#include "vision/VisualDispatchSession.h"

namespace rb::vision {
namespace {
MotionMode motionMode(ProposedCommand command)
{
    switch (command) {
    case ProposedCommand::Forward: return MotionMode::Forward;
    case ProposedCommand::TurnLeft: return MotionMode::TurnLeft;
    case ProposedCommand::TurnRight: return MotionMode::TurnRight;
    case ProposedCommand::Ascend: return MotionMode::Ascend;
    case ProposedCommand::Descend: return MotionMode::Descend;
    default: return MotionMode::Stop;
    }
}
QString terminalName(CommandTerminalResult result)
{
    switch (result) {
    case CommandTerminalResult::Ok: return QStringLiteral("OK");
    case CommandTerminalResult::Busy: return QStringLiteral("BUSY");
    case CommandTerminalResult::Rejected: return QStringLiteral("REJECTED");
    case CommandTerminalResult::OutcomeUnknown: return QStringLiteral("OUTCOME_UNKNOWN");
    }
    return {};
}
DispatchOutcome policyOutcome(CommandTerminalResult result)
{
    switch (result) {
    case CommandTerminalResult::Ok: return DispatchOutcome::Ok;
    case CommandTerminalResult::Busy: return DispatchOutcome::Busy;
    case CommandTerminalResult::Rejected: return DispatchOutcome::Rejected;
    case CommandTerminalResult::OutcomeUnknown: return DispatchOutcome::OutcomeUnknown;
    }
    return DispatchOutcome::OutcomeUnknown;
}
}
VisualDispatchSession::VisualDispatchSession(IConsoleController *c, SnapshotProvider s, NowMs n, VisualDispatchConfig config, QObject *p)
    : QObject(p), controller_(c), snapshot_(std::move(s)), now_(std::move(n)), machine_(*this,config), requiredServoMask_(config.requiredServoMask)
{
    clock_.start();
    if (!now_) now_ = [this] { return clock_.elapsed(); };
    timer_.setTimerType(Qt::PreciseTimer);
    timer_.setInterval(50);
    connect(&timer_, &QTimer::timeout, this, &VisualDispatchSession::timerTick);
    connect(controller_, &IConsoleController::commandTerminal, this,
        [this](quint32 id, CommandTerminalResult result, quint8 raw, qint64 rtt) {
            // Only reentrant submissions defer: ordinary unknown outcomes must
            // trigger the safety STOP before the controller releases authority.
            if (submitting_) {
                QTimer::singleShot(0, this, [this, id, result, raw, rtt] {
                    terminal(id, result, raw, rtt);
                });
            } else {
                terminal(id, result, raw, rtt);
            }
        });
    connect(controller_, &IConsoleController::authorityStateChanged, this,
            [this](ControlAuthorityState, bool active) {
        if (active) return;
        if (enabled_) machine_.evaluate(input());
        else machine_.manualInput(ManualInputKind::Motion);
        clearAssociations(true);
        emit statusChanged();
    });
    connect(controller_, &IConsoleController::connectionStateChanged, this,
            [this](TransportState state) {
        if (state == TransportState::Connected) return;
        if (enabled_) machine_.evaluate(input());
        else machine_.manualInput(ManualInputKind::Motion);
        clearAssociations(true);
        emit statusChanged();
    });
    // The controller can emit this while it is being destroyed: the session is
    // the context object, so the connection dies with the session (or first).
    connect(controller_, &IConsoleController::controlDepthSampleChanged,
            this, &VisualDispatchSession::depthSampleChanged);
    timer_.start();
}
VisualDispatchSession::VisualDispatchSession(IConsoleController *controller,
    VisualDiagnosticSession *diagnostic, VisualDispatchConfig config, QObject *parent)
    : VisualDispatchSession(controller,
        [diagnostic] { return std::optional{diagnostic->snapshot()}; },
        [diagnostic] { return diagnostic->monotonicNowMs(); }, config, parent)
{
    diagnostic_ = diagnostic;
    connect(diagnostic, &VisualDiagnosticSession::diagnosticChanged,
            this, &VisualDispatchSession::diagnosticChanged);
}

void VisualDispatchSession::depthSampleChanged()
{
    // Pitch/Both only, and only while there is something to protect.
    if (!enabled_ || axis_ == VisualAxisMode::Yaw
        || !(machine_.armed() || machine_.stopAwaiting())) return;
    deliverDepth(controller_->controlDepthSample());
}

void VisualDispatchSession::deliverDepth(const std::optional<DepthControlSample> &depth)
{
    // Queue the sample by value: a later in-range sample must not erase a
    // transient hard-limit reading, and arrival order is preserved.
    depthQueue_.push_back(depth);
    drainDepthQueue();
}

void VisualDispatchSession::drainDepthQueue()
{
    if (submitting_ || evaluating_) {
        if (!depthDrainScheduled_) {
            depthDrainScheduled_ = true;
            QTimer::singleShot(0, this, [this] {
                depthDrainScheduled_ = false;
                drainDepthQueue();
            });
        }
        return;
    }
    while (!depthQueue_.empty()) {
        const auto depth = depthQueue_.front();
        depthQueue_.pop_front();
        evaluating_ = true;
        auto in = input();
        in.depth = depth;
        machine_.evaluate(in);
        evaluating_ = false;
        emit statusChanged();
    }
}

void VisualDispatchSession::setAxisMode(VisualAxisMode axis)
{
    if (axis_ == axis) return;
    // Same rule as Disarm: one operator STOP when armed or a STOP is unconfirmed.
    disarm();
    axis_ = axis;
    if (diagnostic_) diagnostic_->setAxisMode(axis);
    reason_ = ArmReason::NotTracking;
    emit statusChanged();
}

VisualDepthCsvInfo VisualDispatchSession::depthCsvInfo() const
{
    VisualDepthCsvInfo info;
    info.envelope = QString::fromLatin1(depthEnvelopeStateName(machine_.depthEnvelope()));
    if (const auto sample = controller_->controlDepthSample()) {
        info.rawM = sample->rawDepthM;
        info.calibratedM = sample->calibratedDepthM;
        if (sample->ageMs >= 0) info.ageMs = sample->ageMs;
    }
    return info;
}

void VisualDispatchSession::diagnosticChanged(const VisualDiagnosticSnapshot &snapshot)
{
    if (!enabled_ || (snapshot.state != VisualState::Stale
        && snapshot.state != VisualState::InferenceOff && snapshot.state != VisualState::Lost)) return;
    if (submitting_ || evaluating_) {
        // Retain the event's value: a newer snapshot may already be TRACKING
        // when submission/evaluation unwinds. Never collapse a safety transient.
        QTimer::singleShot(0, this, [this, snapshot] { diagnosticChanged(snapshot); });
        return;
    }
    evaluating_ = true;
    machine_.evaluate(input(&snapshot));
    evaluating_ = false;
    emit statusChanged();
}

VisualDispatchInput VisualDispatchSession::input(const VisualDiagnosticSnapshot *eventSnapshot) const
{
    VisualDispatchInput result;
    result.nowMs = now_();
    result.linkConnected = controller_->backendKind() == ConsoleBackendKind::RemoteRbrp
        && controller_->isConnected();
    result.controlOwned = controller_->isControlActive();
    for (int i = 0; i < kServoCount; ++i) {
        const auto id = static_cast<ServoId>(i);
        if (controller_->isServoEnabled(id) && !controller_->isServoDisablePending(id))
            result.enabledMask |= 1U << i;
    }
    result.poseKnownMask = controller_->inferredPoseKnownMask();
    result.confirmedTurnSign = confirmedSign_;
    result.axis = axis_;
    result.depth = controller_->controlDepthSample();
    // The diagnostic snapshot owns video grace and freshness. Use its state
    // verbatim and pair its proposal with the filtered error from that result.
    const auto latest = eventSnapshot ? std::nullopt : snapshot_();
    const auto *snapshot = eventSnapshot ? eventSnapshot : latest ? &*latest : nullptr;
    if (snapshot) {
        result.state = snapshot->state;
        result.suggestion = snapshot->command.proposed;
        result.ex = snapshot->command.ex_f;
    }
    return result;
}
void VisualDispatchSession::setFeatureEnabled(bool enabled)
{
    if (enabled_ == enabled) return;
    if (!enabled) disarm();
    enabled_ = enabled;
    emit statusChanged();
}

ArmReason VisualDispatchSession::arm()
{
    if (!enabled_) {
        reason_ = ArmReason::NotTracking;
        emit statusChanged();
        return reason_;
    }
    reason_ = machine_.arm(input());
    emit statusChanged();
    return reason_;
}

void VisualDispatchSession::disarm()
{
    const bool stop = machine_.armed() || machine_.stopAwaiting();
    machine_.manualInput(ManualInputKind::Motion);
    clearAssociations();
    if (stop) {
        operatorStopResult_.reset();
        submit(0, ProposedCommand::Stop, true);
    }
    emit statusChanged();
}

void VisualDispatchSession::manualInput(ManualInputKind kind)
{
    if (kind == ManualInputKind::NonMotion) return;
    machine_.manualInput(kind);
    clearAssociations();
    emit statusChanged();
}

AutoFollowReadiness VisualDispatchSession::readiness() const
{
    // Pure query: reuse input()'s accessors but never touch machine_ state.
    AutoFollowReadiness result;
    result.linkAndControl = controller_->backendKind() == ConsoleBackendKind::RemoteRbrp
        && controller_->isConnected() && controller_->isControlActive();

    quint16 enabledMask = 0;
    for (int i = 0; i < kServoCount; ++i) {
        const auto id = static_cast<ServoId>(i);
        if (controller_->isServoEnabled(id) && !controller_->isServoDisablePending(id))
            enabledMask |= 1U << i;
    }
    result.servosReady = (enabledMask & requiredServoMask_) == requiredServoMask_
        && (controller_->inferredPoseKnownMask() & requiredServoMask_) == requiredServoMask_;

    const auto snapshot = snapshot_();
    result.tracking = snapshot.has_value() && snapshot->state == VisualState::Tracking;

    // Pitch/Both add the front axis servo and a usable depth reading. The
    // yaw-only checklist above keeps its meaning unchanged.
    result.pitchActive = axis_ != VisualAxisMode::Yaw;
    if (result.pitchActive) {
        result.frontAxisReady = (enabledMask & kFrontAxisServoMask) == kFrontAxisServoMask
            && (controller_->inferredPoseKnownMask() & kFrontAxisServoMask) == kFrontAxisServoMask;
        VisualDispatchInput probe;
        probe.depth = controller_->controlDepthSample();
        switch (machine_.envelopeFor(probe)) {
        case DepthEnvelopeState::Unavailable:
            result.depthReady = false;
            result.depthNote = QStringLiteral("Depth unavailable");
            break;
        case DepthEnvelopeState::NotZeroed:
            result.depthReady = false;
            result.depthNote = QStringLiteral("Depth not zeroed");
            break;
        case DepthEnvelopeState::HardLimit:
            result.depthReady = false;
            result.depthNote = QStringLiteral("Depth at hard limit");
            break;
        default:
            break;
        }
    }
    return result;
}
void VisualDispatchSession::timerTick()
{
    if (!enabled_ || submitting_ || evaluating_) return;
    evaluating_ = true;
    machine_.evaluate(input());
    evaluating_ = false;
    emit statusChanged();
}
std::optional<ProposedCommand> VisualDispatchSession::currentMode() const
{
    return machine_.modeConfirmed() ? machine_.currentMode() : std::nullopt;
}

void VisualDispatchSession::send(const DispatchRequest &request)
{
    submit(request.id, request.command, false);
}

void VisualDispatchSession::submit(quint64 policyId, ProposedCommand command,
                                   bool operatorStop)
{
    // Bounded audit correlation. Retired requests cannot update policy state.
    if (associations_.size() >= 64) {
        auto it = associations_.begin();
        const auto id = it.key();
        const auto old = it.value();
        associations_.erase(it);
        emit dispatchRecorded({now_(), old.policyId, id, old.command,
                               QStringLiteral("OUTCOME_UNKNOWN"), 0xff, -1});
    }
    submitting_ = true;
    const auto id = controller_->backendKind() == ConsoleBackendKind::RemoteRbrp
        ? controller_->submitVisualMotion(motionMode(command)) : std::nullopt;
    submitting_ = false;
    if (id) associations_.insert(*id, {policyId, command, operatorStop});
    emit dispatchRecorded({now_(), policyId, id, command,
                           id ? QStringLiteral("SENT") : QStringLiteral("LOCAL_REJECTED"),
                           0xff, -1});
    if (!id) {
        if (operatorStop) {
            operatorStopResult_ = CommandTerminalResult::Rejected;
        } else {
            QTimer::singleShot(0, this, [this, policyId] {
                machine_.acknowledge(policyId, DispatchOutcome::Rejected, now_());
                emit statusChanged();
            });
        }
    }
}

void VisualDispatchSession::terminal(quint32 id, CommandTerminalResult result,
                                     quint8 raw, qint64 rtt)
{
    auto it = associations_.find(id);
    if (it == associations_.end()) return;
    const auto association = *it;
    associations_.erase(it);
    emit dispatchRecorded({now_(), association.policyId, id, association.command,
                           (association.command == ProposedCommand::Stop
                            && result == CommandTerminalResult::Ok && raw == 0xff)
                               ? QStringLiteral("OK (covered)") : terminalName(result), raw, rtt});
    if (association.operatorStop) {
        operatorStopResult_ = result;
    } else {
        if (association.command != ProposedCommand::Stop && raw == 6
            && result == CommandTerminalResult::Rejected
            && requiredServoMask_ != 0
            && (controller_->inferredPoseKnownMask() & machine_.requiredMask(axis_))
                == machine_.requiredMask(axis_)) {
            poseMismatch_ = QStringLiteral("可能是姿态未知（Qt 推断与固件不一致）");
        }
        machine_.acknowledge(association.policyId, policyOutcome(result), now_());
        if (association.command == ProposedCommand::Stop
            && result == CommandTerminalResult::Ok) {
            clearAssociations();
        }
    }
    emit statusChanged();
}

void VisualDispatchSession::clearAssociations(bool includeOperator)
{
    const auto retired = associations_;
    for (auto it = retired.cbegin(); it != retired.cend(); ++it) {
        if (it->operatorStop && !includeOperator) continue;
        associations_.remove(it.key());
        if (it->operatorStop) operatorStopResult_ = CommandTerminalResult::OutcomeUnknown;
        emit dispatchRecorded({now_(), it->policyId, it.key(), it->command,
                               QStringLiteral("OUTCOME_UNKNOWN"), 0xff, -1});
    }
}
} // namespace rb::vision
