#include "vision/VisualDispatchSession.h"

namespace rb::vision {
namespace {
MotionMode motionMode(ProposedCommand command)
{
    switch (command) {
    case ProposedCommand::Forward: return MotionMode::Forward;
    case ProposedCommand::TurnLeft: return MotionMode::TurnLeft;
    case ProposedCommand::TurnRight: return MotionMode::TurnRight;
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
    : QObject(p), controller_(c), snapshot_(std::move(s)), now_(std::move(n)), machine_(*this,config)
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
    timer_.start();
}
VisualDispatchInput VisualDispatchSession::input() const
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
    // The diagnostic snapshot owns video grace and freshness. Use its state
    // verbatim and pair its proposal with the filtered error from that result.
    if (auto snapshot = snapshot_()) {
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

void VisualDispatchSession::selectTurnSign(std::optional<int> sign)
{
    if (sign && *sign != 1 && *sign != -1) sign.reset();
    if (selectedSign_ == sign) return;
    disarm();
    selectedSign_ = sign;
    confirmedSign_.reset();
    emit statusChanged();
}

void VisualDispatchSession::confirmTurnSign()
{
    confirmedSign_ = selectedSign_;
    emit statusChanged();
}
void VisualDispatchSession::manualInput(ManualInputKind kind)
{
    if (kind == ManualInputKind::NonMotion) return;
    machine_.manualInput(kind);
    clearAssociations();
    emit statusChanged();
}
void VisualDispatchSession::timerTick()
{
    if (!enabled_) return;
    machine_.evaluate(input());
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
                           terminalName(result), raw, rtt});
    if (association.operatorStop) {
        operatorStopResult_ = result;
    } else {
        if (association.command != ProposedCommand::Stop && raw == 6
            && result == CommandTerminalResult::Rejected
            && controller_->inferredPoseKnownMask() != 0) {
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
