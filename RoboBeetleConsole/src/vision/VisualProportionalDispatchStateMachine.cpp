#include "vision/VisualProportionalDispatchStateMachine.h"

namespace rb::vision
{
namespace
{
QString outcomeName(CommandTerminalResult r)
{
    switch (r) {
    case CommandTerminalResult::Ok:
        return QStringLiteral("OK");
    case CommandTerminalResult::Busy:
        return QStringLiteral("BUSY");
    case CommandTerminalResult::Rejected:
        return QStringLiteral("REJECTED");
    case CommandTerminalResult::OutcomeUnknown:
        return QStringLiteral("OUTCOME_UNKNOWN");
    }
    return {};
}
bool validArea(const std::optional<VisualDiagnosticSnapshot> &s)
{
    return s && !s->awaitingVideo && s->target && s->target->target.areaCells &&
           *s->target->target.areaCells > 0;
}
} // namespace
VisualProportionalDispatchStateMachine::VisualProportionalDispatchStateMachine(IConsoleController *c,
                                                                               VisualDispatchConfig dc)
    : controller_(c), dispatchConfig_(dc)
{
}

ArmReason
VisualProportionalDispatchStateMachine::eligibility(const VisualDispatchInput &in,
                                                    const std::optional<VisualDiagnosticSnapshot> &s) const
{
    if (!validVisualApproachConfig(config) || !robobeetle::protocol::valid_proportional_config(q))
        return ArmReason::InvalidConfig;
    if (in.nowMs < 0 || (lastTime_ && in.nowMs < *lastTime_))
        return ArmReason::InvalidTime;
    if (!dryRun) {
        if (!in.linkConnected)
            return ArmReason::LinkDisconnected;
        if (!in.controlOwned)
            return ArmReason::ControlNotOwned;
        if ((in.enabledMask & 0x1f) != 0x1f)
            return ArmReason::ServosNotEnabled;
        if ((in.poseKnownMask & 0x1f) != 0x1f)
            return ArmReason::PoseUnknown;
        if (!controller_->hasCpgSchema2() || controller_->cpgFeatureLevel() < 2)
            return ArmReason::ProportionalUnavailable;
    }
    if (channelBusy)
        return ArmReason::ProportionalChannelBusy;
    if (in.axis != VisualAxisMode::Yaw) {
        switch (evaluateDepthEnvelope(in.depth, depth_, dispatchConfig_.depth).state) {
        case DepthEnvelopeState::Unavailable:
            return ArmReason::DepthUnavailable;
        case DepthEnvelopeState::NotZeroed:
            return ArmReason::DepthNotZeroed;
        case DepthEnvelopeState::HardLimit:
            return ArmReason::DepthHardLimit;
        default:
            break;
        }
    }
    if (in.state != VisualState::Tracking)
        return ArmReason::NotTracking;
    if (!validArea(s))
        return ArmReason::AreaUnavailable;
    if (!armed_ && !dryRun &&
        (controller_->motionState() != MotionState::Stopped || controller_->isMotionActive()))
        return ArmReason::MotionNotStopped;
    return ArmReason::Ready;
}
ArmReason VisualProportionalDispatchStateMachine::arm(const VisualDispatchInput &in,
                                                      const std::optional<VisualDiagnosticSnapshot> &s)
{
    const auto reason = eligibility(in, s);
    if (reason != ArmReason::Ready || stopAwaiting_)
        return reason == ArmReason::Ready ? ArmReason::MotionNotStopped : reason;
    armed_ = true;
    area_ = {};
    lastAcquiredFrame_.reset();
    noTargetSince_.reset();
    stopTimeoutCount_ = 0;
    info_ = {};
    info_.proportional = true;
    info_.dryRun = dryRun;
    evaluate(in, s);
    return ArmReason::Ready;
}
void VisualProportionalDispatchStateMachine::note(const QString &name, qint64 now, const QString &detail)
{
    auto entry = info_;
    entry.event = name;
    entry.detail = detail;
    entry.nowMs = now;
    if (event)
        event(entry);
}
void VisualProportionalDispatchStateMachine::dispatch(std::optional<quint32> id, ProposedCommand command,
                                                      const QString &result, qint64 now, quint8 raw,
                                                      qint64 rtt)
{
    if (record)
        record({now, id.value_or(0), id, command, result, raw, rtt, info_});
}
void VisualProportionalDispatchStateMachine::submitStop(qint64 now)
{
    stopSent_ = now;
    if (dryRun) {
        stopAwaiting_ = false;
        lastStopAccepted_ = now;
        submittedPitch_ = 0;
        if (operatorStop_)
            operatorResult_ = CommandTerminalResult::Ok;
        dispatch({}, ProposedCommand::Stop, QStringLiteral("SIMULATED"), now);
        return;
    }
    const auto id = controller_->submitVisualProportionalStop();
    if (id)
        stopIds_.insert(*id);
    info_.submitted = id.has_value();
    dispatch(id, ProposedCommand::Stop, id ? QStringLiteral("SENT") : QStringLiteral("LOCAL_REJECTED"), now);
    if (!id && operatorStop_)
        operatorResult_ = CommandTerminalResult::Rejected;
}
void VisualProportionalDispatchStateMachine::beginStop(qint64 now, bool disarmed, bool op)
{
    if (disarmed)
        armed_ = false;
    startId_.reset();
    streaming_ = false;
    setpoint_ = {};
    info_.setpoint = setpoint_;
    info_.submitted = false;
    if (stopEpisode_ && !op)
        return;
    stopEpisode_ = true;
    operatorStop_ = op;
    stopIds_.clear();
    stopTimeoutCount_ = 0;
    stopAwaiting_ = true;
    if (op)
        operatorResult_.reset();
    submitStop(now);
}
void VisualProportionalDispatchStateMachine::disarm(qint64 now)
{
    const bool needed = armed_ || stopAwaiting_ || streaming_ || startId_;
    armed_ = false;
    area_ = {};
    lastAcquiredFrame_.reset();
    noTargetSince_.reset();
    if (needed)
        beginStop(now, true, true);
}
void VisualProportionalDispatchStateMachine::start(qint64 now)
{
    stopEpisode_ = false;
    operatorStop_ = false;
    stopIds_.clear();
    startSent_ = now;
    if (dryRun) {
        streaming_ = true;
        dispatch({}, mode_, QStringLiteral("SIMULATED"), now);
        stream(now);
        return;
    }
    startId_ = controller_->submitVisualProportionalStart(q);
    info_.submitted = startId_.has_value();
    dispatch(startId_, mode_, startId_ ? QStringLiteral("SENT") : QStringLiteral("LOCAL_REJECTED"), now);
    if (!startId_) {
        note(QStringLiteral("START_FAILED"), now, QStringLiteral("LOCAL_REJECTED"));
        beginStop(now, true);
    }
}
void VisualProportionalDispatchStateMachine::stream(qint64 now)
{
    info_.setpoint = setpoint_;
    if (dryRun)
        return;
    info_.submitted = controller_->setProportionalInput(setpoint_.throttle, setpoint_.turn, setpoint_.pitch);
    if (info_.submitted)
        submittedPitch_ = setpoint_.pitch;
    else {
        note(QStringLiteral("STREAM_REJECTED"), now);
        beginStop(now, true);
    }
}
void VisualProportionalDispatchStateMachine::terminal(quint32 id, CommandTerminalResult result, quint8 raw,
                                                      qint64 rtt, qint64 now)
{
    if (stopIds_.contains(id)) {
        dispatch(id, ProposedCommand::Stop, outcomeName(result), now, raw, rtt);
        if (operatorStop_)
            operatorResult_ = result;
        if (result == CommandTerminalResult::Ok && stopAwaiting_) {
            stopAwaiting_ = false;
            lastStopAccepted_ = now;
            submittedPitch_ = 0;
            stopTimeoutCount_ = 0;
            stopIds_.clear();
        }
        return;
    }
    if (!startId_ || id != *startId_)
        return;
    startId_.reset();
    dispatch(id, mode_, outcomeName(result), now, raw, rtt);
    if (result != CommandTerminalResult::Ok || now - startSent_ >= dispatchConfig_.ackTimeoutMs || !armed_) {
        note(QStringLiteral("START_FAILED"), now, outcomeName(result));
        beginStop(now, true);
        return;
    }
    streaming_ = true;
    // The session immediately re-evaluates the latest snapshot/depth before streaming.
}
void VisualProportionalDispatchStateMachine::telemetry(qint64 now)
{
    // Remote ends proportionalActive only after observing this accepted session's
    // proportional telemetry. Stopped batches from the preceding session can
    // retain their stop reason before and just after a new Start ACK.
    if (!dryRun && armed_ && streaming_ && !controller_->proportionalActive() && !stopEpisode_) {
        const auto reason = controller_->proportionalStopReason();
        if (reason == 3 || reason == 5) {
            if (reason == 3)
                note(QStringLiteral("SETPOINT_TIMEOUT"), now);
            beginStop(now, true);
        }
    }
}
void VisualProportionalDispatchStateMachine::evaluate(const VisualDispatchInput &in,
                                                      const std::optional<VisualDiagnosticSnapshot> &snapshot)
{
    info_.proportional = true;
    info_.dryRun = dryRun;
    info_.submitted = false;
    info_.nowMs = in.nowMs;
    if (snapshot) {
        info_.frameId = snapshot->frameId;
        info_.sessionId = snapshot->sessionId;
    }
    const auto env = evaluateDepthEnvelope(in.depth, depth_, dispatchConfig_.depth);
    depth_ = env.next;
    envelope_ = env.state;
    if (in.nowMs < 0 || (lastTime_ && in.nowMs < *lastTime_)) {
        if (armed_)
            beginStop(lastTime_.value_or(0), true);
        return;
    }
    lastTime_ = in.nowMs;
    if (!dryRun && (!in.linkConnected || !in.controlOwned)) {
        if (armed_)
            beginStop(in.nowMs, true);
        stopAwaiting_ = false;
        stopIds_.clear();
        startId_.reset();
        return;
    }
    if (armed_) {
        const auto why = eligibility(in, snapshot);
        if (why != ArmReason::Ready && why != ArmReason::NotTracking && why != ArmReason::AreaUnavailable)
            beginStop(in.nowMs, true);
        if (in.state == VisualState::Stale || in.state == VisualState::InferenceOff)
            beginStop(in.nowMs, true);
        if (in.axis != VisualAxisMode::Yaw && armed_) {
            const auto forbidden = [&](qint16 pitch) {
                return (envelope_ == DepthEnvelopeState::Surface && pitch > 0) ||
                       (envelope_ == DepthEnvelopeState::SoftFloor && pitch < 0);
            };
            if (forbidden(submittedPitch_) || (startId_ && forbidden(setpoint_.pitch)) ||
                (!dryRun && forbidden(controller_->actualProportionalPitch())))
                beginStop(in.nowMs, false);
        }
    }
    if (stopAwaiting_ && !operatorStop_ && in.nowMs - stopSent_ >= dispatchConfig_.ackTimeoutMs) {
        ++stopTimeoutCount_;
        submitStop(in.nowMs);
    }
    if (!armed_)
        return;
    if (startId_ && in.nowMs - startSent_ >= dispatchConfig_.ackTimeoutMs) {
        note(QStringLiteral("START_FAILED"), in.nowMs, QStringLiteral("OUTCOME_UNKNOWN"));
        beginStop(in.nowMs, true);
        return;
    }
    if (in.state == VisualState::NoTarget) {
        if (!noTargetSince_)
            noTargetSince_ = in.nowMs;
    } else
        noTargetSince_.reset();
    const bool lost = in.state == VisualState::Lost ||
                      (noTargetSince_ && in.nowMs - *noTargetSince_ >= VisualPolicyConfig{}.lost_ms);
    if (lost) {
        area_ = {};
        info_.area.reset();
        info_.state = QStringLiteral("APPROACH");
        lastAcquiredFrame_.reset();
        beginStop(in.nowMs, false);
        return;
    }
    const bool observation =
        snapshot && in.state == VisualState::Tracking && !snapshot->awaitingVideo && snapshot->target;
    if (observation && snapshot->associationStatus == AssociationStatus::Acquired &&
        (!lastAcquiredFrame_ || *lastAcquiredFrame_ != static_cast<qint64>(snapshot->target->frameId))) {
        area_ = {};
        lastAcquiredFrame_ = snapshot->target->frameId;
    }
    const auto areaInput =
        VisualApproachInput{in.nowMs,
                            snapshot ? snapshot->sessionId : 0,
                            observation ? snapshot->target->frameId : 0,
                            observation && snapshot->target->target.areaCells
                                ? std::optional<double>{*snapshot->target->target.areaCells}
                                : std::nullopt,
                            observation,
                            VisualPolicyConfig{}.max_miss_ms};
    VisualApproachResult result;
    if (snapshot && in.state == VisualState::Tracking && snapshot->awaitingVideo) {
        // Video catch-up is not a new area observation or a target loss. Keep
        // dwell and missing-area episode timestamps across the paired events.
        result.next = area_;
        result.area_f = area_.filteredArea;
    } else {
        result = evaluateVisualApproach(area_, areaInput, config);
    }
    // Resume area dwell may finish during STOPPING; preserve ARRIVED until the
    // ordinary STOP is accepted for a full second and the controller is stopped.
    if (result.arrivedExit && (stopAwaiting_ || (lastStopAccepted_ && in.nowMs - *lastStopAccepted_ < 1000) ||
                               (!dryRun && controller_->motionState() != MotionState::Stopped))) {
        result.next.state = ApproachState::Arrived;
        result.next.resumeSinceMs = area_.resumeSinceMs.value_or(in.nowMs);
        result.arrivedExit = false;
    }
    area_ = result.next;
    info_.area = result.area_f;
    info_.state = QString::fromLatin1(approachStateName(area_.state));
    if (result.areaMissingLost) {
        note(QStringLiteral("AREA_MISSING_LOST"), in.nowMs);
        beginStop(in.nowMs, false);
        return;
    }
    if (result.arrivedEnter) {
        setpoint_ = {};
        info_.setpoint = setpoint_;
        note(QStringLiteral("ARRIVED_ENTER"), in.nowMs);
        beginStop(in.nowMs, false);
    }
    if (result.arrivedExit)
        note(QStringLiteral("ARRIVED_EXIT"), in.nowMs);
    if (area_.state == ApproachState::Arrived || stopAwaiting_)
        return;
    if (in.state == VisualState::Tracking && in.suggestion == ProposedCommand::Stop) {
        beginStop(in.nowMs, false);
        return;
    }
    if (observation && info_.area) {
        auto desired = in.suggestion;
        if (desired != ProposedCommand::Hold && desired != ProposedCommand::Stop) {
            if ((desired == ProposedCommand::Ascend && envelope_ == DepthEnvelopeState::Surface) ||
                (desired == ProposedCommand::Descend && envelope_ == DepthEnvelopeState::SoftFloor))
                desired = ProposedCommand::Forward;
            if (!lastSwitch_ || in.nowMs - *lastSwitch_ >= 1000 || desired == mode_) {
                if (mode_ != desired || !lastSwitch_)
                    lastSwitch_ = in.nowMs;
                mode_ = desired;
            }
            // Depth safety overrides ordinary command-switch dwell.
            if ((mode_ == ProposedCommand::Ascend && envelope_ == DepthEnvelopeState::Surface) ||
                (mode_ == ProposedCommand::Descend && envelope_ == DepthEnvelopeState::SoftFloor))
                mode_ = ProposedCommand::Forward;
            setpoint_ = mapVisualSetpoint(mode_, *info_.area, config);
            info_.setpoint = setpoint_;
        }
    }
    if (startId_)
        return;
    if (streaming_) {
        stream(in.nowMs);
        return;
    }
    if (observation && validArea(snapshot) && info_.area &&
        (!lastStopAccepted_ || in.nowMs - *lastStopAccepted_ >= 1000) &&
        (dryRun || (controller_->motionState() == MotionState::Stopped && !controller_->isMotionActive())))
        start(in.nowMs);
}
} // namespace rb::vision
