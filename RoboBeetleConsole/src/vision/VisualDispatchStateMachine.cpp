#include "vision/VisualDispatchStateMachine.h"

#include <cmath>

namespace rb::vision {
VisualDispatchStateMachine::VisualDispatchStateMachine(VisualCommandSendPort &port,
                                                     VisualDispatchConfig config)
    : port_(port), config_(config) {}

ArmReason VisualDispatchStateMachine::eligibility(const VisualDispatchInput &input) const
{
    if (config_.ackTimeoutMs <= 0 || config_.requiredServoMask == 0 ||
        (config_.requiredServoMask & ~0x001fU) != 0) return ArmReason::InvalidConfig;
    if (input.nowMs < 0 || (lastObservedMs_ && input.nowMs < *lastObservedMs_)) return ArmReason::InvalidTime;
    if (!input.linkConnected) return ArmReason::LinkDisconnected;
    if (!input.controlOwned) return ArmReason::ControlNotOwned;
    if ((input.enabledMask & config_.requiredServoMask) != config_.requiredServoMask) return ArmReason::ServosNotEnabled;
    if ((input.poseKnownMask & config_.requiredServoMask) != config_.requiredServoMask) return ArmReason::PoseUnknown;
    if (!input.confirmedTurnSign || (*input.confirmedTurnSign != 1 && *input.confirmedTurnSign != -1)) return ArmReason::TurnSignUnconfirmed;
    if (input.state != VisualState::Tracking) return ArmReason::NotTracking;
    return ArmReason::Ready;
}

bool VisualDispatchStateMachine::observeTime(std::int64_t nowMs)
{
    if (nowMs < 0 || (lastObservedMs_ && nowMs < *lastObservedMs_)) return false;
    lastObservedMs_ = nowMs;
    return true;
}

ArmReason VisualDispatchStateMachine::arm(const VisualDispatchInput &input)
{
    const auto reason = eligibility(input);
    if (reason != ArmReason::Ready) {
        // Rejected operator actions are observations too. Keep the clock
        // consistent with any safety STOP emitted by this call.
        (void)observeTime(input.nowMs);
        if (armed_) {
            failSafe(lastObservedMs_.value_or(0));
        }
        if (!input.linkConnected || !input.controlOwned) {
            armed_ = false;
            invalidateRequests(); // Also applies to a disarmed pending safety STOP.
        }
        return reason;
    }
    (void)observeTime(input.nowMs);
    armed_ = true;
    stopTimeoutAlert_ = false; // Explicit eligible operator action clears the alert.
    return ArmReason::Ready;
}

void VisualDispatchStateMachine::invalidateRequests()
{
    pendingMotion_.reset();
    firstStopId_.reset();
    stopAwaiting_ = false;
    stopEpisode_ = false;
    modeConfirmed_ = false;
}

void VisualDispatchStateMachine::manualInput(ManualInputKind kind)
{
    if (kind == ManualInputKind::NonMotion) return;
    armed_ = false;
    invalidateRequests();
}

void VisualDispatchStateMachine::sendStop(std::int64_t nowMs)
{
    const DispatchRequest request{nextId_++, ProposedCommand::Stop};
    if (!firstStopId_) firstStopId_ = request.id;
    lastStopId_ = request.id;
    lastStopSentMs_ = nowMs;
    stopAwaiting_ = true;
    port_.send(request);
}

void VisualDispatchStateMachine::beginStop(std::int64_t nowMs)
{
    pendingMotion_.reset(); // Its eventual OK cannot overwrite this stop episode.
    if (stopEpisode_) return;
    stopEpisode_ = true;
    firstStopId_.reset();
    stopTimeoutCount_ = 0;
    sendStop(nowMs);
}

void VisualDispatchStateMachine::failSafe(std::int64_t nowMs)
{
    beginStop(nowMs);
    armed_ = false;
}

void VisualDispatchStateMachine::retryStop(std::int64_t nowMs)
{
    if (!stopAwaiting_ || nowMs - lastStopSentMs_ < config_.ackTimeoutMs) return;
    // Count at most one expiry per evaluation. Never burst after a long stall.
    if (stopTimeoutCount_ < 3) ++stopTimeoutCount_;
    if (stopTimeoutCount_ >= 3) stopTimeoutAlert_ = true;
    sendStop(nowMs);
}

void VisualDispatchStateMachine::evaluate(const VisualDispatchInput &input)
{
    if (!observeTime(input.nowMs)) {
        if (armed_) failSafe(lastObservedMs_.value_or(0));
        return;
    }
    if (!input.linkConnected || !input.controlOwned) {
        if (armed_) failSafe(input.nowMs); // One immediate STOP attempt on loss.
        armed_ = false;
        invalidateRequests(); // Link/authority loss ends retries and ACK associations.
        return;
    }
    if (armed_) {
        const auto reason = eligibility(input);
        if (reason != ArmReason::Ready &&
            input.state != VisualState::NoTarget && input.state != VisualState::Lost) {
            failSafe(input.nowMs);
        } else if (reason != ArmReason::Ready && reason != ArmReason::NotTracking) {
            failSafe(input.nowMs); // Servo/pose/direction invalidation even in LOST.
        } else if (input.state == VisualState::NoTarget ||
                   (input.state == VisualState::Tracking && input.suggestion == ProposedCommand::Hold)) {
            // A transient miss is HOLD, not LOST. Never assume an old mode
            // after arming, manual takeover or link-loss invalidation.
            if (!modeConfirmed_) beginStop(input.nowMs);
        } else if (input.state == VisualState::Lost || input.suggestion == ProposedCommand::Stop) {
            beginStop(input.nowMs);
        }
    }
    if (stopAwaiting_) { retryStop(input.nowMs); return; }
    if (!armed_) return;
    if (pendingMotion_) {
        if (input.nowMs - pendingMotion_->sentMs >= config_.ackTimeoutMs) failSafe(input.nowMs);
        return;
    }
    if (input.state != VisualState::Tracking || input.suggestion == ProposedCommand::Stop) return;
    if (input.suggestion == ProposedCommand::Hold) return;
    auto command = input.suggestion;
    if (command == ProposedCommand::TurnLeft || command == ProposedCommand::TurnRight) {
        if (!input.ex || !std::isfinite(*input.ex) || *input.ex == 0.0) {
            failSafe(input.nowMs);
            return;
        }
        command = ((*input.ex > 0.0) == (*input.confirmedTurnSign > 0))
            ? ProposedCommand::TurnRight : ProposedCommand::TurnLeft;
    }
    if (modeConfirmed_ && currentMode_ == command) return;
    // Subtract monotonic timestamps rather than adding deadlines (no overflow).
    if ((lastNonStopSentMs_ && input.nowMs - *lastNonStopSentMs_ < VisualDispatchConfig::minDwellMs) ||
        (lastStopAcceptedMs_ && input.nowMs - *lastStopAcceptedMs_ < VisualDispatchConfig::minDwellMs)) return;
    stopEpisode_ = false;
    firstStopId_.reset();
    const DispatchRequest request{nextId_++, command};
    pendingMotion_ = PendingMotion{request, input.nowMs};
    lastNonStopSentMs_ = input.nowMs;
    port_.send(request);
}

void VisualDispatchStateMachine::acknowledge(std::uint64_t id, DispatchOutcome outcome,
                                           std::int64_t nowMs)
{
    const bool stopId = firstStopId_ && id >= *firstStopId_ && id <= lastStopId_;
    const bool motionId = pendingMotion_ && id == pendingMotion_->request.id;
    if (!stopId && !motionId) return; // Old ACK must not even advance the policy clock.
    if (!observeTime(nowMs)) {
        if (armed_) failSafe(lastObservedMs_.value_or(0));
        return;
    }
    if (stopId) {
        if (outcome == DispatchOutcome::Ok && stopAwaiting_) {
            currentMode_ = ProposedCommand::Stop;
            modeConfirmed_ = true;
            stopAwaiting_ = false;
            stopTimeoutCount_ = 0;
            // Receipt time conservatively bounds actual firmware acceptance.
            lastStopAcceptedMs_ = nowMs;
        }
        // Busy/rejected/unknown STOP keeps the episode retry timer intact.
        return;
    }
    if (nowMs - pendingMotion_->sentMs >= config_.ackTimeoutMs) {
        // ACK delivery can itself be the first observation of a missed deadline.
        // A delayed timer must not let a late OK establish an unsafe mode.
        failSafe(nowMs);
        return;
    }
    const auto command = pendingMotion_->request.command;
    pendingMotion_.reset();
    if (outcome == DispatchOutcome::Ok) {
        currentMode_ = command;
        modeConfirmed_ = true;
    } else if (outcome != DispatchOutcome::Busy) {
        failSafe(nowMs);
    }
}

const char *armReasonName(ArmReason reason) noexcept
{
    switch (reason) {
    case ArmReason::Ready: return "READY";
    case ArmReason::InvalidConfig: return "INVALID_CONFIG";
    case ArmReason::InvalidTime: return "INVALID_TIME";
    case ArmReason::LinkDisconnected: return "LINK_DISCONNECTED";
    case ArmReason::ControlNotOwned: return "CONTROL_NOT_OWNED";
    case ArmReason::ServosNotEnabled: return "SERVOS_NOT_ENABLED";
    case ArmReason::PoseUnknown: return "POSE_UNKNOWN";
    case ArmReason::TurnSignUnconfirmed: return "TURN_SIGN_UNCONFIRMED";
    case ArmReason::NotTracking: return "NOT_TRACKING";
    }
    return "UNKNOWN";
}
} // namespace rb::vision
