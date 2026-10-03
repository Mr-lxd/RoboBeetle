#include "vision/VisualCommandPolicy.h"

#include <algorithm>

namespace rb::vision {
namespace {

bool validError(double ex) noexcept
{
    return std::isfinite(ex) && ex >= -1.0 && ex <= 1.0;
}

bool validEffective(ProposedCommand command) noexcept
{
    return command == ProposedCommand::Forward || command == ProposedCommand::TurnLeft
        || command == ProposedCommand::TurnRight || command == ProposedCommand::Stop;
}

ProposedCommand turnFor(double error, int turnSign) noexcept
{
    const bool right = (error > 0.0) == (turnSign > 0);
    return right ? ProposedCommand::TurnRight : ProposedCommand::TurnLeft;
}

} // namespace

VisualCommandResult evaluateVisualCommand(
    const VisualCommandMemory &previous, const VisualCommandInput &input,
    const VisualPolicyConfig &config)
{
    VisualCommandResult result;
    result.next = previous;

    const bool validTime = input.nowMs >= 0
        && (!previous.lastEvaluatedMs
            || (*previous.lastEvaluatedMs >= 0 && input.nowMs >= *previous.lastEvaluatedMs))
        && (!previous.lastSwitchMs
            || (*previous.lastSwitchMs >= 0 && input.nowMs >= *previous.lastSwitchMs));
    if (validTime) {
        result.next.lastEvaluatedMs = input.nowMs;
    }

    const auto stop = [&]() {
        result.next.filteredEx.reset();
        result.next.effective = ProposedCommand::Stop;
        if (validTime && previous.effective != ProposedCommand::Stop) {
            result.next.lastSwitchMs = input.nowMs;
        }
        // Keep lastFilteredFrameId: a stopped session cannot reuse a consumed sample.
        return result;
    };

    if (!validTime || !validVisualPolicyConfig(config)
        || !validEffective(previous.effective)
        || (previous.filteredEx && !validError(*previous.filteredEx))) {
        return stop();
    }

    switch (input.state) {
    case VisualState::Stale:
    case VisualState::InferenceOff:
    case VisualState::Lost:
        return stop();
    case VisualState::NoTarget:
        result.proposed = ProposedCommand::Hold;
        result.effective = previous.effective;
        result.ex_f = previous.filteredEx;
        result.yaw_cmd = result.ex_f
            ? std::clamp(config.K_yaw * *result.ex_f, -1.0, 1.0) : 0.0;
        return result;
    case VisualState::Tracking:
        break;
    default:
        return stop();
    }

    if (!input.usableFrameId || !input.ex || !validError(*input.ex)) {
        return stop();
    }
    if (!previous.lastFilteredFrameId || *input.usableFrameId > *previous.lastFilteredFrameId) {
        result.next.filteredEx = previous.filteredEx
            ? config.alpha * *input.ex + (1.0 - config.alpha) * *previous.filteredEx
            : *input.ex;
        result.next.lastFilteredFrameId = input.usableFrameId;
    }
    if (!result.next.filteredEx) {
        result.waitingForSample = true;
        return stop();
    }

    result.ex_f = result.next.filteredEx;
    result.yaw_cmd = std::clamp(config.K_yaw * *result.ex_f, -1.0, 1.0);
    const double magnitude = std::abs(*result.ex_f);
    ProposedCommand candidate = previous.effective;
    const bool wasTurning = previous.effective == ProposedCommand::TurnLeft
        || previous.effective == ProposedCommand::TurnRight;
    if (!wasTurning) {
        candidate = magnitude > config.e_on
            ? turnFor(*result.ex_f, config.turn_sign) : ProposedCommand::Forward;
    } else if (magnitude < config.e_off) {
        candidate = ProposedCommand::Forward;
    } else if (magnitude > config.e_on) {
        candidate = turnFor(*result.ex_f, config.turn_sign);
    }

    if (candidate != previous.effective) {
        // Nonnegative monotonic timestamps make this difference overflow-safe.
        result.dwellBlocked = previous.lastSwitchMs
            && input.nowMs - *previous.lastSwitchMs < config.min_dwell_ms;
        if (!result.dwellBlocked) {
            result.next.effective = candidate;
            result.next.lastSwitchMs = input.nowMs;
        }
    }
    result.effective = result.next.effective;
    result.proposed = result.effective;
    return result;
}

const char *visualStateName(VisualState state) noexcept
{
    switch (state) {
    case VisualState::Tracking: return "TRACKING";
    case VisualState::NoTarget: return "NO_TARGET";
    case VisualState::Lost: return "LOST";
    case VisualState::Stale: return "STALE";
    case VisualState::InferenceOff: return "INFERENCE_OFF";
    }
    return "UNKNOWN";
}

const char *proposedCommandName(ProposedCommand command) noexcept
{
    switch (command) {
    case ProposedCommand::Forward: return "FORWARD";
    case ProposedCommand::TurnLeft: return "TURN_LEFT";
    case ProposedCommand::TurnRight: return "TURN_RIGHT";
    case ProposedCommand::Stop: return "STOP";
    case ProposedCommand::Hold: return "HOLD";
    }
    return "UNKNOWN";
}

} // namespace rb::vision
