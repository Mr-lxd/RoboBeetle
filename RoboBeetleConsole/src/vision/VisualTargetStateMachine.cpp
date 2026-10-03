#include "vision/VisualTargetStateMachine.h"
#include <algorithm>
#include <limits>

namespace rb::vision {
namespace {
std::int64_t deadline(std::int64_t start, std::int64_t duration)
{
    const auto limit = std::numeric_limits<std::int64_t>::max();
    return duration > limit - start ? limit : start + duration;
}
}
VisualTargetResult advanceVisualTargetState(const VisualTargetMemory &previous,
    const VisualTargetInput &input, const VisualPolicyConfig &config)
{
    VisualTargetResult result;
    result.next = previous;
    auto &m = result.next;
    auto invalid = [&](VisualState state) {
        m.state = state;
        m.noTargetSinceMs.reset();
        if (input.gate != DetectionDisplayState::AwaitingVideo) {
            m.awaitingVideoSinceMs.reset();
        }
        result.state = state;
        return result;
    };
    if (!validVisualPolicyConfig(config) || input.nowMs < 0
        || (m.lastEvaluatedMs && input.nowMs < *m.lastEvaluatedMs)) {
        return invalid(VisualState::Stale);
    }
    m.lastEvaluatedMs = input.nowMs;
    if (input.arrivedFrameId
        && (!m.highestArrivedFrameId || *input.arrivedFrameId > *m.highestArrivedFrameId)) {
        if (m.lastAdvancedArrivalMs
            && input.nowMs - *m.lastAdvancedArrivalMs >= config.stale_ms) {
            m.noTargetSinceMs.reset();
        }
        m.highestArrivedFrameId = input.arrivedFrameId;
        m.lastAdvancedArrivalMs = input.nowMs;
        result.frameAdvanced = true;
    }
    if (input.gate == DetectionDisplayState::InferenceOff) {
        return invalid(VisualState::InferenceOff);
    }
    if (input.gate == DetectionDisplayState::Stale) {
        return invalid(VisualState::Stale);
    }
    if (!m.lastAdvancedArrivalMs || input.nowMs < *m.lastAdvancedArrivalMs
        || input.nowMs - *m.lastAdvancedArrivalMs >= config.stale_ms) {
        return invalid(VisualState::Stale);
    }
    result.nextDeadlineMs = deadline(*m.lastAdvancedArrivalMs, config.stale_ms);
    // Check before accepting catchup too: its event may precede an overdue timer.
    if (m.awaitingVideoSinceMs
        && input.nowMs - *m.awaitingVideoSinceMs >= config.stale_ms) {
        return invalid(VisualState::Stale);
    }
    if (input.gate == DetectionDisplayState::AwaitingVideo) {
        if (!m.awaitingVideoSinceMs) {
            m.awaitingVideoSinceMs = *m.lastAdvancedArrivalMs;
        }
        if (input.nowMs - *m.awaitingVideoSinceMs >= config.stale_ms) {
            return invalid(VisualState::Stale);
        }
        result.nextDeadlineMs = std::min(*result.nextDeadlineMs,
            deadline(*m.awaitingVideoSinceMs, config.stale_ms));
        result.awaitingVideo = true;
        result.state = m.state; // Do not interrupt a working suggestion while video catches up.
        return result;
    }
    m.awaitingVideoSinceMs.reset();
    if (input.gate == DetectionDisplayState::Target) {
        if (!input.selected || input.selected->frameId != m.highestArrivedFrameId) {
            return invalid(VisualState::Stale);
        }
        m.state = VisualState::Tracking;
        m.noTargetSinceMs.reset();
        result.usableTarget = input.selected;
    } else {
        if (!m.noTargetSinceMs) {
            m.noTargetSinceMs = input.nowMs;
        }
        m.state = input.nowMs - *m.noTargetSinceMs >= config.lost_ms
            ? VisualState::Lost : VisualState::NoTarget;
        if (m.state == VisualState::NoTarget) {
            result.nextDeadlineMs = std::min(*result.nextDeadlineMs,
                deadline(*m.noTargetSinceMs, config.lost_ms));
        }
    }
    result.state = m.state;
    return result;
}
} // namespace rb::vision
