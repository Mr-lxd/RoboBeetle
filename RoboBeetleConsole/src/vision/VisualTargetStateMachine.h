#pragma once

#include "vision/TargetState.h"
#include "vision/VisualPolicyConfig.h"
#include <optional>

namespace rb::vision {

struct VisualTargetMemory {
    std::optional<std::int64_t> lastEvaluatedMs;
    std::optional<std::uint64_t> highestArrivedFrameId;
    std::optional<std::int64_t> lastAdvancedArrivalMs;
    std::optional<std::int64_t> noTargetSinceMs;
    std::optional<std::int64_t> awaitingVideoSinceMs;
    VisualState state{VisualState::Stale};
};

struct VisualTargetInput {
    std::int64_t nowMs;
    std::optional<std::uint64_t> arrivedFrameId;
    DetectionDisplayState gate;
    std::optional<TargetState> selected;
};

struct VisualTargetResult {
    VisualTargetMemory next;
    VisualState state{VisualState::Stale};
    bool frameAdvanced{false};
    std::optional<TargetState> usableTarget;
    std::optional<std::int64_t> nextDeadlineMs;
    bool awaitingVideo{false};
};

[[nodiscard]] VisualTargetResult advanceVisualTargetState(
    const VisualTargetMemory &previous, const VisualTargetInput &input,
    const VisualPolicyConfig &config);

} // namespace rb::vision
