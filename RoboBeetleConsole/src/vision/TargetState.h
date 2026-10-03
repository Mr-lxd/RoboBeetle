#pragma once

#include "vision/DetectionMetadata.h"

#include <optional>

namespace rb::vision {

// Original-frame pixels only. No actuator or motion-command dependency.
struct TargetState {
    quint64 frameId{0};
    quint64 captureTimestampNs{0};
    QSize sourceSize;
    DetectionObservation target;
    double ex{0.0};
    double ey{0.0};
};

// Highest confidence among valid observations; first in stream on a tie.
// Freshness and inference readiness remain the caller's existing responsibility.
[[nodiscard]] std::optional<TargetState> selectTargetState(
    const DetectionFrame &frame);

} // namespace rb::vision
