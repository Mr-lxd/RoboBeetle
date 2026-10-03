#include "vision/TargetState.h"

#include <cmath>

namespace rb::vision {

std::optional<TargetState> selectTargetState(const DetectionFrame &frame)
{
    const int width = frame.sourceSize.width();
    const int height = frame.sourceSize.height();
    if (width <= 0 || height <= 0) {
        return std::nullopt;
    }

    const DetectionObservation *selected = nullptr;
    for (const DetectionObservation &candidate : frame.detections) {
        const double u = candidate.originalPoint.x();
        const double v = candidate.originalPoint.y();
        if (!std::isfinite(candidate.confidence)
            || candidate.confidence < 0.0 || candidate.confidence > 1.0
            || !std::isfinite(u) || !std::isfinite(v)
            || u < 0.0 || u > width - 1 || v < 0.0 || v > height - 1) {
            continue;
        }
        if (selected == nullptr || candidate.confidence > selected->confidence) {
            selected = &candidate;
        }
    }
    if (selected == nullptr) {
        return std::nullopt;
    }

    const double uc = static_cast<double>(width) / 2.0;
    const double vc = static_cast<double>(height) / 2.0;
    return TargetState{
        frame.frameId,
        frame.captureTimestampNs,
        frame.sourceSize,
        *selected,
        (selected->originalPoint.x() - uc) / uc,
        (selected->originalPoint.y() - vc) / vc};
}

} // namespace rb::vision
