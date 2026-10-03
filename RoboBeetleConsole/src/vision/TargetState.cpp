#include "vision/TargetState.h"
#include <cmath>

namespace rb::vision {
std::optional<TargetState> targetStateAt(const DetectionFrame &frame, qsizetype index)
{
    const int width = frame.sourceSize.width();
    const int height = frame.sourceSize.height();
    if (width <= 0 || height <= 0 || index < 0 || index >= frame.detections.size()) return std::nullopt;
    const auto &candidate = frame.detections[index];
    const double u = candidate.originalPoint.x(), v = candidate.originalPoint.y();
    if (!std::isfinite(candidate.confidence) || candidate.confidence < 0 || candidate.confidence > 1
        || !std::isfinite(u) || !std::isfinite(v)
        || u < 0 || u > width - 1 || v < 0 || v > height - 1) return std::nullopt;
    const double uc = width / 2.0, vc = height / 2.0;
    return TargetState{frame.frameId, frame.captureTimestampNs, frame.sourceSize,
                       candidate, (u - uc) / uc, (v - vc) / vc};
}
std::optional<TargetState> selectTargetState(const DetectionFrame &frame)
{
    std::optional<TargetState> selected;
    for (qsizetype i = 0; i < frame.detections.size(); ++i) {
        auto candidate = targetStateAt(frame, i);
        if (candidate && (!selected || candidate->target.confidence > selected->target.confidence)) {
            selected = std::move(candidate);
        }
    }
    return selected;
}
} // namespace rb::vision
