#include "vision/DetectionMetadata.h"

namespace rb::vision {

DetectionDisplayState detectionOverlayState(
    const DetectionFrame &frame,
    const QSize &currentVideoSize,
    quint64 currentVideoCaptureTimestampNs,
    bool inferenceRunning,
    bool statusFresh) noexcept
{
    if (!statusFresh) {
        return DetectionDisplayState::Stale;
    }
    if (!inferenceRunning) {
        return DetectionDisplayState::InferenceOff;
    }
    if (!frame.sourceSize.isValid() || frame.sourceSize != currentVideoSize) {
        return DetectionDisplayState::Stale;
    }
    if (frame.captureTimestampNs == 0U || currentVideoCaptureTimestampNs == 0U
        || frame.captureTimestampNs > currentVideoCaptureTimestampNs) {
        return DetectionDisplayState::Stale;
    }
    if (currentVideoCaptureTimestampNs - frame.captureTimestampNs
        > kDetectionOverlayFreshnessNs) {
        return DetectionDisplayState::Stale;
    }
    return frame.detections.isEmpty()
        ? DetectionDisplayState::NoTarget : DetectionDisplayState::Target;
}

bool detectionOverlayRenderable(
    const DetectionFrame &frame,
    const QSize &currentVideoSize,
    quint64 currentVideoCaptureTimestampNs,
    bool inferenceRunning,
    bool statusFresh) noexcept
{
    return detectionOverlayState(frame, currentVideoSize,
                                 currentVideoCaptureTimestampNs,
                                 inferenceRunning, statusFresh)
        == DetectionDisplayState::Target;
}

} // namespace rb::vision
