#include "vision/DetectionMetadata.h"

namespace rb::vision {

bool detectionOverlayRenderable(
    const DetectionFrame &frame,
    const QSize &currentVideoSize,
    quint64 currentVideoCaptureTimestampNs,
    bool inferenceRunning,
    bool statusFresh) noexcept
{
    if (!inferenceRunning || !statusFresh || frame.detections.isEmpty()) {
        return false;
    }
    if (!frame.sourceSize.isValid() || frame.sourceSize != currentVideoSize) {
        return false;
    }
    if (frame.captureTimestampNs == 0U || currentVideoCaptureTimestampNs == 0U
        || frame.captureTimestampNs > currentVideoCaptureTimestampNs) {
        return false;
    }
    return currentVideoCaptureTimestampNs - frame.captureTimestampNs
        <= kDetectionOverlayFreshnessNs;
}

} // namespace rb::vision
