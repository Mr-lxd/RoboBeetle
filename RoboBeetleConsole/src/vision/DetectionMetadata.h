#pragma once

#include <QMetaType>
#include <QPointF>
#include <QSize>
#include <QString>
#include <QVector>

namespace rb::vision {

inline constexpr quint16 kDetectionStreamDefaultPort = 47012;
inline constexpr int kDetectionStreamVersion = 1;
inline constexpr qsizetype kDetectionStreamMaxLineBytes = 64 * 1024;
inline constexpr qsizetype kDetectionStreamMaxClassNameBytes = 128;
inline constexpr qsizetype kDetectionStreamMaxDetections = 256;
inline constexpr quint64 kDetectionOverlayFreshnessNs = 1'500'000'000ULL;

struct DetectionObservation {
    int classId{0};
    QString className;
    double confidence{0.0};
    QPointF originalPoint;
};

struct DetectionFrame {
    quint64 frameId{0};
    quint64 captureTimestampNs{0};
    QSize sourceSize;
    QVector<DetectionObservation> detections;
};

[[nodiscard]] bool detectionOverlayRenderable(
    const DetectionFrame &frame,
    const QSize &currentVideoSize,
    quint64 currentVideoCaptureTimestampNs,
    bool inferenceRunning,
    bool statusFresh) noexcept;

} // namespace rb::vision

Q_DECLARE_METATYPE(rb::vision::DetectionObservation)
Q_DECLARE_METATYPE(rb::vision::DetectionFrame)
