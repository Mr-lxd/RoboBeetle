#pragma once

#include "vision/DetectionMetadata.h"

#include <QElapsedTimer>
#include <QImage>
#include <QWidget>

#include <cstdint>
#include <optional>

namespace rb::vision {

class VideoView final : public QWidget {
    Q_OBJECT

public:
    explicit VideoView(QWidget *parent = nullptr);

    void setFrame(
        const QImage &image,
        quint64 frameId,
        quint64 captureTimestampNs = 0U);
    void clearFrame();
    void setDetectionOverlay(const DetectionFrame &frame);
    void clearDetectionOverlay();

    [[nodiscard]] bool hasFrame() const noexcept { return !image_.isNull(); }
    [[nodiscard]] quint64 currentFrameId() const noexcept { return frameId_; }
    [[nodiscard]] quint64 currentCaptureTimestampNs() const noexcept
    {
        return captureTimestampNs_;
    }
    [[nodiscard]] QSize currentFrameSize() const noexcept
    {
        return image_.size();
    }
    [[nodiscard]] bool hasDetectionOverlay() const noexcept
    {
        return detectionOverlay_.has_value();
    }
    [[nodiscard]] qsizetype detectionOverlayCount() const noexcept
    {
        return detectionOverlay_.has_value()
            ? detectionOverlay_->detections.size()
            : 0;
    }
    [[nodiscard]] quint64 replacedPendingFrames() const noexcept
    {
        return replacedPendingFrames_;
    }
    [[nodiscard]] double displayedFps() const noexcept { return displayedFps_; }

signals:
    void displayedFpsChanged(double fps);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    QImage image_;
    quint64 frameId_{0};
    quint64 captureTimestampNs_{0};
    std::optional<DetectionFrame> detectionOverlay_;
    quint64 replacedPendingFrames_{0};
    bool paintPending_{false};
    QElapsedTimer fpsTimer_;
    quint64 paintWindowFrames_{0};
    double displayedFps_{0.0};
};

} // namespace rb::vision
