#include "vision/VideoView.h"

#include <QFontMetrics>
#include <QPainter>
#include <QPaintEvent>
#include <QPen>
#include <QRect>

namespace rb::vision {

VideoView::VideoView(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(240, 160);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setAutoFillBackground(false);
}

void VideoView::setFrame(
    const QImage &image,
    quint64 frameId,
    quint64 captureTimestampNs)
{
    if (image.isNull()) {
        return;
    }
    if (paintPending_ && !image_.isNull()) {
        ++replacedPendingFrames_;
    }
    image_ = image;
    frameId_ = frameId;
    captureTimestampNs_ = captureTimestampNs;
    paintPending_ = true;
    update();
}

void VideoView::clearFrame()
{
    image_ = QImage();
    frameId_ = 0U;
    captureTimestampNs_ = 0U;
    detectionOverlay_.reset();
    paintPending_ = false;
    update();
}

void VideoView::setDetectionOverlay(const DetectionFrame &frame)
{
    if (image_.isNull() || frame.sourceSize != image_.size()
        || frame.detections.isEmpty()) {
        clearDetectionOverlay();
        return;
    }
    detectionOverlay_ = frame;
    update();
}

void VideoView::clearDetectionOverlay()
{
    if (!detectionOverlay_.has_value()) {
        return;
    }
    detectionOverlay_.reset();
    update();
}

void VideoView::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.fillRect(rect(), palette().window());

    if (image_.isNull()) {
        return;
    }

    const QSize scaled = image_.size().scaled(size(), Qt::KeepAspectRatio);
    const QRect target(
        (width() - scaled.width()) / 2,
        (height() - scaled.height()) / 2,
        scaled.width(),
        scaled.height());
    painter.drawImage(target, image_);

    if (detectionOverlay_.has_value()
        && detectionOverlay_->sourceSize == image_.size()) {
        painter.save();
        painter.setClipRect(target);
        QFont overlayFont = font();
        overlayFont.setBold(true);
        overlayFont.setPointSizeF(qMax(9.0, overlayFont.pointSizeF()));
        painter.setFont(overlayFont);
        const QFontMetrics metrics(overlayFont);
        const QColor overlayColor(0xFF, 0xB0, 0x00);
        constexpr int markerArmRadiusPx = 6;
        constexpr int markerStrokeWidthPx = 2;
        constexpr int markerDotRadiusPx = 3;
        constexpr int labelOffsetPx = 9;

        for (const DetectionObservation &detection
             : detectionOverlay_->detections) {
            const QString label = QStringLiteral("%1 %2")
                .arg(detection.className)
                .arg(detection.confidence, 0, 'f', 2);

            const double mappedX =
                static_cast<double>(target.left())
                + detection.originalPoint.x()
                    * static_cast<double>(target.width())
                    / static_cast<double>(image_.width());
            const double mappedY =
                static_cast<double>(target.top())
                + detection.originalPoint.y()
                    * static_cast<double>(target.height())
                    / static_cast<double>(image_.height());
            const QPoint marker(qRound(mappedX), qRound(mappedY));

            painter.setPen(QPen(
                overlayColor,
                markerStrokeWidthPx,
                Qt::SolidLine,
                Qt::SquareCap));
            painter.drawLine(
                marker.x() - markerArmRadiusPx,
                marker.y(),
                marker.x() + markerArmRadiusPx,
                marker.y());
            painter.drawLine(
                marker.x(),
                marker.y() - markerArmRadiusPx,
                marker.x(),
                marker.y() + markerArmRadiusPx);
            painter.setPen(Qt::NoPen);
            painter.setBrush(overlayColor);
            painter.drawEllipse(
                marker,
                markerDotRadiusPx,
                markerDotRadiusPx);
            painter.setBrush(Qt::NoBrush);

            const int textWidth = metrics.horizontalAdvance(label);
            const int minimumBaseline = target.top() + metrics.ascent();
            const int maximumBaseline = target.bottom() - metrics.descent();
            const int x = qBound(
                target.left(),
                marker.x() + labelOffsetPx,
                qMax(target.left(), target.right() - textWidth));
            const int baseline = qBound(
                minimumBaseline,
                marker.y() - labelOffsetPx,
                qMax(minimumBaseline, maximumBaseline));

            painter.setPen(QColor(0, 0, 0, 210));
            painter.drawText(x + 1, baseline + 1, label);
            painter.setPen(overlayColor);
            painter.drawText(x, baseline, label);
        }
        painter.restore();
    }

    paintPending_ = false;
    if (!fpsTimer_.isValid()) {
        fpsTimer_.start();
    }
    ++paintWindowFrames_;
    const qint64 elapsed = fpsTimer_.elapsed();
    if (elapsed >= 1000) {
        displayedFps_ =
            static_cast<double>(paintWindowFrames_) * 1000.0 /
            static_cast<double>(elapsed);
        paintWindowFrames_ = 0U;
        fpsTimer_.restart();
        emit displayedFpsChanged(displayedFps_);
    }
}

} // namespace rb::vision
