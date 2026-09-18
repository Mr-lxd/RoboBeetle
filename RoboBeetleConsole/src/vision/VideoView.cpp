#include "vision/VideoView.h"

#include <QPainter>
#include <QPaintEvent>
#include <QRect>

namespace rb::vision {

VideoView::VideoView(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(240, 160);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setAutoFillBackground(false);
}

void VideoView::setFrame(const QImage &image, quint64 frameId)
{
    if (image.isNull()) {
        return;
    }
    if (paintPending_ && !image_.isNull()) {
        ++replacedPendingFrames_;
    }
    image_ = image;
    frameId_ = frameId;
    paintPending_ = true;
    update();
}

void VideoView::clearFrame()
{
    image_ = QImage();
    frameId_ = 0U;
    paintPending_ = false;
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
