#pragma once

#include <QElapsedTimer>
#include <QImage>
#include <QWidget>

#include <cstdint>

namespace rb::vision {

class VideoView final : public QWidget {
    Q_OBJECT

public:
    explicit VideoView(QWidget *parent = nullptr);

    void setFrame(const QImage &image, quint64 frameId);
    void clearFrame();

    [[nodiscard]] bool hasFrame() const noexcept { return !image_.isNull(); }
    [[nodiscard]] quint64 currentFrameId() const noexcept { return frameId_; }
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
    quint64 replacedPendingFrames_{0};
    bool paintPending_{false};
    QElapsedTimer fpsTimer_;
    quint64 paintWindowFrames_{0};
    double displayedFps_{0.0};
};

} // namespace rb::vision
