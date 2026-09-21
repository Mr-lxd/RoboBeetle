#include "vision/VideoView.h"

#include <QApplication>
#include <QImage>

#include <cstdio>
#include <cstdlib>

namespace {

using rb::vision::DetectionFrame;
using rb::vision::DetectionObservation;
using rb::vision::VideoView;

int failures = 0;

void expect(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

QImage image(int marker)
{
    QImage value(4, 3, QImage::Format_RGB32);
    value.fill(qRgb(marker, marker, marker));
    return value;
}

void latestPendingFrameReplacesOlderPendingFrame()
{
    VideoView view;
    view.resize(320, 240);

    view.setFrame(image(1), 1U, 1'000U);
    view.setFrame(image(2), 2U, 2'000U);
    view.setFrame(image(7), 7U, 7'000U);

    expect(view.hasFrame(), "VideoView keeps one current image");
    expect(view.currentFrameId() == 7U,
           "VideoView latest pending frame wins before paint");
    expect(view.currentCaptureTimestampNs() == 7'000U,
           "VideoView retains the current Pi capture timestamp");
    expect(view.replacedPendingFrames() == 2U,
           "VideoView counts two overwritten pending frames");
}

void paintingClearsPendingReplacementWindow()
{
    VideoView view;
    view.resize(320, 240);
    view.show();

    view.setFrame(image(3), 3U, 3'000U);
    for (int i = 0; i < 10; ++i) {
        QApplication::processEvents();
    }
    const quint64 before = view.replacedPendingFrames();

    view.setFrame(image(4), 4U, 4'000U);
    for (int i = 0; i < 10; ++i) {
        QApplication::processEvents();
    }
    expect(view.currentFrameId() == 4U,
           "VideoView paints the latest submitted image");
    expect(view.replacedPendingFrames() == before,
           "a frame submitted after paint is not counted as pending replacement");

    view.clearFrame();
    expect(!view.hasFrame(), "VideoView clear removes stale image");
    view.hide();
}


void textOverlayLifecycleIsIndependentFromVideoFrame()
{
    VideoView view;
    view.resize(320, 240);
    view.show();
    view.setFrame(image(20), 20U, 20'000U);

    DetectionFrame overlay;
    overlay.frameId = 19U;
    overlay.captureTimestampNs = 19'000U;
    overlay.sourceSize = QSize(4, 3);
    overlay.detections.push_back(
        DetectionObservation{
            0,
            QStringLiteral("fish"),
            0.88,
            QPointF(1.0, 1.0)});

    view.setDetectionOverlay(overlay);
    expect(view.hasDetectionOverlay()
               && view.detectionOverlayCount() == 1,
           "VideoView accepts text-overlay metadata for matching source size");

    QImage withOverlay(view.size(), QImage::Format_ARGB32);
    withOverlay.fill(Qt::transparent);
    view.render(&withOverlay);

    view.clearDetectionOverlay();
    expect(!view.hasDetectionOverlay(),
           "overlay can clear without clearing live video");
    expect(view.hasFrame(),
           "clearing overlay does not clear live video");

    QImage withoutOverlay(view.size(), QImage::Format_ARGB32);
    withoutOverlay.fill(Qt::transparent);
    view.render(&withoutOverlay);
    expect(withOverlay != withoutOverlay,
           "text overlay changes rendered pixels");

    DetectionFrame wrongSize = overlay;
    wrongSize.sourceSize = QSize(640, 480);
    view.setDetectionOverlay(wrongSize);
    expect(!view.hasDetectionOverlay(),
           "mismatched source dimensions fail closed in VideoView");

    view.setDetectionOverlay(overlay);
    view.clearFrame();
    expect(!view.hasFrame()
               && !view.hasDetectionOverlay()
               && view.currentCaptureTimestampNs() == 0U,
           "clearing video also clears stale overlay and timestamp");
    view.hide();
}

} // namespace

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    latestPendingFrameReplacesOlderPendingFrame();
    paintingClearsPendingReplacementWindow();
    textOverlayLifecycleIsIndependentFromVideoFrame();
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
