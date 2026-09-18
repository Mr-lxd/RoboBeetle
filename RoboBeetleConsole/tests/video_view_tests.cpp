#include "vision/VideoView.h"

#include <QApplication>
#include <QImage>

#include <cstdio>
#include <cstdlib>

namespace {

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

    view.setFrame(image(1), 1U);
    view.setFrame(image(2), 2U);
    view.setFrame(image(7), 7U);

    expect(view.hasFrame(), "VideoView keeps one current image");
    expect(view.currentFrameId() == 7U,
           "VideoView latest pending frame wins before paint");
    expect(view.replacedPendingFrames() == 2U,
           "VideoView counts two overwritten pending frames");
}

void paintingClearsPendingReplacementWindow()
{
    VideoView view;
    view.resize(320, 240);
    view.show();

    view.setFrame(image(3), 3U);
    for (int i = 0; i < 10; ++i) {
        QApplication::processEvents();
    }
    const quint64 before = view.replacedPendingFrames();

    view.setFrame(image(4), 4U);
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

} // namespace

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    latestPendingFrameReplacesOlderPendingFrame();
    paintingClearsPendingReplacementWindow();
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
