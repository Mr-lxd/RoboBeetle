#include "vision/VideoView.h"

#include <QApplication>
#include <QColor>
#include <QDir>
#include <QImage>
#include <QRect>

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

bool containsColor(
    const QImage &rendered,
    const QRect &region,
    const QColor &color)
{
    const QRect bounded = region.intersected(rendered.rect());
    for (int y = bounded.top(); y <= bounded.bottom(); ++y) {
        for (int x = bounded.left(); x <= bounded.right(); ++x) {
            if (rendered.pixelColor(x, y) == color) {
                return true;
            }
        }
    }
    return false;
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

    view.setDetectionOverlay(overlay, rb::vision::selectTargetState(overlay));
    expect(view.hasDetectionOverlay()
               && view.detectionOverlayCount() == 1,
           "VideoView accepts text-overlay metadata for matching source size");
    expect(view.currentTargetState()
               && view.currentTargetState()->ex == -0.5
               && view.visualDiagnosticText().contains(QStringLiteral("VISION DRY_RUN"))
               && view.visualDiagnosticText().contains(QStringLiteral("ex=-0.500")),
           "accepted metadata exposes normalized errors and DRY_RUN diagnostics");

    QImage withOverlay(view.size(), QImage::Format_ARGB32);
    withOverlay.fill(Qt::transparent);
    view.render(&withOverlay);

    view.clearDetectionOverlay();
    expect(!view.hasDetectionOverlay(),
           "overlay can clear without clearing live video");
    expect(view.hasFrame(),
           "clearing overlay does not clear live video");
    expect(!view.currentTargetState()
               && view.visualDiagnosticText().contains(QStringLiteral("STALE"))
               && view.visualDiagnosticText().contains(QStringLiteral("ex=-- ey=--")),
           "clearing detections removes errors instead of keeping stale numeric values");

    QImage withoutOverlay(view.size(), QImage::Format_ARGB32);
    withoutOverlay.fill(Qt::transparent);
    view.render(&withoutOverlay);
    expect(withOverlay != withoutOverlay,
           "text overlay changes rendered pixels");

    DetectionFrame wrongSize = overlay;
    wrongSize.sourceSize = QSize(640, 480);
    view.setDetectionOverlay(wrongSize, rb::vision::selectTargetState(wrongSize));
    expect(!view.hasDetectionOverlay(),
           "mismatched source dimensions fail closed in VideoView");
    expect(!view.currentTargetState(), "mismatched dimensions cannot produce target errors");

    view.setDetectionOverlay(overlay, rb::vision::selectTargetState(overlay));
    view.clearFrame();
    expect(!view.hasFrame()
               && !view.hasDetectionOverlay()
               && !view.currentTargetState()
               && view.currentCaptureTimestampNs() == 0U,
           "clearing video also clears stale overlay and timestamp");
    view.hide();
}

void amberCentroidMarkersAndLabelsRenderForEveryDetection()
{
    VideoView view;
    view.resize(320, 240);
    view.show();
    view.setFrame(image(20), 20U, 20'000U);

    DetectionFrame overlay;
    overlay.frameId = 20U;
    overlay.captureTimestampNs = 20'000U;
    overlay.sourceSize = QSize(4, 3);
    overlay.detections.push_back(
        DetectionObservation{
            0,
            QStringLiteral("fish"),
            0.88,
            QPointF(1.0, 1.0)});
    overlay.detections.push_back(
        DetectionObservation{
            1,
            QStringLiteral("stingray"),
            0.82,
            QPointF(3.0, 2.0)});
    view.setDetectionOverlay(overlay, rb::vision::selectTargetState(overlay));

    QImage rendered(view.size(), QImage::Format_ARGB32);
    rendered.fill(Qt::transparent);
    view.render(&rendered);

    const QColor amber(0xFF, 0xB0, 0x00);
    expect(rendered.pixelColor(80, 80) == amber,
           "first detection renders an amber centroid marker");
    expect(rendered.pixelColor(240, 160) == amber,
           "every detection renders the same amber centroid marker");
    expect(containsColor(rendered, QRect(88, 45, 120, 30), amber),
           "detection label foreground renders in bright amber");
    expect(containsColor(rendered, QRect(210, 125, 110, 28), amber),
           "every detection renders the same amber label foreground");
    view.hide();
}

void visualErrorGeometryUsesImageRectangle()
{
    VideoView view;
    view.resize(400, 400);
    QImage source(640, 480, QImage::Format_RGB32);
    source.fill(qRgb(20, 20, 20));
    view.setFrame(source, 20U, 20'000U);

    QImage centered(view.size(), QImage::Format_ARGB32);
    view.render(&centered);
    const QColor cyan(0x00, 0xE5, 0xFF);
    expect(containsColor(centered, QRect(197, 197, 7, 7), cyan),
           "image-center cross renders before a target is detected");

    DetectionFrame overlay;
    overlay.frameId = 20U;
    overlay.captureTimestampNs = 20'000U;
    overlay.sourceSize = source.size();
    overlay.detections.push_back(
        DetectionObservation{0, QStringLiteral("fish"), 0.88,
                             QPointF(160.0, 120.0)});
    view.setDetectionOverlay(overlay, rb::vision::selectTargetState(overlay));
    QImage withTarget(view.size(), QImage::Format_ARGB32);
    view.render(&withTarget);
    expect(containsColor(withTarget, QRect(147, 159, 7, 7), cyan),
           "error line maps original pixels through letterboxed video geometry");
    expect(withTarget.pixelColor(100, 125) == QColor(0xFF, 0xB0, 0x00),
           "error line retains the existing amber target marker");
    expect(!containsColor(withTarget, QRect(0, 0, 400, 45), cyan),
           "visual diagnostics do not draw into letterbox margins");

    view.clearDetectionOverlay();
    QImage cleared(view.size(), QImage::Format_ARGB32);
    view.render(&cleared);
    expect(!containsColor(cleared, QRect(147, 159, 7, 7), cyan),
           "clearing detections also removes the target error line");

    view.setDetectionOverlay(overlay, rb::vision::selectTargetState(overlay));
    QImage differentSize(1280, 720, QImage::Format_RGB32);
    differentSize.fill(Qt::black);
    view.setFrame(differentSize, 21U, 21'000U);
    expect(!view.currentTargetState() && !view.hasDetectionOverlay(),
           "a source-size change clears the previous coordinate interpretation");
}

void commandProposalDisplay()
{
    VideoView view;
    rb::vision::VisualDiagnosticSnapshot s;
    s.state = rb::vision::VisualState::Tracking;
    s.command.ex_f = 0.4;
    s.command.yaw_cmd = 0.4;
    s.command.proposed = rb::vision::ProposedCommand::TurnRight;
    s.command.effective = rb::vision::ProposedCommand::TurnRight;
    view.setVisualDiagnostic(s);
    expect(view.visualDiagnosticText().contains(QStringLiteral("u=-- v=--")),
           "no target leaves the area row out");
    s.target = rb::vision::TargetState{1, 1, {640, 480}, {0, QStringLiteral("fish"), 0.9, {10, 20}, 6}, 0.0, 0.0};
    view.setVisualDiagnostic(s);
    expect(view.visualDiagnosticText().contains(QStringLiteral("area=6 cells")), "target area is shown");
    s.target->target.areaCells.reset();
    view.setVisualDiagnostic(s);
    expect(view.visualDiagnosticText().contains(QStringLiteral("area=--")), "missing area is shown as --");
    s.target.reset();
    view.setVisualDiagnostic(s);
    expect(view.visualDiagnosticText().contains(QStringLiteral("manual controls live"))
               && view.visualDiagnosticText().contains(QStringLiteral("no motion output"))
               && view.visualDiagnosticText().contains(QStringLiteral("PROPOSED (not sent): TURN_RIGHT"))
               && view.visualDiagnosticText().contains(QStringLiteral("ex_f=0.400 yaw=0.400")),
           "proposal panel labels dry run and dimensionless suggestions");
    s.state = rb::vision::VisualState::Stale;
    s.command = {};
    view.setVisualDiagnostic(s);
    expect(view.visualDiagnosticText().contains(QStringLiteral("PROPOSED (not sent): STOP"))
               && view.visualDiagnosticText().contains(QStringLiteral("ex_f=-- yaw=0.000")),
           "invalid target does not masquerade as zero filtered error");
}

void associatedSelectionDisplay()
{
    qint64 now=0;
    rb::vision::VisualDiagnosticSession session({},[&]{return now;});
    DetectionFrame f{1,1000,{640,480},{{0,"fish",.89,{450,300}},{1,"other",.85,{547,300}}}};
    session.onDetectionArrival(f,{rb::vision::DetectionDisplayState::Target,f});
    now=40; f.frameId=2; f.detections[0].confidence=.85; f.detections[1].confidence=.89;
    session.onDetectionArrival(f,{rb::vision::DetectionDisplayState::Target,f});
    VideoView view; view.resize(640,480);
    QImage source(640,480,QImage::Format_RGB32); source.fill(Qt::black);
    view.setFrame(source,2,1000);
    view.setDetectionOverlay(f,session.snapshot().target); view.setVisualDiagnostic(session.snapshot());
    expect(view.currentTargetState() && view.currentTargetState()->target.originalPoint.x()==450,
           "view must use associated target rather than higher-confidence target");
    expect(view.visualDiagnosticText().contains("lock: ASSOCIATED d=0.0px")
           && !view.visualDiagnosticText().contains("(highest)"),"panel identifies associated lock and distance");
    QImage rendered(view.size(),QImage::Format_ARGB32); view.render(&rendered);
    expect(rendered.pixelColor(461,300)==QColor(0,0xE5,0xFF),"associated point has distinct cyan ring");
    expect(rendered.pixelColor(547,300)==QColor(0xFF,0xB0,0),"unselected detection remains amber");
    now=80; f.frameId=3; f.detections.removeFirst(); session.onDetectionArrival(f,{rb::vision::DetectionDisplayState::Target,f});
    view.setDetectionOverlay(f,session.snapshot().target); view.setVisualDiagnostic(session.snapshot());
    expect(view.hasDetectionOverlay() && view.detectionOverlayCount()==1 && !view.currentTargetState(),
           "MISS retains other detections without a target error line");
    expect(view.visualDiagnosticText().contains("lock: MISS 0ms"),"MISS elapsed time is shown");
}

void saveDiagnosticPreviews(const QString &directory)
{
    expect(QDir().mkpath(directory), "preview output directory can be created");
    struct Case { const char *name; double u; bool detected; };
    const Case cases[] = {
        {"left", 160.0, true}, {"center", 320.0, true},
        {"right", 480.0, true}, {"no-target", 320.0, false}};
    for (const QSize displaySize : {QSize(640, 480), QSize(240, 180)}) {
        VideoView view;
        view.resize(displaySize);
        QImage source(640, 480, QImage::Format_RGB32);
        source.fill(qRgb(16, 24, 32));
        view.setFrame(source, 42U, 1'000'000U);
        for (const Case &item : cases) {
            DetectionFrame overlay{
                42U, 1'000'000U, source.size(),
                {DetectionObservation{2, QStringLiteral("fish"), 0.88,
                                      QPointF(item.u, 240.0)}}};
            if (!item.detected) {
                overlay.detections.clear();
            }
            view.setDetectionOverlay(overlay, rb::vision::selectTargetState(overlay));
            rb::vision::VisualDiagnosticSession diagnostic({}, [] { return 0; });
            diagnostic.onDetectionArrival(overlay, {
                item.detected ? rb::vision::DetectionDisplayState::Target
                              : rb::vision::DetectionDisplayState::NoTarget,
                overlay});
            view.setVisualDiagnostic(diagnostic.snapshot());
            QImage rendered(view.size(), QImage::Format_ARGB32);
            view.render(&rendered);
            const QString name = QStringLiteral("%1-%2x%3.png")
                .arg(QString::fromLatin1(item.name))
                .arg(displaySize.width()).arg(displaySize.height());
            expect(rendered.save(QDir(directory).filePath(name)),
                   "simulated diagnostic preview can be saved");
        }
    }
}

} // namespace

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    latestPendingFrameReplacesOlderPendingFrame();
    paintingClearsPendingReplacementWindow();
    textOverlayLifecycleIsIndependentFromVideoFrame();
    amberCentroidMarkersAndLabelsRenderForEveryDetection();
    visualErrorGeometryUsesImageRectangle();
    commandProposalDisplay();
    associatedSelectionDisplay();
    const QStringList arguments = app.arguments();
    const int previewOption = arguments.indexOf(QStringLiteral("--preview-dir"));
    if (previewOption >= 0 && previewOption + 1 < arguments.size()) {
        saveDiagnosticPreviews(arguments[previewOption + 1]);
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
