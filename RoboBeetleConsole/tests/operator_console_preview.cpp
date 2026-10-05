#include "helpers/OperatorConsoleFixture.h"
#include "helpers/VisualControllerFixture.h"
#include "ui/MainWindow.h"
#include "vision/VisualDispatchSession.h"
#include "vision/VideoView.h"

#include <QApplication>
#include <QCheckBox>
#include <QCommandLineParser>
#include <QDir>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPixmap>
#include <QProgressBar>
#include <QPushButton>
#include <QScreen>
#include <QScrollArea>
#include <QScrollBar>
#include <QSplitter>
#include <QTabWidget>
#include <QTextStream>

#include <cstdlib>

namespace {

// Task 06 green-region geometry. Reported for the PR so the reviewer can see
// the real vertical budget at both required window sizes instead of a claim.
QJsonObject measureGreenRegion(rb::MainWindow &window)
{
    QJsonObject result;
    auto *summary = window.findChild<QWidget *>(QStringLiteral("visionSummaryPanel"));
    if (summary == nullptr) return result;
    result.insert(QStringLiteral("panel_geometry"),
                  QStringLiteral("%1,%2 %3x%4").arg(summary->x()).arg(summary->y())
                      .arg(summary->width()).arg(summary->height()));
    if (auto *session = window.findChild<rb::vision::VisualDispatchSession *>()) {
        const auto ready = session->readiness();
        result.insert(QStringLiteral("readiness"),
                      QJsonObject{{QStringLiteral("link_and_control"), ready.linkAndControl},
                                  {QStringLiteral("servos_ready"), ready.servosReady},
                                  {QStringLiteral("tracking"), ready.tracking},
                                  {QStringLiteral("missing"), ready.missingCount()}});
        result.insert(QStringLiteral("session_armed"), session->armed());
    }
    for (const auto &pair : QList<QPair<QString, QString>>{
             {QStringLiteral("state_pill_text"), QStringLiteral("autoFollowStatePill")},
             {QStringLiteral("checklist_text"), QStringLiteral("autoFollowChecklist")},
             {QStringLiteral("alert_text"), QStringLiteral("autoFollowAlert")},
             {QStringLiteral("arm_button_text"), QStringLiteral("visualArmButton")}}) {
        auto *label = window.findChild<QWidget *>(pair.second);
        if (label == nullptr) continue;
        if (auto *asLabel = qobject_cast<QLabel *>(label)) {
            result.insert(pair.first, asLabel->text());
        } else if (auto *asButton = qobject_cast<QPushButton *>(label)) {
            result.insert(pair.first, asButton->text());
            result.insert(QStringLiteral("arm_button_enabled"), asButton->isEnabled());
        }
        result.insert(pair.first + QStringLiteral("_visible"), label->isVisible());
    }

    // The status text above the green region is everything up to the stretch.
    auto *controlsGroup =
        window.findChild<QWidget *>(QStringLiteral("visionControlsGroup"));
    auto *autoGroup = window.findChild<QWidget *>(QStringLiteral("autoFollowCard"));
    auto *host = window.findChild<QWidget *>(QStringLiteral("greenRegionHost"));
    auto *message = window.findChild<QWidget *>(QStringLiteral("visionControlMessage"));
    if (host == nullptr) return result;
    const int hostTop = host->mapTo(summary, QPoint(0, 0)).y();
    result.insert(QStringLiteral("status_text_height_px"), hostTop);
    result.insert(QStringLiteral("green_region_available_px"), host->height());
    result.insert(QStringLiteral("green_region_free_px"),
                  summary->height() - hostTop - host->height());
    if (message != nullptr) {
        result.insert(QStringLiteral("notice_bottom_px"),
                      message->mapTo(summary, QPoint(0, 0)).y() + message->height());
    }
    if (autoGroup != nullptr) {
        result.insert(QStringLiteral("auto_follow_geometry"),
                      QStringLiteral("%1,%2 %3x%4")
                          .arg(autoGroup->mapTo(summary, QPoint(0, 0)).x())
                          .arg(autoGroup->mapTo(summary, QPoint(0, 0)).y())
                          .arg(autoGroup->width()).arg(autoGroup->height()));
        QJsonObject rows;
        const QStringList names{
            QStringLiteral("autoFollowStatePill"), QStringLiteral("visualDispatchEnabled"),
            QStringLiteral("autoFollowChecklist"), QStringLiteral("visualArmButton"),
            QStringLiteral("autoFollowAlert"), QStringLiteral("autoFollowDetail"),
            QStringLiteral("autoFollowFooter"), QStringLiteral("autoFollowAxisSelector")};
        for (const QString &name : names) {
            auto *widget = window.findChild<QWidget *>(name);
            if (widget == nullptr) continue;
            rows.insert(name, QJsonObject{
                {QStringLiteral("visible"), widget->isVisible()},
                {QStringLiteral("h"), widget->height()},
                {QStringLiteral("fully_contained"),
                 [&] {
                     if (!widget->isVisible()) return true;
                     // Compare against the group's own rectangle: the styled
                     // title margin and padding are part of the clipping region.
                     const QRect mapped(widget->mapTo(autoGroup, QPoint(0, 0)),
                                        widget->size());
                     return autoGroup->rect().contains(mapped);
                 }()}});
        }
        result.insert(QStringLiteral("auto_follow_rows"), rows);
    }
    if (controlsGroup != nullptr) {
        result.insert(QStringLiteral("vision_controls_geometry"),
                      QStringLiteral("%1,%2 %3x%4")
                          .arg(controlsGroup->mapTo(summary, QPoint(0, 0)).x())
                          .arg(controlsGroup->mapTo(summary, QPoint(0, 0)).y())
                          .arg(controlsGroup->width()).arg(controlsGroup->height()));
    }
    if (controlsGroup != nullptr && autoGroup != nullptr) {
        result.insert(QStringLiteral("side_by_side"),
                      controlsGroup->x() < autoGroup->x()
                      || controlsGroup->y() == autoGroup->y());
        result.insert(QStringLiteral("vision_controls_width_px"), controlsGroup->width());
        result.insert(QStringLiteral("auto_follow_width_px"), autoGroup->width());
    }
    // Widest Vision Controls button, for the side-by-side width verdict.
    if (auto *connect = window.findChild<QWidget *>(QStringLiteral("visionConnectButton"))) {
        result.insert(QStringLiteral("vision_button_width_px"), connect->width());
    }
    result.insert(QStringLiteral("variant"), QStringLiteral("V1-stacked"));
    return result;
}

// Vision Status panel crop at 1:1: the review evidence for this task. The
// region is mapped through the pixmap's device pixel ratio so a DPI-scaled run
// crops the real panel rather than a fraction of it.
void savePanelCrop(rb::MainWindow &window, const QString &path)
{
    auto *summary = window.findChild<QWidget *>(QStringLiteral("visionSummaryPanel"));
    if (summary == nullptr) return;
    const QPixmap full = window.grab();
    const qreal dpr = full.devicePixelRatio() > 0.0 ? full.devicePixelRatio() : 1.0;
    const QPoint origin = summary->mapTo(&window, QPoint(0, 0));
    const QRect region(QPoint(qRound(origin.x() * dpr), qRound(origin.y() * dpr)),
                       QSize(qRound(summary->width() * dpr),
                             qRound(summary->height() * dpr)));
    full.copy(region.intersected(full.rect())).save(path, "PNG");
}

struct PreviewCase {
    QString id;
    QSize size;
    int tab{0};
    QString scenario;
    // Task 06 green-region extra: variant selector and scripted Auto Follow state.
    QString variant;   // "V1" for the green-region suite, empty for the generic suite.
    QString state;     // DRY RUN / NOT READY / READY / ARMED / STOPPING / FAULT / PITCH-READY / PITCH-NOT-ZEROED
    bool panelCrop{false};
};

// Task 06: every required status x size combination.
QList<PreviewCase> greenRegionCases()
{
    QList<PreviewCase> cases;
    const QString variant = QStringLiteral("V1");
    // {w, 0}: default size with fully expanded Operator tools; {1100, 720} is
    // clamped up to the window minimum.
    const QList<QSize> sizes{{1420, 0}, {1100, 720}};
    const QList<QString> states{
        QStringLiteral("DRY RUN"), QStringLiteral("NOT READY"),
        QStringLiteral("READY"), QStringLiteral("ARMED"), QStringLiteral("STOPPING"),
        QStringLiteral("FAULT"), QStringLiteral("PITCH-READY"),
        QStringLiteral("PITCH-NOT-ZEROED")};
    const auto slug = [](const QString &state) {
        return QString(state).replace(QStringLiteral(" "), QStringLiteral("-"))
            .replace(QStringLiteral("+"), QStringLiteral("-"));
    };
    for (const QSize &size : sizes) {
        for (const QString &state : states) {
            PreviewCase item;
            item.id = QStringLiteral("T06-%1-%2-%3x%4")
                          .arg(slug(state), variant)
                          .arg(size.width())
                          .arg(size.height() > 0 ? QString::number(size.height()) : QStringLiteral("default"));
            item.size = size;
            item.tab = 0;
            item.scenario = QStringLiteral("green region %1 %2 %3x%4")
                                .arg(state, variant)
                                .arg(size.width())
                                .arg(size.height() > 0 ? QString::number(size.height()) : QStringLiteral("default"));
            item.variant = variant;
            item.state = state;
            item.panelCrop = true;
            cases.append(item);
        }
    }
    return cases;
}

QList<PreviewCase> standardCases()
{
    return {
        {QStringLiteral("S01-remote-idle-1100x720"), {1100, 720}, 0, QStringLiteral("remote fresh disabled idle")},
        {QStringLiteral("S02-remote-idle-1420x880"), {1420, 880}, 0, QStringLiteral("remote fresh disabled idle")},
        {QStringLiteral("S03-remote-running-1600x1000"), {1600, 1000}, 0, QStringLiteral("remote running")},
        {QStringLiteral("S04-running-recording-1420x880"), {1420, 880}, 2, QStringLiteral("running recording")},
        {QStringLiteral("S05-inference-error-1420x880"), {1420, 880}, 2, QStringLiteral("inference failed long error")},
        {QStringLiteral("S06-inference-stopping-1420x880"), {1420, 880}, 5, QStringLiteral("inference stopping cleanup error")},
        {QStringLiteral("S07-inference-retrying-1420x880"), {1420, 880}, 0, QStringLiteral("inference retrying")},
        {QStringLiteral("S08-control-stale-1420x880"), {1420, 880}, 2, QStringLiteral("control status stale")},
        {QStringLiteral("S09-dirty-long-host-1100x720"), {1100, 720}, 0, QStringLiteral("dirty long host")},
        {QStringLiteral("S10-direct-actuators-1100x720"), {1100, 720}, 1, QStringLiteral("direct idle actuator reflow")},
        {QStringLiteral("S11-remote-actuators-1420x880"), {1420, 880}, 1, QStringLiteral("remote actuator authority")},
        {QStringLiteral("S12-protocol-1420x880"), {1420, 880}, 4, QStringLiteral("protocol details")},
        {QStringLiteral("S13-telemetry-1420x880"), {1420, 880}, 3, QStringLiteral("signed telemetry")},
        {QStringLiteral("S14-plots-1420x880"), {1420, 880}, 6, QStringLiteral("plot placeholders")},
        {QStringLiteral("S15-dpi125-running-1420x880"), {1420, 880}, 0, QStringLiteral("DPI 1.25 running")},
        {QStringLiteral("S16-dpi150-idle-1100x720"), {1100, 720}, 0, QStringLiteral("DPI 1.50 idle")},
    };
}

QList<PreviewCase> selectedCases(const QString &caseId, bool greenOnly)
{
    const auto all = greenOnly ? greenRegionCases() : standardCases();
    if (caseId.isEmpty()) {
        return all;
    }
    for (const auto &item : all) {
        if (item.id.startsWith(caseId)) {
            return {item};
        }
    }
    return {};
}

// Drive the Auto Follow block into the requested status using only the public
// controller/session surface. No hardware, no gateway, no real dispatch peers.
// The diagnostic session ages a snapshot out after 500 ms and the application
// keeps ticking, so a scripted TRACKING state must be refreshed immediately
// before the grab rather than once during setup.
void freshenTracking(rb::MainWindow &window, quint64 frameId)
{
    auto *diagnostic = window.findChild<rb::vision::VisualDiagnosticSession *>();
    if (diagnostic == nullptr) return;
    rb::vision::DetectionFrame frame{
        frameId, 1000 + frameId, {640, 480}, {{1, "fish", 0.91, {352.0, 240.0}}}};
    diagnostic->onDetectionArrival(
        frame, {rb::vision::DetectionDisplayState::Target, frame});
    QApplication::processEvents();
}

// Drive the Auto Follow block into the requested status. State is injected
// through the preview fixture controller, so this needs no production hook.
// Arming and the optional placeholder rows are applied by captureCase, after
// the TRACKING snapshot has been refreshed.
void applyGreenState(rb::MainWindow &window,
                     rb::test::VisualControllerFixture &fixture,
                     const PreviewCase &item)
{
    auto *gate = window.findChild<QCheckBox *>(QStringLiteral("visualDispatchEnabled"));
    if (gate == nullptr) return;

    if (item.state == QStringLiteral("DRY RUN")) return;

    if (item.state == QStringLiteral("NOT READY")) {
        // Exactly two independent items missing: link/control and servos/pose.
        gate->setChecked(true);
        fixture.active = false;
        fixture.enabled = 0;
        fixture.known = 0;
        QApplication::processEvents();
        return;
    }

    gate->setChecked(true);
    QApplication::processEvents();

    if (item.state == QStringLiteral("PITCH-READY") || item.state == QStringLiteral("PITCH-NOT-ZEROED")) {
        // Pitch axis with the front axis ready; depth either zeroed (0.12 m, NORMAL) or not.
        if (auto *pitch = window.findChild<QPushButton *>(QStringLiteral("autoFollowAxisPitch"))) pitch->click();
        fixture.enabled = 0x1f;
        fixture.known = 0x1f;
        rb::DepthControlSample sample;
        sample.rawDepthM = 0.12 + 0.115;
        if (item.state == QStringLiteral("PITCH-READY")) sample.calibratedDepthM = 0.12;
        sample.ageMs = 20;
        sample.fresh = true;
        fixture.publishDepth(sample);
        QApplication::processEvents();
        return;
    }

    if (item.state == QStringLiteral("STOPPING")) {
        // Armed, then a STALE safety event: the automatic STOP is sent and its
        // ACK is deliberately withheld, so the session stays in "stop awaiting".
        freshenTracking(window, 1);
        auto *session = window.findChild<rb::vision::VisualDispatchSession *>();
        if (session != nullptr) session->timerTick();
        if (auto *arm = window.findChild<QPushButton *>(QStringLiteral("visualArmButton"))) {
            arm->click();
        }
        if (auto *diagnostic = window.findChild<rb::vision::VisualDiagnosticSession *>()) {
            rb::vision::VisualViewContext stale;
            stale.gate = rb::vision::DetectionDisplayState::Stale;
            diagnostic->refresh(stale);
        }
        QApplication::processEvents();
        return;
    }

    if (item.state == QStringLiteral("FAULT")) {
        freshenTracking(window, 1);
        if (auto *session = window.findChild<rb::vision::VisualDispatchSession *>()) {
            session->timerTick();
        }
        if (auto *arm = window.findChild<QPushButton *>(QStringLiteral("visualArmButton"))) {
            arm->click();
        }
        // A STALE safety event while armed begins one automatic STOP episode.
        // Its ACK is never delivered, so the three retry deadlines latch the
        // STOP-timeout alarm. The link stays up so retries actually run.
        if (auto *diagnostic = window.findChild<rb::vision::VisualDiagnosticSession *>()) {
            rb::vision::VisualViewContext stale;
            stale.gate = rb::vision::DetectionDisplayState::Stale;
            diagnostic->refresh(stale);
        }
        QElapsedTimer wait;
        wait.start();
        auto *session = window.findChild<rb::vision::VisualDispatchSession *>();
        while (session != nullptr && wait.elapsed() < 6000 && !session->stopTimeoutAlert()) {
            QApplication::processEvents(QEventLoop::AllEvents, 50);
            session->timerTick();
        }
        QApplication::processEvents();
    }
}

QByteArray statusBodyFor(const PreviewCase &item)
{
    QByteArray capture =
        "\"capture\":{\"state\":\"idle\",\"recording\":false,"
        "\"session_id\":\"SIM-001\",\"recorded_frames\":0,"
        "\"snapshot_count\":0,\"queue_bytes\":0,\"max_queue_bytes\":1048576}";
    QByteArray inference =
        "\"inference\":{\"configured\":true,\"control_supported\":true,"
        "\"operation\":null,\"state\":\"disabled\","
        "\"artifact_name\":\"fomo-sim.onnx\","
        "\"model_sha256\":\"3dea74511bf2aabbccddeeff00112233445566778899aabbccddeeff00112233\"}";
    if (item.id.contains(QStringLiteral("running"))) {
        inference =
            "\"inference\":{\"configured\":true,\"control_supported\":true,"
            "\"operation\":null,\"state\":\"running\","
            "\"artifact_name\":\"fomo-sim.onnx\","
            "\"model_sha256\":\"3dea74511bf2aabbccddeeff00112233445566778899aabbccddeeff00112233\","
            "\"confidence_threshold\":0.65,\"latest_frame_id\":42,"
            "\"processed_frames\":40,\"skipped_frames\":2,\"inference_fps\":19.5,"
            "\"latency_ms\":12.0,\"detection_count\":0}";
    }
    if (item.id.contains(QStringLiteral("recording"))) {
        capture =
            "\"capture\":{\"state\":\"recording\",\"recording\":true,"
            "\"session_id\":\"SIM-REC\",\"recorded_frames\":120,"
            "\"snapshot_count\":3,\"queue_bytes\":4096,\"max_queue_bytes\":1048576}";
    }
    if (item.id.contains(QStringLiteral("error"))) {
        inference =
            "\"inference\":{\"configured\":true,\"control_supported\":true,"
            "\"operation\":null,\"state\":\"failed\","
            "\"artifact_name\":\"fomo-sim.onnx\","
            "\"model_sha256\":\"3dea74511bf2aabbccddeeff00112233445566778899aabbccddeeff00112233\","
            "\"last_error\":\"SIMULATED inference failure: worker did not produce a frame\"}";
    }
    if (item.id.contains(QStringLiteral("stopping"))) {
        inference =
            "\"inference\":{\"configured\":true,\"control_supported\":true,"
            "\"operation\":\"stopping\",\"state\":\"running\","
            "\"last_error\":\"SIMULATED cleanup error\"}";
    }
    if (item.id.contains(QStringLiteral("retrying"))) {
        inference =
            "\"inference\":{\"configured\":true,\"control_supported\":true,"
            "\"operation\":\"retrying\",\"state\":\"failed\"}";
    }
    return QByteArrayLiteral("{\"ok\":true,\"camera\":{\"running\":true},")
        + capture + QByteArrayLiteral(",") + inference + QByteArrayLiteral("}");
}

QJsonObject captureCase(rb::MainWindow &window,
                        const PreviewCase &item,
                        const QString &directory,
                        const QString &qpa)
{
    // Height 0 means "the default size": fully expanded Operator tools.
    window.resize(item.size.height() > 0 ? item.size
                                         : QSize(item.size.width(), window.fullyExpandedWindowHeight()));
    window.show();
    QApplication::processEvents();
    if (item.size.height() <= 0) {
        window.settleStartupGeometry();
        window.resize(item.size.width(), window.fullyExpandedWindowHeight());
        QApplication::processEvents();
    }
    if (auto *tabs = window.findChild<QTabWidget *>(QStringLiteral("operatorToolsTabs"))) {
        tabs->setCurrentIndex(item.tab);
    }
    QApplication::processEvents();
    // Keep a scripted TRACKING state fresh across the layout settle above.
    const bool wantsTracking = item.state == QStringLiteral("READY")
        || item.state == QStringLiteral("ARMED")
        || item.state == QStringLiteral("PITCH-READY")
        || item.state == QStringLiteral("PITCH-NOT-ZEROED");
    if (wantsTracking) {
        freshenTracking(window, 2);
        if (auto *session = window.findChild<rb::vision::VisualDispatchSession *>()) {
            session->timerTick();
        }
        QApplication::processEvents();
        if (item.state == QStringLiteral("ARMED")) {
            if (auto *arm = window.findChild<QPushButton *>(QStringLiteral("visualArmButton"))) {
                arm->click();
            }
            if (auto *session = window.findChild<rb::vision::VisualDispatchSession *>()) {
                session->timerTick();
            }
        }
        QApplication::processEvents();
    }
    const auto image = window.grab();
    const QString fileName = item.id + QStringLiteral(".png");
    image.save(QDir(directory).filePath(fileName), "PNG");
    if (item.panelCrop) {
        savePanelCrop(window, QDir(directory).filePath(item.id + QStringLiteral("-panel.png")));
        if (auto *depthCard = window.findChild<QWidget *>(QStringLiteral("depthCard"))) {
            depthCard->grab().save(
                QDir(directory).filePath(item.id + QStringLiteral("-depthcard.png")), "PNG");
        }
    }

    QJsonObject result;
    result.insert(QStringLiteral("case_id"), item.id);
    result.insert(QStringLiteral("requested_logical_size"),
                  QStringLiteral("%1x%2").arg(item.size.width()).arg(item.size.height()));
    result.insert(QStringLiteral("actual_window_size"),
                  QStringLiteral("%1x%2").arg(window.size().width()).arg(window.size().height()));
    result.insert(QStringLiteral("actual_frame_size"),
                  QStringLiteral("%1x%2").arg(image.width()).arg(image.height()));
    result.insert(QStringLiteral("qt_version"), QString::fromLatin1(qVersion()));
    result.insert(QStringLiteral("qpa_platform"), qpa);
    result.insert(QStringLiteral("scale_environment"),
                  QString::fromLocal8Bit(qgetenv("QT_SCALE_FACTOR")));
    result.insert(QStringLiteral("device_pixel_ratio"), image.devicePixelRatio());
    result.insert(QStringLiteral("png_size"), QStringLiteral("%1x%2").arg(image.width()).arg(image.height()));
    result.insert(QStringLiteral("selected_tab"), item.tab);
    result.insert(QStringLiteral("scenario"), item.scenario);
    result.insert(QStringLiteral("output"), fileName);
    if (!item.variant.isEmpty()) {
        result.insert(QStringLiteral("variant"), item.variant);
        result.insert(QStringLiteral("auto_follow_requested_state"), item.state);
        result.insert(QStringLiteral("green_region"), measureGreenRegion(window));
    }
    if (auto *screen = window.screen(); screen != nullptr) {
        const QRect available = screen->availableGeometry();
        result.insert(
            QStringLiteral("available_screen_geometry"),
            QStringLiteral("%1,%2 %3x%4")
                .arg(available.x())
                .arg(available.y())
                .arg(available.width())
                .arg(available.height()));
    }
    QJsonObject scrollPositions;
    for (auto *scroll : window.findChildren<QScrollArea *>()) {
        scrollPositions.insert(
            scroll->objectName(),
            QJsonObject{{QStringLiteral("value"), scroll->verticalScrollBar()->value()},
                        {QStringLiteral("maximum"), scroll->verticalScrollBar()->maximum()}});
    }
    result.insert(QStringLiteral("scroll_positions"), scrollPositions);
    if (auto *sidebar = window.findChild<QWidget *>(QStringLiteral("telemetrySidebar"))) {
        result.insert(QStringLiteral("sidebar_geometry"),
                      QStringLiteral("%1,%2 %3x%4").arg(sidebar->x()).arg(sidebar->y()).arg(sidebar->width()).arg(sidebar->height()));
    }
    if (auto *video = window.findChild<rb::vision::VideoView *>(QStringLiteral("videoView"))) {
        result.insert(QStringLiteral("video_geometry"),
                      QStringLiteral("%1,%2 %3x%4").arg(video->x()).arg(video->y()).arg(video->width()).arg(video->height()));
    }
    if (auto *splitter = window.findChild<QSplitter *>(QStringLiteral("workspaceSplitter"))) {
        QJsonArray sizes;
        for (const int size : splitter->sizes()) sizes.append(size);
        result.insert(QStringLiteral("splitter_sizes"), sizes);
    }
    return result;
}

} // namespace

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    QCommandLineParser parser;
    parser.addHelpOption();
    QCommandLineOption outputOption({QStringLiteral("o"), QStringLiteral("output-dir")},
                                    QStringLiteral("Output directory"), QStringLiteral("directory"));
    QCommandLineOption suiteOption(QStringLiteral("suite"), QStringLiteral("Suite name"), QStringLiteral("suite"));
    QCommandLineOption caseOption(QStringLiteral("case"), QStringLiteral("Single case prefix"), QStringLiteral("case"));
    QCommandLineOption greenOption(
        QStringLiteral("green-region"),
        QStringLiteral("Run the Task 06 green-region suite instead of the standard suite"));
    QCommandLineOption scaleLabelOption(
        QStringLiteral("scale-label"),
        QStringLiteral("Suffix added to emitted case ids and file names"), QStringLiteral("label"));
    parser.addOption(outputOption);
    parser.addOption(suiteOption);
    parser.addOption(caseOption);
    parser.addOption(greenOption);
    parser.addOption(scaleLabelOption);
    parser.process(app);
    const QString directory = parser.value(outputOption);
    if (directory.isEmpty()) {
        return 2;
    }
    QDir().mkpath(directory);
    const bool greenSuite = parser.isSet(greenOption);
    const QString scaleLabel = parser.value(scaleLabelOption);
    auto cases = selectedCases(parser.value(caseOption), greenSuite);
    if (cases.isEmpty()) {
        return 3;
    }
    if (!scaleLabel.isEmpty()) {
        // Keep scaled runs distinct in the merged manifest and in file names.
        for (auto &item : cases) {
            item.id += QStringLiteral("-") + scaleLabel;
        }
    }

    const QString qpa = QString::fromLocal8Bit(qgetenv("QT_QPA_PLATFORM"));
    rb::test::LoopbackVisionStatusServer server;
    if (!server.listen()) {
        return 5;
    }
    QJsonArray entries;
    for (const auto &item : cases) {
        rb::test::OperatorConsoleFixture fixture;
        fixture.create(item.id.startsWith(QStringLiteral("S10")), true);
        server.setBody(statusBodyFor(item));
        fixture.visionControlClient->setEndpoint(QStringLiteral("127.0.0.1"), server.port());
        fixture.visionControlClient->refreshStatus();
        for (int tick = 0; tick < 50 && !fixture.visionControlClient->hasFreshStatus(); ++tick) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
        if (greenSuite) {
            // The green-region statuses need a scriptable authority/servo state.
            rb::test::VisualControllerFixture controller;
            rb::MainWindow window(&controller, fixture.visionClient.get(),
                                  fixture.visionControlClient.get());
            if (auto *video = window.findChild<rb::vision::VideoView *>(QStringLiteral("videoView"))) {
                video->setFrame(rb::test::OperatorConsoleFixture::syntheticVideo(), 1);
            }
            applyGreenState(window, controller, item);
            entries.append(captureCase(window, item, directory, qpa));
            window.hide();
            continue;
        }
        rb::MainWindow window(fixture.controller(), fixture.visionClient.get(),
                              fixture.visionControlClient.get());
        if (auto *video = window.findChild<rb::vision::VideoView *>(QStringLiteral("videoView"))) {
            video->setFrame(rb::test::OperatorConsoleFixture::syntheticVideo(), 1);
        }
        entries.append(captureCase(window, item, directory, qpa));
        if (item.id.startsWith(QStringLiteral("S10"))) {
            if (auto *scroll = window.findChild<QScrollArea *>(QStringLiteral("actuatorScrollArea"))) {
                scroll->verticalScrollBar()->setValue(scroll->verticalScrollBar()->maximum());
                QApplication::processEvents();
                const auto lower = window.grab();
                lower.save(QDir(directory).filePath(QStringLiteral("S10-direct-actuators-1100x720-lower.png")), "PNG");
            }
        }
        window.hide();
    }
    QJsonObject manifest;
    manifest.insert(QStringLiteral("base_sha"), QStringLiteral("d84548c46f8edd862595b9edf38ffad1963b318c"));
    manifest.insert(QStringLiteral("dirty"), true);
    manifest.insert(QStringLiteral("suite"), parser.value(suiteOption));
    QFile file(QDir(directory).filePath(QStringLiteral("manifest.json")));
    QJsonArray mergedEntries;
    if (file.exists() && file.open(QIODevice::ReadOnly)) {
        const auto existing = QJsonDocument::fromJson(file.readAll()).object().value(QStringLiteral("entries")).toArray();
        for (const auto &entry : existing) {
            mergedEntries.append(entry);
        }
        file.close();
    }
    for (const auto &entry : entries) {
        const QString id = entry.toObject().value(QStringLiteral("case_id")).toString();
        for (int index = mergedEntries.size() - 1; index >= 0; --index) {
            if (mergedEntries.at(index).toObject().value(QStringLiteral("case_id")).toString() == id) {
                mergedEntries.removeAt(index);
            }
        }
        mergedEntries.append(entry);
    }
    manifest.insert(QStringLiteral("entries"), mergedEntries);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return 4;
    }
    file.write(QJsonDocument(manifest).toJson(QJsonDocument::Indented));
    file.close();
    return 0;
}
