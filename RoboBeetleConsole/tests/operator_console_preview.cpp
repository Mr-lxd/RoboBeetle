#include "helpers/OperatorConsoleFixture.h"
#include "ui/MainWindow.h"
#include "vision/VideoView.h"

#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QEventLoop>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QScreen>
#include <QScrollArea>
#include <QScrollBar>
#include <QSplitter>
#include <QTabWidget>

#include <cstdlib>

namespace {

struct PreviewCase {
    QString id;
    QSize size;
    int tab{0};
    QString scenario;
};

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

QList<PreviewCase> selectedCases(const QString &caseId)
{
    const auto all = standardCases();
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
    window.resize(item.size);
    window.show();
    QApplication::processEvents();
    if (auto *tabs = window.findChild<QTabWidget *>(QStringLiteral("operatorToolsTabs"))) {
        tabs->setCurrentIndex(item.tab);
    }
    QApplication::processEvents();
    const auto image = window.grab();
    const QString fileName = item.id + QStringLiteral(".png");
    image.save(QDir(directory).filePath(fileName), "PNG");

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
    parser.addOption(outputOption);
    parser.addOption(suiteOption);
    parser.addOption(caseOption);
    parser.process(app);
    const QString directory = parser.value(outputOption);
    if (directory.isEmpty()) {
        return 2;
    }
    QDir().mkpath(directory);
    const auto cases = selectedCases(parser.value(caseOption));
    if (cases.isEmpty()) {
        return 3;
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
