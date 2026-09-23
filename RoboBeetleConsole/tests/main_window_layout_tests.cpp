#include "helpers/OperatorConsoleFixture.h"
#include "ui/MainWindow.h"
#include "ui/ElidedLabel.h"
#include "vision/VideoView.h"

#include <QApplication>
#include <QAbstractSpinBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGroupBox>
#include <QGridLayout>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QScreen>
#include <QScrollArea>
#include <QScrollBar>
#include <QSplitter>
#include <QSpinBox>
#include <QTabWidget>

#include <algorithm>
#include <cstdio>

namespace {
int failures = 0;
void expect(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

void testScreenAwareStartupGeometry()
{
    rb::test::OperatorConsoleFixture fixture;
    fixture.create(false, true);
    rb::MainWindow window(fixture.controller(), fixture.visionClient.get(),
                          fixture.visionControlClient.get());

    QScreen *screen = window.screen();
    if (screen == nullptr) {
        screen = QGuiApplication::primaryScreen();
    }
    expect(screen != nullptr, "startup geometry has an available Qt screen");
    if (screen == nullptr) {
        return;
    }
    const QRect available = screen->availableGeometry();
    const int expectedWidth = qMax(1100, qMin(1420, available.width() - 32));
    const int expectedHeight = qMax(720, qMin(1000, available.height() - 80));
    std::fprintf(stdout,
                 "startup_available=%dx%d expected=%dx%d actual=%dx%d\n",
                 available.width(), available.height(), expectedWidth, expectedHeight,
                 window.size().width(), window.size().height());
    expect(window.minimumSize() == QSize(1100, 720),
           "startup geometry preserves the exact 1100x720 minimum");
    expect(window.size() == QSize(expectedWidth, expectedHeight),
           "startup geometry clamps the comfortable target to available screen space");

    window.show();
    QApplication::processEvents();
    const QPoint availableCenter = available.center();
    const QPoint windowCenter = window.frameGeometry().center();
    std::fprintf(stdout,
                 "startup_center=available(%d,%d) actual(%d,%d) delta=%d\n",
                 availableCenter.x(), availableCenter.y(), windowCenter.x(),
                 windowCenter.y(), (windowCenter - availableCenter).manhattanLength());
    const bool centerCheckReliable = QGuiApplication::platformName() != QStringLiteral("windows");
    expect(!centerCheckReliable || (windowCenter - availableCenter).manhattanLength() <= 4,
           centerCheckReliable
               ? "startup geometry centers the window on the available screen"
               : "native Windows QPA reports the OS-adjusted frame center separately");
    window.close();
}

QGroupBox *group(rb::MainWindow &window, const QString &title)
{
    for (auto *box : window.findChildren<QGroupBox *>()) {
        if (box->title() == title) {
            return box;
        }
    }
    return nullptr;
}

void testTreeAndSizing()
{
    rb::test::OperatorConsoleFixture fixture;
    fixture.create(false, true);
    rb::MainWindow window(fixture.controller(), fixture.visionClient.get(),
                          fixture.visionControlClient.get());
    window.resize(1100, 720);
    window.show();
    QApplication::processEvents();

    auto *splitter = window.findChild<QSplitter *>(QStringLiteral("workspaceSplitter"));
    expect(splitter != nullptr, "U01: workspace has a named splitter");
    expect(splitter != nullptr && splitter->orientation() == Qt::Vertical,
           "U01: workspace splitter is vertical");
    expect(splitter != nullptr && splitter->count() == 2,
           "U01: workspace splitter has exactly two panes");
    expect(window.findChildren<rb::vision::VideoView *>().size() == 1,
           "U02: exactly one VideoView");
    expect(group(window, QStringLiteral("Realtime Video")) != nullptr,
           "U02: one video card exists");
    expect(window.findChildren<QLineEdit *>(QStringLiteral("piHost")).size() == 1,
           "U02: one editable Pi Host exists");
    expect(group(window, QStringLiteral("Depth Sensor")) != nullptr,
           "U03: frozen Depth Sensor title");
    expect(group(window, QStringLiteral("Depth Sensor — ROVMAKER")) == nullptr,
           "U03: old Depth Sensor title is absent");

    auto *tabs = window.findChild<QTabWidget *>(QStringLiteral("operatorToolsTabs"));
    expect(tabs != nullptr, "U15: operator tools tabs exist");
    const QStringList expected = {QStringLiteral("Motion / Gait"),
                                  QStringLiteral("Servo Fine Control"),
                                  QStringLiteral("Vision Details"),
                                  QStringLiteral("Telemetry Details"),
                                  QStringLiteral("Protocol Details"),
                                  QStringLiteral("Data Plots")};
    expect(tabs != nullptr && tabs->count() == expected.size(),
           "U15: operator tabs exclude Actuators moved to dashboard and Log merged into Motion/Gait");
    if (tabs != nullptr) {
        for (int i = 0; i < expected.size() && i < tabs->count(); ++i) {
            expect(tabs->tabText(i) == expected.at(i), "U15: frozen operator tab order");
        }
    }
    expect(window.minimumSize() == QSize(1100, 720), "U08: root minimum is exactly 1100x720");
    expect(window.size() == QSize(1100, 720), "U08: minimum request settles at 1100x720");
    expect(window.findChild<QPushButton *>(QStringLiteral("motionStopButton")) != nullptr,
           "U17: D-pad center Motion Stop exists");
    expect(window.findChild<QPushButton *>(QStringLiteral("enableAllButton")) != nullptr,
           "U17: persistent Enable All exists");
    expect(window.findChild<QPushButton *>(QStringLiteral("disableAllButton")) != nullptr,
           "U17: persistent Disable All exists");
    auto *estop = window.findChild<QPushButton *>(QStringLiteral("emergencyStopButton"));
    expect(estop != nullptr && !estop->isEnabled(), "U17: Emergency Stop remains disabled");
    window.close();
}

void testElidedLabel()
{
    rb::ui::ElidedLabel label;
    label.resize(96, 20);
    label.setFullText(QStringLiteral("a very long operator-facing diagnostic message"));
    expect(label.fullText().startsWith(QStringLiteral("a very long")), "U33: full text retained");
    expect(label.toolTip() == label.fullText(), "U33: tooltip retains full text");
    expect(label.text().contains(QChar(0x2026)), "U33: displayed text is elided");
    expect(label.minimumSizeHint().width() < label.fullText().size() * 10,
           "U33: minimum size does not demand full diagnostic width");
}

void testCompactVisionSurfaceAndDetailsTree()
{
    rb::test::OperatorConsoleFixture fixture;
    fixture.create(false, true);
    rb::MainWindow window(fixture.controller(), fixture.visionClient.get(),
                          fixture.visionControlClient.get());
    window.resize(1100, 720);
    window.show();
    QApplication::processEvents();

    auto *videoCard = window.findChild<QWidget *>(QStringLiteral("videoCard"));
    expect(videoCard != nullptr, "compact vision test has the main video card");
    if (videoCard != nullptr) {
        expect(videoCard->findChild<QLabel *>(QStringLiteral("visionDiagnostics")) == nullptr,
               "main video card does not own full visionDiagnostics");
        expect(videoCard->findChild<QLabel *>(QStringLiteral("inferenceDiagnostics")) == nullptr,
               "main video card does not own full inferenceDiagnostics");
        expect(videoCard->findChild<QLabel *>(QStringLiteral("captureDiagnostics")) == nullptr,
               "main video card does not own full captureDiagnostics");
        expect(videoCard->findChild<QLineEdit *>(QStringLiteral("inferenceShaValue")) == nullptr,
               "main video card does not own inference identity SHA");
        expect(videoCard->findChild<QLabel *>(QStringLiteral("videoFpsSummary")) != nullptr,
               "main video card exposes the compact received-FPS summary");
        auto *resolutionSummary =
            videoCard->findChild<QLabel *>(QStringLiteral("videoResolutionSummary"));
        expect(resolutionSummary != nullptr
                   && resolutionSummary->text() == QStringLiteral("Resolution --"),
               "Vision Status exposes a camera resolution field before a live frame exists");
        expect(videoCard->findChild<QLabel *>(QStringLiteral("inferencePerformanceSummary")) != nullptr,
               "main video card exposes the compact inference performance summary");
        expect(videoCard->findChild<QLabel *>(QStringLiteral("inferenceDetectionSummary")) != nullptr,
               "main video card exposes the compact inference detection summary");
        expect(videoCard->findChild<QLabel *>(QStringLiteral("captureCountSummary")) != nullptr,
               "main video card exposes the compact capture count summary");
        expect(videoCard->findChild<QWidget *>(QStringLiteral("visionSummaryPanel")) != nullptr,
               "main video card places inference/capture status in a dedicated side summary panel");
        expect(videoCard->findChild<QPushButton *>(QStringLiteral("visionDetailsButton")) == nullptr,
               "redundant Details shortcut is absent from the video card");
        auto *summaryPanel =
            videoCard->findChild<QWidget *>(QStringLiteral("visionSummaryPanel"));
        const QStringList visibleControlNames = {
            QStringLiteral("visionConnectButton"),
            QStringLiteral("startInferenceButton"),
            QStringLiteral("snapshotButton"),
            QStringLiteral("startRecordingButton")};
        auto *hiddenRefresh =
            videoCard->findChild<QPushButton *>(QStringLiteral("visionRefreshStatusButton"));
        auto *hiddenInferenceStop =
            videoCard->findChild<QPushButton *>(QStringLiteral("stopInferenceButton"));
        auto *hiddenRecordingStop =
            videoCard->findChild<QPushButton *>(QStringLiteral("stopRecordingButton"));
        expect(hiddenRefresh != nullptr && !hiddenRefresh->isVisible(),
               "redundant Refresh Status action is hidden from the operator surface");
        expect(hiddenInferenceStop != nullptr && !hiddenInferenceStop->isVisible()
                   && hiddenRecordingStop != nullptr && !hiddenRecordingStop->isVisible(),
               "normal operation presents one inference toggle and one recording toggle");

        int previousY = -1;
        for (const QString &name : visibleControlNames) {
            auto *button = videoCard->findChild<QPushButton *>(name);
            expect(button != nullptr && button->isVisible()
                       && summaryPanel != nullptr
                       && summaryPanel->isAncestorOf(button),
                   "Vision Status owns every visible video/capture action");
            if (button != nullptr) {
                const int y = button->mapTo(summaryPanel, QPoint(0, 0)).y();
                expect(y > previousY,
                       "Vision Status actions are stacked vertically");
                previousY = y;
            }
        }
        auto *fpsSummary = videoCard->findChild<QLabel *>(QStringLiteral("videoFpsSummary"));
        auto *performance = videoCard->findChild<QLabel *>(QStringLiteral("inferencePerformanceSummary"));
        auto *detections = videoCard->findChild<QLabel *>(QStringLiteral("inferenceDetectionSummary"));
        auto *memory = videoCard->findChild<QLabel *>(QStringLiteral("inferenceMemorySummary"));
        auto *httpState = videoCard->findChild<QLabel *>(QStringLiteral("visionControlState"));
        auto *captureCounts = videoCard->findChild<QLabel *>(QStringLiteral("captureCountSummary"));
        expect(fpsSummary != nullptr
                   && fpsSummary->toolTip() == QStringLiteral(
                       "Received RBVS frame rate; not inference FPS or unique display FPS.")
                   && fpsSummary->styleSheet().contains(QStringLiteral("#257A9E"))
                   && fpsSummary->styleSheet().contains(QStringLiteral("13px")),
               "received Video FPS is visually emphasized with its dedicated color");
        expect(performance != nullptr
                   && performance->text() == QStringLiteral("-- FPS / -- ms")
                   && performance->styleSheet().contains(QStringLiteral("#6C5AAE"))
                   && performance->styleSheet().contains(QStringLiteral("13px")),
               "inference FPS/latency uses a second emphasized color while masking inactive values");
        expect(detections != nullptr && detections->text() == QStringLiteral("Detections --"),
               "absent inference detection count remains distinct from zero");
        expect(memory != nullptr && memory->text() == QStringLiteral("Memory -- / --")
                   && memory->toolTip() == QStringLiteral(
                       "Vision process RSS / total system physical memory"),
               "memory summary starts masked and identifies its process/system scope");
        expect(detections != nullptr && memory != nullptr && httpState != nullptr
                   && detections->mapTo(summaryPanel, QPoint(0, 0)).y()
                       < memory->mapTo(summaryPanel, QPoint(0, 0)).y()
                   && memory->mapTo(summaryPanel, QPoint(0, 0)).y()
                       < httpState->mapTo(summaryPanel, QPoint(0, 0)).y(),
               "memory summary is placed between detections and HTTP status");
        auto *captureState = videoCard->findChild<QLabel *>(QStringLiteral("captureState"));
        expect(captureCounts != nullptr
                   && captureState != nullptr
                   && captureState->text() == QStringLiteral("Capture Unknown")
                   && captureCounts->text().contains(QStringLiteral("Recorded --"))
                   && captureCounts->text().contains(QStringLiteral("Snapshots --")),
               "stale or missing capture status masks main counters");
    }

    expect(window.findChild<QLabel *>(QStringLiteral("visionDiagnostics")) != nullptr,
           "Vision Details owns the sole visionDiagnostics widget");
    expect(window.findChild<QLabel *>(QStringLiteral("visionEndpointDetails")) != nullptr,
           "Vision Details owns the committed endpoint summary");
    expect(window.findChild<QLabel *>(QStringLiteral("controlResponseDetails")) != nullptr,
           "Vision Details owns the control response summary");
    auto *message = window.findChild<QLabel *>(QStringLiteral("visionControlMessage"));
    expect(message != nullptr && !message->isVisible(),
           "empty vision notice is hidden rather than reserving a visible row");
    window.close();
}

void testMinimumToolPagesUseVerticalScrollOnly()
{
    rb::test::OperatorConsoleFixture fixture;
    fixture.create(false, true);
    rb::MainWindow window(fixture.controller(), fixture.visionClient.get(),
                          fixture.visionControlClient.get());
    window.resize(1100, 720);
    window.show();
    QApplication::processEvents();

    const QStringList pages = {
        QStringLiteral("motionPage"), QStringLiteral("servoFineControlPage"),
        QStringLiteral("visionDetailsPage"), QStringLiteral("telemetryDetailsPage"),
        QStringLiteral("protocolDetailsPage"), QStringLiteral("dataPlotsPage")};
    for (const QString &pageName : pages) {
        expect(window.findChild<QWidget *>(pageName) != nullptr,
               "frozen operator page has its structural objectName");
    }
    const QStringList scrollAreas = {
        QStringLiteral("motionScrollArea"),
        QStringLiteral("servoFineControlScrollArea")};
    for (const QString &scrollName : scrollAreas) {
        auto *scroll = window.findChild<QScrollArea *>(scrollName);
        expect(scroll != nullptr, "minimum operator page has a named scroll area");
        if (scroll != nullptr) {
            expect(scroll->widgetResizable(), "operator scroll area resizes its page widget");
            expect(scroll->frameShape() == QFrame::NoFrame,
                   "operator scroll area has no extra frame");
            expect(scroll->horizontalScrollBarPolicy() == Qt::ScrollBarAlwaysOff,
                   "operator scroll area never introduces horizontal scrolling");
            expect(scroll->verticalScrollBarPolicy() == Qt::ScrollBarAsNeeded,
                   "operator scroll area allows vertical scrolling when needed");
        }
    }
    auto *motionStop = window.findChild<QPushButton *>(QStringLiteral("motionStopButton"));
    auto *enableAll = window.findChild<QPushButton *>(QStringLiteral("enableAllButton"));
    auto *disableAll = window.findChild<QPushButton *>(QStringLiteral("disableAllButton"));
    auto *actionBar = window.findChild<QWidget *>(QStringLiteral("operatorActionBar"));
    expect(motionStop != nullptr && actionBar != nullptr && !actionBar->isAncestorOf(motionStop),
           "Motion Stop lives in the Motion/Gait D-pad rather than the persistent action bar");
    expect(enableAll != nullptr && enableAll->parentWidget() == actionBar,
           "persistent Enable All remains outside page scroll content");
    expect(disableAll != nullptr && disableAll->parentWidget() == actionBar,
           "persistent Disable All remains outside page scroll content");
    window.close();
}

bool fullyContainedInVisibleAncestors(QWidget *widget, QWidget *root)
{
    if (widget == nullptr || root == nullptr || !widget->isVisible()
        || widget->width() <= 0 || widget->height() <= 0) {
        return false;
    }
    QWidget *current = widget;
    while (current != root) {
        QWidget *parent = current->parentWidget();
        if (parent == nullptr || !parent->isVisible()) {
            return false;
        }
        const QRect mapped(current->mapTo(parent, QPoint(0, 0)), current->size());
        if (!parent->rect().contains(mapped)) {
            return false;
        }
        current = parent;
    }
    return true;
}

void testReviewerClosureContracts()
{
    rb::test::OperatorConsoleFixture fixture;
    fixture.create(false, true);
    rb::MainWindow window(fixture.controller(), fixture.visionClient.get(),
                          fixture.visionControlClient.get());
    window.resize(1100, 720);
    window.show();
    QApplication::processEvents();

    const QStringList requiredNames = {
        QStringLiteral("dashboard"), QStringLiteral("leakCard"),
        QStringLiteral("imuCard"), QStringLiteral("depthCard"),
        QStringLiteral("protocolSummaryCard"), QStringLiteral("protocolCountersDetails")};
    for (const QString &name : requiredNames) {
        expect(window.findChild<QWidget *>(name) != nullptr,
               "review closure: frozen structural objectName exists");
    }
    auto *leakCard = window.findChild<QWidget *>(QStringLiteral("leakCard"));
    auto *imuCard = window.findChild<QWidget *>(QStringLiteral("imuCard"));
    auto *depthCard = window.findChild<QWidget *>(QStringLiteral("depthCard"));
    auto *protocolCard = window.findChild<QWidget *>(QStringLiteral("protocolSummaryCard"));
    auto *actuatorPanel = window.findChild<QWidget *>(QStringLiteral("actuatorPage"));
    auto *telemetryRail =
        window.findChild<QWidget *>(QStringLiteral("telemetrySidebar"));
    auto *dashboard = window.findChild<QWidget *>(QStringLiteral("dashboard"));
    auto *videoCardForTelemetry =
        window.findChild<QWidget *>(QStringLiteral("videoCard"));

    expect(leakCard != nullptr && imuCard != nullptr
               && depthCard != nullptr && protocolCard != nullptr
               && actuatorPanel != nullptr
               && leakCard->y() < imuCard->y()
               && imuCard->y() < depthCard->y()
               && depthCard->y() < protocolCard->y()
               && protocolCard->y() < actuatorPanel->y()
               && qAbs(leakCard->x() - imuCard->x()) <= 2
               && qAbs(imuCard->x() - depthCard->x()) <= 2
               && qAbs(depthCard->x() - protocolCard->x()) <= 2
               && qAbs(protocolCard->x() - actuatorPanel->x()) <= 2,
           "feedback: telemetry and actuator status cards form one compact vertical stack");
    expect(telemetryRail != nullptr
               && telemetryRail->isAncestorOf(leakCard)
               && telemetryRail->isAncestorOf(imuCard)
               && telemetryRail->isAncestorOf(depthCard)
               && telemetryRail->isAncestorOf(protocolCard)
               && telemetryRail->isAncestorOf(actuatorPanel),
           "feedback: Leak/IMU/Depth/Protocol/Actuator share one telemetry rail");

    if (dashboard != nullptr && videoCardForTelemetry != nullptr
        && telemetryRail != nullptr) {
        const int videoRight =
            videoCardForTelemetry->mapTo(dashboard, QPoint(0, 0)).x()
            + videoCardForTelemetry->width();
        const int telemetryLeft =
            telemetryRail->mapTo(dashboard, QPoint(0, 0)).x();
        expect(videoRight <= telemetryLeft,
               "feedback: dashboard uses Video then one compact telemetry/status rail");
    } else {
        expect(false, "feedback: two-column dashboard widgets all exist");
    }
    expect(window.findChild<QScrollArea *>(
               QStringLiteral("actuatorScrollArea")) == nullptr,
           "feedback: Actuator Control no longer owns a separate dashboard column");

    expect(leakCard != nullptr && leakCard->height() >= 60
               && imuCard != nullptr && imuCard->height() >= 96
               && depthCard != nullptr && depthCard->height() >= 92
               && protocolCard != nullptr && protocolCard->height() >= 110
               && actuatorPanel != nullptr && actuatorPanel->height() >= 145,
           "feedback: telemetry/status cards keep their minimum content budget while expanding vertically");
    expect(videoCardForTelemetry != nullptr && telemetryRail != nullptr
               && qAbs(videoCardForTelemetry->height() - telemetryRail->height()) <= 4,
           "feedback: the five right-side status modules expand to the same overall dashboard height as Realtime Video");
    for (QWidget *card :
         {leakCard, imuCard, depthCard, protocolCard, actuatorPanel}) {
        expect(card != nullptr && card->maximumWidth() <= 230,
               "feedback: middle status rail keeps a narrow horizontal footprint");
    }

    for (const QString &name : {QStringLiteral("visionDiagnostics"),
                                QStringLiteral("inferenceDiagnostics"),
                                QStringLiteral("captureDiagnostics")}) {
        auto *label = window.findChild<QLabel *>(name);
        expect(label != nullptr
                   && (label->textInteractionFlags() & Qt::TextSelectableByMouse),
               "review closure: Vision Details diagnostics are mouse-selectable");
    }
    for (const QString &name : {QStringLiteral("inferenceDiagnostics"),
                                QStringLiteral("captureDiagnostics"),
                                QStringLiteral("inferenceLastErrorValue"),
                                QStringLiteral("controlResponseDetails")}) {
        auto *label = window.findChild<QLabel *>(name);
        expect(label != nullptr && label->textFormat() == Qt::PlainText,
               "review closure: externally supplied Details text is forced to PlainText");
    }
    auto *protocolCounters =
        window.findChild<QLabel *>(QStringLiteral("protocolCountersDetails"));
    expect(protocolCounters != nullptr
               && protocolCounters->text().contains(QStringLiteral("TX"))
               && protocolCounters->text().contains(QStringLiteral("RX"))
               && protocolCounters->text().contains(QStringLiteral("CRC"))
               && protocolCounters->text().contains(QStringLiteral("Timeout"))
               && protocolCounters->text().contains(QStringLiteral("RTT")),
           "review closure: Protocol Details retains the full counters summary");
    rb::ProtocolMonitor monitor;
    monitor.txPacketCount = 41;
    monitor.rxPacketCount = 37;
    monitor.crcErrorCount = 3;
    monitor.timeoutCount = 5;
    monitor.lastAckRttMs = 17;
    monitor.ackStatus = QStringLiteral("ACK OK");
    fixture.controller()->protocolMonitorChanged(monitor);
    QApplication::processEvents();
    expect(protocolCounters != nullptr
               && protocolCounters->text().contains(QStringLiteral("41"))
               && protocolCounters->text().contains(QStringLiteral("37"))
               && protocolCounters->text().contains(QStringLiteral("3"))
               && protocolCounters->text().contains(QStringLiteral("5"))
               && protocolCounters->text().contains(QStringLiteral("17")),
           "U25: Protocol Details counters update from the same live ProtocolMonitor signal");

    const QStringList criticalNames = {
        QStringLiteral("piHost"), QStringLiteral("applyPiHostButton"),
        QStringLiteral("connectRobotButton"), QStringLiteral("visionConnectButton"),
        QStringLiteral("startInferenceButton"), QStringLiteral("snapshotButton"),
        QStringLiteral("startRecordingButton"),
        QStringLiteral("enableAllButton"), QStringLiteral("disableAllButton"),
        QStringLiteral("emergencyStopButton")};
    auto *compatInferenceStop =
        window.findChild<QPushButton *>(QStringLiteral("stopInferenceButton"));
    auto *compatRecordingStop =
        window.findChild<QPushButton *>(QStringLiteral("stopRecordingButton"));
    expect(compatInferenceStop != nullptr && !compatInferenceStop->isVisible()
               && compatRecordingStop != nullptr && !compatRecordingStop->isVisible(),
           "U10: compatibility Stop actions are not duplicate visible controls");
    QList<QWidget *> critical;
    for (const QString &name : criticalNames) {
        QWidget *widget = window.findChild<QWidget *>(name);
        critical.push_back(widget);
        expect(fullyContainedInVisibleAncestors(widget, &window),
               "U10: critical visible control is fully contained by active ancestors");
    }
    for (int i = 0; i < critical.size(); ++i) {
        for (int j = i + 1; j < critical.size(); ++j) {
            if (critical.at(i) == nullptr || critical.at(j) == nullptr) continue;
            const QRect a(critical.at(i)->mapTo(&window, QPoint(0, 0)), critical.at(i)->size());
            const QRect b(critical.at(j)->mapTo(&window, QPoint(0, 0)), critical.at(j)->size());
            expect(!a.intersects(b), "U10: critical controls do not overlap at minimum size");
        }
    }

    auto *tabs = window.findChild<QTabWidget *>(QStringLiteral("operatorToolsTabs"));
    auto *motionStop = window.findChild<QPushButton *>(QStringLiteral("motionStopButton"));
    auto *enableAll = window.findChild<QPushButton *>(QStringLiteral("enableAllButton"));
    auto *disableAll = window.findChild<QPushButton *>(QStringLiteral("disableAllButton"));
    expect(tabs != nullptr, "U17: operator tabs exist for persistent-action sweep");
    if (tabs != nullptr) {
        for (int i = 0; i < tabs->count(); ++i) {
            tabs->setCurrentIndex(i);
            QApplication::processEvents();
            for (auto *scroll : tabs->widget(i)->findChildren<QScrollArea *>()) {
                scroll->verticalScrollBar()->setValue(scroll->verticalScrollBar()->maximum());
            }
            QApplication::processEvents();
            expect(fullyContainedInVisibleAncestors(enableAll, &window)
                       && fullyContainedInVisibleAncestors(disableAll, &window),
                   "U17: persistent actuator actions remain visible after every tab/page scroll");
        }
        tabs->setCurrentIndex(0);
        auto *motionScroll = window.findChild<QScrollArea *>(QStringLiteral("motionScrollArea"));
        if (motionScroll != nullptr) {
            motionScroll->verticalScrollBar()->setValue(0);
        }
        QApplication::processEvents();
        bool motionStopReachable = false;
        if (motionScroll != nullptr && motionStop != nullptr) {
            motionScroll->ensureWidgetVisible(motionStop, 0, 0);
            QApplication::processEvents();
            const QRect stopInViewport(
                motionStop->mapTo(motionScroll->viewport(), QPoint(0, 0)),
                motionStop->size());
            motionStopReachable =
                stopInViewport.intersects(motionScroll->viewport()->rect());
        }
        expect(motionStopReachable,
               "U17: minimum Motion/Gait can scroll directly to the center Stop button");
    }

    auto *video = window.findChild<rb::vision::VideoView *>(QStringLiteral("videoView"));
    auto *videoCard = window.findChild<QWidget *>(QStringLiteral("videoCard"));
    auto *sidebar = window.findChild<QWidget *>(QStringLiteral("telemetrySidebar"));
    auto *splitter = window.findChild<QSplitter *>(QStringLiteral("workspaceSplitter"));
    expect(dashboard != nullptr && videoCard != nullptr && sidebar != nullptr
               && dashboard->isAncestorOf(videoCard)
               && dashboard->isAncestorOf(sidebar)
               && !videoCard->isAncestorOf(sidebar),
           "feedback: minimum dashboard keeps Video and telemetry as sibling columns");
    expect(video != nullptr
               && video->width() * 3 == video->height() * 4,
           "U04: VideoView uses an exact 4:3 source-ratio surface with no letterbox edge");
    expect(splitter != nullptr && splitter->sizes().size() == 2
               && splitter->sizes().at(0) > 0 && splitter->sizes().at(1) > 0,
           "U06: both splitter panes remain nonzero at minimum size");

    if (tabs != nullptr) {
        window.resize(1600, 1000);
        QApplication::processEvents();

        auto *cardsHost =
            window.findChild<QWidget *>(QStringLiteral("actuatorCardsHost"));
        auto *finePage =
            window.findChild<QWidget *>(QStringLiteral("servoFineControlPage"));
        expect(cardsHost != nullptr && finePage != nullptr,
               "Actuator status list and Servo Fine Control page exist");
        expect(window.findChild<QScrollArea *>(
                   QStringLiteral("actuatorScrollArea")) == nullptr,
               "Actuator status list does not consume a separate scroll column");

        QList<QWidget *> originalRows;
        for (int index = 0; index < rb::kServoCount; ++index) {
            QWidget *row = window.findChild<QWidget *>(
                QStringLiteral("servoStatusRow%1").arg(index));
            if (row != nullptr) {
                originalRows.append(row);
                expect(row->parentWidget() == cardsHost,
                       "every actuator status row belongs to one compact host");
                expect(row->findChildren<QPushButton *>().isEmpty()
                           && row->findChildren<QSpinBox *>().isEmpty()
                           && row->findChildren<QDoubleSpinBox *>().isEmpty()
                           && row->findChildren<QSlider *>().isEmpty(),
                       "Actuator status rows contain status only");
            }
        }
        expect(originalRows.size() == rb::kServoCount,
               "exactly five actuator status rows exist");
        if (originalRows.size() == rb::kServoCount) {
            const int firstX = originalRows.first()->x();
            int previousY = -1;
            bool oneColumn = true;
            for (QWidget *row : originalRows) {
                oneColumn = oneColumn
                    && qAbs(row->x() - firstX) <= 2
                    && row->y() > previousY;
                previousY = row->y();
            }
            expect(oneColumn,
                   "all five actuator status rows form one vertical list");

            QList<int> centerGaps;
            for (int index = 1; index < originalRows.size(); ++index) {
                const int previousCenter =
                    originalRows.at(index - 1)->geometry().center().y();
                const int currentCenter =
                    originalRows.at(index)->geometry().center().y();
                centerGaps.append(currentCenter - previousCenter);
            }
            if (!centerGaps.isEmpty()) {
                const auto [minGap, maxGap] =
                    std::minmax_element(centerGaps.cbegin(), centerGaps.cend());
                expect(*maxGap - *minGap <= 4,
                       "Actuator Control distributes the five status rows evenly through its available height");
            }
        }

        auto *savedPwm =
            window.findChild<QSpinBox *>(QStringLiteral("servoPwmSpin0"));
        auto *savedAngle =
            window.findChild<QDoubleSpinBox *>(QStringLiteral("servoAngleSpin0"));
        auto *savedSlider =
            window.findChild<QSlider *>(QStringLiteral("servoPwmSlider0"));
        auto *applyPwm =
            window.findChild<QPushButton *>(QStringLiteral("servoApplyButton0"));
        auto *applyAngle =
            window.findChild<QPushButton *>(QStringLiteral("servoAngleApplyButton0"));
        auto *enable =
            window.findChild<QPushButton *>(QStringLiteral("servoEnableButton0"));
        auto *neutral =
            window.findChild<QPushButton *>(QStringLiteral("servoNeutralButton0"));
        expect(savedPwm != nullptr && savedAngle != nullptr
                   && savedSlider != nullptr && applyPwm != nullptr
                   && applyAngle != nullptr && enable != nullptr
                   && neutral != nullptr,
               "Servo Fine Control owns PWM/Angle dual-Apply/Enable/Neutral controls");
        expect(savedAngle != nullptr
                   && savedAngle->buttonSymbols() == QAbstractSpinBox::NoButtons,
               "Angle editor keeps numeric setting but removes up/down buttons");
        expect(applyPwm != nullptr
                   && applyPwm->text() == QStringLiteral("Apply PWM"),
               "Servo Fine Control exposes an explicit Apply PWM action");
        expect(applyAngle != nullptr
                   && applyAngle->text() == QStringLiteral("Apply Angle"),
               "Servo Fine Control exposes an explicit Apply Angle action");

        if (savedPwm != nullptr && savedPwm->maximum() > savedPwm->minimum()) {
            savedPwm->setValue(
                savedPwm->minimum()
                + (savedPwm->maximum() - savedPwm->minimum()) / 3);
        }
        if (savedAngle != nullptr && savedAngle->maximum() > savedAngle->minimum()) {
            savedAngle->setValue(
                savedAngle->minimum()
                + (savedAngle->maximum() - savedAngle->minimum()) / 3.0);
        }
        const int expectedPwm = savedPwm != nullptr ? savedPwm->value() : 0;
        const double expectedAngle =
            savedAngle != nullptr ? savedAngle->value() : 0.0;

        window.resize(1100, 720);
        QApplication::processEvents();

        bool samePointers = true;
        for (int index = 0; index < originalRows.size(); ++index) {
            samePointers = samePointers
                && window.findChild<QWidget *>(
                       QStringLiteral("servoStatusRow%1").arg(index))
                       == originalRows.at(index);
        }
        expect(samePointers,
               "minimum-size resize preserves every actuator status row pointer");
        expect(savedPwm != nullptr && savedPwm->value() == expectedPwm
                   && savedAngle != nullptr
                   && qFuzzyCompare(savedAngle->value() + 1.0,
                                    expectedAngle + 1.0),
               "Servo Fine Control PWM/Angle values survive window resize");

        window.resize(1600, 1000);
        QApplication::processEvents();
        expect(video != nullptr && video->width() >= 420,
               "comfortable layout keeps a substantial live-video surface");
        expect(video != nullptr
                   && video->width() * 3 == video->height() * 4,
               "comfortable VideoView remains exactly source-aspect matched");
    }

    window.close();
}

void testPresentationDetailsReflowAndNoSideEffects()
{
    rb::test::OperatorConsoleFixture fixture;
    fixture.create(false, true);
    rb::MainWindow window(fixture.controller(), fixture.visionClient.get(),
                          fixture.visionControlClient.get());
    window.resize(1420, 880);
    window.show();
    QApplication::processEvents();
    expect(window.size() == QSize(1420, 880),
           "U09: 1420x880 settles exactly");

    auto *splitter = window.findChild<QSplitter *>(QStringLiteral("workspaceSplitter"));
    auto *tabs = window.findChild<QTabWidget *>(QStringLiteral("operatorToolsTabs"));
    auto *video = window.findChild<rb::vision::VideoView *>(QStringLiteral("videoView"));
    auto *sidebar = window.findChild<QWidget *>(QStringLiteral("telemetrySidebar"));
    expect(splitter != nullptr && splitter->count() == 2
               && !splitter->isCollapsible(0) && !splitter->isCollapsible(1),
           "U06: splitter panes are nonzero and noncollapsible");
    auto *videoCard = window.findChild<QWidget *>(QStringLiteral("videoCard"));
    auto *dashboard = window.findChild<QWidget *>(QStringLiteral("dashboard"));
    auto *actuatorPanel = window.findChild<QWidget *>(QStringLiteral("actuatorPage"));
    auto *protocolCard = window.findChild<QWidget *>(QStringLiteral("protocolSummaryCard"));
    expect(dashboard != nullptr && videoCard != nullptr && sidebar != nullptr
               && actuatorPanel != nullptr && protocolCard != nullptr
               && dashboard->isAncestorOf(videoCard)
               && dashboard->isAncestorOf(sidebar)
               && sidebar->isAncestorOf(actuatorPanel)
               && actuatorPanel->y() > protocolCard->y()
               && !videoCard->isAncestorOf(sidebar),
           "feedback: normal dashboard uses Video plus one telemetry/status rail with Actuator below Protocol");
    expect(video != nullptr && qAbs(video->width() * 3 - video->height() * 4) <= 6,
           "U04: normal VideoView remains source-aspect matched");
    const int videoHeight880 = video == nullptr ? 0 : video->height();
    window.resize(1420, 1000);
    QApplication::processEvents();
    expect(video != nullptr
               && video->height() >= videoHeight880
               && qAbs(video->width() * 3 - video->height() * 4) <= 6,
           "U05: extra vertical space never shrinks or distorts the source-matched video");
    expect(window.size() == QSize(1420, 1000), "U09: 1420x1000 settles exactly");
    window.resize(1600, 1000);
    QApplication::processEvents();
    expect(window.size() == QSize(1600, 1000),
           "U09: 1600x1000 settles exactly");
    if (tabs != nullptr) {
        tabs->setCurrentIndex(0);
        QApplication::processEvents();
        auto *motionScroll = window.findChild<QScrollArea *>(QStringLiteral("motionScrollArea"));
        expect(motionScroll != nullptr && motionScroll->verticalScrollBar()->maximum() == 0,
               "feedback: Motion/Gait is fully visible at the comfortable window size");
    }

    if (splitter != nullptr && tabs != nullptr) {
        splitter->setSizes({splitter->height() / 2, splitter->height() / 2});
        QApplication::processEvents();
        const auto before = splitter->sizes();
        tabs->setCurrentIndex(4);
        QApplication::processEvents();
        expect(splitter->sizes() == before,
               "U07: tab changes do not reset user splitter sizes");
    }

    expect(window.findChild<QPushButton *>(QStringLiteral("visionDetailsButton")) == nullptr,
           "U16: redundant video Details shortcut is removed; Vision Details remains a direct tab");
    expect(tabs != nullptr && tabs->count() > 2
               && tabs->tabText(2) == QStringLiteral("Vision Details"),
           "U16: Vision Details remains directly available after Servo Fine Control");
    expect(fixture.directTransport.writes().isEmpty(),
           "U22/U38: presentation changes emit no robot writes");

    videoCard = window.findChild<QWidget *>(QStringLiteral("videoCard"));
    expect(videoCard != nullptr
               && videoCard->findChild<QLabel *>(QStringLiteral("inferenceDiagnostics")) == nullptr
               && videoCard->findChild<QLineEdit *>(QStringLiteral("inferenceShaValue")) == nullptr,
           "U26: main video contains compact summaries, not full identity diagnostics");
    expect(window.findChild<QLineEdit *>(QStringLiteral("inferenceShaValue")) != nullptr
               && window.findChild<QLineEdit *>(QStringLiteral("inferenceShaValue"))->isReadOnly(),
           "U27: full inference identity is copyable in Vision Details");
    expect(window.findChild<QTabWidget *>(QStringLiteral("dataPlotTabs")) != nullptr,
           "U15: Data Plots retains IMU/Depth/Actuator placeholders");

    auto *visionPort = window.findChild<QSpinBox *>(QStringLiteral("visionPort"));
    auto *robotPort = window.findChild<QSpinBox *>(QStringLiteral("robotTcpPort"));
    auto *piHost = window.findChild<QLineEdit *>(QStringLiteral("piHost"));
    auto *connectRobot = window.findChild<QPushButton *>(QStringLiteral("connectRobotButton"));
    auto *applyHost = window.findChild<QPushButton *>(QStringLiteral("applyPiHostButton"));
    auto *serialRefresh = window.findChild<QPushButton *>(QStringLiteral("serialRefreshButton"));
    auto *connectionState = window.findChild<QLabel *>(QStringLiteral("connectionStatus"));
    auto *authorityState = window.findChild<QLabel *>(QStringLiteral("authorityStatus"));
    expect(visionPort != nullptr && visionPort->buttonSymbols() == QAbstractSpinBox::NoButtons
               && visionPort->minimum() == 1 && visionPort->maximum() == 65535
               && robotPort != nullptr,
           "U11/U12: remote endpoint fields retain labels, ranges and no buttons");
    expect(piHost != nullptr && piHost->maximumWidth() <= 230,
           "feedback: Pi Host editor is compact rather than consuming the full header");
    expect(piHost != nullptr && robotPort != nullptr && connectRobot != nullptr
               && qAbs(piHost->mapTo(&window, piHost->rect().center()).y()
                       - robotPort->mapTo(&window, robotPort->rect().center()).y()) <= 6
               && qAbs(piHost->mapTo(&window, piHost->rect().center()).y()
                       - connectRobot->mapTo(&window, connectRobot->rect().center()).y()) <= 6,
           "feedback: Robot TCP and Connect share the same top row as Pi Host");
    expect(applyHost != nullptr && !applyHost->isEnabled()
               && applyHost->text() == QStringLiteral("Applied"),
           "feedback: clean Pi Host state has explicit applied feedback");
    expect(serialRefresh != nullptr && !serialRefresh->isVisible(),
           "feedback: Remote mode never exposes the serial Refresh button");
    expect(connectionState != nullptr && connectionState->isVisible()
               && connectionState->text() == QStringLiteral("Disconnected")
               && authorityState != nullptr && authorityState->isVisible()
               && authorityState->text() == QStringLiteral("Unowned"),
           "feedback: top header keeps transport State and control ownership visible");
    expect(window.findChild<QPushButton *>(QStringLiteral("motionStopButton"))
               ->property("consoleActionRole").toString() == QStringLiteral("stop")
               && window.findChild<QPushButton *>(QStringLiteral("emergencyStopButton"))->isEnabled() == false,
           "U17/U36: persistent stop/danger roles are distinct and Emergency Stop disabled");

    auto *cardsHost =
        window.findChild<QWidget *>(QStringLiteral("actuatorCardsHost"));
    auto *actuatorPanelStatus =
        window.findChild<QWidget *>(QStringLiteral("actuatorPage"));
    auto *protocolCardStatus =
        window.findChild<QWidget *>(QStringLiteral("protocolSummaryCard"));
    expect(cardsHost != nullptr && actuatorPanelStatus != nullptr
               && protocolCardStatus != nullptr,
           "U20: compact actuator status list exists");
    if (cardsHost != nullptr) {
        QList<QWidget *> rows;
        for (int index = 0; index < rb::kServoCount; ++index) {
            QWidget *row = window.findChild<QWidget *>(
                QStringLiteral("servoStatusRow%1").arg(index));
            if (row != nullptr) {
                rows.append(row);
            }
        }
        expect(rows.size() == rb::kServoCount,
               "feedback: actuator status list retains five servo rows");
        bool oneColumn = rows.size() == rb::kServoCount;
        int lastY = -1;
        const int firstX = rows.isEmpty() ? 0 : rows.first()->x();
        for (QWidget *row : rows) {
            oneColumn = oneColumn
                && qAbs(row->x() - firstX) <= 2
                && row->y() > lastY;
            lastY = row->y();
        }
        expect(oneColumn,
               "feedback: actuator status rows stay in one vertical list");
        expect(actuatorPanelStatus->y() > protocolCardStatus->y(),
               "feedback: Actuator Control is positioned below Protocol");

        window.resize(1100, 720);
        QApplication::processEvents();
        bool minimumOneColumn = true;
        lastY = -1;
        const int minimumFirstX = rows.isEmpty() ? 0 : rows.first()->x();
        for (QWidget *row : rows) {
            minimumOneColumn = minimumOneColumn
                && qAbs(row->x() - minimumFirstX) <= 2
                && row->y() > lastY;
            lastY = row->y();
        }
        expect(minimumOneColumn,
               "U21: minimum actuator status list remains one compact column");
    }
    window.close();
}
}

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    testScreenAwareStartupGeometry();
    testTreeAndSizing();
    testElidedLabel();
    testCompactVisionSurfaceAndDetailsTree();
    testMinimumToolPagesUseVerticalScrollOnly();
    testReviewerClosureContracts();
    testPresentationDetailsReflowAndNoSideEffects();
    return failures == 0 ? 0 : 1;
}
