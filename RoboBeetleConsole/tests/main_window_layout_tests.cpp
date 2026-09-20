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
    const int expectedWidth = qMax(1100, qMin(1600, available.width() - 32));
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
    const QStringList expected = {QStringLiteral("Motion / Gait"), QStringLiteral("Actuators"),
                                  QStringLiteral("Vision Details"), QStringLiteral("Telemetry Details"),
                                  QStringLiteral("Protocol Details"), QStringLiteral("Log"),
                                  QStringLiteral("Data Plots")};
    expect(tabs != nullptr && tabs->count() == expected.size(), "U15: exactly seven operator tabs");
    if (tabs != nullptr) {
        for (int i = 0; i < expected.size() && i < tabs->count(); ++i) {
            expect(tabs->tabText(i) == expected.at(i), "U15: frozen operator tab order");
        }
    }
    expect(window.minimumSize() == QSize(1100, 720), "U08: root minimum is exactly 1100x720");
    expect(window.size() == QSize(1100, 720), "U08: minimum request settles at 1100x720");
    expect(window.findChild<QPushButton *>(QStringLiteral("motionStopButton")) != nullptr,
           "U17: persistent Motion Stop exists");
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
        expect(videoCard->findChild<QLabel *>(QStringLiteral("inferencePerformanceSummary")) != nullptr,
               "main video card exposes the compact inference performance summary");
        expect(videoCard->findChild<QLabel *>(QStringLiteral("inferenceDetectionSummary")) != nullptr,
               "main video card exposes the compact inference detection summary");
        expect(videoCard->findChild<QLabel *>(QStringLiteral("captureCountSummary")) != nullptr,
               "main video card exposes the compact capture count summary");
        auto *fpsSummary = videoCard->findChild<QLabel *>(QStringLiteral("videoFpsSummary"));
        auto *performance = videoCard->findChild<QLabel *>(QStringLiteral("inferencePerformanceSummary"));
        auto *detections = videoCard->findChild<QLabel *>(QStringLiteral("inferenceDetectionSummary"));
        auto *captureCounts = videoCard->findChild<QLabel *>(QStringLiteral("captureCountSummary"));
        expect(fpsSummary != nullptr
                   && fpsSummary->toolTip() == QStringLiteral(
                       "Received RBVS frame rate; not inference FPS or unique display FPS."),
               "received FPS summary uses the frozen tooltip");
        expect(performance != nullptr && performance->text() == QStringLiteral("-- FPS / -- ms"),
               "non-active inference masks both performance values");
        expect(detections != nullptr && detections->text() == QStringLiteral("Detections --"),
               "absent inference detection count remains distinct from zero");
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
        QStringLiteral("motionPage"), QStringLiteral("actuatorPage"),
        QStringLiteral("visionDetailsPage"), QStringLiteral("telemetryDetailsPage"),
        QStringLiteral("protocolDetailsPage"), QStringLiteral("dataPlotsPage")};
    for (const QString &pageName : pages) {
        expect(window.findChild<QWidget *>(pageName) != nullptr,
               "frozen operator page has its structural objectName");
    }
    const QStringList scrollAreas = {
        QStringLiteral("motionScrollArea"), QStringLiteral("actuatorScrollArea")};
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
    auto *disableAll = window.findChild<QPushButton *>(QStringLiteral("disableAllButton"));
    expect(motionStop != nullptr && motionStop->parentWidget() != nullptr
               && motionStop->parentWidget()->objectName() == QStringLiteral("operatorActionBar"),
           "persistent Motion Stop remains outside page scroll content");
    expect(disableAll != nullptr && disableAll->parentWidget() != nullptr
               && disableAll->parentWidget()->objectName() == QStringLiteral("operatorActionBar"),
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
    expect(leakCard != nullptr && imuCard != nullptr && depthCard != nullptr && protocolCard != nullptr
               && leakCard->y() < imuCard->y() && imuCard->y() < depthCard->y()
               && depthCard->y() < protocolCard->y(),
           "U03: telemetry sidebar cards form one top-to-bottom vertical stack");
    expect(leakCard != nullptr && imuCard != nullptr && leakCard->height() < imuCard->height(),
           "U03: Leak Detection remains the smallest telemetry card");

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
        QStringLiteral("visionRefreshStatusButton"), QStringLiteral("startInferenceButton"),
        QStringLiteral("stopInferenceButton"), QStringLiteral("snapshotButton"),
        QStringLiteral("startRecordingButton"), QStringLiteral("stopRecordingButton"),
        QStringLiteral("motionStopButton"), QStringLiteral("disableAllButton"),
        QStringLiteral("emergencyStopButton")};
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
            expect(fullyContainedInVisibleAncestors(motionStop, &window)
                       && fullyContainedInVisibleAncestors(disableAll, &window),
                   "U17: persistent stop actions remain visible after every tab/page scroll");
        }
    }

    auto *video = window.findChild<rb::vision::VideoView *>(QStringLiteral("videoView"));
    auto *sidebar = window.findChild<QWidget *>(QStringLiteral("telemetrySidebar"));
    auto *splitter = window.findChild<QSplitter *>(QStringLiteral("workspaceSplitter"));
    expect(video != nullptr && sidebar != nullptr
               && static_cast<double>(video->width()) >= 2.5 * sidebar->width(),
           "U04: minimum dashboard keeps video at least 2.5 times sidebar width");
    expect(splitter != nullptr && splitter->sizes().size() == 2
               && splitter->sizes().at(0) > 0 && splitter->sizes().at(1) > 0,
           "U06: both splitter panes remain nonzero at minimum size");

    if (tabs != nullptr) {
        tabs->setCurrentIndex(1);
        window.resize(1600, 1000);
        QApplication::processEvents();
        auto *cardsHost = window.findChild<QWidget *>(QStringLiteral("actuatorCardsHost"));
        auto *actuatorScroll = window.findChild<QScrollArea *>(QStringLiteral("actuatorScrollArea"));
        const QList<QGroupBox *> originalCards =
            cardsHost != nullptr ? cardsHost->findChildren<QGroupBox *>(QString(), Qt::FindDirectChildrenOnly)
                                 : QList<QGroupBox *>{};
        expect(originalCards.size() == 5, "U20: exactly five actuator card objects exist before reflow");
        QSpinBox *savedPwm = originalCards.isEmpty() ? nullptr : originalCards.first()->findChild<QSpinBox *>();
        QDoubleSpinBox *savedAngle = originalCards.isEmpty() ? nullptr : originalCards.first()->findChild<QDoubleSpinBox *>();
        if (savedPwm != nullptr && savedPwm->maximum() > savedPwm->minimum()) {
            savedPwm->setValue(savedPwm->minimum() + (savedPwm->maximum() - savedPwm->minimum()) / 3);
        }
        if (savedAngle != nullptr && savedAngle->maximum() > savedAngle->minimum()) {
            savedAngle->setValue(savedAngle->minimum() + (savedAngle->maximum() - savedAngle->minimum()) / 3.0);
        }
        const int expectedPwm = savedPwm != nullptr ? savedPwm->value() : 0;
        const double expectedAngle = savedAngle != nullptr ? savedAngle->value() : 0.0;

        window.resize(1100, 720);
        QApplication::processEvents();
        const QList<QGroupBox *> minimumCards =
            cardsHost != nullptr ? cardsHost->findChildren<QGroupBox *>(QString(), Qt::FindDirectChildrenOnly)
                                 : QList<QGroupBox *>{};
        bool samePointers = minimumCards.size() == originalCards.size();
        for (QGroupBox *card : originalCards) samePointers = samePointers && minimumCards.contains(card);
        expect(samePointers, "U20: 5-to-3-plus-2 reflow preserves every actuator QWidget pointer");
        expect(savedPwm != nullptr && savedPwm->value() == expectedPwm
                   && savedAngle != nullptr && qFuzzyCompare(savedAngle->value() + 1.0, expectedAngle + 1.0),
               "U20: actuator PWM/angle values survive the narrow reflow");
        expect(actuatorScroll != nullptr && actuatorScroll->horizontalScrollBar()->maximum() == 0
                   && actuatorScroll->verticalScrollBar()->maximum() > 0,
               "U21: minimum actuator page uses vertical scrolling only");
        if (actuatorScroll != nullptr && !originalCards.isEmpty()) {
            actuatorScroll->verticalScrollBar()->setValue(actuatorScroll->verticalScrollBar()->maximum());
            QApplication::processEvents();
            const QRect lastCardInViewport(
                originalCards.last()->mapTo(actuatorScroll->viewport(), QPoint(0, 0)),
                originalCards.last()->size());
            expect(lastCardInViewport.intersects(actuatorScroll->viewport()->rect()),
                   "U21: scrolling to maximum makes the final actuator card reachable");
        }

        window.resize(1600, 1000);
        QApplication::processEvents();
        const QList<QGroupBox *> finalCards =
            cardsHost != nullptr ? cardsHost->findChildren<QGroupBox *>(QString(), Qt::FindDirectChildrenOnly)
                                 : QList<QGroupBox *>{};
        bool restoredPointers = finalCards.size() == originalCards.size();
        for (QGroupBox *card : originalCards) restoredPointers = restoredPointers && finalCards.contains(card);
        expect(restoredPointers
                   && savedPwm != nullptr && savedPwm->value() == expectedPwm
                   && savedAngle != nullptr && qFuzzyCompare(savedAngle->value() + 1.0, expectedAngle + 1.0),
               "U20: 5-to-3-plus-2-to-5 reflow preserves identity and values");
        expect(video != nullptr && sidebar != nullptr && video->width() >= sidebar->width() * 3,
               "U04: comfortable dashboard keeps video at least three times sidebar width");
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
    expect(video != nullptr && sidebar != nullptr && video->width() >= sidebar->width() * 3,
           "U04: normal dashboard keeps video at least three times sidebar width");
    const int videoHeight880 = video == nullptr ? 0 : video->height();
    window.resize(1420, 1000);
    QApplication::processEvents();
    expect(video != nullptr && video->height() - videoHeight880 >= 84,
           "U05: 120 added pixels yield at least 70 percent video-height gain");
    expect(window.size() == QSize(1420, 1000), "U09: 1420x1000 settles exactly");
    window.resize(1600, 1000);
    QApplication::processEvents();
    expect(window.size() == QSize(1600, 1000),
           "U09: 1600x1000 settles exactly");

    if (splitter != nullptr && tabs != nullptr) {
        splitter->setSizes({splitter->height() / 2, splitter->height() / 2});
        QApplication::processEvents();
        const auto before = splitter->sizes();
        tabs->setCurrentIndex(4);
        QApplication::processEvents();
        expect(splitter->sizes() == before,
               "U07: tab changes do not reset user splitter sizes");
    }

    auto *details = window.findChild<QPushButton *>(QStringLiteral("visionDetailsButton"));
    expect(details != nullptr, "U16: video Details shortcut exists");
    if (details != nullptr && tabs != nullptr) {
        tabs->setCurrentIndex(0);
        details->click();
        expect(tabs->currentIndex() == 2, "U16: Details selects Vision Details only");
    }
    expect(fixture.directTransport.writes().isEmpty(),
           "U22/U38: presentation changes emit no robot writes");

    auto *videoCard = window.findChild<QWidget *>(QStringLiteral("videoCard"));
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
    expect(visionPort != nullptr && visionPort->buttonSymbols() == QAbstractSpinBox::NoButtons
               && visionPort->minimum() == 1 && visionPort->maximum() == 65535
               && robotPort != nullptr,
           "U11/U12: remote endpoint fields retain labels, ranges and no buttons");
    expect(window.findChild<QPushButton *>(QStringLiteral("motionStopButton"))
               ->property("consoleActionRole").toString() == QStringLiteral("stop")
               && window.findChild<QPushButton *>(QStringLiteral("emergencyStopButton"))->isEnabled() == false,
           "U17/U36: persistent stop/danger roles are distinct and Emergency Stop disabled");

    tabs->setCurrentIndex(1);
    QApplication::processEvents();
    auto *cardsHost = window.findChild<QWidget *>(QStringLiteral("actuatorCardsHost"));
    expect(cardsHost != nullptr, "U20: actuator card host exists");
    if (cardsHost != nullptr) {
        int firstRow = 0;
        for (auto *box : cardsHost->findChildren<QGroupBox *>()) {
            if (box->y() < 100) {
                ++firstRow;
            }
        }
        expect(firstRow >= 5, "U20/U21: normal actuator layout has five cards in first row");
        window.resize(1100, 720);
        QApplication::processEvents();
        int rows = 0;
        int firstRowAtMinimum = 0;
        int firstY = -1;
        for (auto *box : cardsHost->findChildren<QGroupBox *>()) {
            if (firstY < 0) firstY = box->y();
            if (box->y() <= firstY + 4) ++firstRowAtMinimum;
            rows = qMax(rows, box->y());
        }
        expect(firstRowAtMinimum == 3 && rows > firstY,
               "U21: minimum actuator layout reflows to three plus two rows");
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
