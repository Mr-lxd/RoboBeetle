#include "protocol/PacketCodec.h"
#include "robot/DepthSnapshot.h"
#include "robot/ImuSnapshot.h"
#include "robot/RobotController.h"
#include "remote/RemoteRobotController.h"
#include "transport/FakeTransport.h"
#include "ui/MainWindow.h"
#include "vision/VisionClient.h"
#include "vision/VisionControlClient.h"
#include "vision/VideoView.h"

#include <QApplication>
#include <QAbstractSpinBox>
#include <QComboBox>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QGroupBox>
#include <QHostAddress>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTabWidget>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>

#include <cstdio>
#include <functional>
#include <memory>

namespace {

int failures = 0;

void expect(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

void injectImuSnapshot(rb::FakeTransport &transport)
{
    rb::ImuSnapshot snapshot;
    snapshot.validityFlags = rb::ImuSnapshot::AccValid
        | rb::ImuSnapshot::GyroValid | rb::ImuSnapshot::AngleValid;
    snapshot.accMg = {1000, -2000, 0};
    snapshot.gyroDecidps = {10, -20, 0};
    snapshot.angleCentidegrees = {300, -400, 500};
    snapshot.diagnostics.validFrameCount = 42;
    transport.injectBytes(rb::PacketCodec::encodeWire(
        {rb::MessageType::ImuSnapshot, 0x6200, rb::ImuSnapshot::encodePayload(snapshot)}));
}

void injectDepthSnapshot(rb::FakeTransport &transport)
{
    rb::DepthSnapshot snapshot;
    snapshot.validityFlags = rb::DepthSnapshot::DepthValid
        | rb::DepthSnapshot::TemperatureValid;
    snapshot.depthMm = 1234;
    snapshot.temperatureCentiC = 2534;
    snapshot.sampleAgeMs = 25;
    snapshot.diagnostics.rxByteCount = 100;
    snapshot.diagnostics.validLineCount = 4;
    transport.injectBytes(rb::PacketCodec::encodeWire(
        {rb::MessageType::DepthSnapshot, 0x6300,
         rb::DepthSnapshot::encodePayload(snapshot)}));
}

void injectLeakState(rb::FakeTransport &transport, rb::LeakState state)
{
    QByteArray payload;
    payload.append(static_cast<char>(state));
    transport.injectBytes(rb::PacketCodec::encodeWire(
        {rb::MessageType::LeakStatus, 0x6100, payload}));
}

QGroupBox *imuPanel(rb::MainWindow &window)
{
    for (QGroupBox *box : window.findChildren<QGroupBox *>()) {
        if (box->title() == QStringLiteral("IMU — JY901S")) {
            return box;
        }
    }
    return nullptr;
}

QGroupBox *depthPanel(rb::MainWindow &window)
{
    for (QGroupBox *box : window.findChildren<QGroupBox *>()) {
        if (box->title() == QStringLiteral("Depth Sensor — ROVMAKER")) {
            return box;
        }
    }
    return nullptr;
}

QGroupBox *leakPanel(rb::MainWindow &window)
{
    for (QGroupBox *box : window.findChildren<QGroupBox *>()) {
        if (box->title() == QStringLiteral("Leak Detection")) {
            return box;
        }
    }
    return nullptr;
}

QGroupBox *motionPanel(rb::MainWindow &window)
{
    for (QGroupBox *box : window.findChildren<QGroupBox *>()) {
        if (box->title() == QStringLiteral("Motion / Gait — Bench")) {
            return box;
        }
    }
    return nullptr;
}

QComboBox *gaitBackendCombo(const QWidget *root)
{
    for (QComboBox *combo : root->findChildren<QComboBox *>()) {
        if (combo->objectName() == QStringLiteral("gaitBackendCombo")) {
            return combo;
        }
    }
    return nullptr;
}

bool hasLabelText(const QWidget *root, const QString &text)
{
    for (QLabel *label : root->findChildren<QLabel *>()) {
        if (label->text() == text) {
            return true;
        }
    }
    return false;
}

QPushButton *buttonWithText(const QWidget *root, const QString &text)
{
    for (QPushButton *button : root->findChildren<QPushButton *>()) {
        if (button->text() == text) {
            return button;
        }
    }
    return nullptr;
}

rb::Packet lastPacket(const rb::FakeTransport &transport)
{
    const QByteArray wire = transport.writes().last();
    return rb::PacketCodec::decodeWire(wire.first(wire.size() - 1)).packet;
}

void acknowledgeLast(rb::FakeTransport &transport)
{
    const rb::Packet request = lastPacket(transport);
    QByteArray payload;
    payload.append(static_cast<char>(request.sequence & 0xffU));
    payload.append(static_cast<char>((request.sequence >> 8U) & 0xffU));
    payload.append(static_cast<char>(request.type));
    payload.append(static_cast<char>(rb::AckResult::Ok));
    transport.injectBytes(rb::PacketCodec::encodeWire(
        {rb::MessageType::Ack, 0x8000, payload}));
}

void acknowledgeLastWithResult(rb::FakeTransport &transport, rb::AckResult result)
{
    const rb::Packet request = lastPacket(transport);
    QByteArray payload;
    payload.append(static_cast<char>(request.sequence & 0xffU));
    payload.append(static_cast<char>((request.sequence >> 8U) & 0xffU));
    payload.append(static_cast<char>(request.type));
    payload.append(static_cast<char>(result));
    transport.injectBytes(rb::PacketCodec::encodeWire(
        {rb::MessageType::Ack, 0x8000, payload}));
}

void waitForMs(int milliseconds)
{
    QEventLoop loop;
    QTimer::singleShot(milliseconds, &loop, &QEventLoop::quit);
    loop.exec();
}

bool waitUntil(const std::function<bool()> &predicate, int timeoutMs = 2000)
{
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeoutMs) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        if (predicate()) {
            return true;
        }
        waitForMs(2);
    }
    QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    return predicate();
}

void answerNextMessageBox(QMessageBox::StandardButton button)
{
    QTimer::singleShot(50, [button] {
        for (QWidget *widget : QApplication::topLevelWidgets()) {
            auto *box = qobject_cast<QMessageBox *>(widget);
            if (box != nullptr) {
                if (auto *push = box->button(button)) {
                    push->click();
                } else {
                    box->done(static_cast<int>(button));
                }
                return;
            }
        }
    });
}

std::shared_ptr<bool> answerNextMessageBoxAndTrack(QMessageBox::StandardButton button)
{
    const auto seen = std::make_shared<bool>(false);
    QTimer::singleShot(50, [button, seen] {
        for (QWidget *widget : QApplication::topLevelWidgets()) {
            auto *box = qobject_cast<QMessageBox *>(widget);
            if (box != nullptr) {
                *seen = true;
                if (auto *push = box->button(button)) {
                    push->click();
                } else {
                    box->done(static_cast<int>(button));
                }
                return;
            }
        }
    });
    return seen;
}

QByteArray readHttpRequest(QTcpSocket *socket)
{
    QByteArray request;
    for (int i = 0; i < 50 && !request.contains("\r\n\r\n"); ++i) {
        if (socket->bytesAvailable() > 0) {
            request += socket->readAll();
        }
        waitForMs(5);
    }
    return request;
}

void sendHttpJson(QTcpSocket *socket, const QByteArray &body)
{
    const QByteArray response =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: " + QByteArray::number(body.size())
        + "\r\nConnection: close\r\n\r\n" + body;
    socket->write(response);
    socket->flush();
    socket->disconnectFromHost();
}

void enablePaddles(rb::FakeTransport &transport, rb::RobotController &controller)
{
    controller.connectTransport({QStringLiteral("COM_TEST"), 9600});
    transport.simulateConnected();
    const rb::ServoId paddles[] = {
        rb::ServoId::FrontRight,
        rb::ServoId::FrontLeft,
        rb::ServoId::RearRight,
        rb::ServoId::RearLeft,
    };
    for (const rb::ServoId id : paddles) {
        controller.enableServo(id);
        acknowledgeLast(transport);
    }
}

QGroupBox *findGroupBox(const QWidget *root, const QString &title);

void testImuPanelLifecycle()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::MainWindow window(&controller);

    QGroupBox *panel = imuPanel(window);
    expect(panel != nullptr, "MainWindow must expose an IMU — JY901S panel");
    if (panel == nullptr) {
        return;
    }
    expect(hasLabelText(panel, QStringLiteral("Unknown")),
           "IMU panel must start with Unknown status");
    expect(hasLabelText(panel, QStringLiteral("--")),
           "IMU panel must not show live values before reception");

    controller.connectTransport({QStringLiteral("COM_TEST"), 9600});
    transport.simulateConnected();
    injectImuSnapshot(transport);
    expect(hasLabelText(panel, QStringLiteral("Receiving")),
           "IMU panel must show Receiving after a valid snapshot");
    expect(hasLabelText(panel, QStringLiteral("1.000, -2.000, 0.000 g")),
           "IMU panel must display fixed-point Acc values");
    expect(hasLabelText(panel, QStringLiteral("1.0, -2.0, 0.0 dps")),
           "IMU panel must display fixed-point Gyro values");
    expect(hasLabelText(panel, QStringLiteral("3.00, -4.00, 5.00 deg")),
           "IMU panel must display fixed-point Angle values");

    controller.imuMonitor()->tick(controller.imuState().lastReceivedAtMs
                                  + rb::ImuMonitor::StaleTimeoutMs);
    expect(hasLabelText(panel, QStringLiteral("Stale")),
           "IMU panel must show Stale after the liveness window");
    expect(hasLabelText(panel, QStringLiteral("--")),
           "IMU panel must clear old values after becoming stale");
}

void testDepthPanelLifecycle()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::MainWindow window(&controller);

    QGroupBox *panel = depthPanel(window);
    expect(panel != nullptr, "MainWindow must expose a Depth Sensor — ROVMAKER panel");
    if (panel == nullptr) {
        return;
    }
    expect(hasLabelText(panel, QStringLiteral("Unknown")),
           "Depth panel must start with Unknown status");
    expect(hasLabelText(panel, QStringLiteral("--")),
           "Depth panel must not show live values before reception");

    controller.connectTransport({QStringLiteral("COM_TEST"), 9600});
    transport.simulateConnected();
    injectDepthSnapshot(transport);
    expect(hasLabelText(panel, QStringLiteral("Receiving")),
           "Depth panel must show Receiving after a valid snapshot");
    expect(hasLabelText(panel, QStringLiteral("1.234 m")),
           "Depth panel must display millimetre values as metres");
    expect(hasLabelText(panel, QStringLiteral("25.34 C")),
           "Depth panel must display centi-degree values as Celsius");

    controller.depthMonitor()->tick(controller.depthState().lastReceivedAtMs
                                    + rb::DepthMonitor::StaleTimeoutMs);
    expect(hasLabelText(panel, QStringLiteral("Stale")),
           "Depth panel must show Stale after the liveness window");
    expect(hasLabelText(panel, QStringLiteral("--")),
           "Depth panel must clear old depth values after becoming stale");
}

void testLeakPanelLifecycle()
{
    rb::FakeTransport transport;
    rb::RobotController controller(
        &transport,
        rb::RobotControllerConfig::bringUpProvisional());
    rb::MainWindow window(&controller);

    QGroupBox *panel = leakPanel(window);
    expect(panel != nullptr,
           "MainWindow must expose a Leak Detection panel");
    if (panel == nullptr) {
        return;
    }

    expect(hasLabelText(panel, QStringLiteral("Unknown")),
           "Leak panel must start with Unknown status");

    controller.connectTransport({QStringLiteral("COM_TEST"), 9600});
    transport.simulateConnected();

    injectLeakState(transport, rb::LeakState::Dry);
    expect(hasLabelText(panel, QStringLiteral("Dry")),
           "Leak panel must display Dry for dry telemetry");

    injectLeakState(transport, rb::LeakState::Wet);
    expect(hasLabelText(panel, QStringLiteral("LEAK DETECTED")),
           "Leak panel must retain the critical LEAK DETECTED warning");
}

void testMotionPanelLifecycleAndManualArbitration()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    rb::MainWindow window(&controller);

    QGroupBox *panel = motionPanel(window);
    expect(panel != nullptr,
           "MainWindow must expose the Motion / Gait — Bench panel");
    if (panel == nullptr) {
        return;
    }

    // The motion panel splits into two visually distinct subareas: a D-pad-like
    // Motion Control block and a separate Gait / Vertical block.
    expect(findGroupBox(panel, QStringLiteral("Motion Control")) != nullptr,
           "Motion panel must expose a Motion Control subarea");
    expect(findGroupBox(panel, QStringLiteral("Gait / Vertical")) != nullptr,
           "Motion panel must expose a Gait / Vertical subarea");

    expect(gaitBackendCombo(panel) != nullptr,
           "Motion panel must expose a dedicated gait backend combo");
    QPushButton *forwardButton = buttonWithText(panel, QStringLiteral("Forward"));
    QPushButton *backwardButton = buttonWithText(panel, QStringLiteral("Backward (Pending)"));
    QPushButton *turnLeftButton = buttonWithText(panel, QStringLiteral("Turn Left"));
    QPushButton *turnRightButton = buttonWithText(panel, QStringLiteral("Turn Right"));
    QPushButton *ascendButton = buttonWithText(panel, QStringLiteral("Ascend"));
    QPushButton *descendButton = buttonWithText(panel, QStringLiteral("Descend"));
    QPushButton *stopButton = buttonWithText(panel, QStringLiteral("Stop"));
    expect(forwardButton != nullptr && backwardButton != nullptr
               && turnLeftButton != nullptr && turnRightButton != nullptr
               && ascendButton != nullptr && descendButton != nullptr
               && stopButton != nullptr,
           "Motion panel must expose direct Forward/Backward/Turn/Axis/Stop buttons");
    if (forwardButton == nullptr || backwardButton == nullptr
        || turnLeftButton == nullptr || turnRightButton == nullptr
        || ascendButton == nullptr || descendButton == nullptr
        || stopButton == nullptr) {
        return;
    }

    expect(hasLabelText(panel, QStringLiteral("Stopped")),
           "Motion panel must start with Stopped status");
    expect(!forwardButton->isEnabled() && !turnLeftButton->isEnabled()
               && !turnRightButton->isEnabled() && !ascendButton->isEnabled()
               && !descendButton->isEnabled() && !stopButton->isEnabled(),
           "Motion controls must be disabled while disconnected");
    expect(!backwardButton->isEnabled()
               && backwardButton->toolTip().contains(QStringLiteral("Pending")),
           "Backward must remain disabled and visibly Pending");

    enablePaddles(transport, controller);
    expect(forwardButton->isEnabled() && turnLeftButton->isEnabled()
               && turnRightButton->isEnabled()
               && !ascendButton->isEnabled() && !descendButton->isEnabled(),
           "paddle-only setup should enable horizontal Motion buttons only");
    expect(controller.enableServo(rb::ServoId::FrontAxis),
           "direct Motion UI setup should enable FrontAxis for vertical modes");
    acknowledgeLast(transport);
    expect(ascendButton->isEnabled() && descendButton->isEnabled(),
           "Ascend and Descend should enable after FrontAxis ACK");
    expect(!backwardButton->isEnabled(),
           "Backward must remain disabled after the link is ready");
    expect(buttonWithText(&window, QStringLiteral("Release PWM")) != nullptr,
           "enabled individual Servo controls must use Release PWM semantics");
    forwardButton->click();
    acknowledgeLast(transport);
    expect(controller.motionState() == rb::MotionState::Running,
           "direct Forward button should reach Running after ACK");
    expect(hasLabelText(panel, QStringLiteral("Running — Forward")),
           "Motion panel should display the running mode");
    expect(forwardButton->isCheckable()
               && forwardButton->styleSheet().contains(QStringLiteral(":checked")),
           "Motion buttons must provide an explicit checked highlight style");
    expect(forwardButton->isChecked()
               && !turnLeftButton->isChecked()
               && !turnRightButton->isChecked()
               && !ascendButton->isChecked()
               && !descendButton->isChecked(),
           "the active Forward button must be checked exclusively");

    for (QPushButton *button : window.findChildren<QPushButton *>()) {
        if (button->text() == QStringLiteral("Set Angle")
            || button->text() == QStringLiteral("Apply PWM")
            || button->text() == QStringLiteral("Neutral")) {
            expect(!button->isEnabled(),
                   "manual Servo controls must be disabled while Motion runs");
        }
    }
    QGroupBox *global = nullptr;
    for (QGroupBox *box : window.findChildren<QGroupBox *>()) {
        if (box->title() == QStringLiteral("Actuator Control")) {
            global = box;
            break;
        }
    }
    expect(global != nullptr,
           "MainWindow must retain its Actuator Control panel");
    if (global != nullptr) {
        QPushButton *disableAll = buttonWithText(global, QStringLiteral("Disable All"));
        expect(disableAll != nullptr && disableAll->isEnabled(),
               "Disable All must remain available during Motion");
    }

    stopButton->click();
    acknowledgeLast(transport);
    expect(controller.motionState() == rb::MotionState::Stopping,
           "Motion panel should display the acceptance-time Stopping state");
    expect(hasLabelText(panel, QStringLiteral("Stopping")),
           "Motion panel should show Stopping during the provisional ramp");
    waitForMs(rb::kMotionTransitionDurationMs + 50);
    expect(controller.motionState() == rb::MotionState::Stopped,
           "Motion panel should settle at Stopped after the provisional duration");
    expect(hasLabelText(panel, QStringLiteral("Stopped")),
           "Motion panel should show Stopped after the ramp timer");
    expect(!forwardButton->isChecked() && !turnLeftButton->isChecked()
               && !turnRightButton->isChecked()
               && !ascendButton->isChecked() && !descendButton->isChecked(),
           "Motion button highlight must clear after graceful STOP completes");
}

void testGaitBackendPanelLifecycle()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    rb::MainWindow window(&controller);

    QGroupBox *panel = motionPanel(window);
    expect(panel != nullptr, "gait backend test must find the Motion / Gait panel");
    if (panel == nullptr) {
        return;
    }
    QComboBox *combo = gaitBackendCombo(panel);
    expect(combo != nullptr, "Motion panel must expose the gait backend combo");
    if (combo == nullptr) {
        return;
    }

    expect(combo->findText(QStringLiteral("Unknown")) >= 0,
           "gait backend combo must expose an explicit Unknown state");
    expect(combo->findText(QStringLiteral("SimpleGait")) >= 0,
           "gait backend combo must expose SimpleGait");
    expect(combo->findText(QStringLiteral("CPG")) >= 0,
           "gait backend combo must expose CPG");
    expect(!combo->isEnabled(),
           "gait backend selection must be disabled while disconnected");

    controller.connectTransport({QStringLiteral("COM_TEST"), 9600});
    transport.simulateConnected();
    expect(combo->isEnabled(),
           "gait backend selection must enable after connection");

    const int simpleIndex = combo->findData(
        static_cast<int>(rb::GaitBackend::SimpleGait));
    expect(simpleIndex >= 0, "SimpleGait combo item must carry its backend value");
    if (simpleIndex < 0) {
        return;
    }
    combo->setCurrentIndex(simpleIndex);
    expect(!transport.writes().isEmpty()
               && lastPacket(transport).type == rb::MessageType::SetGaitBackend,
           "selecting a backend must send SetGaitBackend");
    expect(lastPacket(transport).payload == QByteArray(1, '\0'),
           "SimpleGait UI selection must send the one-byte zero payload");
    expect(!controller.confirmedGaitBackend().has_value(),
           "backend UI selection must not confirm before ACK");
    expect(!combo->isEnabled(),
           "gait backend combo must reject a second selection while pending");

    acknowledgeLastWithResult(transport, rb::AckResult::Busy);
    expect(!controller.confirmedGaitBackend().has_value(),
           "BUSY must preserve the previous confirmed backend in the UI path");
    expect(combo->isEnabled(),
           "gait backend combo must re-enable after BUSY");

    combo->setCurrentIndex(simpleIndex);
    acknowledgeLast(transport);
    expect(controller.confirmedGaitBackend().has_value()
               && *controller.confirmedGaitBackend() == rb::GaitBackend::SimpleGait,
           "matching UI selector ACK must confirm SimpleGait");
    expect(combo->currentData().toInt()
               == static_cast<int>(rb::GaitBackend::SimpleGait),
           "UI combo must reflect the ACK-confirmed backend");
}

QGroupBox *findGroupBox(const QWidget *root, const QString &title)
{
    for (QGroupBox *box : root->findChildren<QGroupBox *>()) {
        if (box->title() == title) {
            return box;
        }
    }
    return nullptr;
}

QTabWidget *tabWidgetWithText(const QWidget *root, const QString &tabText)
{
    for (QTabWidget *tabs : root->findChildren<QTabWidget *>()) {
        for (int i = 0; i < tabs->count(); ++i) {
            if (tabs->tabText(i) == tabText) {
                return tabs;
            }
        }
    }
    return nullptr;
}

void testVisionConnectionIsIndependentFromControlTransport()
{
    rb::FakeTransport transport;
    rb::RobotController controller(
        &transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::vision::VisionClient visionClient;
    rb::MainWindow window(&controller, &visionClient);

    QGroupBox *panel = findGroupBox(&window, QStringLiteral("Realtime Video"));
    expect(panel != nullptr, "Vision integration must retain Realtime Video card");
    if (panel == nullptr) {
        return;
    }

    auto *host = window.findChild<QLineEdit *>(QStringLiteral("piHost"));
    auto *port = window.findChild<QSpinBox *>(QStringLiteral("visionPort"));
    auto *view = panel->findChild<rb::vision::VideoView *>(
        QStringLiteral("videoView"));
    QPushButton *connectVideo =
        buttonWithText(panel, QStringLiteral("Connect Video"));
    expect(host != nullptr && port != nullptr && view != nullptr
               && connectVideo != nullptr,
           "Realtime Video card exposes independent host/port/client controls");
    if (host == nullptr || port == nullptr || connectVideo == nullptr) {
        return;
    }

    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "Vision isolation test server must listen");
    if (!server.isListening()) {
        return;
    }

    host->setText(QStringLiteral("127.0.0.1"));
    port->setValue(server.serverPort());
    if (auto *apply = window.findChild<QPushButton *>(QStringLiteral("applyPiHostButton"))) {
        apply->click();
    }
    expect(transport.writes().isEmpty(),
           "control transport is quiet before Vision connect");

    connectVideo->click();
    for (int i = 0; i < 20 && !server.hasPendingConnections(); ++i) {
        waitForMs(10);
    }
    expect(server.hasPendingConnections(),
           "Vision Connect reaches dedicated Vision server");
    QTcpSocket *peer = server.nextPendingConnection();

    for (int i = 0; i < 20
         && !hasLabelText(panel, QStringLiteral("Connected")); ++i) {
        waitForMs(10);
    }
    expect(hasLabelText(panel, QStringLiteral("Connected")),
           "Vision card reports its own connected state");
    expect(transport.writes().isEmpty(),
           "Vision connect must not emit any robot-control transport bytes");

    window.close();
    if (peer != nullptr) {
        peer->deleteLater();
    }
}

void testVisionCaptureControlsAreIndependentFromRobotTransport()
{
    rb::FakeTransport transport;
    rb::RobotController controller(
        &transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::vision::VisionClient visionClient;
    rb::vision::VisionControlClient controlClient;

    QTcpServer controlServer;
    expect(controlServer.listen(QHostAddress::LocalHost, 0),
           "Vision capture fake HTTP server must listen");
    if (!controlServer.isListening()) {
        return;
    }
    controlClient.setEndpoint(
        QStringLiteral("127.0.0.1"),
        controlServer.serverPort());

    rb::MainWindow window(&controller, &visionClient, &controlClient);
    QGroupBox *panel = findGroupBox(&window, QStringLiteral("Realtime Video"));
    expect(panel != nullptr, "capture integration retains Realtime Video card");
    if (panel == nullptr) {
        return;
    }

    auto *host = window.findChild<QLineEdit *>(QStringLiteral("piHost"));
    auto *port = window.findChild<QSpinBox *>(QStringLiteral("visionPort"));
    auto *snapshot =
        panel->findChild<QPushButton *>(QStringLiteral("snapshotButton"));
    auto *startRecording =
        panel->findChild<QPushButton *>(QStringLiteral("startRecordingButton"));
    auto *stopRecording =
        panel->findChild<QPushButton *>(QStringLiteral("stopRecordingButton"));
    auto *captureDiagnostics =
        panel->findChild<QLabel *>(QStringLiteral("captureDiagnostics"));
    QPushButton *connectVideo =
        buttonWithText(panel, QStringLiteral("Connect Video"));
    expect(host != nullptr && port != nullptr && snapshot != nullptr
               && startRecording != nullptr && stopRecording != nullptr
               && captureDiagnostics != nullptr && connectVideo != nullptr,
           "capture UI exposes the frozen controls");
    if (host == nullptr || port == nullptr || snapshot == nullptr
        || startRecording == nullptr || stopRecording == nullptr
        || captureDiagnostics == nullptr || connectVideo == nullptr) {
        return;
    }

    expect(!snapshot->isEnabled() && !startRecording->isEnabled()
               && !stopRecording->isEnabled(),
           "capture actions start disabled before Vision connection");

    QTcpServer videoServer;
    expect(videoServer.listen(QHostAddress::LocalHost, 0),
           "capture integration fake RBVS server must listen");
    if (!videoServer.isListening()) {
        return;
    }

    host->setText(QStringLiteral("127.0.0.1"));
    port->setValue(videoServer.serverPort());
    connectVideo->click();

    for (int i = 0; i < 50 && !videoServer.hasPendingConnections(); ++i) {
        waitForMs(5);
    }
    expect(videoServer.hasPendingConnections(),
           "video connection reaches independent RBVS endpoint");
    QTcpSocket *videoPeer = videoServer.nextPendingConnection();

    for (int i = 0; i < 50 && !controlServer.hasPendingConnections(); ++i) {
        waitForMs(5);
    }
    expect(controlServer.hasPendingConnections(),
           "video connection starts independent capture status polling");
    QTcpSocket *statusPeer = controlServer.nextPendingConnection();
    if (statusPeer != nullptr) {
        const QByteArray request = readHttpRequest(statusPeer);
        expect(
            request.startsWith("GET /api/v1/vision/status HTTP/1.1"),
            "capture status uses HTTP control plane, not RBVS or RBRP");
        sendHttpJson(
            statusPeer,
            QByteArrayLiteral(
                "{\"ok\":true,"
                "\"camera\":{\"running\":true,\"latest_frame_id\":12},"
                "\"capture\":{\"state\":\"idle\",\"recording\":false,"
                "\"session_id\":null,\"segment\":null,"
                "\"recorded_frames\":0,\"snapshot_count\":0,"
                "\"queue_bytes\":0,\"max_queue_bytes\":67108864,"
                "\"last_error\":null}}"));
    }

    for (int i = 0; i < 50 && !snapshot->isEnabled(); ++i) {
        waitForMs(5);
    }
    expect(snapshot->isEnabled() && startRecording->isEnabled()
               && !stopRecording->isEnabled(),
           "capture status enables idle snapshot/start actions");
    expect(transport.writes().isEmpty(),
           "capture status polling emits no robot-control bytes");

    snapshot->click();
    for (int i = 0; i < 50 && !controlServer.hasPendingConnections(); ++i) {
        waitForMs(5);
    }
    expect(controlServer.hasPendingConnections(),
           "Snapshot button reaches capture HTTP endpoint");
    QTcpSocket *snapshotPeer = controlServer.nextPendingConnection();
    if (snapshotPeer != nullptr) {
        const QByteArray request = readHttpRequest(snapshotPeer);
        expect(
            request.startsWith("POST /api/v1/vision/snapshot HTTP/1.1"),
            "Snapshot button uses the dedicated capture action path");
        sendHttpJson(
            snapshotPeer,
            QByteArrayLiteral(
                "{\"ok\":true,\"capture\":{\"state\":\"idle\","
                "\"recording\":false,\"session_id\":\"capture-test-001\","
                "\"segment\":null,\"recorded_frames\":0,"
                "\"snapshot_count\":1,\"queue_bytes\":0,"
                "\"max_queue_bytes\":67108864,\"last_error\":null}}"));
    }

    for (int i = 0; i < 50
         && !captureDiagnostics->text().contains(QStringLiteral("Snapshots 1"));
         ++i) {
        waitForMs(5);
    }
    expect(captureDiagnostics->text().contains(QStringLiteral("Snapshots 1")),
           "capture action response updates UI diagnostics");
    expect(transport.writes().isEmpty(),
           "Snapshot action emits no robot-control bytes");

    // Keep this integration test deterministic; action responses themselves
    // still update status, so periodic polling is not needed below.
    controlClient.stopPolling();

    startRecording->click();
    for (int i = 0; i < 50 && !controlServer.hasPendingConnections(); ++i) {
        waitForMs(5);
    }
    expect(controlServer.hasPendingConnections(),
           "Start Recording reaches capture HTTP endpoint");
    QTcpSocket *startPeer = controlServer.nextPendingConnection();
    if (startPeer != nullptr) {
        const QByteArray request = readHttpRequest(startPeer);
        expect(
            request.startsWith(
                "POST /api/v1/vision/recording/start HTTP/1.1"),
            "Start Recording uses dedicated capture action path");
        sendHttpJson(
            startPeer,
            QByteArrayLiteral(
                "{\"ok\":true,\"capture\":{\"state\":\"recording\","
                "\"recording\":true,\"session_id\":\"capture-test-001\","
                "\"segment\":\"raw.avi\",\"recorded_frames\":5,"
                "\"snapshot_count\":1,\"queue_bytes\":0,"
                "\"max_queue_bytes\":67108864,"
                "\"free_disk_bytes\":1073741824,"
                "\"last_error\":null}}"));
    }
    for (int i = 0; i < 50 && !stopRecording->isEnabled(); ++i) {
        waitForMs(5);
    }
    expect(stopRecording->isEnabled() && !startRecording->isEnabled(),
           "recording status enables Stop and disables Start");

    // Drop RBVS only. Capture control must remain usable so a recording can
    // always be stopped even when the realtime video path fails.
    connectVideo->click();
    for (int i = 0; i < 50
         && !hasLabelText(panel, QStringLiteral("Disconnected")); ++i) {
        waitForMs(5);
    }
    expect(stopRecording->isEnabled(),
           "Stop Recording remains enabled after RBVS disconnect");
    expect(transport.writes().isEmpty(),
           "RBVS disconnect while recording emits no robot-control bytes");

    stopRecording->click();
    for (int i = 0; i < 50 && !controlServer.hasPendingConnections(); ++i) {
        waitForMs(5);
    }
    expect(controlServer.hasPendingConnections(),
           "Stop Recording remains reachable without RBVS");
    QTcpSocket *stopPeer = controlServer.nextPendingConnection();
    if (stopPeer != nullptr) {
        const QByteArray request = readHttpRequest(stopPeer);
        expect(
            request.startsWith(
                "POST /api/v1/vision/recording/stop HTTP/1.1"),
            "Stop Recording uses dedicated capture action path");
        sendHttpJson(
            stopPeer,
            QByteArrayLiteral(
                "{\"ok\":true,\"capture\":{\"state\":\"idle\","
                "\"recording\":false,\"session_id\":\"capture-test-001\","
                "\"segment\":null,\"recorded_frames\":8,"
                "\"snapshot_count\":1,\"queue_bytes\":0,"
                "\"max_queue_bytes\":67108864,"
                "\"free_disk_bytes\":1073741824,"
                "\"last_error\":null}}"));
    }
    for (int i = 0; i < 50 && stopRecording->isEnabled(); ++i) {
        waitForMs(5);
    }
    for (int i = 0; i < 50 && controlClient.actionBusy(); ++i) {
        waitForMs(5);
    }
    expect(!stopRecording->isEnabled() && startRecording->isEnabled(),
           "Stop response returns capture UI to idle");
    expect(transport.writes().isEmpty(),
           "capture start/stop emits no robot-control bytes");

    window.close();
    if (statusPeer != nullptr) {
        statusPeer->deleteLater();
    }
    if (snapshotPeer != nullptr) {
        snapshotPeer->deleteLater();
    }
    if (startPeer != nullptr) {
        startPeer->deleteLater();
    }
    if (stopPeer != nullptr) {
        stopPeer->deleteLater();
    }
    if (videoPeer != nullptr) {
        videoPeer->deleteLater();
    }
}

void testVisionCaptureActionDefersWindowClose()
{
    rb::FakeTransport transport;
    rb::RobotController controller(
        &transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::vision::VisionClient visionClient;
    rb::vision::VisionControlClient controlClient;

    QTcpServer controlServer;
    expect(controlServer.listen(QHostAddress::LocalHost, 0),
           "close-guard capture server must listen");
    if (!controlServer.isListening()) {
        return;
    }
    controlClient.setEndpoint(
        QStringLiteral("127.0.0.1"),
        controlServer.serverPort());

    rb::MainWindow window(&controller, &visionClient, &controlClient);
    window.show();
    waitForMs(20);

    controlClient.startRecording();
    for (int i = 0; i < 50 && !controlServer.hasPendingConnections(); ++i) {
        waitForMs(5);
    }
    expect(controlServer.hasPendingConnections(),
           "close-guard Start Recording reaches capture endpoint");
    QTcpSocket *peer = controlServer.nextPendingConnection();
    if (peer == nullptr) {
        return;
    }
    const QByteArray request = readHttpRequest(peer);
    expect(
        request.startsWith(
            "POST /api/v1/vision/recording/start HTTP/1.1"),
        "close-guard request is recording/start");
    expect(controlClient.actionInFlight(),
           "MainWindow sees capture action in flight");

    expect(!window.close(),
           "MainWindow close is deferred while capture action is unresolved");
    expect(window.isVisible(),
           "deferred close leaves the application window open");
    expect(controlClient.actionInFlight(),
           "deferred close does not abort the active capture action");

    sendHttpJson(
        peer,
        QByteArrayLiteral(
            "{\"ok\":true,\"capture\":{\"state\":\"recording\","
            "\"recording\":true,\"recorded_frames\":0,"
            "\"snapshot_count\":0,\"queue_bytes\":0,"
            "\"max_queue_bytes\":67108864,"
            "\"free_disk_bytes\":1073741824,\"last_error\":null}}"));
    for (int i = 0; i < 50 && controlClient.actionInFlight(); ++i) {
        waitForMs(5);
    }
    for (int i = 0; i < 50 && controlClient.actionBusy(); ++i) {
        waitForMs(5);
    }
    expect(!controlClient.actionInFlight(),
           "capture action obtains a definite result before close");

    answerNextMessageBox(QMessageBox::Yes);
    expect(window.close(),
           "MainWindow closes after the capture action completes");

    peer->deleteLater();
}

void testVisionInferenceDiagnosticsAreRenderedWithoutRobotWrites()
{
    const QString fullSha = QStringLiteral(
        "3dea74511bf2aabbccddeeff00112233445566778899aabbccddeeff00112233");

    rb::FakeTransport transport;
    rb::RobotController controller(
        &transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::vision::VisionClient visionClient;
    rb::vision::VisionControlClient controlClient;

    QTcpServer controlServer;
    QTcpServer videoServer;
    expect(controlServer.listen(QHostAddress::LocalHost, 0),
           "inference diagnostics control server must listen");
    expect(videoServer.listen(QHostAddress::LocalHost, 0),
           "inference diagnostics video server must listen");
    if (!controlServer.isListening() || !videoServer.isListening()) {
        return;
    }
    controlClient.setEndpoint(
        QStringLiteral("127.0.0.1"), controlServer.serverPort());

    rb::MainWindow window(&controller, &visionClient, &controlClient);
    QGroupBox *panel = findGroupBox(&window, QStringLiteral("Realtime Video"));
    expect(panel != nullptr, "inference diagnostics retain the Realtime Video card");
    if (panel == nullptr) {
        return;
    }

    auto *host = window.findChild<QLineEdit *>(QStringLiteral("piHost"));
    auto *port = window.findChild<QSpinBox *>(QStringLiteral("visionPort"));
    auto *state = panel->findChild<QLabel *>(QStringLiteral("inferenceState"));
    auto *diagnostics =
        panel->findChild<QLabel *>(QStringLiteral("inferenceDiagnostics"));
    QPushButton *connectVideo =
        buttonWithText(panel, QStringLiteral("Connect Video"));
    expect(host != nullptr && port != nullptr && state != nullptr
               && diagnostics != nullptr && connectVideo != nullptr,
           "Realtime Video card exposes inference state and diagnostics labels");
    if (host == nullptr || port == nullptr || state == nullptr
        || diagnostics == nullptr || connectVideo == nullptr) {
        return;
    }

    expect(state->text() == QStringLiteral("Inference Unknown"),
           "inference state starts unknown before status reception");

    host->setText(QStringLiteral("127.0.0.1"));
    port->setValue(videoServer.serverPort());
    connectVideo->click();
    for (int i = 0; i < 50 && !controlServer.hasPendingConnections(); ++i) {
        waitForMs(5);
    }
    expect(controlServer.hasPendingConnections(),
           "inference diagnostics use the existing status polling endpoint");
    QTcpSocket *statusPeer = controlServer.nextPendingConnection();
    QTcpSocket *videoPeer = nullptr;
    for (int i = 0; i < 50 && !videoServer.hasPendingConnections(); ++i) {
        waitForMs(5);
    }
    if (videoServer.hasPendingConnections()) {
        videoPeer = videoServer.nextPendingConnection();
    }
    if (statusPeer == nullptr) {
        return;
    }

    expect(readHttpRequest(statusPeer).startsWith(
               "GET /api/v1/vision/status HTTP/1.1"),
           "inference diagnostics use the existing GET status request");
    sendHttpJson(
        statusPeer,
        QByteArrayLiteral(
            "{\"ok\":true,\"camera\":{\"running\":true,"
            "\"latest_frame_id\":12},\"capture\":{\"state\":\"idle\","
            "\"recording\":false,\"last_error\":null},"
            "\"inference\":{\"state\":\"running\","
            "\"artifact_name\":\"lab_pool_d2_seed42_e20.onnx\","
            "\"model_sha256\":\"3dea74511bf2aabbccddeeff00112233445566778899aabbccddeeff00112233\",\"latest_frame_id\":0,"
            "\"skipped_frames\":0,\"inference_fps\":0.0,"
            "\"latency_ms\":0.0,\"detection_count\":0}}"));
    for (int i = 0; i < 50 && state->text() != QStringLiteral("Inference Running");
         ++i) {
        waitForMs(5);
    }
    expect(state->text() == QStringLiteral("Inference Running"),
           "running inference status is rendered in the state label");
    const QString shaLabel = QStringLiteral("SHA-256: ");
    const int shaStart = diagnostics->text().indexOf(shaLabel);
    const int shaEnd = diagnostics->text().indexOf(QStringLiteral("\nFPS:"), shaStart);
    const QString displayedSha =
        shaStart >= 0 && shaEnd > shaStart
        ? diagnostics->text().mid(
              shaStart + shaLabel.size(), shaEnd - shaStart - shaLabel.size())
        : QString();
    expect(controlClient.status().inferenceModelSha256 == fullSha
               && controlClient.status().inferenceModelSha256.size() == 64,
           "parsed inference status retains the complete model SHA-256");
    expect(diagnostics->text().contains(QStringLiteral("Artifact: lab_pool_d2_seed42_e20.onnx"))
               && displayedSha == fullSha.left(12)
               && !diagnostics->text().contains(fullSha)
               && diagnostics->text().contains(QStringLiteral("FPS: 0.0"))
               && diagnostics->text().contains(QStringLiteral("Latency ms: 0.0"))
               && diagnostics->text().contains(QStringLiteral("Latest Frame ID: 0"))
               && diagnostics->text().contains(QStringLiteral("Detections: 0"))
               && diagnostics->text().contains(QStringLiteral("Skipped: 0")),
           "running inference diagnostics preserve validated identity and zero values");
    expect(transport.writes().isEmpty(),
           "inference status updates emit no robot-control transport writes");

    controlClient.stopPolling();
    statusPeer->deleteLater();

    const auto applyInferenceStatus = [&](const QByteArray &inference) {
        controlClient.refreshStatus();
        for (int i = 0; i < 50 && !controlServer.hasPendingConnections(); ++i) {
            waitForMs(5);
        }
        expect(controlServer.hasPendingConnections(),
               "inference state changes use the existing status request");
        QTcpSocket *peer = controlServer.nextPendingConnection();
        if (peer == nullptr) {
            return;
        }
        readHttpRequest(peer);
        sendHttpJson(
            peer,
            QByteArrayLiteral("{\"ok\":true,\"inference\":")
                + inference + QByteArrayLiteral("}"));
        for (int i = 0; i < 50 && controlClient.requestInFlight(); ++i) {
            waitForMs(5);
        }
        peer->deleteLater();
    };

    applyInferenceStatus(QByteArrayLiteral("{\"state\":\"starting\"}"));
    expect(state->text() == QStringLiteral("Inference Starting"),
           "starting inference status is rendered in the state label");

    applyInferenceStatus(
        QByteArrayLiteral(
            "{\"state\":\"failed\",\"last_error\":\"model load failed\"}"));
    expect(state->text() == QStringLiteral("Inference Error")
               && diagnostics->text().contains(
                   QStringLiteral("Last error: model load failed")),
           "failed inference status renders a short last error");

    applyInferenceStatus(QByteArrayLiteral("{\"state\":\"unsupported\"}"));
    expect(state->text() == QStringLiteral("Inference Unavailable")
               && diagnostics->text().contains(QStringLiteral("FPS: --"))
               && diagnostics->text().contains(QStringLiteral("Latest Frame ID: --"))
               && diagnostics->text().contains(QStringLiteral("Detections: --"))
               && diagnostics->text().contains(QStringLiteral("Skipped: --")),
           "unsupported inference status renders unavailable diagnostics");
    expect(transport.writes().isEmpty(),
           "inference state changes emit no robot-control transport writes");

    if (videoPeer != nullptr) {
        videoPeer->deleteLater();
    }
    window.close();
}

void testTask02SharedHostWidgetsAndHttpOnlyControls()
{
    rb::FakeTransport transport;
    rb::RobotController controller(
        &transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::vision::VisionControlClient controlClient;
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "Task 02 HTTP-only server must listen");
    controlClient.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());

    rb::MainWindow window(&controller, nullptr, &controlClient);
    auto *host = window.findChild<QLineEdit *>(QStringLiteral("piHost"));
    auto *apply = window.findChild<QPushButton *>(QStringLiteral("applyPiHostButton"));
    auto *refresh = window.findChild<QPushButton *>(QStringLiteral("visionRefreshStatusButton"));
    auto *visionPort = window.findChild<QSpinBox *>(QStringLiteral("visionPort"));
    auto *start = window.findChild<QPushButton *>(QStringLiteral("startInferenceButton"));
    auto *stop = window.findChild<QPushButton *>(QStringLiteral("stopInferenceButton"));
    auto *connectRobot = window.findChild<QPushButton *>(QStringLiteral("connectRobotButton"));
    expect(host != nullptr && apply != nullptr && refresh != nullptr
               && visionPort != nullptr && start != nullptr && stop != nullptr
               && connectRobot != nullptr,
           "Task 02 exposes one shared Host and HTTP/inference controls");
    expect(refresh != nullptr
               && refresh->toolTip().contains(QString::number(server.serverPort())),
           "Refresh Status identifies the injected nonstandard HTTP endpoint");
    window.resize(1420, 880);
    window.show();
    waitForMs(10);
    expect(host != nullptr && host->isVisible()
               && refresh != nullptr && refresh->isVisible()
               && start != nullptr && start->isVisible()
               && stop != nullptr && stop->isVisible(),
           "Task 02 Host, Refresh, and inference buttons are reachable at normal size");
    window.hide();
    expect(window.findChild<QLineEdit *>(QStringLiteral("visionHost")) == nullptr,
           "Task 02 removes the per-card visionHost editor");
    if (host != nullptr && apply != nullptr) {
        expect(host->text() == QStringLiteral("127.0.0.1"),
               "injected control endpoint seeds the committed shared Host");
        host->setText(QStringLiteral(" 127.0.0.2 "));
        expect(apply->isEnabled(), "Apply Host enables for a changed candidate");
        apply->click();
        expect(host->text() == QStringLiteral("127.0.0.2"),
               "Apply Host commits the normalized candidate");
         host->setText(QStringLiteral("127.0.0.2 other"));
         expect(!refresh->isEnabled() && !start->isEnabled() && !connectRobot->isEnabled(),
                "dirty Host disables new monitoring/inference actions");
        host->setFocus();
        QKeyEvent escape(QEvent::KeyPress, Qt::Key_Escape, Qt::NoModifier);
        QApplication::sendEvent(host, &escape);
        expect(host->text() == QStringLiteral("127.0.0.2"),
               "Escape restores the committed Host");
        for (const QString &invalid : {QStringLiteral("http://127.0.0.3"),
                                       QStringLiteral("127.0.0.3:47011"),
                                       QStringLiteral("bad host")}) {
            host->setText(invalid);
            expect(!apply->isEnabled(),
                   "invalid Host syntax cannot be applied or retarget an endpoint");
        }
        host->setText(QStringLiteral("LOCALHOST"));
        expect(apply->isEnabled(), "valid hostname candidate enables Apply Host");
        apply->click();
        expect(host->text() == QStringLiteral("localhost"),
               "hostname commit normalizes case consistently");
    }
    expect(visionPort->buttonSymbols() == QAbstractSpinBox::NoButtons,
           "HTTP/video port uses a compact no-arrow spin box");
    expect(transport.writes().isEmpty(),
           "constructing HTTP-only controls emits no robot-control writes");
    window.close();
}

void testTask02RefreshAndInferenceUseOnlyCommittedHost()
{
    rb::FakeTransport transport;
    rb::RobotController controller(
        &transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::vision::VisionControlClient controlClient;
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "Task 02 refresh server must listen");
    controlClient.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    rb::MainWindow window(&controller, nullptr, &controlClient);
    auto *refresh = window.findChild<QPushButton *>(QStringLiteral("visionRefreshStatusButton"));
    auto *start = window.findChild<QPushButton *>(QStringLiteral("startInferenceButton"));
    auto *state = window.findChild<QLabel *>(QStringLiteral("inferenceState"));
    auto *controlState = window.findChild<QLabel *>(QStringLiteral("visionControlState"));
    expect(refresh != nullptr && start != nullptr && state != nullptr && controlState != nullptr,
           "HTTP-only refresh and inference widgets exist without VisionClient");
    if (refresh == nullptr || start == nullptr) {
        return;
    }
    refresh->click();
    expect(waitUntil([&server] { return server.hasPendingConnections(); }),
           "Refresh Status reaches the HTTP control endpoint without Robot/RBVS");
    QTcpSocket *statusPeer = server.nextPendingConnection();
    if (statusPeer != nullptr) {
        expect(readHttpRequest(statusPeer).startsWith(
                   "GET /api/v1/vision/status HTTP/1.1"),
               "Refresh Status uses the authoritative GET status path");
        sendHttpJson(
            statusPeer,
            QByteArrayLiteral(
                "{\"ok\":true,\"camera\":{\"running\":true},"
                "\"capture\":{\"state\":\"idle\",\"recording\":false},"
                "\"inference\":{\"configured\":true,"
                "\"control_supported\":true,\"operation\":null,"
                "\"state\":\"disabled\"}}"));
    }
    expect(waitUntil([&controlClient] { return controlClient.hasFreshStatus(); }),
           "HTTP-only status becomes fresh");
    expect(start->isEnabled(), "confirmed disabled inference enables Start");
    start->click();
    expect(waitUntil([&server] { return server.hasPendingConnections(); }),
           "Start Inference reaches HTTP without a Robot connection");
    QTcpSocket *actionPeer = server.nextPendingConnection();
    if (actionPeer != nullptr) {
        expect(readHttpRequest(actionPeer).startsWith(
                   "POST /api/v1/vision/inference/start HTTP/1.1"),
               "Start Inference uses the frozen endpoint");
        sendHttpJson(
            actionPeer,
            QByteArrayLiteral(
                "{\"ok\":true,\"action\":\"inference/start\","
                "\"outcome\":\"accepted\"}"));
    }
    QTcpSocket *refreshPeer = nullptr;
    expect(waitUntil([&server] { return server.hasPendingConnections(); }),
           "Start ACK schedules one authoritative reconciliation GET");
    if (server.hasPendingConnections()) {
        refreshPeer = server.nextPendingConnection();
        readHttpRequest(refreshPeer);
        sendHttpJson(
            refreshPeer,
            QByteArrayLiteral(
                "{\"ok\":true,\"camera\":{\"running\":true},"
                "\"capture\":{\"state\":\"idle\",\"recording\":false},"
                "\"inference\":{\"configured\":true,"
                "\"control_supported\":true,\"operation\":null,"
                "\"state\":\"running\"}}"));
    }
    expect(waitUntil([&] { return state->text() == QStringLiteral("Inference Running"); }),
           "UI state is updated only by reconciled GET status");
    expect(controlState->text().contains(QStringLiteral("Reachable")),
           "vision control reachability is separate from capture state");
    auto *host = window.findChild<QLineEdit *>(QStringLiteral("piHost"));
    auto *stop = window.findChild<QPushButton *>(QStringLiteral("stopInferenceButton"));
    expect(host != nullptr && stop != nullptr,
           "running HTTP-only status exposes committed-host Stop access");
    if (host != nullptr && stop != nullptr) {
        host->setText(QStringLiteral("127.0.0.2"));
        expect(!start->isEnabled() && !refresh->isEnabled() && stop->isEnabled(),
               "dirty Host disables new actions but keeps committed Stop available");
        host->setText(QStringLiteral("127.0.0.1"));
    }
    expect(transport.writes().isEmpty(),
           "HTTP-only inference controls emit no Robot writes");
    answerNextMessageBox(QMessageBox::Yes);
    window.close();
    if (statusPeer != nullptr) statusPeer->deleteLater();
    if (actionPeer != nullptr) actionPeer->deleteLater();
    if (refreshPeer != nullptr) refreshPeer->deleteLater();
}

void testTask02CaptureAndInferenceErrorsStaySeparated()
{
    rb::FakeTransport transport;
    rb::RobotController controller(
        &transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::vision::VisionControlClient controlClient;
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "Task 02 error server must listen");
    controlClient.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    rb::MainWindow window(&controller, nullptr, &controlClient);
    auto *refresh = window.findChild<QPushButton *>(QStringLiteral("visionRefreshStatusButton"));
    auto *message = window.findChild<QLabel *>(QStringLiteral("visionControlMessage"));
    auto *captureState = window.findChild<QLabel *>(QStringLiteral("captureState"));
    expect(refresh != nullptr && message != nullptr && captureState != nullptr,
           "Task 02 separates control message from Capture state");
    if (refresh == nullptr) {
        return;
    }
    refresh->click();
    expect(waitUntil([&server] { return server.hasPendingConnections(); }),
           "error test refresh connects");
    QTcpSocket *peer = server.nextPendingConnection();
    if (peer != nullptr) {
        readHttpRequest(peer);
        sendHttpJson(
            peer,
            QByteArrayLiteral(
                "{\"ok\":true,\"camera\":{\"running\":true},"
                "\"capture\":{\"state\":\"idle\",\"recording\":false},"
                "\"inference\":{\"configured\":true,"
                "\"control_supported\":true,\"operation\":null,"
                "\"state\":\"disabled\"}}"));
    }
    expect(waitUntil([&controlClient] { return controlClient.hasFreshStatus(); }),
           "error test status is fresh");
    auto *start = window.findChild<QPushButton *>(QStringLiteral("startInferenceButton"));
    if (start != nullptr) {
        start->click();
    }
    expect(waitUntil([&server] { return server.hasPendingConnections(); }),
           "error test Start connects");
    QTcpSocket *action = server.nextPendingConnection();
    if (action != nullptr) {
        readHttpRequest(action);
        sendHttpJson(action,
                     QByteArrayLiteral(
                         "{\"ok\":false,\"error\":\"inference_busy\","
                         "\"message\":\"worker is stopping\"}"));
    }
    expect(waitUntil([message] {
        return message != nullptr
            && message->text().contains(QStringLiteral("worker is stopping"));
    }), "typed inference error is shown in the Vision control message");
    expect(captureState->text() != QStringLiteral("Capture Error"),
           "inference error does not become Capture Error");
    answerNextMessageBox(QMessageBox::Yes);
    window.close();
    if (peer != nullptr) peer->deleteLater();
    if (action != nullptr) action->deleteLater();
}

void testTask02ApplyHostBlocksQueuedMutation()
{
    rb::FakeTransport transport;
    rb::RobotController controller(
        &transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::vision::VisionControlClient controlClient;
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "queued-host server must listen");
    controlClient.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    rb::MainWindow window(&controller, nullptr, &controlClient);
    auto *refresh = window.findChild<QPushButton *>(QStringLiteral("visionRefreshStatusButton"));
    auto *apply = window.findChild<QPushButton *>(QStringLiteral("applyPiHostButton"));
    auto *host = window.findChild<QLineEdit *>(QStringLiteral("piHost"));
    expect(refresh != nullptr && apply != nullptr && host != nullptr,
           "queued-host test exposes Apply and committed Host widgets");
    if (refresh == nullptr || apply == nullptr || host == nullptr) {
        return;
    }

    refresh->click();
    expect(waitUntil([&server] { return server.hasPendingConnections(); }),
           "queued-host initial refresh connects");
    QTcpSocket *initial = server.nextPendingConnection();
    if (initial != nullptr) {
        readHttpRequest(initial);
        sendHttpJson(initial,
                     QByteArrayLiteral(
                         "{\"ok\":true,\"camera\":{\"running\":true},"
                         "\"capture\":{\"state\":\"idle\",\"recording\":false},"
                         "\"inference\":{\"configured\":true,"
                         "\"control_supported\":true,\"operation\":null,"
                         "\"state\":\"disabled\"}}"));
    }
    expect(waitUntil([&controlClient] { return controlClient.hasFreshStatus(); }),
           "queued-host initial status is fresh");

    controlClient.refreshStatus();
    expect(waitUntil([&server] { return server.hasPendingConnections(); }),
           "queued-host second GET is active");
    QTcpSocket *pendingStatus = server.nextPendingConnection();
    if (pendingStatus != nullptr) {
        readHttpRequest(pendingStatus);
    }
    controlClient.startInference();
    expect(controlClient.actionBusy(),
           "inference mutation is queued behind the active GET");

    host->setText(QStringLiteral("127.0.0.2"));
    expect(!apply->isEnabled() && controlClient.host() == QStringLiteral("127.0.0.1"),
           "UI Apply Host blocks queued mutation without retargeting the client");

    if (pendingStatus != nullptr) {
        sendHttpJson(pendingStatus,
                     QByteArrayLiteral(
                         "{\"ok\":true,\"camera\":{\"running\":true},"
                         "\"capture\":{\"state\":\"idle\",\"recording\":false},"
                         "\"inference\":{\"configured\":true,"
                         "\"control_supported\":true,\"operation\":null,"
                         "\"state\":\"disabled\"}}"));
    }
    expect(waitUntil([&server] { return server.hasPendingConnections(); }),
           "queued-host mutation dispatches only after GET completion");
    QTcpSocket *action = server.nextPendingConnection();
    if (action != nullptr) {
        expect(readHttpRequest(action).startsWith(
                   "POST /api/v1/vision/inference/start HTTP/1.1"),
               "queued-host mutation retains its original committed endpoint");
        sendHttpJson(action,
                     QByteArrayLiteral(
                         "{\"ok\":true,\"action\":\"inference/start\","
                         "\"outcome\":\"accepted\"}"));
    }
    expect(waitUntil([&server] { return server.hasPendingConnections(); }),
           "queued-host mutation schedules reconciliation GET");
    QTcpSocket *reconcile = server.nextPendingConnection();
    if (reconcile != nullptr) {
        readHttpRequest(reconcile);
        sendHttpJson(reconcile,
                     QByteArrayLiteral(
                         "{\"ok\":true,\"camera\":{\"running\":true},"
                         "\"capture\":{\"state\":\"idle\",\"recording\":false},"
                         "\"inference\":{\"configured\":true,"
                         "\"control_supported\":true,\"operation\":null,"
                         "\"state\":\"running\"}}"));
    }
    expect(waitUntil([&controlClient] { return !controlClient.actionBusy(); }),
           "queued-host action retires after reconciliation");
    expect(controlClient.host() == QStringLiteral("127.0.0.1"),
           "queued-host completion never changes the committed endpoint");
    answerNextMessageBox(QMessageBox::Yes);
    window.close();
    for (QTcpSocket *peer : {initial, pendingStatus, action, reconcile}) {
        if (peer != nullptr) {
            peer->deleteLater();
        }
    }
}

void testTask02CloseNeverImplicitlyStopsRemoteWork()
{
    rb::FakeTransport transport;
    rb::RobotController controller(
        &transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::vision::VisionControlClient controlClient;
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "Task 02 close server must listen");
    controlClient.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    rb::MainWindow window(&controller, nullptr, &controlClient);
    auto *refresh = window.findChild<QPushButton *>(QStringLiteral("visionRefreshStatusButton"));
    if (refresh != nullptr) {
        refresh->click();
        expect(waitUntil([&server] { return server.hasPendingConnections(); }),
               "close test refresh connects");
        QTcpSocket *peer = server.nextPendingConnection();
        if (peer != nullptr) {
            readHttpRequest(peer);
            sendHttpJson(peer,
                         QByteArrayLiteral(
                             "{\"ok\":true,\"camera\":{\"running\":true},"
                             "\"capture\":{\"state\":\"recording\",\"recording\":true},"
                             "\"inference\":{\"configured\":true,"
                             "\"control_supported\":true,\"operation\":null,"
                             "\"state\":\"running\"}}"));
            peer->deleteLater();
        }
        waitUntil([&controlClient] { return controlClient.hasFreshStatus(); });
    }
    answerNextMessageBox(QMessageBox::Yes);
    window.close();
    expect(!server.hasPendingConnections(),
           "closing does not create an implicit inference/stop or recording/stop POST");
    expect(transport.writes().isEmpty(),
           "closing Vision work does not create Robot-control writes");
}

void testTask02CloseWarnsForRemoteVisionState(const QByteArray &statusBody)
{
    rb::FakeTransport transport;
    rb::RobotController controller(
        &transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::vision::VisionControlClient controlClient;
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "remote-work close warning server must listen");
    controlClient.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    rb::MainWindow window(&controller, nullptr, &controlClient);
    auto *refresh = window.findChild<QPushButton *>(QStringLiteral("visionRefreshStatusButton"));
    expect(refresh != nullptr, "remote-work close warning exposes Refresh Status");
    if (refresh == nullptr) {
        return;
    }
    refresh->click();
    expect(waitUntil([&server] { return server.hasPendingConnections(); }),
           "remote-work close warning refresh connects");
    QTcpSocket *peer = server.nextPendingConnection();
    if (peer != nullptr) {
        readHttpRequest(peer);
        sendHttpJson(peer, statusBody);
        peer->deleteLater();
    }
    expect(waitUntil([&controlClient] { return controlClient.hasFreshStatus(); }),
           "remote-work close warning status becomes fresh");

    const auto messageBoxSeen = answerNextMessageBoxAndTrack(QMessageBox::Yes);
    const bool closed = window.close();
    waitForMs(80);
    expect(closed, "close accepts after the user confirms remote-work warning");
    expect(*messageBoxSeen,
           "close warns when authoritative remote Vision work may continue");
    expect(!server.hasPendingConnections(),
           "remote-work warning close emits no inference/recording Stop POST");
    expect(transport.writes().isEmpty(),
           "remote-work warning close emits no Robot-control writes");
}

void testTask02CloseWarnsForStoppingAndRetryingRemoteWork()
{
    testTask02CloseWarnsForRemoteVisionState(QByteArrayLiteral(
        "{\"ok\":true,\"camera\":{\"running\":true},"
        "\"capture\":{\"state\":\"idle\",\"recording\":false},"
        "\"inference\":{\"configured\":true,\"control_supported\":true,"
        "\"operation\":\"stopping\",\"state\":\"disabled\"}}"));
    testTask02CloseWarnsForRemoteVisionState(QByteArrayLiteral(
        "{\"ok\":true,\"camera\":{\"running\":true},"
        "\"capture\":{\"state\":\"idle\",\"recording\":false},"
        "\"inference\":{\"configured\":true,\"control_supported\":true,"
        "\"operation\":\"retrying\",\"state\":\"disabled\"}}"));
    testTask02CloseWarnsForRemoteVisionState(QByteArrayLiteral(
        "{\"ok\":true,\"camera\":{\"running\":true},"
        "\"capture\":{\"state\":\"stopping\",\"recording\":false},"
        "\"inference\":{\"configured\":true,\"control_supported\":true,"
        "\"operation\":null,\"state\":\"disabled\"}}"));
}

void seedAmbiguousInferenceReconciliation(
    rb::vision::VisionControlClient &controlClient,
    QTcpServer &server,
    bool successful)
{
    controlClient.refreshStatus();
    expect(waitUntil([&server] { return server.hasPendingConnections(); }),
           "ambiguous MainWindow setup initial GET connects");
    QTcpSocket *initial = server.nextPendingConnection();
    if (initial != nullptr) {
        readHttpRequest(initial);
        sendHttpJson(
            initial,
            QByteArrayLiteral(
                "{\"ok\":true,\"camera\":{\"running\":true},"
                "\"capture\":{\"state\":\"idle\",\"recording\":false},"
                "\"inference\":{\"configured\":true,"
                "\"control_supported\":true,\"operation\":null,"
                "\"state\":\"disabled\"}}"));
        initial->deleteLater();
    }
    expect(waitUntil([&controlClient] { return controlClient.hasFreshStatus(); }),
           "ambiguous MainWindow setup starts from fresh status");

    controlClient.startInference();
    expect(waitUntil([&server] { return server.hasPendingConnections(); }),
           "ambiguous MainWindow setup POST connects");
    QTcpSocket *action = server.nextPendingConnection();
    if (action != nullptr) {
        readHttpRequest(action);
        action->abort();
        action->deleteLater();
    }
    expect(waitUntil([&controlClient] { return controlClient.inferenceReconcilePending(); }),
           "ambiguous MainWindow setup schedules reconciliation GET");

    expect(waitUntil([&server] { return server.hasPendingConnections(); }),
           "ambiguous MainWindow setup reconciliation GET connects");
    QTcpSocket *reconcile = server.nextPendingConnection();
    if (reconcile != nullptr) {
        readHttpRequest(reconcile);
        if (successful) {
            sendHttpJson(
                reconcile,
                QByteArrayLiteral(
                    "{\"ok\":true,\"camera\":{\"running\":true},"
                    "\"capture\":{\"state\":\"idle\",\"recording\":false},"
                    "\"inference\":{\"configured\":true,"
                    "\"control_supported\":true,\"operation\":null,"
                    "\"state\":\"disabled\"}}"));
        } else {
            const QByteArray body = QByteArrayLiteral(
                "{\"ok\":false,\"error\":\"internal_error\","
                "\"message\":\"status unavailable\"}");
            const QByteArray response =
                "HTTP/1.1 500 Internal Server Error\r\n"
                "Content-Type: application/json\r\n"
                "Content-Length: " + QByteArray::number(body.size())
                + "\r\nConnection: close\r\n\r\n" + body;
            reconcile->write(response);
            reconcile->flush();
            reconcile->disconnectFromHost();
        }
        reconcile->deleteLater();
    }
    expect(waitUntil([&controlClient] {
        return !controlClient.requestInFlight()
            && !controlClient.inferenceReconcilePending();
    }), "ambiguous MainWindow setup reconciliation completes");
    expect(controlClient.hasFreshStatus() == successful,
           "ambiguous MainWindow setup preserves GET freshness outcome");
}

void testTask02SuccessfulReconciliationClearsUncertaintyBeforeClose()
{
    rb::FakeTransport transport;
    rb::RobotController controller(
        &transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::vision::VisionControlClient controlClient;
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "successful reconciliation close server must listen");
    controlClient.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    rb::MainWindow window(&controller, nullptr, &controlClient);

    seedAmbiguousInferenceReconciliation(controlClient, server, true);
    const auto messageBoxSeen = answerNextMessageBoxAndTrack(QMessageBox::Yes);
    const bool closed = window.close();
    waitForMs(80);
    expect(closed, "successful authoritative reconciliation permits close");
    expect(!*messageBoxSeen,
           "successful authoritative reconciliation clears uncertainty-only close warning");
    expect(!server.hasPendingConnections(),
           "successful reconciliation close emits no automatic Stop POST");
    expect(transport.writes().isEmpty(),
           "successful reconciliation close emits no Robot-control writes");
}

void testTask02SuccessfulReconciliationClearsUncertaintyBeforeApplyHost()
{
    rb::FakeTransport transport;
    rb::RobotController controller(
        &transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::vision::VisionControlClient controlClient;
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "successful reconciliation Apply Host server must listen");
    controlClient.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    rb::MainWindow window(&controller, nullptr, &controlClient);
    auto *host = window.findChild<QLineEdit *>(QStringLiteral("piHost"));
    auto *apply = window.findChild<QPushButton *>(QStringLiteral("applyPiHostButton"));
    expect(host != nullptr && apply != nullptr,
           "successful reconciliation Apply Host widgets exist");
    if (host == nullptr || apply == nullptr) {
        return;
    }

    seedAmbiguousInferenceReconciliation(controlClient, server, true);
    host->setText(QStringLiteral("127.0.0.2"));
    const auto messageBoxSeen = answerNextMessageBoxAndTrack(QMessageBox::Yes);
    apply->click();
    waitForMs(80);
    expect(!*messageBoxSeen,
           "successful authoritative reconciliation clears uncertainty-only Apply Host warning");
    expect(host->text() == QStringLiteral("127.0.0.2")
               && controlClient.host() == QStringLiteral("127.0.0.2"),
           "Apply Host commits directly after uncertainty is reconciled");
    expect(transport.writes().isEmpty(),
           "Apply Host uncertainty reconciliation emits no Robot-control writes");
    window.close();
}

void testTask02FailedReconciliationKeepsUncertaintyAndDoesNotRetry()
{
    rb::FakeTransport transport;
    rb::RobotController controller(
        &transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::vision::VisionControlClient controlClient;
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "failed reconciliation close server must listen");
    controlClient.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    rb::MainWindow window(&controller, nullptr, &controlClient);

    seedAmbiguousInferenceReconciliation(controlClient, server, false);
    const auto messageBoxSeen = answerNextMessageBoxAndTrack(QMessageBox::Yes);
    const bool closed = window.close();
    waitForMs(100);
    expect(closed, "failed reconciliation still permits confirmed close");
    expect(*messageBoxSeen,
           "failed reconciliation keeps the uncertainty-only close warning latched");
    expect(!server.hasPendingConnections(),
           "failed reconciliation does not retry the inference POST");
    expect(transport.writes().isEmpty(),
           "failed reconciliation close emits no Robot-control writes");
}

void testTask02RemoteAndDirectEndpointWidgetsStayDistinct()
{
    rb::RemoteRobotController remote;
    rb::MainWindow remoteWindow(&remote);
    expect(remoteWindow.findChild<QLineEdit *>(QStringLiteral("piHost")) != nullptr,
           "Remote mode retains the shared Pi Host editor");
    expect(remoteWindow.findChild<QSpinBox *>(QStringLiteral("robotTcpPort")) != nullptr,
           "Remote mode exposes a dedicated Robot TCP port");
    expect(remoteWindow.findChild<QComboBox *>(QStringLiteral("serialPortCombo")) == nullptr,
           "Remote mode does not replace Pi Host with a serial combo");
    remoteWindow.close();

    rb::FakeTransport transport;
    rb::RobotController direct(
        &transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::MainWindow directWindow(&direct);
    expect(directWindow.findChild<QComboBox *>(QStringLiteral("serialPortCombo")) != nullptr,
           "Direct mode exposes a dedicated serial port combo");
    expect(directWindow.findChild<QSpinBox *>(QStringLiteral("robotTcpPort")) == nullptr,
           "Direct mode has no remote Robot TCP editor");
    directWindow.close();
}

void testDashboardLayout()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::MainWindow window(&controller);

    // Realtime Video placeholder exists.
    expect(findGroupBox(&window, QStringLiteral("Realtime Video")) != nullptr,
           "MainWindow must expose a Realtime Video placeholder");

    // Emergency Stop exists and stays disabled in Phase 1.
    QPushButton *estop = buttonWithText(&window, QStringLiteral("Emergency Stop"));
    expect(estop != nullptr, "MainWindow must expose an Emergency Stop button");
    expect(estop != nullptr && !estop->isEnabled(),
           "Emergency Stop must remain disabled (no Phase 1 message)");

    // The 2x2 status grid keeps all four compact status cards present.
    expect(findGroupBox(&window, QStringLiteral("Leak Detection")) != nullptr,
           "dashboard status grid must keep the Leak Detection card");
    expect(findGroupBox(&window, QStringLiteral("IMU — JY901S")) != nullptr,
           "dashboard status grid must keep the IMU card");
    expect(findGroupBox(&window, QStringLiteral("Depth Sensor — ROVMAKER")) != nullptr,
           "dashboard status grid must keep the Depth card");
    expect(findGroupBox(&window, QStringLiteral("Protocol / Link")) != nullptr,
           "dashboard status grid must keep the Protocol / Link card");

    // Actuator Control exists.
    expect(findGroupBox(&window, QStringLiteral("Actuator Control")) != nullptr,
           "MainWindow must expose the Actuator Control panel");

    // All five semantic servo panels exist.
    const char *servoNames[] = {
        "FrontRight", "FrontLeft", "Depth", "RearRight", "RearLeft",
    };
    int servoPanels = 0;
    for (const char *name : servoNames) {
        if (findGroupBox(&window, QString::fromLatin1(name)) != nullptr) {
            ++servoPanels;
        }
    }
    expect(servoPanels == 5, "MainWindow must expose all five servo panels");

    // Data Plots region exists as an independent tab widget with IMU/Depth/
    // Actuator placeholder sub-tabs.
    bool imuPlot = tabWidgetWithText(&window, QStringLiteral("IMU")) != nullptr;
    bool depthPlot = tabWidgetWithText(&window, QStringLiteral("Depth")) != nullptr;
    bool actuatorPlot = tabWidgetWithText(&window, QStringLiteral("Actuator")) != nullptr;
    expect(imuPlot && depthPlot && actuatorPlot,
           "MainWindow must expose the Data Plots region (IMU/Depth/Actuator)");

    // Log and Protocol Details tabs exist.
    QTabWidget *logTabs = tabWidgetWithText(&window, QStringLiteral("Log"));
    QTabWidget *detailsTabs = tabWidgetWithText(&window, QStringLiteral("Protocol Details"));
    expect(logTabs != nullptr && detailsTabs != nullptr,
           "MainWindow must expose Log and Protocol Details tabs");

    // ACK status must be owned by the Protocol Details page only, never also
    // placed in the Protocol/Link summary card. The summary card must not
    // contain an ACK-state label, and the details page must retain one.
    QGroupBox *summary = findGroupBox(&window, QStringLiteral("Protocol / Link"));
    expect(summary != nullptr, "MainWindow must expose the Protocol / Link summary");
    bool summaryHasAckLabel = false;
    for (QLabel *label : summary->findChildren<QLabel *>()) {
        if (label->text() == QStringLiteral("Idle")) {
            summaryHasAckLabel = true;
        }
    }
    expect(!summaryHasAckLabel,
           "Protocol/Link summary must not contain an ACK-state label");
    if (detailsTabs != nullptr) {
        bool detailsHasAckLabel = false;
        for (int i = 0; i < detailsTabs->count(); ++i) {
            for (QLabel *label : detailsTabs->widget(i)->findChildren<QLabel *>()) {
                if (label->text() == QStringLiteral("Idle")) {
                    detailsHasAckLabel = true;
                }
            }
        }
        expect(detailsHasAckLabel,
               "Protocol Details page must retain the ACK-status label");
    }

    // Log/Details tab widget keeps its three pages.
    if (detailsTabs != nullptr) {
        expect(detailsTabs->count() == 3,
               "Log/Details tab must keep Log, Telemetry Details, and Protocol Details pages");
    }

    // A Telemetry Details page exists and hosts the IMU/Depth diagnostics.
    QTabWidget *telemetryTabs = tabWidgetWithText(&window, QStringLiteral("Telemetry Details"));
    expect(telemetryTabs != nullptr,
           "MainWindow must expose a Telemetry Details tab");
    if (telemetryTabs != nullptr) {
        bool foundTelemetryPage = false;
        for (int i = 0; i < telemetryTabs->count(); ++i) {
            if (telemetryTabs->tabText(i) == QStringLiteral("Telemetry Details")) {
                foundTelemetryPage = true;
            }
        }
        expect(foundTelemetryPage,
               "Telemetry Details tab must contain its page");
    }

    // Diagnostics still flow to the Telemetry Details page after a real
    // protocol update (no widget loss from re-adding to layouts).
    controller.connectTransport({QStringLiteral("COM_TEST"), 9600});
    transport.simulateConnected();
    if (telemetryTabs != nullptr) {
        bool imuDiagLive = false;
        bool depthDiagLive = false;
        injectImuSnapshot(transport);
        for (int i = 0; i < telemetryTabs->count(); ++i) {
            const QWidget *page = telemetryTabs->widget(i);
            for (const QLabel *label : page->findChildren<QLabel *>()) {
                if (label->text().contains(QStringLiteral("valid 42"))) {
                    imuDiagLive = true;
                }
            }
        }
        injectDepthSnapshot(transport);
        for (int i = 0; i < telemetryTabs->count(); ++i) {
            const QWidget *page = telemetryTabs->widget(i);
            for (const QLabel *label : page->findChildren<QLabel *>()) {
                if (label->text().contains(QStringLiteral("RX 100"))) {
                    depthDiagLive = true;
                }
            }
        }
        expect(imuDiagLive && depthDiagLive,
               "Telemetry Details page must show live IMU/Depth diagnostics");
    }

    // Window fits the approved minimum; the dashboard needs no scroll at the
    // default size.
    window.resize(1420, 880);
    const QSize ws = window.size();
    expect(ws.width() >= 1100 && ws.height() >= 720,
           "window must fit the approved minimum (1100x720)");

    // Structural one-row check: all five servo cards share the same parent
    // (the Actuator Control cards layout), i.e. they are siblings in one row.
    // This avoids pixel coordinates and platform font metrics.
    const QWidget *cardsParent = nullptr;
    bool allSameParent = true;
    for (QGroupBox *g : window.findChildren<QGroupBox *>()) {
        const QString t = g->title();
        if (t == QStringLiteral("FrontRight")
            || t == QStringLiteral("FrontLeft")
            || t == QStringLiteral("Depth")
            || t == QStringLiteral("RearRight")
            || t == QStringLiteral("RearLeft")) {
            if (cardsParent == nullptr) {
                cardsParent = g->parentWidget();
            } else if (g->parentWidget() != cardsParent) {
                allSameParent = false;
            }
        }
    }
    expect(allSameParent,
           "all five servo cards must be siblings in one row");

    // Relative horizontal-clip check: no servo card wider than the window.
    // A card wider than the window would be horizontally clipped.
    bool anyClipped = false;
    for (QGroupBox *g : window.findChildren<QGroupBox *>()) {
        const QString t = g->title();
        if (t == QStringLiteral("FrontRight")
            || t == QStringLiteral("FrontLeft")
            || t == QStringLiteral("Depth")
            || t == QStringLiteral("RearRight")
            || t == QStringLiteral("RearLeft")) {
            if (g->width() > ws.width()) {
                anyClipped = true;
            }
        }
    }
    expect(!anyClipped,
           "no servo card may be wider than the window (horizontal clip)");

    // Vertical-clipping regression: after showing at the default size and
    // letting the layout settle, every critical dashboard telemetry label must
    // actually be laid out at a height at least its required (minimum) height.
    // This is relative to each widget's own minimumSizeHint(), so it is
    // independent of font metrics and fixed pixel coordinates.
    window.show();
    for (int i = 0; i < 20; ++i) {
        QApplication::processEvents();
    }
    auto checkLabelHeight = [&window](const QString &title) {
        QGroupBox *box = findGroupBox(&window, title);
        if (box == nullptr) {
            return false;
        }
        bool ok = true;
        const auto labels = box->findChildren<QLabel *>();
        for (const QLabel *label : labels) {
            if (label->text().isEmpty()) {
                continue;
            }
            const int required = label->minimumSizeHint().height();
            const int actual = label->height();
            if (actual > 0 && actual < required) {
                ok = false;
            }
        }
        return ok;
    };
    expect(checkLabelHeight(QStringLiteral("IMU — JY901S")),
           "IMU card status/metric labels must not be vertically clipped");
    expect(checkLabelHeight(QStringLiteral("Depth Sensor — ROVMAKER")),
           "Depth card status/metric labels must not be vertically clipped");
    expect(checkLabelHeight(QStringLiteral("Protocol / Link")),
           "Protocol/Link card TX/RX/CRC/Timeout/ACK RTT labels must not be "
           "vertically clipped");
    expect(checkLabelHeight(QStringLiteral("Leak Detection")),
           "Leak card label must not be vertically clipped");
    window.hide();
}

} // namespace

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
#ifdef RB_MAIN_WINDOW_TASK02_ONLY
    testTask02SharedHostWidgetsAndHttpOnlyControls();
    testTask02RefreshAndInferenceUseOnlyCommittedHost();
    testTask02CaptureAndInferenceErrorsStaySeparated();
    testTask02ApplyHostBlocksQueuedMutation();
    testTask02CloseNeverImplicitlyStopsRemoteWork();
    testTask02CloseWarnsForStoppingAndRetryingRemoteWork();
    testTask02SuccessfulReconciliationClearsUncertaintyBeforeClose();
    testTask02SuccessfulReconciliationClearsUncertaintyBeforeApplyHost();
    testTask02FailedReconciliationKeepsUncertaintyAndDoesNotRetry();
    testTask02RemoteAndDirectEndpointWidgetsStayDistinct();
#else
    testImuPanelLifecycle();
    testDepthPanelLifecycle();
    testLeakPanelLifecycle();
    testMotionPanelLifecycleAndManualArbitration();
    testGaitBackendPanelLifecycle();
    testVisionConnectionIsIndependentFromControlTransport();
    testVisionCaptureControlsAreIndependentFromRobotTransport();
    testVisionCaptureActionDefersWindowClose();
    testVisionInferenceDiagnosticsAreRenderedWithoutRobotWrites();
    testTask02SharedHostWidgetsAndHttpOnlyControls();
    testTask02RefreshAndInferenceUseOnlyCommittedHost();
    testTask02CaptureAndInferenceErrorsStaySeparated();
    testTask02ApplyHostBlocksQueuedMutation();
    testTask02CloseNeverImplicitlyStopsRemoteWork();
    testTask02CloseWarnsForStoppingAndRetryingRemoteWork();
    testTask02SuccessfulReconciliationClearsUncertaintyBeforeClose();
    testTask02SuccessfulReconciliationClearsUncertaintyBeforeApplyHost();
    testTask02FailedReconciliationKeepsUncertaintyAndDoesNotRetry();
    testTask02RemoteAndDirectEndpointWidgetsStayDistinct();
    testDashboardLayout();
#endif
    std::fflush(stderr);
    if (failures == 0) {
        std::fprintf(stdout, "All MainWindow tests passed\n");
    }
    return failures == 0 ? 0 : 1;
}
