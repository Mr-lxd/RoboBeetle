#include "protocol/PacketCodec.h"
#include "robot/DepthSnapshot.h"
#include "robot/ImuSnapshot.h"
#include "robot/RobotController.h"
#include "transport/FakeTransport.h"
#include "ui/MainWindow.h"

#include <QApplication>
#include <QComboBox>
#include <QEventLoop>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

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

} // namespace

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    testImuPanelLifecycle();
    testDepthPanelLifecycle();
    testMotionPanelLifecycleAndManualArbitration();
    testGaitBackendPanelLifecycle();
    if (failures == 0) {
        std::fprintf(stdout, "All MainWindow tests passed\n");
    }
    return failures == 0 ? 0 : 1;
}
