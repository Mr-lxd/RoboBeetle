#include "protocol/PacketCodec.h"
#include "robot/DepthSnapshot.h"
#include "robot/ImuSnapshot.h"
#include "robot/RobotController.h"
#include "transport/FakeTransport.h"
#include "ui/MainWindow.h"

#include <QApplication>
#include <QGroupBox>
#include <QLabel>

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

bool hasLabelText(const QWidget *root, const QString &text)
{
    for (QLabel *label : root->findChildren<QLabel *>()) {
        if (label->text() == text) {
            return true;
        }
    }
    return false;
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

} // namespace

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    testImuPanelLifecycle();
    testDepthPanelLifecycle();
    if (failures == 0) {
        std::fprintf(stdout, "All MainWindow IMU panel tests passed\n");
    }
    return failures == 0 ? 0 : 1;
}
