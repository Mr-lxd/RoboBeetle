#include "protocol/PacketCodec.h"
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

QGroupBox *imuPanel(rb::MainWindow &window)
{
    for (QGroupBox *box : window.findChildren<QGroupBox *>()) {
        if (box->title() == QStringLiteral("IMU — JY901S")) {
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

} // namespace

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    testImuPanelLifecycle();
    if (failures == 0) {
        std::fprintf(stdout, "All MainWindow IMU panel tests passed\n");
    }
    return failures == 0 ? 0 : 1;
}
