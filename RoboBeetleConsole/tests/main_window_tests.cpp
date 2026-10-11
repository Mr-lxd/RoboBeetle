#include "protocol/PacketCodec.h"
#include "robot/DepthSnapshot.h"
#include "robot/ImuSnapshot.h"
#include "robot/RobotController.h"
#include "remote/RemoteRobotController.h"
#include "transport/FakeTransport.h"
#include "ui/MainWindow.h"
#include "vision/DetectionClient.h"
#include "vision/VisionClient.h"
#include "vision/VisionControlClient.h"
#include "vision/VideoView.h"

#include <QApplication>
#include <QAbstractSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QDir>
#include <QFile>
#include <QElapsedTimer>
#include <QDoubleSpinBox>
#include <QEventLoop>
#include <QGroupBox>
#include <QGridLayout>
#include <QHostAddress>
#include <QKeyEvent>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QLayout>
#include <QMessageBox>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QStandardPaths>
#include <QTemporaryDir>
#include <QSlider>
#include <QTabWidget>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>

#include <cstdio>
#include <cmath>
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
        if (box->title() == QStringLiteral("IMU")) {
            return box;
        }
    }
    return nullptr;
}

QGroupBox *depthPanel(rb::MainWindow &window)
{
    for (QGroupBox *box : window.findChildren<QGroupBox *>()) {
        if (box->title() == QStringLiteral("Depth Sensor")) {
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

QWidget *motionPanel(rb::MainWindow &window)
{
    return window.findChild<QWidget *>(QStringLiteral("motionPage"));
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

QComboBox *frontRearCoordinationCombo(const QWidget *root)
{
    for (QComboBox *combo : root->findChildren<QComboBox *>()) {
        if (combo->objectName() == QStringLiteral("frontRearCoordinationCombo")) {
            return combo;
        }
    }
    return nullptr;
}

int gridRowForWidget(const QGridLayout *layout, const QWidget *widget)
{
    if (layout == nullptr || widget == nullptr) {
        return -1;
    }
    const int index = layout->indexOf(widget);
    if (index < 0) {
        return -1;
    }
    int row = -1;
    int column = 0;
    int rowSpan = 0;
    int columnSpan = 0;
    layout->getItemPosition(index, &row, &column, &rowSpan, &columnSpan);
    return row;
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


QByteArray slice5StatusBody(
    quint16 detectionPort,
    const QString &inferenceState = QStringLiteral("running"),
    bool advertiseDetection = true,
    int detectionVersion = rb::vision::kDetectionStreamVersion)
{
    QJsonObject inference{
        {QStringLiteral("configured"), true},
        {QStringLiteral("control_supported"), true},
        {QStringLiteral("operation"), QJsonValue::Null},
        {QStringLiteral("state"), inferenceState},
        {QStringLiteral("artifact_name"), QJsonValue::Null},
        {QStringLiteral("model_sha256"), QJsonValue::Null},
        {QStringLiteral("last_error"), QJsonValue::Null},
    };
    if (advertiseDetection) {
        inference.insert(
            QStringLiteral("detection_stream_supported"),
            true);
        inference.insert(
            QStringLiteral("detection_stream_port"),
            static_cast<int>(detectionPort));
        inference.insert(
            QStringLiteral("detection_stream_version"),
            detectionVersion);
    }

    const QJsonObject root{
        {QStringLiteral("ok"), true},
        {QStringLiteral("camera"),
         QJsonObject{
             {QStringLiteral("running"), true},
             {QStringLiteral("latest_frame_id"), 100},
         }},
        {QStringLiteral("capture"),
         QJsonObject{
             {QStringLiteral("state"), QStringLiteral("idle")},
             {QStringLiteral("recording"), false},
             {QStringLiteral("recorded_frames"), 0},
             {QStringLiteral("snapshot_count"), 0},
             {QStringLiteral("queue_bytes"), 0},
         }},
        {QStringLiteral("inference"), inference},
    };
    return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

QByteArray slice5DetectionLine(
    quint64 frameId,
    quint64 captureTimestampNs,
    const QString &className = QStringLiteral("fish"),
    double confidence = 0.88,
    double x = 120.0,
    double y = 160.0)
{
    const QJsonObject detection{
        {QStringLiteral("class_id"), 0},
        {QStringLiteral("class_name"), className},
        {QStringLiteral("confidence"), confidence},
        {QStringLiteral("original_x"), x},
        {QStringLiteral("original_y"), y},
    };
    const QJsonObject record{
        {QStringLiteral("type"), QStringLiteral("detections")},
        {QStringLiteral("version"), rb::vision::kDetectionStreamVersion},
        {QStringLiteral("frame_id"), static_cast<qint64>(frameId)},
        {QStringLiteral("capture_timestamp_ns"),
         static_cast<qint64>(captureTimestampNs)},
        {QStringLiteral("width"), 640},
        {QStringLiteral("height"), 480},
        {QStringLiteral("coordinate_space"),
         QStringLiteral("original_frame_pixels")},
        {QStringLiteral("detections"), QJsonArray{detection}},
    };
    return QJsonDocument(record).toJson(QJsonDocument::Compact) + '\n';
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

QString statusDotTip(const QWidget *panel, const char *dotName)
{
    const auto *dot = panel->findChild<QLabel *>(QString::fromLatin1(dotName));
    return dot != nullptr ? dot->toolTip() : QString();
}

void testImuPanelLifecycle()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::MainWindow window(&controller);

    QGroupBox *panel = imuPanel(window);
    expect(panel != nullptr, "MainWindow must expose an IMU panel");
    if (panel == nullptr) {
        return;
    }
    expect(statusDotTip(panel, "imuStatusDot") == QStringLiteral("Unknown")
               && !hasLabelText(panel, QStringLiteral("Unknown")),
           "IMU panel must start with Unknown status");
    expect(hasLabelText(panel, QStringLiteral("--")),
           "IMU panel must not show live values before reception");

    controller.connectTransport({QStringLiteral("COM_TEST"), 9600});
    transport.simulateConnected();
    injectImuSnapshot(transport);
    expect(statusDotTip(panel, "imuStatusDot") == QStringLiteral("Receiving")
               && !hasLabelText(panel, QStringLiteral("Receiving")),
           "IMU panel must show Receiving after a valid snapshot");
    expect(hasLabelText(panel, QStringLiteral("1.000, -2.000, 0.000 g")),
           "IMU panel must display fixed-point Acc values");
    expect(hasLabelText(panel, QStringLiteral("1.0, -2.0, 0.0 dps")),
           "IMU panel must display fixed-point Gyro values");
    expect(hasLabelText(panel, QStringLiteral("3.00, -4.00, 5.00 deg")),
           "IMU panel must display fixed-point Angle values");

    controller.imuMonitor()->tick(controller.imuState().lastReceivedAtMs
                                  + rb::ImuMonitor::StaleTimeoutMs);
    expect(statusDotTip(panel, "imuStatusDot") == QStringLiteral("Stale")
               && !hasLabelText(panel, QStringLiteral("Stale")),
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
    expect(panel != nullptr, "MainWindow must expose a Depth Sensor panel");
    if (panel == nullptr) {
        return;
    }
    expect(statusDotTip(panel, "depthStatusDot") == QStringLiteral("Unknown")
               && !hasLabelText(panel, QStringLiteral("Unknown")),
           "Depth panel must start with Unknown status");
    expect(hasLabelText(panel, QStringLiteral("--")),
           "Depth panel must not show live values before reception");

    controller.connectTransport({QStringLiteral("COM_TEST"), 9600});
    transport.simulateConnected();
    injectDepthSnapshot(transport);
    expect(statusDotTip(panel, "depthStatusDot") == QStringLiteral("Receiving")
               && !hasLabelText(panel, QStringLiteral("Receiving")),
           "Depth panel must show Receiving after a valid snapshot");
    expect(hasLabelText(panel, QStringLiteral("1.23 m")),
           "Depth panel must display millimetre values as metres");
    expect(hasLabelText(panel, QStringLiteral("25.34 C")),
           "Depth panel must display centi-degree values as Celsius");

    controller.depthMonitor()->tick(controller.depthState().lastReceivedAtMs
                                    + rb::DepthMonitor::StaleTimeoutMs);
    expect(statusDotTip(panel, "depthStatusDot") == QStringLiteral("Stale")
               && !hasLabelText(panel, QStringLiteral("Stale")),
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

    QWidget *panel = motionPanel(window);
    expect(panel != nullptr,
           "MainWindow must expose the Motion / Gait panel");
    if (panel == nullptr) {
        return;
    }

    // The motion panel splits into two visually distinct subareas: a D-pad-like
    // Motion Control block and a separate Gait block.
    expect(findGroupBox(panel, QStringLiteral("Motion Control")) != nullptr,
           "Motion panel must expose a Motion Control subarea");
    expect(findGroupBox(panel, QStringLiteral("Gait")) != nullptr,
           "Motion panel must expose a Gait subarea");

    expect(gaitBackendCombo(panel) != nullptr,
           "Motion panel must expose a dedicated gait backend combo");
    QPushButton *forwardButton = buttonWithText(panel, QStringLiteral("Forward"));
    QPushButton *backwardButton = buttonWithText(panel, QStringLiteral("Backward"));
    QPushButton *turnLeftButton = buttonWithText(panel, QStringLiteral("Turn Left"));
    QPushButton *turnRightButton = buttonWithText(panel, QStringLiteral("Turn Right"));
    QPushButton *ascendButton = buttonWithText(panel, QStringLiteral("Ascend"));
    QPushButton *descendButton = buttonWithText(panel, QStringLiteral("Descend"));
    QPushButton *stopButton = window.findChild<QPushButton *>(QStringLiteral("motionStopButton"));
    expect(forwardButton != nullptr && backwardButton != nullptr && turnLeftButton != nullptr &&
               turnRightButton != nullptr && ascendButton != nullptr && descendButton != nullptr &&
               stopButton != nullptr,
           "Motion panel must expose direct Forward/Backward/Turn/Axis/Stop buttons");
    if (forwardButton == nullptr || backwardButton == nullptr
        || turnLeftButton == nullptr || turnRightButton == nullptr
        || ascendButton == nullptr || descendButton == nullptr
        || stopButton == nullptr) {
        return;
    }

    expect(hasLabelText(&window, QStringLiteral("Stopped")),
           "Motion panel must start with Stopped status");
    expect(!forwardButton->isEnabled() && !turnLeftButton->isEnabled()
               && !turnRightButton->isEnabled() && !ascendButton->isEnabled()
               && !descendButton->isEnabled() && !stopButton->isEnabled(),
           "Motion controls must be disabled while disconnected");
    expect(!backwardButton->isEnabled() &&
               backwardButton->toolTip().contains(QStringLiteral("not supported"),
                                                  Qt::CaseInsensitive),
           "Brake remains disabled until the existing motion backend is verified");

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
           "Brake must remain disabled after the link is ready");
    expect(buttonWithText(&window, QStringLiteral("Release")) != nullptr,
           "enabled individual Servo controls must use Release semantics");

    auto *frontRightAngle =
        window.findChild<QDoubleSpinBox *>(QStringLiteral("servoAngleSpin0"));
    expect(frontRightAngle != nullptr && frontRightAngle->isEnabled(),
           "Servo Fine Control keeps FrontRight Angle editable after Enable ACK");
    expect(frontRightAngle != nullptr
               && frontRightAngle->buttonSymbols() == QAbstractSpinBox::NoButtons,
           "Angle input removes its up/down buttons");
    auto *frontRightAngleApply =
        window.findChild<QPushButton *>(QStringLiteral("servoAngleApplyButton0"));
    auto *frontRightPwm =
        window.findChild<QSpinBox *>(QStringLiteral("servoPwmSpin0"));
    auto *frontRightPwmSlider =
        window.findChild<QSlider *>(QStringLiteral("servoPwmSlider0"));
    auto *frontRightPwmApply =
        window.findChild<QPushButton *>(QStringLiteral("servoApplyButton0"));
    expect(frontRightAngleApply != nullptr && frontRightAngleApply->isEnabled(),
           "Servo Fine Control exposes an explicit Apply Angle action after Enable ACK");
    expect(frontRightPwm != nullptr && frontRightPwmSlider != nullptr
               && frontRightPwmApply != nullptr,
           "Servo Fine Control keeps PWM value, slider, and Apply PWM controls");
    if (frontRightAngle != nullptr && frontRightAngle->isEnabled()
        && frontRightAngleApply != nullptr
        && frontRightPwm != nullptr && frontRightPwmSlider != nullptr
        && frontRightPwmApply != nullptr) {
        const qsizetype writesBeforePreview = transport.writes().size();
        frontRightPwmSlider->setValue(1600);
        expect(frontRightPwm->value() == 1600
                   && frontRightAngle->value() == -15.0,
               "PWM slider updates the PWM field and equivalent angle preview");
        expect(transport.writes().size() == writesBeforePreview,
               "PWM slider movement must remain preview-only");

        frontRightAngle->setValue(10.0);
        expect(frontRightPwm->value() == 1350
                   && frontRightPwmSlider->value() == 1350,
               "Angle editing updates equivalent PWM field and slider position");
        expect(transport.writes().size() == writesBeforePreview,
               "Angle editing must remain preview-only");

        frontRightAngle->setValue(45.0);
        expect(frontRightPwm->value() == 1000
                   && !frontRightPwmApply->isEnabled()
                   && frontRightAngleApply->isEnabled(),
               "Angle-only calibration endpoints remain previewable while raw PWM Apply stays inside its safety envelope");

        frontRightPwmSlider->setValue(1500);
        expect(frontRightPwmApply->isEnabled(),
               "Apply PWM re-enables when the preview returns inside the raw PWM envelope");
        const qsizetype writesBeforePwm = transport.writes().size();
        frontRightPwmApply->click();
        expect(transport.writes().size() == writesBeforePwm + 1
                   && lastPacket(transport).type == rb::MessageType::SetServoPwm,
               "Apply PWM sends exactly one SetServoPwm command");
        if (transport.writes().size() == writesBeforePwm + 1) {
            acknowledgeLast(transport);
        }

        const qsizetype writesBeforeAngle = transport.writes().size();
        frontRightAngle->setValue(12.3);
        QMetaObject::invokeMethod(
            frontRightAngle, "editingFinished", Qt::DirectConnection);
        expect(transport.writes().size() == writesBeforeAngle,
               "editing or finishing Angle must remain preview-only");
        frontRightAngleApply->click();
        expect(transport.writes().size() == writesBeforeAngle + 1,
               "Apply Angle sends exactly one command");
        expect(!transport.writes().isEmpty()
                   && lastPacket(transport).type
                          == rb::MessageType::SetServoAngle,
               "Apply Angle must preserve the existing SetServoAngle protocol command");
        if (transport.writes().size() == writesBeforeAngle + 1) {
            acknowledgeLast(transport);
        }
    }

    forwardButton->click();
    acknowledgeLast(transport);
    expect(controller.motionState() == rb::MotionState::Running,
           "direct Forward button should reach Running after ACK");
    expect(hasLabelText(&window, QStringLiteral("Running — Forward")),
           "Motion panel should display the running mode");
    expect(forwardButton->isCheckable()
               && forwardButton->property("consoleActionRole").toString()
                      == QStringLiteral("secondary"),
           "Motion buttons must use the shared secondary action role");
    expect(forwardButton->isChecked()
               && !turnLeftButton->isChecked()
               && !turnRightButton->isChecked()
               && !ascendButton->isChecked()
               && !descendButton->isChecked(),
           "the active Forward button must be checked exclusively");

    for (QPushButton *button : window.findChildren<QPushButton *>()) {
        if (button->text() == QStringLiteral("Apply")
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
        QPushButton *disableAll = window.findChild<QPushButton *>(QStringLiteral("disableAllButton"));
        expect(disableAll != nullptr && disableAll->isEnabled(),
               "Disable All must remain available during Motion");
    }

    const qsizetype writesBeforeMotionStop = transport.writes().size();
    stopButton->click();
    expect(transport.writes().size() == writesBeforeMotionStop + 1,
           "persistent Motion Stop invokes exactly one controller command");
    acknowledgeLast(transport);
    expect(controller.motionState() == rb::MotionState::Stopping,
           "Motion panel should display the acceptance-time Stopping state");
    expect(hasLabelText(&window, QStringLiteral("Stopping")),
           "Motion panel should show Stopping during the provisional ramp");
    waitForMs(rb::kMotionTransitionDurationMs + 50);
    expect(controller.motionState() == rb::MotionState::Stopped,
           "Motion panel should settle at Stopped after the provisional duration");
    expect(hasLabelText(&window, QStringLiteral("Stopped")),
           "Motion panel should show Stopped after the ramp timer");
    expect(!forwardButton->isChecked() && !turnLeftButton->isChecked()
               && !turnRightButton->isChecked()
               && !ascendButton->isChecked() && !descendButton->isChecked(),
           "Motion button highlight must clear after graceful STOP completes");

    QPushButton *disableAll = window.findChild<QPushButton *>(QStringLiteral("disableAllButton"));
    expect(disableAll != nullptr && disableAll->isEnabled(),
           "persistent Disable All remains enabled after Motion Stop");
    if (disableAll != nullptr && disableAll->isEnabled()) {
        const qsizetype writesBeforeDisableAll = transport.writes().size();
        disableAll->click();
        expect(transport.writes().size() == writesBeforeDisableAll + 1,
               "persistent Disable All invokes exactly one controller command");
    }
    QPushButton *emergencyStop = window.findChild<QPushButton *>(QStringLiteral("emergencyStopButton"));
    expect(emergencyStop != nullptr && !emergencyStop->isEnabled(),
           "Emergency Stop remains disabled after lifecycle actions");
    if (emergencyStop != nullptr) {
        const qsizetype writesBeforeEmergency = transport.writes().size();
        emergencyStop->click();
        expect(transport.writes().size() == writesBeforeEmergency,
               "disabled Emergency Stop emits no controller command");
    }
}

void testGaitBackendPanelLifecycle()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    rb::MainWindow window(&controller);

    QWidget *panel = motionPanel(window);
    expect(panel != nullptr, "gait backend test must find the Motion / Gait panel");
    if (panel == nullptr) {
        return;
    }
    QComboBox *combo = gaitBackendCombo(panel);
    QComboBox *coordinationCombo = frontRearCoordinationCombo(panel);
    expect(combo != nullptr, "Motion panel must expose the gait backend combo");
    expect(coordinationCombo != nullptr,
           "Motion panel must expose the Front / Rear coordination combo");
    if (combo == nullptr || coordinationCombo == nullptr) {
        return;
    }

    expect(combo->findText(QStringLiteral("Unknown")) >= 0,
           "gait backend combo must expose an explicit Unknown state");
    expect(combo->findText(QStringLiteral("SimpleGait")) >= 0,
           "gait backend combo must expose SimpleGait");
    expect(combo->findText(QStringLiteral("CPG")) >= 0,
           "gait backend combo must expose CPG");
    expect(combo->findText(QStringLiteral("Experimental Flex")) >= 0,
           "gait backend combo must expose Experimental Flex");
    expect(coordinationCombo->findText(QStringLiteral("Unknown")) >= 0
               && coordinationCombo->findText(QStringLiteral("Same Direction")) >= 0
               && coordinationCombo->findText(QStringLiteral("Opposite Direction")) >= 0,
           "Front / Rear combo must expose Unknown and both coordination values");
    auto *selectorLayout = qobject_cast<QGridLayout *>(
        combo->parentWidget()->layout());
    QLabel *gaitConfirmed = panel->findChild<QLabel *>(
        QStringLiteral("gaitBackendStatus"));
    QLabel *coordinationConfirmed = panel->findChild<QLabel *>(
        QStringLiteral("frontRearCoordinationStatus"));
    QLabel *gaitCurrent = panel->findChild<QLabel *>(
        QStringLiteral("gaitBackendCurrentLabel"));
    QLabel *gaitLabel = nullptr;
    QLabel *coordinationLabel = nullptr;
    for (QLabel *label : panel->findChildren<QLabel *>()) {
        if (label->text() == QStringLiteral("Backend"))
        {
            gaitLabel = label;
        } else if (label->text() == QStringLiteral("Front / Rear"))
        {
            coordinationLabel = label;
        }
    }
    expect(selectorLayout != nullptr && gaitLabel != nullptr && coordinationLabel != nullptr &&
               gridRowForWidget(selectorLayout, gaitLabel) == 0 &&
               gridRowForWidget(selectorLayout, coordinationLabel) == 0,
           "Backend and Front / Rear headings share the A prime header row");
    expect(selectorLayout != nullptr && gaitConfirmed != nullptr && gaitCurrent != nullptr &&
               gridRowForWidget(selectorLayout, gaitCurrent) == 0 &&
               gridRowForWidget(selectorLayout, combo) == 1 &&
               gridRowForWidget(selectorLayout, gaitConfirmed) == 1 &&
               gridRowForWidget(selectorLayout, coordinationCombo) == 1,
           "Backend, Current and Front / Rear values share the A prime value row");
    expect(coordinationConfirmed != nullptr && coordinationConfirmed->isHidden(),
           "coordination confirmation uses the existing selector without a duplicate status row");
    expect(!combo->isEnabled(),
           "gait backend selection must be disabled while disconnected");
    expect(!coordinationCombo->isEnabled(),
           "coordination selection must be disabled while disconnected");

    controller.connectTransport({QStringLiteral("COM_TEST"), 9600});
    transport.simulateConnected();
    expect(combo->isEnabled(),
           "gait backend selection must enable after connection");
    expect(coordinationCombo->isEnabled(),
           "coordination selection must enable after connection");

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
    expect(!coordinationCombo->isEnabled(),
           "a pending gait selector must also disable coordination selection");
    expect(gaitConfirmed != nullptr
               && gaitConfirmed->text() == QStringLiteral("Unknown"),
           "backend confirmation label stays Unknown until ACK");

    acknowledgeLastWithResult(transport, rb::AckResult::Busy);
    expect(!controller.confirmedGaitBackend().has_value(),
           "BUSY must preserve the previous confirmed backend in the UI path");
    expect(combo->isEnabled(),
           "gait backend combo must re-enable after BUSY");
    expect(coordinationCombo->isEnabled(),
           "coordination combo must re-enable after gait BUSY");

    combo->setCurrentIndex(simpleIndex);
    acknowledgeLast(transport);
    expect(controller.confirmedGaitBackend().has_value()
               && *controller.confirmedGaitBackend() == rb::GaitBackend::SimpleGait,
           "matching UI selector ACK must confirm SimpleGait");
    expect(combo->currentData().toInt()
                == static_cast<int>(rb::GaitBackend::SimpleGait),
           "UI combo must reflect the ACK-confirmed backend");
    expect(gaitConfirmed != nullptr
               && gaitConfirmed->text() == QStringLiteral("Confirmed — SimpleGait"),
           "backend confirmation label shows only the ACK-confirmed value");

    const int oppositeIndex = coordinationCombo->findData(
        static_cast<int>(rb::FrontRearCoordination::OppositeDirection));
    const qsizetype writesBeforeCoordination = transport.writes().size();
    coordinationCombo->setCurrentIndex(oppositeIndex);
    expect(transport.writes().size() == writesBeforeCoordination + 1,
           "coordination selection must emit exactly one robot write");
    expect(lastPacket(transport).type == rb::MessageType::SetFrontRearCoordination
               && lastPacket(transport).payload == QByteArray(1, '\1'),
           "coordination selector must send only Protocol V2 0x17 and one value byte");
    expect(!combo->isEnabled() && !coordinationCombo->isEnabled(),
           "a pending coordination selector must disable both selectors");
    expect(coordinationConfirmed != nullptr
               && coordinationConfirmed->text() == QStringLiteral("Unknown"),
           "coordination confirmation label stays Unknown until ACK");
    acknowledgeLast(transport);
    expect(controller.confirmedFrontRearCoordination().has_value()
               && *controller.confirmedFrontRearCoordination()
                   == rb::FrontRearCoordination::OppositeDirection,
           "matching UI selector ACK must confirm Opposite Direction");
    expect(coordinationConfirmed != nullptr
               && coordinationConfirmed->text()
                   == QStringLiteral("Confirmed — Opposite Direction"),
           "coordination confirmation label shows its ACK-confirmed value");
    expect(combo->isEnabled() && coordinationCombo->isEnabled(),
           "both selectors re-enable after the matching coordination ACK");

    const rb::ServoId motionServos[] = {
        rb::ServoId::FrontRight, rb::ServoId::FrontLeft,
        rb::ServoId::RearRight, rb::ServoId::RearLeft,
    };
    for (const rb::ServoId servo : motionServos) {
        expect(controller.enableServo(servo),
               "Motion UI gate fixture must enable the four paddles");
        acknowledgeLast(transport);
    }
    expect(controller.startMotion(rb::MotionMode::Forward),
           "Motion UI gate fixture must start Forward");
    acknowledgeLast(transport);
    expect(!combo->isEnabled() && !coordinationCombo->isEnabled(),
           "both selectors must disable during active Motion");
    const qsizetype writesBeforeActiveRejection = transport.writes().size();
    expect(!controller.setGaitBackend(rb::GaitBackend::CPG)
               && !controller.setFrontRearCoordination(
                   rb::FrontRearCoordination::SameDirection),
           "controller must reject either selector during active Motion");
    expect(transport.writes().size() == writesBeforeActiveRejection,
           "active-Motion selector rejection must emit no robot writes");
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
           "Vision Status reports its own connected state");
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
        window.findChild<QLabel *>(QStringLiteral("captureDiagnostics"));
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
         && !captureDiagnostics->text().contains(QStringLiteral("Snapshots: 1"));
         ++i) {
        waitForMs(5);
    }
    expect(captureDiagnostics->text().contains(QStringLiteral("Snapshots: 1")),
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
    for (int i = 0; i < 50
         && !(startRecording->isEnabled()
              && startRecording->text() == QStringLiteral("Stop Recording"));
         ++i) {
        waitForMs(5);
    }
    expect(startRecording->isEnabled()
               && startRecording->text() == QStringLiteral("Stop Recording")
               && stopRecording->isEnabled()
               && !stopRecording->isVisible(),
           "recording status turns the single visible Recording action into Stop");

    // Drop RBVS only. Capture control must remain usable so a recording can
    // always be stopped even when the realtime video path fails.
    connectVideo->click();
    for (int i = 0; i < 50
         && !hasLabelText(panel, QStringLiteral("Disconnected")); ++i) {
        waitForMs(5);
    }
    expect(startRecording->isEnabled()
               && startRecording->text() == QStringLiteral("Stop Recording"),
           "the visible Recording toggle remains a committed-host Stop after RBVS disconnect");
    expect(transport.writes().isEmpty(),
           "RBVS disconnect while recording emits no robot-control bytes");

    startRecording->click();
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
    for (int i = 0; i < 50
         && startRecording->text() != QStringLiteral("Start Recording"); ++i) {
        waitForMs(5);
    }
    for (int i = 0; i < 50 && controlClient.actionBusy(); ++i) {
        waitForMs(5);
    }
    expect(!stopRecording->isEnabled()
               && !stopRecording->isVisible()
               && startRecording->isEnabled()
               && startRecording->text() == QStringLiteral("Start Recording"),
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
    window.show();
    QApplication::processEvents();
    QGroupBox *panel = findGroupBox(&window, QStringLiteral("Realtime Video"));
    expect(panel != nullptr, "inference diagnostics retain the Realtime Video card");
    if (panel == nullptr) {
        return;
    }

    auto *host = window.findChild<QLineEdit *>(QStringLiteral("piHost"));
    auto *port = window.findChild<QSpinBox *>(QStringLiteral("visionPort"));
    auto *state = panel->findChild<QLabel *>(QStringLiteral("inferenceState"));
    auto *diagnostics =
        window.findChild<QLabel *>(QStringLiteral("inferenceDiagnostics"));
    auto *visionDiagnostics =
        window.findChild<QLabel *>(QStringLiteral("visionDiagnostics"));
    auto *endpointDetails =
        window.findChild<QLabel *>(QStringLiteral("visionEndpointDetails"));
    auto *performance =
        panel->findChild<QLabel *>(QStringLiteral("inferencePerformanceSummary"));
    auto *detectionSummary =
        panel->findChild<QLabel *>(QStringLiteral("inferenceDetectionSummary"));
    auto *memorySummary =
        panel->findChild<QLabel *>(QStringLiteral("inferenceMemorySummary"));
    auto *httpState =
        panel->findChild<QLabel *>(QStringLiteral("visionControlState"));
    QPushButton *connectVideo =
        buttonWithText(panel, QStringLiteral("Connect Video"));
    expect(host != nullptr && port != nullptr && state != nullptr
               && diagnostics != nullptr && visionDiagnostics != nullptr
               && endpointDetails != nullptr && performance != nullptr
               && detectionSummary != nullptr && memorySummary != nullptr
               && httpState != nullptr && connectVideo != nullptr,
           "Realtime Video card exposes inference state and diagnostics labels");
    if (host == nullptr || port == nullptr || state == nullptr
        || diagnostics == nullptr || visionDiagnostics == nullptr
        || endpointDetails == nullptr || performance == nullptr
        || detectionSummary == nullptr || memorySummary == nullptr
        || httpState == nullptr || connectVideo == nullptr) {
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
            "\"recording\":false,\"session_id\":\"capture-test\","
            "\"segment\":\"raw.avi\",\"recorded_frames\":7,"
            "\"snapshot_count\":2,\"queue_bytes\":1234,"
            "\"max_queue_bytes\":67108864,\"free_disk_bytes\":2147483648,"
            "\"last_error\":null},"
            "\"inference\":{\"configured\":true,\"control_supported\":true,"
            "\"operation\":null,\"state\":\"running\","
            "\"artifact_name\":\"lab_pool_d2_seed42_e20.onnx\","
            "\"model_sha256\":\"3dea74511bf2aabbccddeeff00112233445566778899aabbccddeeff00112233\",\"latest_frame_id\":0,"
            "\"capture_timestamp_ns\":987654321,\"processed_frames\":99,"
            "\"skipped_frames\":0,\"inference_fps\":12.3,"
            "\"latency_ms\":45.6,\"detection_count\":0,"
            "\"vision_process_rss_bytes\":327786496,"
            "\"system_total_memory_bytes\":4089446400,"
            "\"confidence_threshold\":0.75,\"last_error\":null}}"));
    for (int i = 0; i < 50 && state->text() != QStringLiteral("Inference Running");
         ++i) {
        waitForMs(5);
    }
    expect(state->text() == QStringLiteral("Inference Running"),
           "running inference status is rendered in the state label");
    auto *inferenceDot =
        window.findChild<QLabel *>(QStringLiteral("inferenceStatusDot"));
    expect(inferenceDot != nullptr
               && inferenceDot->styleSheet().contains(QStringLiteral("#2F80ED")),
           "running inference uses the dedicated blue status light");
    expect(controlClient.status().inferenceModelSha256 == fullSha
               && controlClient.status().inferenceModelSha256.size() == 64,
           "parsed inference status retains the complete model SHA-256");
    expect(diagnostics->text().contains(QStringLiteral("Fresh"))
               && diagnostics->text().contains(QStringLiteral("Artifact: lab_pool_d2_seed42_e20.onnx"))
               && diagnostics->text().contains(fullSha)
               && diagnostics->text().contains(QStringLiteral("Confidence threshold: 0.75"))
               && diagnostics->text().contains(QStringLiteral("FPS: 12.3"))
               && diagnostics->text().contains(QStringLiteral("Latency ms: 45.6"))
               && diagnostics->text().contains(QStringLiteral("Latest inference frame: 0"))
               && diagnostics->text().contains(QStringLiteral("Capture timestamp ns: 987654321"))
               && diagnostics->text().contains(QStringLiteral("Processed frames: 99"))
               && diagnostics->text().contains(QStringLiteral("Detection count: 0")),
           "Vision Details preserve every authoritative inference field including zero values");
    expect(performance->text() == QStringLiteral("12.3 FPS / 45.6 ms")
               && detectionSummary->text() == QStringLiteral("Detections 0")
               && memorySummary->text() == QStringLiteral(
                   "Memory 312.6 MiB / 3.8 GiB")
               && memorySummary->toolTip() == QStringLiteral(
                   "Vision process RSS / total system physical memory")
               && httpState->text() == QStringLiteral("HTTP: Reachable")
               && performance->toolTip() == QStringLiteral(
                   "Camera capture to inference completion; not ORT-only duration."),
           "main inference row renders active metrics with frozen formatting and tooltip");
    expect(visionDiagnostics->text().contains(QStringLiteral("RX FPS:"))
               && visionDiagnostics->text().contains(QStringLiteral("Display paint-event FPS:"))
               && !panel->findChild<QLabel *>(QStringLiteral("visionDiagnostics")),
           "full video diagnostics live only in Vision Details");
    expect(endpointDetails->text().contains(
               QStringLiteral("Vision HTTP endpoint: http://127.0.0.1:%1")
                   .arg(controlServer.serverPort())),
           "Vision Details retains the test-injected HTTP control port");
    auto *captureDetails = window.findChild<QLabel *>(QStringLiteral("captureDiagnostics"));
    expect(captureDetails != nullptr
               && captureDetails->text().contains(QStringLiteral("Session: capture-test"))
               && captureDetails->text().contains(QStringLiteral("Segment: raw.avi"))
               && captureDetails->text().contains(QStringLiteral("Recorded frames: 7"))
               && captureDetails->text().contains(QStringLiteral("Snapshots: 2"))
               && captureDetails->text().contains(QStringLiteral("Queue bytes: 1234 / 67108864")),
           "Vision Details preserve capture session, segment, counters, and queue data");
    expect(transport.writes().isEmpty(),
           "inference status updates emit no robot-control transport writes");

    controlClient.stopPolling();
    waitForMs(3600);
    QApplication::processEvents();
    expect(performance->text() == QStringLiteral("-- FPS / -- ms")
               && detectionSummary->text() == QStringLiteral("Detections --")
               && memorySummary->text() == QStringLiteral("Memory -- / --")
               && httpState->text() == QStringLiteral("HTTP: Stale"),
           "stale status masks retained inference metrics on the main card");
    expect(diagnostics->text().contains(QStringLiteral("Last received / stale"))
               && diagnostics->text().contains(QStringLiteral("FPS: 12.3"))
               && captureDetails->text().contains(QStringLiteral("Last received / stale"))
               && captureDetails->text().contains(QStringLiteral("Recorded frames: 7")),
           "stale status keeps last-received inference and capture values in Details");

    controlClient.requestFailed(QStringLiteral("Start Inference"),
                                QStringLiteral("timeout"),
                                QStringLiteral("simulated timeout detail"),
                                true);
    QApplication::processEvents();
    auto *notice = window.findChild<QLabel *>(QStringLiteral("visionControlMessage"));
    auto *controlDetails = window.findChild<QLabel *>(QStringLiteral("controlResponseDetails"));
    expect(notice != nullptr && notice->isVisible()
               && notice->text().startsWith(QStringLiteral("Control:"))
               && notice->toolTip().contains(QStringLiteral("simulated timeout detail"))
               && controlDetails != nullptr
               && controlDetails->text().contains(QStringLiteral("Failure: Start Inference")),
           "local HTTP failure is visible with Control prefix and full Details text");
    const QString lastControlDetail =
        controlDetails != nullptr ? controlDetails->text() : QString();
    controlClient.authoritativeStatusRefreshed();
    QApplication::processEvents();
    expect(notice != nullptr && !notice->isVisible(),
           "empty authoritative control response hides the notice row");
    expect(controlDetails != nullptr
               && controlDetails->text() == lastControlDetail
               && controlDetails->text().contains(QStringLiteral("Failure: Start Inference")),
           "authoritative reconciliation hides the notice without erasing the last local control detail");

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
    expect(memorySummary->text() == QStringLiteral("Memory -- / --")
               && httpState->text() == QStringLiteral("HTTP: Reachable"),
           "starting inference masks memory while preserving HTTP reachability");

    applyInferenceStatus(
        QByteArrayLiteral(
            "{\"state\":\"failed\",\"last_error\":\"model load failed\"}"));
    expect(state->text() == QStringLiteral("Inference Error")
               && memorySummary->text() == QStringLiteral("Memory -- / --")
               && diagnostics->text().contains(
                   QStringLiteral("Last error: model load failed")),
           "failed inference status renders a short last error");

    applyInferenceStatus(QByteArrayLiteral(
        "{\"state\":\"running\",\"vision_process_rss_bytes\":327786496}"));
    expect(memorySummary->text() == QStringLiteral("Memory 312.6 MiB / --"),
           "active inference renders RSS when total RAM is unavailable");
    applyInferenceStatus(QByteArrayLiteral(
        "{\"state\":\"running\",\"system_total_memory_bytes\":4089446400}"));
    expect(memorySummary->text() == QStringLiteral("Memory -- / 3.8 GiB"),
           "active inference renders total RAM when RSS is unavailable");
    applyInferenceStatus(QByteArrayLiteral("{\"state\":\"unsupported\"}"));
    expect(state->text() == QStringLiteral("Inference Unavailable")
               && memorySummary->text() == QStringLiteral("Memory -- / --")
               && diagnostics->text().contains(QStringLiteral("FPS: --"))
               && diagnostics->text().contains(QStringLiteral("Latest inference frame: --"))
               && diagnostics->text().contains(QStringLiteral("Detection count: --"))
               && diagnostics->text().contains(QStringLiteral("Skipped frames: --")),
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
               && refresh != nullptr && !refresh->isVisible()
               && start != nullptr && start->isVisible()
               && stop != nullptr && !stop->isVisible(),
           "Task 02 exposes one visible inference Start/Stop toggle while redundant Refresh Status stays hidden");
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
           "running HTTP-only status retains compatibility Clear/Stop action");
    if (host != nullptr && stop != nullptr) {
        host->setText(QStringLiteral("127.0.0.2"));
        expect(start->isEnabled()
                   && start->text() == QStringLiteral("Stop Inference")
                   && !refresh->isEnabled()
                   && !stop->isVisible(),
               "dirty Host blocks new actions but keeps the committed inference Stop toggle available");
        host->setText(QStringLiteral("127.0.0.1"));
    }
    controlClient.stopPolling();
    expect(start->text() == QStringLiteral("Stop Inference")
               && start->isEnabled(),
           "running inference is represented by one visible Stop toggle");
    start->click();
    expect(waitUntil([&server] { return server.hasPendingConnections(); }),
           "the visible inference toggle reaches Stop when authoritative state is running");
    QTcpSocket *stopActionPeer = server.nextPendingConnection();
    if (stopActionPeer != nullptr) {
        expect(readHttpRequest(stopActionPeer).startsWith(
                   "POST /api/v1/vision/inference/stop HTTP/1.1"),
               "running inference toggle uses the frozen Stop endpoint");
        sendHttpJson(
            stopActionPeer,
            QByteArrayLiteral(
                "{\"ok\":true,\"action\":\"inference/stop\","
                "\"outcome\":\"accepted\"}"));
    }

    QTcpSocket *stopRefreshPeer = nullptr;
    expect(waitUntil([&server] { return server.hasPendingConnections(); }),
           "Stop ACK schedules one authoritative reconciliation GET");
    if (server.hasPendingConnections()) {
        stopRefreshPeer = server.nextPendingConnection();
        expect(readHttpRequest(stopRefreshPeer).startsWith(
                   "GET /api/v1/vision/status HTTP/1.1"),
               "Stop reconciliation uses authoritative status GET");
        sendHttpJson(
            stopRefreshPeer,
            QByteArrayLiteral(
                "{\"ok\":true,\"camera\":{\"running\":true},"
                "\"capture\":{\"state\":\"idle\",\"recording\":false},"
                "\"inference\":{\"configured\":true,"
                "\"control_supported\":true,\"operation\":null,"
                "\"state\":\"disabled\"}}"));
    }
    expect(waitUntil([&] {
        return start->text() == QStringLiteral("Start Inference")
            && start->isEnabled();
    }), "authoritative disabled status returns the same toggle to Start Inference");

    expect(transport.writes().isEmpty(),
           "HTTP-only inference controls emit no Robot writes");
    window.close();
    if (statusPeer != nullptr) statusPeer->deleteLater();
    if (actionPeer != nullptr) actionPeer->deleteLater();
    if (refreshPeer != nullptr) refreshPeer->deleteLater();
    if (stopActionPeer != nullptr) stopActionPeer->deleteLater();
    if (stopRefreshPeer != nullptr) stopRefreshPeer->deleteLater();
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
            && (message->text().contains(QStringLiteral("worker is stopping"))
                || message->toolTip().contains(QStringLiteral("worker is stopping")));
    }), "typed inference error is shown in the Vision control message");
    expect(message != nullptr && message->text().startsWith(QStringLiteral("Control:")),
           "local HTTP action failure uses the Control notice prefix");
    expect(captureState->text() != QStringLiteral("Capture Error"),
           "inference error does not become Capture Error");
    answerNextMessageBox(QMessageBox::Yes);
    window.close();
    if (peer != nullptr) peer->deleteLater();
    if (action != nullptr) action->deleteLater();
}

void testTask03VisionNoticePrefixes()
{
    rb::FakeTransport transport;
    rb::RobotController controller(
        &transport, rb::RobotControllerConfig::bringUpProvisional());
    rb::vision::VisionControlClient controlClient;
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "notice-prefix server must listen");
    controlClient.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    rb::MainWindow window(&controller, nullptr, &controlClient);
    window.show();
    QApplication::processEvents();
    auto *refresh = window.findChild<QPushButton *>(QStringLiteral("visionRefreshStatusButton"));
    auto *message = window.findChild<QLabel *>(QStringLiteral("visionControlMessage"));
    expect(refresh != nullptr && message != nullptr,
           "notice-prefix test exposes Refresh and notice widgets");
    if (refresh == nullptr || message == nullptr) {
        return;
    }

    const auto sendStatus = [&](const QByteArray &body) {
        refresh->click();
        expect(waitUntil([&server] { return server.hasPendingConnections(); }),
               "notice-prefix status request connects");
        QTcpSocket *peer = server.nextPendingConnection();
        if (peer == nullptr) {
            return;
        }
        readHttpRequest(peer);
        sendHttpJson(peer, body);
        expect(waitUntil([&controlClient] {
            return !controlClient.requestInFlight() && controlClient.hasFreshStatus();
        }), "notice-prefix status request completes authoritatively");
        peer->deleteLater();
    };

    sendStatus(QByteArrayLiteral(
        "{\"ok\":true,\"camera\":{\"running\":true},"
        "\"capture\":{\"state\":\"idle\",\"recording\":false,"
        "\"last_error\":\"disk full\"},"
        "\"inference\":{\"configured\":true,\"control_supported\":true,"
        "\"operation\":null,\"state\":\"failed\","
        "\"last_error\":\"model load failed\"}}"));
    expect(message->isVisible() && message->text().startsWith(QStringLiteral("Inference:")),
           "inference status error takes the Inference notice prefix");

    sendStatus(QByteArrayLiteral(
        "{\"ok\":true,\"camera\":{\"running\":true},"
        "\"capture\":{\"state\":\"idle\",\"recording\":false,"
        "\"last_error\":\"disk full\"},"
        "\"inference\":{\"configured\":true,\"control_supported\":true,"
        "\"operation\":null,\"state\":\"disabled\","
        "\"last_error\":null}}"));
    expect(message->isVisible() && message->text().startsWith(QStringLiteral("Capture:")),
           "capture status error takes the Capture notice prefix");

    sendStatus(QByteArrayLiteral(
        "{\"ok\":true,\"camera\":{\"running\":true},"
        "\"capture\":{\"state\":\"idle\",\"recording\":false,"
        "\"last_error\":null},"
        "\"inference\":{\"configured\":true,\"control_supported\":true,"
        "\"operation\":null,\"state\":\"disabled\","
        "\"last_error\":null}}"));
    controlClient.errorOccurred(QStringLiteral("request timed out"));
    QApplication::processEvents();
    expect(message->isVisible() && message->text().startsWith(QStringLiteral("Control:")),
           "local action error takes the Control notice prefix");
    window.close();
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


void testSlice5DetectionTextOverlayLifecycle()
{
    rb::FakeTransport transport;
    rb::RobotController controller(
        &transport,
        rb::RobotControllerConfig::bringUpProvisional());
    rb::vision::VisionClient visionClient;
    rb::vision::VisionControlClient controlClient;
    rb::vision::DetectionClient detectionClient;

    QTcpServer videoServer;
    QTcpServer controlServer;
    QTcpServer detectionServer;
    expect(videoServer.listen(QHostAddress::LocalHost, 0),
           "Slice 5 fake RBVS server must listen");
    expect(controlServer.listen(QHostAddress::LocalHost, 0),
           "Slice 5 fake Vision HTTP server must listen");
    expect(detectionServer.listen(QHostAddress::LocalHost, 0),
           "Slice 5 fake detection server must listen");
    if (!videoServer.isListening()
        || !controlServer.isListening()
        || !detectionServer.isListening()) {
        return;
    }

    controlClient.setEndpoint(
        QStringLiteral("127.0.0.1"),
        controlServer.serverPort());

    rb::MainWindow window(
        &controller,
        &visionClient,
        &controlClient,
        &detectionClient);
    auto *view = window.findChild<rb::vision::VideoView *>(
        QStringLiteral("videoView"));
    expect(view != nullptr,
           "Slice 5 integration retains the single VideoView");
    if (view == nullptr) {
        return;
    }

    QTemporaryDir csvDirectory;
    auto *csvEnabled = window.findChild<QCheckBox *>(QStringLiteral("visualCsvEnabled"));
    auto *csvFolder = window.findChild<QLineEdit *>(QStringLiteral("visualCsvDirectory"));
    auto *csvBrowse = window.findChild<QPushButton *>(QStringLiteral("visualCsvBrowse"));
    auto *csvStatus = window.findChild<QLabel *>(QStringLiteral("visualCsvStatus"));
    expect(csvEnabled && !csvEnabled->isChecked(), "CSV recording defaults to off");
    expect(csvFolder && csvFolder->text() == QDir(
               QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation))
                   .filePath(QStringLiteral("RoboBeetle/visual-logs")),
           "CSV defaults to the user's Documents/RoboBeetle/visual-logs");
    expect(csvBrowse && csvBrowse->isEnabled(), "CSV directory is user selectable");
    expect(csvStatus && csvStatus->text().contains(QStringLiteral("OFF")),
           "CSV off state is explicit");
    if (csvEnabled && csvFolder && csvBrowse && csvStatus && csvDirectory.isValid()) {
        csvFolder->setText(csvDirectory.path());
        csvEnabled->setChecked(true);
        expect(csvEnabled->isChecked() && !csvFolder->isEnabled() && !csvBrowse->isEnabled()
                   && csvStatus->text().contains(QStringLiteral("Recording")),
               "CSV recording locks directory editing and displays active file");
    }

    visionClient.connectToHost(
        QStringLiteral("127.0.0.1"),
        videoServer.serverPort());
    expect(waitUntil([&] { return videoServer.hasPendingConnections(); }),
           "Slice 5 video session connects to loopback RBVS server");
    QTcpSocket *videoPeer = videoServer.nextPendingConnection();
    expect(waitUntil([&] { return visionClient.isConnected(); }),
           "Slice 5 loopback RBVS session reaches Connected");

    // An old Pi / legacy status must not cause speculative 47012 traffic.
    controlClient.refreshStatus();
    expect(waitUntil([&] { return controlServer.hasPendingConnections(); }),
           "legacy capability status GET reaches loopback HTTP server");
    QTcpSocket *legacyStatusPeer = controlServer.nextPendingConnection();
    if (legacyStatusPeer != nullptr) {
        readHttpRequest(legacyStatusPeer);
        sendHttpJson(
            legacyStatusPeer,
            slice5StatusBody(
                detectionServer.serverPort(),
                QStringLiteral("running"),
                false));
        legacyStatusPeer->deleteLater();
    }
    expect(waitUntil([&] { return controlClient.hasFreshStatus(); }),
           "legacy status remains a valid fresh Vision status");
    waitForMs(60);
    expect(!detectionServer.hasPendingConnections()
               && detectionClient.state()
                   == rb::vision::DetectionConnectionState::Disconnected,
           "missing detection capability never probes 47012");

    // A future/unknown metadata version is advertised but must fail closed.
    controlClient.refreshStatus();
    expect(waitUntil([&] { return controlServer.hasPendingConnections(); }),
           "future-version capability status GET reaches loopback HTTP server");
    QTcpSocket *futureVersionPeer = controlServer.nextPendingConnection();
    if (futureVersionPeer != nullptr) {
        readHttpRequest(futureVersionPeer);
        sendHttpJson(
            futureVersionPeer,
            slice5StatusBody(
                detectionServer.serverPort(),
                QStringLiteral("running"),
                true,
                rb::vision::kDetectionStreamVersion + 1));
        futureVersionPeer->deleteLater();
    }
    expect(waitUntil([&] { return controlClient.hasFreshStatus(); }),
           "future-version capability status remains a valid HTTP status");
    waitForMs(60);
    expect(!detectionServer.hasPendingConnections()
               && detectionClient.state()
                   == rb::vision::DetectionConnectionState::Disconnected,
           "unsupported detection stream version never probes 47012");

    // Fresh authoritative support/version/port enables exactly one attempt.
    controlClient.refreshStatus();
    expect(waitUntil([&] { return controlServer.hasPendingConnections(); }),
           "Slice 5 capability status GET reaches loopback HTTP server");
    QTcpSocket *supportedStatusPeer = controlServer.nextPendingConnection();
    if (supportedStatusPeer != nullptr) {
        readHttpRequest(supportedStatusPeer);
        sendHttpJson(
            supportedStatusPeer,
            slice5StatusBody(detectionServer.serverPort()));
        supportedStatusPeer->deleteLater();
    }
    expect(waitUntil([&] { return detectionServer.hasPendingConnections(); }),
           "fresh supported v1 capability opens detection metadata stream");
    QTcpSocket *detectionPeer = detectionServer.nextPendingConnection();
    expect(waitUntil([&] { return detectionClient.isConnected(); }),
           "MainWindow-owned detection client reaches Connected");

    const auto parsedCapability = controlClient.status();
    expect(parsedCapability.detectionStreamSupported.has_value()
               && *parsedCapability.detectionStreamSupported
               && parsedCapability.detectionStreamPort.has_value()
               && *parsedCapability.detectionStreamPort
                   == detectionServer.serverPort()
               && parsedCapability.detectionStreamVersion.has_value()
               && *parsedCapability.detectionStreamVersion
                   == rb::vision::kDetectionStreamVersion,
           "MainWindow consumes the authoritative advertised detection endpoint");
    expect(transport.writes().isEmpty(),
           "opening detection metadata emits zero Robot transport writes");

    if (detectionPeer != nullptr) {
        detectionPeer->write(
            slice5DetectionLine(
                10U,
                1'000'000'000ULL,
                QStringLiteral("fish"),
                0.88));
        detectionPeer->flush();
    }
    expect(waitUntil([&] { return detectionClient.lastFrameId() == 10U; }),
           "real loopback NDJSON reaches DetectionClient");
    expect(!view->hasDetectionOverlay(),
           "metadata alone cannot overlay before a timestamped video frame");

    QImage live(640, 480, QImage::Format_RGB32);
    live.fill(qRgb(16, 24, 32));
    visionClient.frameReady(
        live,
        11U,
        1'100'000'000ULL);
    expect(waitUntil([&] {
        return view->hasDetectionOverlay()
            && view->detectionOverlayCount() == 1;
    }), "fresh frame-associated detection text reaches VideoView");
    expect(view->currentTargetState()
               && std::abs(view->currentTargetState()->ex + 0.625) < 1e-9
               && std::abs(view->currentTargetState()->ey + 1.0 / 3.0) < 1e-9
               && view->visualDiagnosticText().contains(QStringLiteral("VISION DRY_RUN")),
           "live metadata calculates original-pixel errors through the existing UI gate");
    {
        auto *screen = window.findChild<QLabel *>(QStringLiteral("visionDiagnosticScreen"));
        expect(screen != nullptr && screen->text() == view->visualDiagnosticText()
                   && screen->text().contains(QStringLiteral("VISION DRY_RUN")),
               "the diagnostic screen shows the same text as visualDiagnosticText()");
    }
    expect(transport.writes().isEmpty(),
           "visual error calculation sends no robot commands");
    expect(view->visualDiagnosticText().contains(QStringLiteral("TRACKING"))
               && view->visualDiagnosticText().contains(QStringLiteral("PROPOSED (not sent): TURN_LEFT")),
           "real metadata produces a display-only left proposal");

    // Independent TCP streams: a newer detection may precede its video image.
    if (detectionPeer) {
        detectionPeer->write(slice5DetectionLine(11U, 1'200'000'000ULL,
                                                QStringLiteral("fish"), 0.88, 135.0, 175.0));
        detectionPeer->flush();
    }
    expect(waitUntil([&] { return detectionClient.lastFrameId() == 11U; }),
           "ahead-of-video detection arrives on the independent stream");
    expect(view->currentVisualDiagnostic() && view->currentVisualDiagnostic()->awaitingVideo
               && view->currentVisualDiagnostic()->command.effective == rb::vision::ProposedCommand::TurnLeft
               && !view->hasDetectionOverlay(),
           "AwaitingVideo preserves the proposal but keeps original raw overlay suppression");
    waitForMs(15);
    visionClient.frameReady(live, 12U, 1'200'000'000ULL);
    expect(view->currentVisualDiagnostic() && !view->currentVisualDiagnostic()->awaitingVideo
               && view->currentVisualDiagnostic()->command.ex_f
               && std::abs(*view->currentVisualDiagnostic()->command.ex_f + 0.6109375) < 1e-9
               && view->currentVisualDiagnostic()->command.effective == rb::vision::ProposedCommand::TurnLeft,
           "15 ms video catchup makes one EMA update and never inserts STOP");
    visionClient.frameReady(live, 12U, 1'200'000'000ULL);
    expect(view->currentVisualDiagnostic()->command.ex_f
               && std::abs(*view->currentVisualDiagnostic()->command.ex_f + 0.6109375) < 1e-9,
           "repeated video rendering does not repeat the EMA update");

    // Exercise the real Stop Inference button, POST ACK and authoritative GET.
    auto *inferenceToggle = window.findChild<QPushButton *>(
        QStringLiteral("startInferenceButton"));
    expect(inferenceToggle && inferenceToggle->isEnabled()
               && inferenceToggle->text() == QStringLiteral("Stop Inference"),
           "running inference exposes the existing Stop Inference action");
    if (inferenceToggle) {
        inferenceToggle->click();
    }
    expect(waitUntil([&] { return controlServer.hasPendingConnections(); }),
           "Stop Inference reaches the Vision HTTP server");
    QTcpSocket *stopInferencePeer = controlServer.nextPendingConnection();
    if (stopInferencePeer) {
        expect(readHttpRequest(stopInferencePeer).startsWith(
                   "POST /api/v1/vision/inference/stop HTTP/1.1"),
               "Stop Inference uses its existing dedicated endpoint");
        sendHttpJson(stopInferencePeer, QByteArrayLiteral(
            "{\"ok\":true,\"action\":\"inference/stop\","
            "\"outcome\":\"already_stopping\"}"));
        stopInferencePeer->deleteLater();
    }
    expect(waitUntil([&] { return controlClient.inferenceReconcilePending(); }),
           "Stop ACK waits for authoritative inference status");
    expect(!view->currentTargetState()
               && view->visualDiagnosticText().contains(QStringLiteral("STALE")),
           "Stop acknowledgement clears errors while awaiting reconciliation");
    expect(waitUntil([&] { return controlServer.hasPendingConnections(); }),
           "Stop acknowledgement triggers the existing status GET");
    QTcpSocket *stopStatusPeer = controlServer.nextPendingConnection();
    if (stopStatusPeer) {
        expect(readHttpRequest(stopStatusPeer).startsWith(
                   "GET /api/v1/vision/status HTTP/1.1"),
               "Stop reconciliation reads authoritative status");
        QJsonObject body = QJsonDocument::fromJson(
            slice5StatusBody(detectionServer.serverPort())).object();
        QJsonObject inference = body.value(QStringLiteral("inference")).toObject();
        inference.insert(QStringLiteral("operation"), QStringLiteral("stopping"));
        body.insert(QStringLiteral("inference"), inference);
        sendHttpJson(stopStatusPeer, QJsonDocument(body).toJson(QJsonDocument::Compact));
        stopStatusPeer->deleteLater();
    }
    expect(waitUntil([&] {
        return controlClient.hasFreshStatus() && !controlClient.inferenceReconcilePending()
            && controlClient.status().inferenceOperation == QStringLiteral("stopping");
    }), "fresh stopping status becomes authoritative");
    expect(!view->currentTargetState() && !view->hasDetectionOverlay()
               && view->visualDiagnosticText().contains(QStringLiteral("INFERENCE_OFF"))
               && view->visualDiagnosticText().contains(QStringLiteral("ex=-- ey=--")),
           "Stop Inference clears errors before the worker finishes stopping");

    controlClient.refreshStatus();
    expect(waitUntil([&] { return controlServer.hasPendingConnections(); }),
           "running status can be confirmed after Stop test");
    QTcpSocket *resumePeer = controlServer.nextPendingConnection();
    if (resumePeer) {
        readHttpRequest(resumePeer);
        sendHttpJson(resumePeer, slice5StatusBody(detectionServer.serverPort()));
        resumePeer->deleteLater();
    }
    expect(waitUntil([&] { return controlClient.status().inferenceState == QStringLiteral("running")
               && controlClient.status().inferenceOperation.isEmpty(); }),
           "fresh authoritative running state is received after stopping");
    expect(!view->currentTargetState(), "running status cannot reacquire a released old ID after Stop");
    if (detectionPeer) {
        QJsonObject empty = QJsonDocument::fromJson(
            slice5DetectionLine(12U, 1'000'000'000ULL)).object();
        empty.insert(QStringLiteral("detections"), QJsonArray{});
        detectionPeer->write(QJsonDocument(empty).toJson(QJsonDocument::Compact) + '\n');
        detectionPeer->flush();
    }
    expect(waitUntil([&] { return detectionClient.lastFrameId() == 12U; }),
           "fresh empty detection result is received");
    expect(!view->currentTargetState()
               && view->visualDiagnosticText().contains(QStringLiteral("NO_TARGET")),
           "fresh empty detections have a distinct NO_TARGET reason");
    if (detectionPeer) {
        detectionPeer->write(slice5DetectionLine(13U, 1'000'000'000ULL));
        detectionPeer->flush();
    }
    expect(waitUntil([&] { return view->currentTargetState().has_value(); }),
           "fresh nonempty result restores the diagnostic target");

    controlClient.refreshStatus();
    expect(waitUntil([&] { return controlServer.hasPendingConnections(); }),
           "unknown-worker status GET reaches the HTTP server");
    QTcpSocket *unknownStatePeer = controlServer.nextPendingConnection();
    if (unknownStatePeer) {
        readHttpRequest(unknownStatePeer);
        sendHttpJson(unknownStatePeer, slice5StatusBody(
            detectionServer.serverPort(), QStringLiteral("unsupported")));
        unknownStatePeer->deleteLater();
    }
    expect(waitUntil([&] {
        return controlClient.status().inferenceState == QStringLiteral("unsupported");
    }), "unknown worker state is received without pretending it is disabled");
    expect(!view->currentTargetState()
               && view->visualDiagnosticText().contains(QStringLiteral("STALE"))
               && !view->visualDiagnosticText().contains(QStringLiteral("INFERENCE_OFF")),
           "unknown inference worker state is STALE rather than confirmed off");

    controlClient.refreshStatus();
    expect(waitUntil([&] { return controlServer.hasPendingConnections(); }),
           "known running status can recover from an unknown state");
    QTcpSocket *knownStatePeer = controlServer.nextPendingConnection();
    if (knownStatePeer) {
        readHttpRequest(knownStatePeer);
        sendHttpJson(knownStatePeer, slice5StatusBody(detectionServer.serverPort()));
        knownStatePeer->deleteLater();
    }
    expect(waitUntil([&] { return controlClient.status().inferenceState == QStringLiteral("running"); }),
           "known running status recovers after unknown state");
    expect(!view->currentTargetState(), "known status cannot reacquire a released old ID after STALE");
    expect(transport.writes().isEmpty(),
           "Stop Inference and display reason transitions send zero robot bytes");

    // HTTP freshness controls visibility but must not tear down a healthy 47012.
    waitForMs(3600);
    expect(!controlClient.hasFreshStatus(),
           "Vision HTTP freshness expires without polling");
    expect(!view->hasDetectionOverlay(),
           "stale HTTP status immediately suppresses overlay text");
    expect(!view->currentTargetState()
               && view->visualDiagnosticText().contains(QStringLiteral("STALE")),
           "stale inference status clears the selected target and visual errors");
    expect(detectionClient.isConnected(),
           "HTTP freshness expiry preserves the established 47012 socket");

    // Running state is an explicit UI gate independent of the metadata socket.
    controlClient.refreshStatus();
    expect(waitUntil([&] { return controlServer.hasPendingConnections(); }),
           "disabled-inference status GET reaches loopback HTTP server");
    QTcpSocket *disabledStatusPeer = controlServer.nextPendingConnection();
    if (disabledStatusPeer != nullptr) {
        readHttpRequest(disabledStatusPeer);
        sendHttpJson(
            disabledStatusPeer,
            slice5StatusBody(
                detectionServer.serverPort(),
                QStringLiteral("disabled")));
        disabledStatusPeer->deleteLater();
    }
    expect(waitUntil([&] {
        return controlClient.hasFreshStatus()
            && controlClient.status().inferenceState
                == QStringLiteral("disabled");
    }), "disabled inference status becomes authoritative");
    expect(waitUntil([&] { return !view->hasDetectionOverlay(); }),
           "non-running inference clears operator overlay text");
    expect(!view->currentTargetState()
               && view->visualDiagnosticText().contains(QStringLiteral("INFERENCE_OFF"))
               && view->visualDiagnosticText().contains(QStringLiteral("ex=-- ey=--")),
           "fresh disabled inference is distinguished from empty detections and stale data");

    // Fresh HTTP alone cannot resurrect locally expired detection metadata.
    controlClient.refreshStatus();
    expect(waitUntil([&] { return controlServer.hasPendingConnections(); }),
           "running-inference status GET reaches loopback HTTP server");
    QTcpSocket *runningStatusPeer = controlServer.nextPendingConnection();
    if (runningStatusPeer != nullptr) {
        readHttpRequest(runningStatusPeer);
        sendHttpJson(
            runningStatusPeer,
            slice5StatusBody(detectionServer.serverPort()));
        runningStatusPeer->deleteLater();
    }
    expect(waitUntil([&] {
        return controlClient.hasFreshStatus()
            && controlClient.status().inferenceState
                == QStringLiteral("running");
    }), "running inference status becomes authoritative");
    expect(!view->hasDetectionOverlay()
               && view->visualDiagnosticText().contains(QStringLiteral("STALE")),
           "running HTTP recovery cannot revive an expired local detection ID");
    if (detectionPeer) {
        detectionPeer->write(slice5DetectionLine(14U, 1'000'000'000ULL));
        detectionPeer->flush();
    }
    expect(waitUntil([&] { return view->hasDetectionOverlay(); }),
           "a new detection ID restores errors after local timeout");

    // A newer live frame more than 1500 ms ahead must suppress stale metadata.
    visionClient.frameReady(
        live,
        12U,
        2'500'000'001ULL);
    expect(waitUntil([&] { return !view->hasDetectionOverlay(); }),
           "metadata older than 1500 ms is suppressed on the latest video frame");
    expect(!view->currentTargetState(),
           "expired metadata cannot keep stale visual errors");

    // New metadata for the current time becomes visible without replaying old video.
    if (detectionPeer != nullptr) {
        detectionPeer->write(
            slice5DetectionLine(
                15U,
                2'500'000'000ULL,
                QStringLiteral("penguin"),
                0.91));
        detectionPeer->flush();
    }
    expect(waitUntil([&] {
        return detectionClient.lastFrameId() == 15U
            && view->hasDetectionOverlay();
    }), "new fresh metadata overlays the current latest video frame");

    // HTTP freshness expiry is display fail-closed, not a metadata disconnect.
    expect(waitUntil([&] {
        return !controlClient.hasFreshStatus();
    }, 4200), "Vision status naturally expires after the 3500 ms freshness window");
    expect(!view->hasDetectionOverlay(),
           "stale Vision HTTP status hides detection text immediately");
    expect(detectionClient.isConnected(),
           "HTTP freshness expiry keeps the established 47012 socket alive");
    expect(!detectionServer.hasPendingConnections(),
           "freshness expiry does not create a replacement 47012 connection");

    controlClient.refreshStatus();
    expect(waitUntil([&] { return controlServer.hasPendingConnections(); }),
           "freshness recovery GET reaches loopback HTTP server");
    QTcpSocket *freshnessRecoveryPeer = controlServer.nextPendingConnection();
    if (freshnessRecoveryPeer != nullptr) {
        readHttpRequest(freshnessRecoveryPeer);
        sendHttpJson(
            freshnessRecoveryPeer,
            slice5StatusBody(detectionServer.serverPort()));
        freshnessRecoveryPeer->deleteLater();
    }
    expect(waitUntil([&] {
        return controlClient.hasFreshStatus();
    }), "fresh status recovery reaches the original session");
    expect(!view->hasDetectionOverlay(),
           "fresh status cannot renew the old frame's local wall-clock lifetime");
    if (detectionPeer) {
        detectionPeer->write(slice5DetectionLine(16U, 2'500'000'000ULL));
        detectionPeer->flush();
    }
    expect(waitUntil([&] { return view->hasDetectionOverlay(); }),
           "new metadata is required after HTTP and local watchdog expiry");
    expect(detectionClient.isConnected()
               && !detectionServer.hasPendingConnections(),
           "freshness recovery reuses the original 47012 socket");

    // Freeze both Pi timestamps while HTTP remains fresh: local timer must clear errors.
    expect(waitUntil([&] {
        return view->visualDiagnosticText().contains(QStringLiteral("STALE"))
            && view->visualDiagnosticText().contains(QStringLiteral("PROPOSED (not sent): STOP"));
    }, 700), "local 500 ms watchdog stops the proposal without a new video/status event");
    expect(controlClient.hasFreshStatus() && !view->currentTargetState(),
           "local expiry applies even with fresh HTTP and clears original errors");
    QElapsedTimer emptyResultsClock;
    for (quint64 id = 17U; id <= 33U; ++id) {
        if (detectionPeer) {
            QJsonObject empty = QJsonDocument::fromJson(
                slice5DetectionLine(id, 2'500'000'000ULL)).object();
            empty.insert(QStringLiteral("detections"), QJsonArray{});
            detectionPeer->write(QJsonDocument(empty).toJson(QJsonDocument::Compact) + '\n');
            detectionPeer->flush();
        }
        expect(waitUntil([&] { return detectionClient.lastFrameId() == id; }),
               "continued empty results have advancing detection IDs");
        if (id == 17U) {
            emptyResultsClock.start();
            expect(view->visualDiagnosticText().contains(QStringLiteral("NO_TARGET"))
                       && view->visualDiagnosticText().contains(QStringLiteral("PROPOSED (not sent): HOLD")),
                   "first fresh empty result holds the previous proposal");
        }
        // Coarse timer wakeups may arrive early: count elapsed monotonic time,
        // not seventeen nominal sleeps, while continuing fresh metadata.
        const qint64 nextArrivalMs = static_cast<qint64>(id - 16U) * 100;
        expect(waitUntil([&] { return emptyResultsClock.elapsed() >= nextArrivalMs; }),
               "empty-result cadence reaches its monotonic deadline");
    }
    expect(view->visualDiagnosticText().contains(QStringLiteral("LOST"))
               && view->visualDiagnosticText().contains(QStringLiteral("PROPOSED (not sent): STOP")),
           "continued fresh empty frames reach LOST STOP after 1500 ms");
    expect(transport.writes().isEmpty(), "all proposal states send zero robot bytes");

    // Task04: actual NDJSON on the established independent TCP detection socket.
    controlClient.refreshStatus();
    expect(waitUntil([&]{return controlServer.hasPendingConnections();}), "association experiment refreshes HTTP freshness");
    if(auto *peer=controlServer.nextPendingConnection()) {
        readHttpRequest(peer); sendHttpJson(peer,slice5StatusBody(detectionServer.serverPort())); peer->deleteLater();
    }
    expect(waitUntil([&]{return controlClient.hasFreshStatus();}), "association experiment has fresh status");
    auto sendAssociated=[&](quint64 id,double ca,double cb,bool onlyOther=false,quint64 ts=2'500'000'000ULL) {
        QJsonObject a=QJsonDocument::fromJson(slice5DetectionLine(id,ts,"fish",ca,450,240)).object();
        const auto b=QJsonDocument::fromJson(slice5DetectionLine(id,ts,"other",cb,547,240)).object();
        QJsonArray detections;
        if(!onlyOther)detections.append(a.value("detections").toArray().first());
        detections.append(b.value("detections").toArray().first());
        a.insert("detections",detections);
        if(detectionPeer){detectionPeer->write(QJsonDocument(a).toJson(QJsonDocument::Compact)+'\n');detectionPeer->flush();}
        expect(waitUntil([&]{return detectionClient.lastFrameId()==id;}), "association fixture arrives over TCP");
    };
    sendAssociated(34,.89,.85);
    expect(view->currentTargetState() && view->currentTargetState()->target.originalPoint.x()==450
           && view->currentVisualDiagnostic()->associationStatus==rb::vision::AssociationStatus::Acquired,
           "fresh target after LOST acquires 450");
    for(quint64 id=35;id<=40;++id) {
        sendAssociated(id,id%2?.85:.89,id%2?.89:.85);
        expect(view->currentTargetState() && view->currentTargetState()->target.originalPoint.x()==450
               && view->currentVisualDiagnostic()->highestConfidenceTarget
               && view->currentVisualDiagnostic()->highestConfidenceTarget->target.originalPoint.x()==(id%2?547:450)
               && view->currentVisualDiagnostic()->associationStatus==rb::vision::AssociationStatus::Associated,
               "loopback confidence alternation keeps lock and records jumping highest baseline");
        expect(transport.writes().isEmpty(),"associated suggestions send zero robot bytes");
    }
    sendAssociated(41,.85,.89,true);
    const auto missStart=view->currentVisualDiagnostic()->localMonoMs;
    expect(view->hasDetectionOverlay() && !view->currentTargetState()
           && view->currentVisualDiagnostic()->state==rb::vision::VisualState::NoTarget
           && view->currentVisualDiagnostic()->command.proposed==rb::vision::ProposedCommand::Hold,
           "out-of-gate candidate remains visible and maps to HOLD");
    std::optional<qint64> reacquiredAt;
    for(quint64 id=42;id<=47;++id) {
        waitForMs(100); sendAssociated(id,.85,.89,true);
        if(!reacquiredAt && view->currentVisualDiagnostic()->target)
            reacquiredAt=view->currentVisualDiagnostic()->localMonoMs;
    }
    expect(reacquiredAt && *reacquiredAt-missStart>=500 && *reacquiredAt-missStart<800
           && view->currentTargetState() && view->currentTargetState()->target.originalPoint.x()==547,
           "new advancing ID reacquires other target after local MISS timeout");
    sendAssociated(48,.85,.89,true,3'000'000'000ULL);
    expect(view->currentVisualDiagnostic()->awaitingVideo,"new association metadata waits for video");
    expect(waitUntil([&]{return view->currentVisualDiagnostic()->state==rb::vision::VisualState::Stale;},700),
           "awaiting association metadata expires at local 500 ms watchdog");
    visionClient.frameReady(live,49,3'000'000'000ULL);
    expect(!view->currentTargetState() && view->currentVisualDiagnostic()->associationStatus==rb::vision::AssociationStatus::Unlocked
           && view->currentVisualDiagnostic()->command.effective==rb::vision::ProposedCommand::Stop,
           "late video cannot resurrect association or produce output");
    expect(transport.writes().isEmpty(),"MISS/reacquire/awaiting expiry emit zero robot bytes");

    // Losing only 47012 clears text but leaves video and HTTP state alive.
    if (detectionPeer != nullptr) {
        detectionPeer->disconnectFromHost();
    }
    expect(waitUntil([&] {
        return detectionClient.state()
            == rb::vision::DetectionConnectionState::Disconnected;
    }), "47012 peer disconnect is isolated to metadata client");
    expect(!view->hasDetectionOverlay(),
           "47012 disconnect clears retained overlay text");
    expect(!view->currentTargetState(), "detection disconnect clears visual errors");
    expect(visionClient.isConnected(),
           "47012 disconnect does not disconnect RBVS video");
    expect(controlClient.hasFreshStatus(),
           "47012 disconnect does not invalidate fresh Vision HTTP status");
    expect(transport.writes().isEmpty(),
           "complete detection overlay lifecycle emits zero Robot writes");

    // Re-advertising the same endpoint in this video session must not retry.
    controlClient.refreshStatus();
    expect(waitUntil([&] { return controlServer.hasPendingConnections(); }),
           "post-disconnect status refresh reaches HTTP server");
    QTcpSocket *sameEndpointStatusPeer = controlServer.nextPendingConnection();
    if (sameEndpointStatusPeer != nullptr) {
        readHttpRequest(sameEndpointStatusPeer);
        sendHttpJson(
            sameEndpointStatusPeer,
            slice5StatusBody(detectionServer.serverPort()));
        sameEndpointStatusPeer->deleteLater();
    }
    expect(waitUntil([&] { return controlClient.hasFreshStatus(); }),
           "same-endpoint status refresh remains fresh");
    waitForMs(80);
    expect(!detectionServer.hasPendingConnections(),
           "same video session does not retry a failed 47012 endpoint");

    // A genuinely new video session resets the one-attempt fence.
    visionClient.disconnectFromHost();
    expect(waitUntil([&] { return !visionClient.isConnected(); }),
           "first Slice 5 video session disconnects");
    if (videoPeer != nullptr) {
        videoPeer->deleteLater();
        videoPeer = nullptr;
    }

    visionClient.connectToHost(
        QStringLiteral("127.0.0.1"),
        videoServer.serverPort());
    expect(waitUntil([&] { return videoServer.hasPendingConnections(); }),
           "second Slice 5 video session connects");
    QTcpSocket *secondVideoPeer = videoServer.nextPendingConnection();
    expect(waitUntil([&] { return visionClient.isConnected(); }),
           "second RBVS session reaches Connected");
    expect(waitUntil([&] { return detectionServer.hasPendingConnections(); }),
           "new video session permits one new 47012 connection attempt");
    QTcpSocket *secondDetectionPeer = detectionServer.nextPendingConnection();

    expect(transport.writes().isEmpty(),
           "new detection session still emits zero Robot writes");

    if (csvEnabled && csvFolder && csvBrowse && csvStatus) {
        csvEnabled->setChecked(false);
        expect(csvFolder->isEnabled() && csvBrowse->isEnabled()
                   && csvStatus->text().contains(QStringLiteral("OFF")),
               "stopping CSV closes/flushed output and enables directory editing");
        const auto files = QDir(csvDirectory.path()).entryList({QStringLiteral("*.csv")}, QDir::Files);
        expect(files.size() == 1, "loopback recording creates one CSV outside the repository");
        if (files.size() == 1) {
            QFile csv(QDir(csvDirectory.path()).filePath(files.front()));
            expect(csv.open(QIODevice::ReadOnly), "closed loopback CSV is readable");
            const auto lines = csv.readAll().split('\n');
            expect(!lines.isEmpty() && lines.front().startsWith("row_kind,local_mono_ms,arrival_mono_ms,frame_id"),
                   "CSV publishes frame/transition schema");
            int aheadFrameCount = 0;
            bool tracking = false, noTarget = false, lost = false, stale = false, off = false;
            bool usableErrors = false;
            for (qsizetype i = 1; i < lines.size(); ++i) {
                const auto columns = lines[i].trimmed().split(',');
                if (columns.size() != 48) { continue; }
                expect(columns[21]=="visual-csv-v6","loopback CSV rows identify new schema");
                if (columns[0] == "frame" && columns[3] == "11") {
                    ++aheadFrameCount;
                    expect(columns[6] == "fish" && std::abs(columns[10].toDouble() + 0.578125) < 1e-9
                               && std::abs(columns[12].toDouble() + 0.6109375) < 1e-9
                               && columns[20] == "0" && columns[1].toLongLong() >= columns[2].toLongLong(),
                           "caught-up frame includes the exact new ex/EMA and arrival/evaluation times");
                }
                if (columns[0] == "frame" && columns[3] == "13") {
                    usableErrors = std::abs(columns[10].toDouble() + 0.625) < 1e-9;
                }
                if (columns[0] != "transition") { continue; }
                for (int c = 2; c <= 11; ++c) {
                    expect(columns[c].isEmpty(), "transition CSV leaves frame-related fields blank");
                }
                tracking |= columns[14] == "TRACKING";
                noTarget |= columns[14] == "NO_TARGET" && columns[15] == "HOLD";
                lost |= columns[14] == "LOST" && columns[15] == "STOP";
                stale |= columns[14] == "STALE" && columns[15] == "STOP";
                off |= columns[14] == "INFERENCE_OFF" && columns[15] == "STOP";
            }
            expect(aheadFrameCount == 1, "video catchup does not duplicate CSV frame ID");
            expect(usableErrors, "accepted loopback frame records original normalized ex");
            expect(tracking && noTarget && lost && stale && off,
                   "CSV records all five real loopback state transitions including timer expiry");
        }
        QFile blocker(QDir(csvDirectory.path()).filePath(QStringLiteral("not-a-directory")));
        expect(blocker.open(QIODevice::WriteOnly), "error fixture is a regular file");
        blocker.close();
        csvFolder->setText(blocker.fileName());
        csvEnabled->setChecked(true);
        expect(!csvEnabled->isChecked() && csvFolder->isEnabled()
                   && csvStatus->text().contains(QStringLiteral("Error")),
               "open failure stops CSV and shows the reason without restarting");
        expect(transport.writes().isEmpty(), "CSV start/record/stop/error emit zero robot bytes");
    }

    window.hide();
    detectionClient.shutdown();
    controlClient.shutdown();
    visionClient.shutdown();
    if (secondDetectionPeer != nullptr) {
        secondDetectionPeer->deleteLater();
    }
    if (secondVideoPeer != nullptr) {
        secondVideoPeer->deleteLater();
    }
    if (detectionPeer != nullptr) {
        detectionPeer->deleteLater();
    }
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

    // The compact telemetry rail keeps all four status cards present.
    expect(findGroupBox(&window, QStringLiteral("Leak Detection")) != nullptr,
           "dashboard status grid must keep the Leak Detection card");
    expect(findGroupBox(&window, QStringLiteral("IMU")) != nullptr,
           "dashboard status grid must keep the IMU card");
    expect(findGroupBox(&window, QStringLiteral("Depth Sensor")) != nullptr,
           "dashboard status grid must keep the Depth card");
    expect(findGroupBox(&window, QStringLiteral("Protocol")) != nullptr,
           "dashboard status rail must keep the Protocol card");

    // Actuator Control exists.
    expect(findGroupBox(&window, QStringLiteral("Actuator Control")) != nullptr,
           "MainWindow must expose the Actuator Control panel");

    // Actuator Control is a status-only list; all controls live in Servo Fine Control.
    int servoStatusRows = 0;
    for (int index = 0; index < rb::kServoCount; ++index) {
        QWidget *row = window.findChild<QWidget *>(
            QStringLiteral("servoStatusRow%1").arg(index));
        if (row != nullptr) {
            ++servoStatusRows;
            expect(row->findChildren<QPushButton *>().isEmpty(),
                   "Actuator status rows must not contain control buttons");
            expect(row->findChildren<QSpinBox *>().isEmpty()
                       && row->findChildren<QDoubleSpinBox *>().isEmpty()
                       && row->findChildren<QSlider *>().isEmpty(),
                   "Actuator status rows must not duplicate fine-control editors");
        }
    }
    expect(servoStatusRows == rb::kServoCount,
           "MainWindow must expose five actuator status rows");

    auto *finePage =
        window.findChild<QWidget *>(QStringLiteral("servoFineControlPage"));
    expect(finePage != nullptr,
           "MainWindow must expose Servo Fine Control");
    if (finePage != nullptr) {
        for (int index = 0; index < rb::kServoCount; ++index) {
            expect(finePage->findChild<QPushButton *>(
                       QStringLiteral("servoEnableButton%1").arg(index)) != nullptr
                       && finePage->findChild<QPushButton *>(
                              QStringLiteral("servoNeutralButton%1").arg(index)) != nullptr
                       && finePage->findChild<QPushButton *>(
                              QStringLiteral("servoApplyButton%1").arg(index)) != nullptr,
                   "Servo Fine Control owns Enable/Neutral/Apply for every servo");
        }
    }

    // Data Plots region exists as an independent tab widget with IMU/Depth/
    // Actuator placeholder sub-tabs.
    bool imuPlot = tabWidgetWithText(&window, QStringLiteral("IMU")) != nullptr;
    bool depthPlot = tabWidgetWithText(&window, QStringLiteral("Depth")) != nullptr;
    bool actuatorPlot = tabWidgetWithText(&window, QStringLiteral("Actuator")) != nullptr;
    expect(imuPlot && depthPlot && actuatorPlot,
           "MainWindow must expose the Data Plots region (IMU/Depth/Actuator)");

    // Log and Protocol Details tabs exist.
    QTabWidget *logTabs = window.findChild<QTabWidget *>(QStringLiteral("operatorToolsTabs"));
    QTabWidget *detailsTabs = logTabs;
    expect(logTabs != nullptr && detailsTabs != nullptr,
           "MainWindow must expose operator tabs");

    // ACK status must be owned by the Protocol Details page only, never also
    // placed in the Protocol/Link summary card. The summary card must not
    // contain an ACK-state label, and the details page must retain one.
    QGroupBox *summary = findGroupBox(&window, QStringLiteral("Protocol"));
    expect(summary != nullptr, "MainWindow must expose the Protocol summary");
    bool summaryHasAckLabel = false;
    for (QLabel *label : summary->findChildren<QLabel *>()) {
        if (label->text() == QStringLiteral("Idle")) {
            summaryHasAckLabel = true;
        }
    }
    expect(!summaryHasAckLabel,
           "Protocol summary must not contain an ACK-state label");
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

    // Operator tabs include the Servo Fine Control page.
    if (detailsTabs != nullptr) {
        expect(detailsTabs->count() == 6,
               "Operator tab widget includes Motion/Gait, Servo Fine Control, and four detail/data pages");
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

    // Structural status-list check: five servo rows are siblings under
    // the single Actuator Control card beneath Protocol.
    QWidget *statusHost =
        window.findChild<QWidget *>(QStringLiteral("actuatorCardsHost"));
    QWidget *actuatorPanel =
        window.findChild<QWidget *>(QStringLiteral("actuatorPage"));
    QWidget *protocolCard =
        window.findChild<QWidget *>(QStringLiteral("protocolSummaryCard"));
    QWidget *telemetrySidebar =
        window.findChild<QWidget *>(QStringLiteral("telemetrySidebar"));
    expect(statusHost != nullptr && actuatorPanel != nullptr
               && protocolCard != nullptr && telemetrySidebar != nullptr
               && telemetrySidebar->isAncestorOf(actuatorPanel)
               && telemetrySidebar->layout() != nullptr
               && telemetrySidebar->layout()->indexOf(protocolCard) >= 0
               && telemetrySidebar->layout()->indexOf(actuatorPanel)
                      > telemetrySidebar->layout()->indexOf(protocolCard),
           "Actuator status list lives below Protocol in the telemetry rail");

    const QWidget *rowParent = nullptr;
    bool allStatusRowsSameParent = true;
    for (int index = 0; index < rb::kServoCount; ++index) {
        QWidget *row = window.findChild<QWidget *>(
            QStringLiteral("servoStatusRow%1").arg(index));
        if (row == nullptr) {
            allStatusRowsSameParent = false;
            continue;
        }
        if (rowParent == nullptr) {
            rowParent = row->parentWidget();
        } else if (row->parentWidget() != rowParent) {
            allStatusRowsSameParent = false;
        }
        expect(row->width() <= ws.width(),
               "no actuator status row may be wider than the window");
    }
    expect(allStatusRowsSameParent && rowParent == statusHost,
           "all five actuator status rows are siblings in one compact list");

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
            if (actual < required) {
                std::fprintf(stderr, "clipped %s label=%s actual=%d required=%d card=%d hint=%d\n",
                             qPrintable(title), qPrintable(label->text()), actual, required,
                             box->height(), box->minimumSizeHint().height());
                ok = false;
            }
        }
        return ok;
    };
    expect(checkLabelHeight(QStringLiteral("IMU")),
           "IMU card status/metric labels must not be vertically clipped");
    expect(checkLabelHeight(QStringLiteral("Depth Sensor")),
           "Depth card status/metric labels must not be vertically clipped");
    expect(checkLabelHeight(QStringLiteral("Protocol")),
           "Protocol card TX/RX/CRC/Timeout/ACK RTT labels must not be "
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
    testTask03VisionNoticePrefixes();
    testTask02ApplyHostBlocksQueuedMutation();
    testTask02CloseNeverImplicitlyStopsRemoteWork();
    testTask02CloseWarnsForStoppingAndRetryingRemoteWork();
    testTask02SuccessfulReconciliationClearsUncertaintyBeforeClose();
    testTask02SuccessfulReconciliationClearsUncertaintyBeforeApplyHost();
    testTask02FailedReconciliationKeepsUncertaintyAndDoesNotRetry();
    testTask02RemoteAndDirectEndpointWidgetsStayDistinct();
#elif defined(RB_MAIN_WINDOW_VISUAL_ERROR_ONLY)
    testSlice5DetectionTextOverlayLifecycle();
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
    testTask03VisionNoticePrefixes();
    testTask02ApplyHostBlocksQueuedMutation();
    testTask02CloseNeverImplicitlyStopsRemoteWork();
    testTask02CloseWarnsForStoppingAndRetryingRemoteWork();
    testTask02SuccessfulReconciliationClearsUncertaintyBeforeClose();
    testTask02SuccessfulReconciliationClearsUncertaintyBeforeApplyHost();
    testTask02FailedReconciliationKeepsUncertaintyAndDoesNotRetry();
    testTask02RemoteAndDirectEndpointWidgetsStayDistinct();
    testSlice5DetectionTextOverlayLifecycle();
    testDashboardLayout();
#endif
    std::fflush(stderr);
    if (failures == 0) {
        std::fprintf(stdout, "All MainWindow tests passed\n");
    }
    return failures == 0 ? 0 : 1;
}
