#include "protocol/PacketCodec.h"
#include "robot/DepthSnapshot.h"
#include "robot/LeakStatus.h"
#include "robot/ImuSnapshot.h"
#include "robot/RobotController.h"
#include "transport/FakeTransport.h"

#include <QCoreApplication>
#include <QEventLoop>
#include <QStringList>
#include <QTimer>

#include <cstdlib>
#include <chrono>
#include <iostream>
#include <limits>
#include <thread>
#include <utility>
#include <vector>

namespace {

int failures = 0;

struct PendingSignalLog {
    std::vector<std::pair<int, bool>> events;
};

void expect(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

rb::Packet lastPacket(const rb::FakeTransport &transport)
{
    const QByteArray wire = transport.writes().last();
    return rb::PacketCodec::decodeWire(wire.first(wire.size() - 1)).packet;
}

rb::Packet packetAt(const rb::FakeTransport &transport, qsizetype index)
{
    const QByteArray wire = transport.writes().at(index);
    return rb::PacketCodec::decodeWire(wire.first(wire.size() - 1)).packet;
}

void waitForMs(int milliseconds)
{
    QEventLoop loop;
    QTimer::singleShot(milliseconds, &loop, &QEventLoop::quit);
    loop.exec();
}

qint64 monotonicNowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

void acknowledgeLast(rb::FakeTransport &transport);

void connectApcAndAcknowledgeHeartbeat(rb::FakeTransport &transport,
                                      rb::RobotController &controller)
{
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();
    expect(!transport.writes().isEmpty(),
           "APC220 connection should send an initial heartbeat");
    if (!transport.writes().isEmpty()
        && lastPacket(transport).type == rb::MessageType::Heartbeat) {
        acknowledgeLast(transport);
    }
}

void acknowledgeLast(rb::FakeTransport &transport)
{
    const rb::Packet request = lastPacket(transport);
    QByteArray payload;
    payload.append(static_cast<char>(request.sequence & 0xff));
    payload.append(static_cast<char>((request.sequence >> 8) & 0xff));
    payload.append(static_cast<char>(request.type));
    payload.append(static_cast<char>(rb::AckResult::Ok));
    transport.injectBytes(rb::PacketCodec::encodeWire({rb::MessageType::Ack, 0x8000, payload}));
}

void acknowledge(rb::FakeTransport &transport,
                 const rb::Packet &request,
                 rb::AckResult result,
                 rb::MessageType acknowledgedType)
{
    QByteArray payload;
    payload.append(static_cast<char>(request.sequence & 0xff));
    payload.append(static_cast<char>((request.sequence >> 8) & 0xff));
    payload.append(static_cast<char>(acknowledgedType));
    payload.append(static_cast<char>(result));
    transport.injectBytes(rb::PacketCodec::encodeWire({rb::MessageType::Ack, 0x8000, payload}));
}

void injectLeakStatus(rb::FakeTransport &transport, quint8 state)
{
    transport.injectBytes(rb::PacketCodec::encodeWire(
        {rb::MessageType::LeakStatus, 0x6000, QByteArray(1, static_cast<char>(state))}));
}

void injectImuSnapshot(rb::FakeTransport &transport, quint16 sequence)
{
    rb::ImuSnapshot snapshot;
    snapshot.validityFlags = rb::ImuSnapshot::AccValid
        | rb::ImuSnapshot::GyroValid | rb::ImuSnapshot::AngleValid;
    snapshot.accMg = {1000, -2000, 0};
    snapshot.gyroDecidps = {10, -20, 0};
    snapshot.angleCentidegrees = {300, -400, 500};
    const QByteArray payload = rb::ImuSnapshot::encodePayload(snapshot);
    transport.injectBytes(rb::PacketCodec::encodeWire(
        {rb::MessageType::ImuSnapshot, sequence, payload}));
}

void injectDepthSnapshot(rb::FakeTransport &transport, quint16 sequence)
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
        {rb::MessageType::DepthSnapshot, sequence,
         rb::DepthSnapshot::encodePayload(snapshot)}));
}

void installSynchronousAcks(rb::FakeTransport &transport,
                            rb::AckResult disableResult,
                            bool mismatchDisableType)
{
    transport.setWriteCallback([&transport, disableResult, mismatchDisableType](
                                    const QByteArray &wire) {
        const rb::DecodeResult decoded = rb::PacketCodec::decodeWire(
            wire.first(wire.size() - 1));
        if (!decoded.ok()) {
            return;
        }
        const rb::MessageType requestType = decoded.packet.type;
        const bool expectsAck = requestType == rb::MessageType::ServoDisable
            || requestType == rb::MessageType::Heartbeat
            || requestType == rb::MessageType::ServoEnable
            || requestType == rb::MessageType::SetServoPwm
            || requestType == rb::MessageType::SetServoAngle
            || requestType == rb::MessageType::Neutral;
        if (!expectsAck) {
            return;
        }
        const rb::MessageType acknowledgedType = requestType == rb::MessageType::ServoDisable
            && mismatchDisableType
            ? rb::MessageType::ServoEnable
            : requestType;
        const rb::AckResult result = requestType == rb::MessageType::ServoDisable
            ? disableResult
            : rb::AckResult::Ok;
        QByteArray payload;
        payload.append(static_cast<char>(decoded.packet.sequence & 0xffU));
        payload.append(static_cast<char>((decoded.packet.sequence >> 8U) & 0xffU));
        payload.append(static_cast<char>(acknowledgedType));
        payload.append(static_cast<char>(result));
        transport.injectBytes(rb::PacketCodec::encodeWire(
            {rb::MessageType::Ack, 0x8000, payload}));
    });
}

void connectAndEnableServo1(rb::FakeTransport &transport, rb::RobotController &controller)
{
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();
    expect(controller.enableServo(rb::ServoId::Servo1),
           "Servo1 enable command should be sent");
    acknowledgeLast(transport);
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "Servo1 should be enabled after ACK");
}

void capturePendingSignals(rb::RobotController &controller, PendingSignalLog &log)
{
    QObject::connect(&controller, &rb::RobotController::servoDisablePendingChanged,
                     [&log](int index, bool pending) {
                         log.events.emplace_back(index, pending);
                     });
}

void testNoAutomaticEnableAndPwmRequiresEnable()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    expect(transport.writes().isEmpty(), "connection must not automatically enable servos");
    expect(!controller.setServoPwm(rb::ServoId::Servo1, 1500),
           "PWM must be rejected until servo is enabled");
    expect(transport.writes().isEmpty(), "rejected PWM must not write a frame");
    expect(!controller.setServoAngle(rb::ServoId::Servo1, 0),
           "angle must be rejected until servo is enabled");
    expect(transport.writes().isEmpty(), "rejected angle must not write a frame");

    expect(controller.enableServo(rb::ServoId::Servo1), "enable command should be sent");
    expect(lastPacket(transport).type == rb::MessageType::ServoEnable,
           "enable must use ServoEnable message");
    expect(!controller.setServoPwm(rb::ServoId::Servo1, 1500),
           "PWM must remain blocked until ServoEnable is acknowledged");
    acknowledgeLast(transport);
    expect(controller.setServoPwm(rb::ServoId::Servo1, 1500),
           "PWM should be sent after enable acknowledgement");
    expect(lastPacket(transport).type == rb::MessageType::SetServoPwm,
           "PWM must use SetServoPwm message");
}

void testLeakStatusMappingAndPendingAckIsolation()
{
    expect(rb::leakStateDisplayText(rb::LeakState::Unknown)
               == QStringLiteral("Leak: Unknown"),
           "UNKNOWN leak state should map to the neutral display text");
    expect(rb::leakStateDisplayText(rb::LeakState::Dry)
               == QStringLiteral("Leak: Dry"),
           "DRY leak state should map to the dry display text");
    expect(rb::leakStateDisplayText(rb::LeakState::Wet)
               == QStringLiteral("LEAK DETECTED"),
           "WET leak state should map to the warning display text");

    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();
    expect(controller.enableServo(rb::ServoId::FrontRight),
           "status isolation test should create a pending command");
    const qsizetype writesBeforeStatus = transport.writes().size();

    injectLeakStatus(transport, 1U);
    expect(controller.leakState() == rb::LeakState::Dry,
           "valid DRY telemetry should update controller state");
    expect(transport.writes().size() == writesBeforeStatus,
           "leak telemetry must not write or alter a pending command");
    expect(!controller.isServoEnabled(rb::ServoId::FrontRight),
           "leak telemetry must not satisfy a pending Enable ACK");

    injectLeakStatus(transport, 3U);
    expect(controller.leakState() == rb::LeakState::Unknown,
           "invalid leak telemetry must fail closed to UNKNOWN");
    acknowledgeLast(transport);
    expect(controller.isServoEnabled(rb::ServoId::FrontRight),
           "the real matching ACK must still complete the pending command");
}

void testLeakStatusStaleAndDisconnectTransitions()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 20;
    config.leakTelemetryStaleTimeoutMs = 30;
    rb::RobotController controller(&transport, config);
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    injectLeakStatus(transport, 1U);
    expect(controller.leakState() == rb::LeakState::Dry,
           "fresh DRY telemetry should be visible before stale timeout");
    waitForMs(70);
    expect(controller.leakState() == rb::LeakState::Unknown,
           "stale telemetry must return to UNKNOWN");

    injectLeakStatus(transport, 2U);
    expect(controller.leakState() == rb::LeakState::Wet,
           "a fresh valid telemetry frame should recover from stale UNKNOWN");
    controller.disconnectTransport();
    expect(controller.leakState() == rb::LeakState::Unknown,
           "disconnect must clear trusted leak state");
}

void testImuSnapshotDoesNotTouchAckOrLeakState()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    expect(controller.enableServo(rb::ServoId::FrontRight),
           "IMU isolation test should create a pending Enable");
    const qsizetype writesBeforeImu = transport.writes().size();
    injectImuSnapshot(transport, 0x6200);

    expect(controller.imuState().status == rb::ImuStatus::Receiving
               && controller.imuState().snapshot.has_value(),
           "valid IMU telemetry should update the monitor");
    expect(transport.writes().size() == writesBeforeImu,
           "IMU telemetry must not create a command write");
    expect(!controller.isServoEnabled(rb::ServoId::FrontRight),
           "IMU telemetry must not satisfy a pending Enable ACK");
    expect(controller.leakState() == rb::LeakState::Unknown,
           "IMU telemetry must not alter LeakStatus state");

    acknowledgeLast(transport);
    expect(controller.isServoEnabled(rb::ServoId::FrontRight),
           "the real matching ACK must still complete the pending Enable");
}

void testDepthSnapshotDoesNotTouchAckOrLeakState()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    expect(controller.enableServo(rb::ServoId::FrontRight),
           "Depth isolation test should create a pending Enable");
    const qsizetype writesBeforeDepth = transport.writes().size();
    injectDepthSnapshot(transport, 0x6300);

    expect(controller.depthState().status == rb::DepthStatus::Receiving
               && controller.depthState().snapshot.has_value(),
           "valid DepthSnapshot telemetry should update the monitor");
    expect(transport.writes().size() == writesBeforeDepth,
           "DepthSnapshot telemetry must not create a command write");
    expect(!controller.isServoEnabled(rb::ServoId::FrontRight),
           "DepthSnapshot telemetry must not satisfy a pending Enable ACK");
    expect(controller.leakState() == rb::LeakState::Unknown,
           "DepthSnapshot telemetry must not alter LeakStatus state");

    acknowledgeLast(transport);
    expect(controller.isServoEnabled(rb::ServoId::FrontRight),
           "the real matching ACK must still complete the pending Enable");
}

void testApcHeartbeatLossClearsLeakState()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 5;
    config.ackTimeoutMs = 5;
    config.maxRetries = 0;
    config.heartbeatSafetyBudgetMs = 100;
    config.leakTelemetryStaleTimeoutMs = 10000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    injectLeakStatus(transport, 1U);
    expect(controller.leakState() == rb::LeakState::Dry,
           "APC liveness test should begin with trusted DRY telemetry");

    waitForMs(40);
    expect(controller.leakState() == rb::LeakState::Unknown,
           "APC Heartbeat loss must clear trusted leak state immediately");
}

void testAngleDegreesConvertToCentidegrees()
{
    expect(rb::angleDegreesToCentidegrees(-90.0) == -9000,
           "-90.0 degrees must convert to -9000 cdeg");
    expect(rb::angleDegreesToCentidegrees(-45.0) == -4500,
           "-45.0 degrees must convert to -4500 cdeg");
    expect(rb::angleDegreesToCentidegrees(0.0) == 0,
           "0.0 degrees must convert to 0 cdeg");
    expect(rb::angleDegreesToCentidegrees(45.0) == 4500,
           "+45.0 degrees must convert to +4500 cdeg");
    expect(rb::angleDegreesToCentidegrees(90.0) == 9000,
           "+90.0 degrees must convert to +9000 cdeg");
}

void testSetAngleBlockedDuringDisableRequest()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();
    controller.enableServo(rb::ServoId::Servo1);
    acknowledgeLast(transport);

    const qsizetype beforeDisable = transport.writes().size();
    expect(controller.disableServo(rb::ServoId::Servo1),
           "disable request should be sent for an enabled Servo1");
    expect(!controller.setServoAngle(rb::ServoId::Servo1, 0),
           "angle must be blocked while Servo1 disable is awaiting ACK");
    expect(transport.writes().size() == beforeDisable + 1,
           "blocked angle must not add a frame while disable is pending");
}

void testApc220DisablePendingBlocksAllMotionAndErrorReleasesNoStaleWork()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 1000;
    config.heartbeatSafetyBudgetMs = 5000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    expect(controller.enableServo(rb::ServoId::FrontRight),
           "APC disable barrier setup should enable FrontRight");
    acknowledgeLast(transport);

    expect(controller.disableServo(rb::ServoId::FrontRight),
           "APC disable barrier should send Disable");
    const rb::Packet disable = lastPacket(transport);
    const qsizetype writesAfterDisable = transport.writes().size();
    const int queuedAfterDisable = controller.queuedCommandCount();
    expect(controller.isServoDisablePending(rb::ServoId::FrontRight),
           "FrontRight must be pending while Disable awaits ACK");
    expect(!controller.setServoPwm(rb::ServoId::FrontRight, 1500),
           "Set PWM must be rejected while Disable is pending");
    expect(!controller.neutralServo(rb::ServoId::FrontRight),
           "Neutral must be rejected while Disable is pending");
    expect(!controller.setServoAngle(rb::ServoId::FrontRight, 0),
           "Set Angle must be rejected while Disable is pending");
    expect(transport.writes().size() == writesAfterDisable,
           "pending-Disable motion rejection must not add wire frames");
    expect(controller.queuedCommandCount() == queuedAfterDisable,
           "pending-Disable motion rejection must not add APC queue entries");

    QByteArray payload;
    payload.append(static_cast<char>(disable.sequence & 0xff));
    payload.append(static_cast<char>((disable.sequence >> 8) & 0xff));
    payload.append(static_cast<char>(disable.type));
    payload.append(static_cast<char>(0x34));
    payload.append(static_cast<char>(0x12));
    transport.injectBytes(rb::PacketCodec::encodeWire(
        {rb::MessageType::Error, 0x8000, payload}));
    expect(!controller.isServoDisablePending(rb::ServoId::FrontRight),
           "matching Disable Error must clear pending state");
    expect(transport.writes().size() == writesAfterDisable,
           "Disable Error must not release rejected stale motion onto the wire");
    expect(controller.queuedCommandCount() == 0,
           "Disable Error must not leave rejected stale motion queued");
}

void testApc220DisableTimeoutReleasesNoStaleMotion()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 5;
    config.maxRetries = 0;
    config.heartbeatSafetyBudgetMs = 5000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    expect(controller.enableServo(rb::ServoId::FrontRight),
           "APC timeout barrier setup should enable FrontRight");
    acknowledgeLast(transport);
    expect(controller.disableServo(rb::ServoId::FrontRight),
           "APC timeout barrier should send Disable");
    const qsizetype writesAfterDisable = transport.writes().size();

    expect(!controller.setServoPwm(rb::ServoId::FrontRight, 1500),
           "Set PWM must not queue behind a pending Disable timeout");
    expect(!controller.neutralServo(rb::ServoId::FrontRight),
           "Neutral must not queue behind a pending Disable timeout");
    expect(!controller.setServoAngle(rb::ServoId::FrontRight, 0),
           "Set Angle must not queue behind a pending Disable timeout");
    waitForMs(40);

    expect(!controller.isServoDisablePending(rb::ServoId::FrontRight),
           "terminal Disable timeout must clear pending state");
    expect(controller.queuedCommandCount() == 0,
           "terminal Disable timeout must not release stale motion from the queue");
    expect(transport.writes().size() == writesAfterDisable,
           "terminal Disable timeout must not send rejected stale motion");
}

void testDisableAckClearsPendingWhenAlreadyDisabled()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    PendingSignalLog pending;
    capturePendingSignals(controller, pending);

    expect(controller.disableAll(), "Disable All should be sent while connected");
    expect(controller.isServoDisablePending(rb::ServoId::Servo1),
           "Disable All should mark Servo1 pending before ACK");
    const rb::Packet request = lastPacket(transport);
    expect(request.type == rb::MessageType::ServoDisable,
           "Disable All must use ServoDisable message");
    acknowledge(transport, request, rb::AckResult::Ok, rb::MessageType::ServoDisable);
    expect(!controller.isServoDisablePending(rb::ServoId::Servo1),
           "successful Disable All ACK should clear pending for already-disabled Servo1");
    expect(pending.events == std::vector<std::pair<int, bool>>{
               {0, true}, {1, true}, {2, true}, {3, true}, {4, true},
               {0, false}, {1, false}, {2, false}, {3, false}, {4, false}},
           "Disable All should emit pending transitions for all five semantic servos");

    expect(controller.enableServo(rb::ServoId::Servo1),
           "Servo1 should remain re-enableable after an already-disabled Disable All ACK");
    acknowledgeLast(transport);
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "Servo1 should be enabled after the subsequent ACK");
}

void testDisablePendingWriteFailureDoesNotLockAngle()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    connectAndEnableServo1(transport, controller);

    PendingSignalLog pending;
    capturePendingSignals(controller, pending);
    transport.setWriteSucceeds(false);
    const qsizetype before = transport.writes().size();

    expect(!controller.disableServo(rb::ServoId::Servo1),
           "Disable write failure should be reported");
    expect(!controller.isServoDisablePending(rb::ServoId::Servo1),
           "failed Disable write must not leave Servo1 pending");
    expect(transport.writes().size() == before,
           "failed Disable write must not append a frame");
    expect(pending.events.empty(),
           "failed Disable write must not emit a pending transition");

    transport.setWriteSucceeds(true);
    expect(controller.setServoAngle(rb::ServoId::Servo1, 0),
           "Set Angle should remain available after Disable write failure");
}

void testDisablePendingAckRejectedRestoresAngle()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    connectAndEnableServo1(transport, controller);

    PendingSignalLog pending;
    capturePendingSignals(controller, pending);
    expect(controller.disableServo(rb::ServoId::Servo1),
           "Disable command should be sent before rejected ACK");
    const rb::Packet request = lastPacket(transport);
    acknowledge(transport, request, rb::AckResult::HardwareFailure, rb::MessageType::ServoDisable);

    expect(!controller.isServoDisablePending(rb::ServoId::Servo1),
           "rejected Disable ACK should clear pending");
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "rejected Disable ACK should leave Servo1 enabled");
    expect(pending.events == std::vector<std::pair<int, bool>>{{0, true}, {0, false}},
           "rejected Disable ACK should emit pending clear");
    expect(controller.setServoAngle(rb::ServoId::Servo1, 0),
           "Set Angle should recover after rejected Disable ACK");
}

void testDisablePendingAckTypeMismatchRestoresAngle()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    connectAndEnableServo1(transport, controller);

    PendingSignalLog pending;
    capturePendingSignals(controller, pending);
    expect(controller.disableServo(rb::ServoId::Servo1),
           "Disable command should be sent before mismatched ACK");
    const rb::Packet request = lastPacket(transport);
    acknowledge(transport, request, rb::AckResult::Ok, rb::MessageType::ServoEnable);

    expect(!controller.isServoDisablePending(rb::ServoId::Servo1),
           "mismatched ACK should clear pending");
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "mismatched ACK should leave Servo1 enabled");
    expect(pending.events == std::vector<std::pair<int, bool>>{{0, true}, {0, false}},
           "mismatched ACK should emit pending clear");
    expect(controller.setServoAngle(rb::ServoId::Servo1, 0),
           "Set Angle should recover after mismatched ACK");
}

void testDisablePendingErrorRestoresAngle()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    connectAndEnableServo1(transport, controller);

    PendingSignalLog pending;
    capturePendingSignals(controller, pending);
    expect(controller.disableServo(rb::ServoId::Servo1),
           "Disable command should be sent before Error");
    const rb::Packet request = lastPacket(transport);
    QByteArray payload;
    payload.append(static_cast<char>(request.sequence & 0xff));
    payload.append(static_cast<char>((request.sequence >> 8) & 0xff));
    payload.append(static_cast<char>(request.type));
    payload.append(static_cast<char>(0x34));
    payload.append(static_cast<char>(0x12));
    transport.injectBytes(rb::PacketCodec::encodeWire(
        {rb::MessageType::Error, 0x8000, payload}));

    expect(!controller.isServoDisablePending(rb::ServoId::Servo1),
           "Disable Error should clear pending");
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "Disable Error should leave Servo1 enabled");
    expect(pending.events == std::vector<std::pair<int, bool>>{{0, true}, {0, false}},
           "Disable Error should emit pending clear");
    expect(controller.setServoAngle(rb::ServoId::Servo1, 0),
           "Set Angle should recover after Disable Error");
}

void testDisablePendingTimeoutRestoresAngle()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 1;
    config.maxRetries = 0;
    rb::RobotController controller(&transport, config);
    connectAndEnableServo1(transport, controller);

    PendingSignalLog pending;
    capturePendingSignals(controller, pending);
    expect(controller.disableServo(rb::ServoId::Servo1),
           "Disable command should be sent before timeout");
    QEventLoop loop;
    QTimer::singleShot(50, &loop, &QEventLoop::quit);
    loop.exec();

    expect(!controller.isServoDisablePending(rb::ServoId::Servo1),
           "Disable timeout should clear pending");
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "Disable timeout should leave Servo1 enabled");
    expect(pending.events == std::vector<std::pair<int, bool>>{{0, true}, {0, false}},
           "Disable timeout should emit pending clear");
    expect(controller.setServoAngle(rb::ServoId::Servo1, 0),
           "Set Angle should recover after Disable timeout");
}

void testDisablePendingDisconnectClearsState()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    connectAndEnableServo1(transport, controller);

    PendingSignalLog pending;
    capturePendingSignals(controller, pending);
    expect(controller.disableServo(rb::ServoId::Servo1),
           "Disable command should be sent before disconnect");
    transport.simulateError(QStringLiteral("link lost"));

    expect(!controller.isServoDisablePending(rb::ServoId::Servo1),
           "disconnect/reset should clear pending");
    expect(pending.events == std::vector<std::pair<int, bool>>{{0, true}, {0, false}},
           "disconnect/reset should emit pending clear");
}

void testDisablePendingSuccessDisablesAngle()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    connectAndEnableServo1(transport, controller);

    PendingSignalLog pending;
    capturePendingSignals(controller, pending);
    expect(controller.disableServo(rb::ServoId::Servo1),
           "Disable command should be sent before successful ACK");
    const rb::Packet request = lastPacket(transport);
    acknowledge(transport, request, rb::AckResult::Ok, rb::MessageType::ServoDisable);

    expect(!controller.isServoDisablePending(rb::ServoId::Servo1),
           "successful Disable ACK should clear pending");
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "successful Disable ACK should disable Servo1");
    expect(pending.events == std::vector<std::pair<int, bool>>{{0, true}, {0, false}},
           "successful Disable ACK should emit pending clear");
    const qsizetype before = transport.writes().size();
    expect(!controller.setServoAngle(rb::ServoId::Servo1, 0),
           "Set Angle must remain disabled after successful Disable ACK");
    expect(transport.writes().size() == before,
           "Set Angle after successful Disable ACK must not write a frame");
}

void testPwmCalibrationAndBounds()
{
    rb::FakeTransport transport;
    const rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    rb::RobotController controller(&transport, config);
    const rb::ServoDescriptor *descriptor = rb::servoDescriptor(rb::ServoId::FrontRight);
    expect(descriptor != nullptr, "FrontRight descriptor must exist");
    expect(descriptor->commandMinPwmUs == 1000, "SAVOX command minimum must be 1000 us");
    expect(descriptor->neutralPwmUs == 1450, "SAVOX neutral must be 1450 us");
    expect(descriptor->commandMaxPwmUs == 1900, "SAVOX command maximum must be 1900 us");
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();
    controller.enableServo(rb::ServoId::Servo1);
    acknowledgeLast(transport);
    const qsizetype before = transport.writes().size();

    expect(!controller.setServoPwm(rb::ServoId::Servo1, 999),
           "SAVOX PWM below command range must be rejected");
    expect(!controller.setServoPwm(rb::ServoId::Servo1, 1901),
           "SAVOX PWM above command range must be rejected");
    expect(transport.writes().size() == before, "rejected commands must not write frames");

    expect(controller.setServoPwm(rb::ServoId::Servo1, 1000),
           "SAVOX PWM minimum boundary must be accepted");
    expect(lastPacket(transport).payload == QByteArray::fromHex("0100e803"),
           "SAVOX PWM minimum must be encoded as count, FrontRight, uint16 LE");
    expect(controller.setServoPwm(rb::ServoId::Servo1, 1900),
           "SAVOX PWM maximum boundary must be accepted");
    expect(lastPacket(transport).payload == QByteArray::fromHex("01006c07"),
           "SAVOX PWM maximum must be encoded as count, FrontRight, uint16 LE");
}

void testSetAngleEncodingAndBounds()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();
    controller.enableServo(rb::ServoId::Servo1);
    acknowledgeLast(transport);

    const qsizetype before = transport.writes().size();
    expect(!controller.setServoAngle(rb::ServoId::Servo1, -4501),
           "FrontRight angle below -45 degrees must be rejected");
    expect(!controller.setServoAngle(rb::ServoId::Servo1, 4501),
           "FrontRight angle above +45 degrees must be rejected");
    expect(transport.writes().size() == before, "out-of-range angles must not write frames");

    expect(controller.setServoAngle(rb::ServoId::Servo1, -4500),
           "-45 degrees must be accepted");
    expect(lastPacket(transport).type == rb::MessageType::SetServoAngle,
           "angle must use SetServoAngle message");
    expect(lastPacket(transport).payload == QByteArray::fromHex("01006cee"),
           "-4500 cdeg must be encoded as int16 LE");

    expect(controller.setServoAngle(rb::ServoId::Servo1, 0),
           "zero degrees must be accepted");
    expect(lastPacket(transport).payload == QByteArray::fromHex("01000000"),
           "zero cdeg must be encoded as int16 LE");

    expect(controller.setServoAngle(rb::ServoId::Servo1, 4500),
           "+45 degrees must be accepted");
    expect(lastPacket(transport).payload == QByteArray::fromHex("01009411"),
           "+4500 cdeg must be encoded as int16 LE");
}

void testDisconnectAttemptsDisableAll()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();
    controller.disconnectTransport();

    expect(!transport.writes().isEmpty(), "disconnect must attempt to send Disable All");
    expect(lastPacket(transport).type == rb::MessageType::ServoDisable,
           "disconnect safety frame must be ServoDisable");
    expect(lastPacket(transport).payload == QByteArray::fromHex("1f00"),
           "disconnect must disable the fixed five-servo supported mask");
    expect(transport.closeCallCount() == 1, "disconnect must close the transport");
}

void testNeutralEncodingAndAck()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    expect(!controller.neutralServo(rb::ServoId::Servo1),
           "neutral must be rejected for a disabled servo");
    controller.enableServo(rb::ServoId::Servo1);
    acknowledgeLast(transport);
    expect(controller.neutralServo(rb::ServoId::Servo1),
           "neutral should be sent for an enabled servo");
    const rb::Packet request = lastPacket(transport);
    expect(request.type == rb::MessageType::Neutral,
           "neutral must use Neutral message");
    expect(request.payload == QByteArray::fromHex("0100"),
           "neutral must encode the Servo1 mask as uint16 LE");
    acknowledge(transport, request, rb::AckResult::Ok, rb::MessageType::Neutral);
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "successful neutral must leave the servo enabled");
    expect(controller.monitor().ackStatus.contains(QString::number(request.sequence)),
           "successful neutral ACK must be matched to its request");
}

void testFrontAxisUsesCalibratedPwmAndAngles()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    const rb::ServoDescriptor *descriptor = rb::servoDescriptor(rb::ServoId::FrontAxis);
    expect(descriptor != nullptr && descriptor->supported,
           "FrontAxis must be supported");
    expect(descriptor != nullptr && descriptor->angleSupported,
           "FrontAxis must support calibrated angles");
    expect(descriptor != nullptr && !descriptor->calibrationPending,
           "FrontAxis calibration must not remain pending");
    const qsizetype beforeEnable = transport.writes().size();
    const bool enableSent = controller.enableServo(rb::ServoId::FrontAxis);
    expect(enableSent, "FrontAxis enable should be sent");
    const bool enableWrote = transport.writes().size() == beforeEnable + 1;
    expect(!enableSent || enableWrote,
           "FrontAxis enable acceptance must produce exactly one wire frame");
    if (!enableSent || !enableWrote) {
        return;
    }
    acknowledgeLast(transport);
    const qsizetype beforeCommands = transport.writes().size();
    const qsizetype beforePwmBelow = transport.writes().size();
    const bool pwmBelowAccepted = controller.setServoPwm(rb::ServoId::FrontAxis, 1059);
    expect(!pwmBelowAccepted,
           "FrontAxis PWM below 1060 us must be rejected");
    expect(transport.writes().size() == beforePwmBelow,
           "rejected FrontAxis PWM below 1060 us must not write a frame");

    const qsizetype beforePwmMin = transport.writes().size();
    const bool pwmMinAccepted = controller.setServoPwm(rb::ServoId::FrontAxis, 1060);
    expect(pwmMinAccepted,
           "FrontAxis PWM 1060 us must be accepted");
    const bool pwmMinWrote = transport.writes().size() == beforePwmMin + 1;
    expect(!pwmMinAccepted || pwmMinWrote,
           "accepted FrontAxis PWM 1060 us must produce exactly one wire frame");
    if (pwmMinAccepted && pwmMinWrote) {
        const rb::Packet packet = lastPacket(transport);
        expect(packet.type == rb::MessageType::SetServoPwm
                   && packet.payload == QByteArray::fromHex("01022404"),
               "FrontAxis PWM 1060 us must encode semantic ID 2 and little-endian pulse");
        acknowledgeLast(transport);
    }

    const qsizetype beforePwmMax = transport.writes().size();
    const bool pwmMaxAccepted = controller.setServoPwm(rb::ServoId::FrontAxis, 2430);
    expect(pwmMaxAccepted,
           "FrontAxis PWM 2430 us must be accepted");
    const bool pwmMaxWrote = transport.writes().size() == beforePwmMax + 1;
    expect(!pwmMaxAccepted || pwmMaxWrote,
           "accepted FrontAxis PWM 2430 us must produce exactly one wire frame");
    if (pwmMaxAccepted && pwmMaxWrote) {
        const rb::Packet packet = lastPacket(transport);
        expect(packet.type == rb::MessageType::SetServoPwm
                   && packet.payload == QByteArray::fromHex("01027e09"),
               "FrontAxis PWM 2430 us must encode semantic ID 2 and little-endian pulse");
        acknowledgeLast(transport);
    }

    const qsizetype beforePwmAbove = transport.writes().size();
    const bool pwmAboveAccepted = controller.setServoPwm(rb::ServoId::FrontAxis, 2431);
    expect(!pwmAboveAccepted,
           "FrontAxis PWM above 2430 us must be rejected");
    expect(transport.writes().size() == beforePwmAbove,
           "rejected FrontAxis PWM above 2430 us must not write a frame");

    const qsizetype beforeAngleBelow = transport.writes().size();
    const bool angleBelowAccepted = controller.setServoAngle(rb::ServoId::FrontAxis, -9001);
    expect(!angleBelowAccepted,
           "FrontAxis angle below -90 degrees must be rejected");
    expect(transport.writes().size() == beforeAngleBelow,
           "rejected FrontAxis angle below -90 degrees must not write a frame");

    const qsizetype beforeAngleMin = transport.writes().size();
    const bool angleMinAccepted = controller.setServoAngle(rb::ServoId::FrontAxis, -9000);
    expect(angleMinAccepted,
           "FrontAxis -90 degrees must be accepted");
    const bool angleMinWrote = transport.writes().size() == beforeAngleMin + 1;
    expect(!angleMinAccepted || angleMinWrote,
           "accepted FrontAxis -90 degrees must produce exactly one wire frame");
    if (angleMinAccepted && angleMinWrote) {
        const rb::Packet packet = lastPacket(transport);
        expect(packet.type == rb::MessageType::SetServoAngle
                   && packet.payload == QByteArray::fromHex("0102d8dc"),
               "FrontAxis -90 degrees must encode semantic ID 2 and little-endian angle");
        acknowledgeLast(transport);
    }

    const qsizetype beforeAngleZero = transport.writes().size();
    const bool angleZeroAccepted = controller.setServoAngle(rb::ServoId::FrontAxis, 0);
    expect(angleZeroAccepted,
           "FrontAxis zero degrees must be accepted");
    const bool angleZeroWrote = transport.writes().size() == beforeAngleZero + 1;
    expect(!angleZeroAccepted || angleZeroWrote,
           "accepted FrontAxis zero degrees must produce exactly one wire frame");
    if (angleZeroAccepted && angleZeroWrote) {
        const rb::Packet packet = lastPacket(transport);
        expect(packet.type == rb::MessageType::SetServoAngle
                   && packet.payload == QByteArray::fromHex("01020000"),
               "FrontAxis zero degrees must encode semantic ID 2 and little-endian angle");
        acknowledgeLast(transport);
    }

    const qsizetype beforeAngleMax = transport.writes().size();
    const bool angleMaxAccepted = controller.setServoAngle(rb::ServoId::FrontAxis, 9000);
    expect(angleMaxAccepted,
           "FrontAxis +90 degrees must be accepted");
    const bool angleMaxWrote = transport.writes().size() == beforeAngleMax + 1;
    expect(!angleMaxAccepted || angleMaxWrote,
           "accepted FrontAxis +90 degrees must produce exactly one wire frame");
    if (angleMaxAccepted && angleMaxWrote) {
        const rb::Packet packet = lastPacket(transport);
        expect(packet.type == rb::MessageType::SetServoAngle
                   && packet.payload == QByteArray::fromHex("01022823"),
               "FrontAxis +90 degrees must encode semantic ID 2 and little-endian angle");
        acknowledgeLast(transport);
    }

    const qsizetype beforeAngleAbove = transport.writes().size();
    const bool angleAboveAccepted = controller.setServoAngle(rb::ServoId::FrontAxis, 9001);
    expect(!angleAboveAccepted,
           "FrontAxis angle above +90 degrees must be rejected");
    expect(transport.writes().size() == beforeAngleAbove,
           "rejected FrontAxis angle above +90 degrees must not write a frame");
    expect(transport.writes().size() == beforeCommands + 5,
           "FrontAxis rejected commands must not write frames");
}

void testSemanticServoCommandBoundaries()
{
    struct BoundaryCase {
        rb::ServoId id;
        quint16 pwmMin;
        quint16 pwmMax;
        qint16 angleMin;
        qint16 angleMax;
        bool angleSupported;
    };
    const BoundaryCase cases[] = {
        {rb::ServoId::FrontRight, 1000, 1900, -4500, 4500, true},
        {rb::ServoId::FrontLeft, 1140, 2020, -4500, 4500, true},
        {rb::ServoId::FrontAxis, 1060, 2430, -9000, 9000, true},
        {rb::ServoId::RearRight, 1110, 2030, -4500, 4500, true},
        {rb::ServoId::RearLeft, 960, 1940, -4500, 4500, true},
    };

    for (const BoundaryCase &boundary : cases) {
        rb::FakeTransport transport;
        rb::RobotController controller(
            &transport, rb::RobotControllerConfig::bringUpProvisional());
        controller.connectTransport({"COM_TEST", 9600});
        transport.simulateConnected();
        expect(controller.enableServo(boundary.id),
               "every semantic servo should accept Enable");
        acknowledgeLast(transport);
        const qsizetype beforeCommands = transport.writes().size();

        const qint64 pwmBelow = static_cast<qint64>(boundary.pwmMin) - 1;
        const qint64 pwmAbove = static_cast<qint64>(boundary.pwmMax) + 1;
        const bool pwmBelowRepresentable =
            pwmBelow >= std::numeric_limits<quint16>::min()
            && pwmBelow <= std::numeric_limits<quint16>::max();
        const bool pwmAboveRepresentable =
            pwmAbove >= std::numeric_limits<quint16>::min()
            && pwmAbove <= std::numeric_limits<quint16>::max();
        expect(pwmBelowRepresentable,
               "PWM below-edge value must be representable as quint16");
        expect(pwmAboveRepresentable,
               "PWM above-edge value must be representable as quint16");
        if (pwmBelowRepresentable) {
            expect(!controller.setServoPwm(boundary.id, static_cast<quint16>(pwmBelow)),
                   "PWM below the descriptor command envelope must be rejected");
        }
        if (pwmAboveRepresentable) {
            expect(!controller.setServoPwm(boundary.id, static_cast<quint16>(pwmAbove)),
                   "PWM above the descriptor command envelope must be rejected");
        }
        expect(controller.setServoPwm(boundary.id, boundary.pwmMin),
               "descriptor PWM minimum must be accepted");
        rb::Packet packet = lastPacket(transport);
        expect(static_cast<quint8>(packet.payload[0]) == 1
                   && static_cast<quint8>(packet.payload[1])
                          == static_cast<quint8>(boundary.id),
               "PWM payload must carry count and semantic servo ID");
        expect(controller.setServoPwm(boundary.id, boundary.pwmMax),
               "descriptor PWM maximum must be accepted");

        if (!boundary.angleSupported) {
            expect(!controller.setServoAngle(boundary.id, 0),
                   "FrontAxis Set Angle must remain unsupported");
        } else {
            const qint64 angleBelow = static_cast<qint64>(boundary.angleMin) - 1;
            const qint64 angleAbove = static_cast<qint64>(boundary.angleMax) + 1;
            const bool angleBelowRepresentable =
                angleBelow >= std::numeric_limits<qint16>::min()
                && angleBelow <= std::numeric_limits<qint16>::max();
            const bool angleAboveRepresentable =
                angleAbove >= std::numeric_limits<qint16>::min()
                && angleAbove <= std::numeric_limits<qint16>::max();
            expect(angleBelowRepresentable,
                   "angle below-edge value must be representable as qint16");
            expect(angleAboveRepresentable,
                   "angle above-edge value must be representable as qint16");
            if (angleBelowRepresentable) {
                expect(!controller.setServoAngle(boundary.id,
                                                 static_cast<qint16>(angleBelow)),
                       "angle below the descriptor command envelope must be rejected");
            }
            if (angleAboveRepresentable) {
                expect(!controller.setServoAngle(boundary.id,
                                                 static_cast<qint16>(angleAbove)),
                       "angle above the descriptor command envelope must be rejected");
            }
            expect(controller.setServoAngle(boundary.id, boundary.angleMin),
                   "angle -45 degrees must be accepted");
            expect(controller.setServoAngle(boundary.id, 0),
                   "angle zero must be accepted");
            expect(controller.setServoAngle(boundary.id, boundary.angleMax),
                   "angle +45 degrees must be accepted");
        }

        const qsizetype expectedAcceptedWrites = boundary.angleSupported ? 5 : 2;
        expect(transport.writes().size() == beforeCommands + expectedAcceptedWrites,
               "out-of-range or unsupported commands must not write frames");
    }
}

void testAckRejectionAndMatching()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    controller.enableServo(rb::ServoId::Servo1);
    const rb::Packet rejected = lastPacket(transport);
    acknowledge(transport, rejected, rb::AckResult::UnsupportedServo, rb::MessageType::ServoEnable);
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "rejected enable ACK must not enable Servo1");
    expect(controller.monitor().ackStatus.contains(QStringLiteral("UnsupportedServo")),
           "ACK result 3 must be reported as UnsupportedServo");

    controller.enableServo(rb::ServoId::Servo1);
    const rb::Packet mismatchedType = lastPacket(transport);
    acknowledge(transport, mismatchedType, rb::AckResult::Ok, rb::MessageType::ServoDisable);
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "ACK with a mismatched type must not enable Servo1");
    expect(controller.monitor().ackStatus.contains(QStringLiteral("type mismatch")),
           "ACK type mismatch must be reported");

    controller.enableServo(rb::ServoId::Servo1);
    const rb::Packet matched = lastPacket(transport);
    rb::Packet wrongSequence = matched;
    ++wrongSequence.sequence;
    acknowledge(transport, wrongSequence, rb::AckResult::Ok, rb::MessageType::ServoEnable);
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "unmatched ACK sequence must not enable Servo1");
    acknowledge(transport, matched, rb::AckResult::Ok, rb::MessageType::ServoEnable);
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "matching successful ACK must enable Servo1");
}

void testRetryReusesIdenticalSequenceAndFrame()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 1;
    config.maxRetries = 1;
    rb::RobotController controller(&transport, config);
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    expect(controller.enableServo(rb::ServoId::Servo1), "enable command should be sent");
    const QByteArray originalFrame = transport.writes().last();
    QEventLoop loop;
    QTimer::singleShot(30, &loop, &QEventLoop::quit);
    loop.exec();

    expect(transport.writes().size() == 2, "one timeout must produce exactly one retry");
    expect(transport.writes().last() == originalFrame,
           "retry must reuse the identical sequence and encoded frame");
}

void testUnexpectedTransportLossRecordsDisableFailure()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    QStringList messages;
    QObject::connect(&controller, &rb::RobotController::logMessage,
                     [&messages](const QString &message) { messages.append(message); });
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();
    transport.simulateError(QStringLiteral("link lost"));

    bool found = false;
    for (const QString &message : messages) {
        found = found || message.contains(QStringLiteral("Disable All could not be delivered"));
    }
    expect(found, "unexpected disconnect must explicitly log that Disable All could not be delivered");
}

void testApc220ProfileUsesHalfDuplexTiming()
{
    const rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    expect(config.linkProfile == rb::LinkProfile::Apc220HalfDuplex,
           "APC220 factory must select the half-duplex link profile");
    expect(config.heartbeatIntervalMs == 250,
           "APC220 profile must use a 250 ms heartbeat interval");
    expect(config.ackTimeoutMs == 250,
           "APC220 profile must use a 250 ms ACK timeout");
    expect(config.heartbeatSafetyBudgetMs == 490,
           "APC220 profile must reserve a 10 ms margin below the 500 ms watchdog");
}

void testDirectUartRetainsMultiplePendingRequests()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    expect(controller.enableServo(rb::ServoId::Servo1),
           "DirectUart Enable should occupy one pending slot");
    const rb::Packet enable = lastPacket(transport);
    expect(controller.disableAll(),
           "DirectUart Disable All should be allowed while Enable is pending");
    const rb::Packet disable = lastPacket(transport);
    expect(transport.writes().size() == 2,
           "DirectUart must retain its multi-pending behavior");
    expect(controller.isServoDisablePending(rb::ServoId::Servo1),
           "DirectUart Disable All should mark pending independently");

    acknowledge(transport, enable, rb::AckResult::Ok, rb::MessageType::ServoEnable);
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "DirectUart Enable ACK should update state while Disable remains pending");
    expect(controller.isServoDisablePending(rb::ServoId::Servo1),
           "DirectUart must keep the second request pending after the first ACK");
    acknowledge(transport, disable, rb::AckResult::Ok, rb::MessageType::ServoDisable);
    expect(!controller.isServoDisablePending(rb::ServoId::Servo1),
           "DirectUart Disable ACK should release its independent pending request");
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "DirectUart Disable ACK should update the final enabled state");
}

void testApc220AllowsOnlyOneAckRequiringFrameInFlight()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);

    const qsizetype beforeEnable = transport.writes().size();
    expect(controller.enableServo(rb::ServoId::Servo1),
           "APC220 enable should be accepted");
    const rb::Packet enable = lastPacket(transport);
    expect(controller.disableAll(),
           "APC220 Disable All should be queued while enable is in flight");
    expect(transport.writes().size() == beforeEnable + 1,
           "APC220 must not write a second ACK-requiring frame before the first ACK");
    expect(controller.queuedCommandCount() == 1,
           "APC220 queued command count should include the waiting Disable All");

    acknowledge(transport, enable, rb::AckResult::Ok, rb::MessageType::ServoEnable);
    expect(transport.writes().size() == beforeEnable + 2,
           "ACK should release the APC220 queue and dispatch the next command");
    expect(lastPacket(transport).type == rb::MessageType::ServoDisable,
           "the queued command should dispatch after the matching ACK");
    expect(controller.queuedCommandCount() == 0,
           "dispatched APC220 command should leave the bounded queue");
}

void testApc220HeartbeatTicksCoalesceWhileCommandIsInFlight()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 5;
    config.ackTimeoutMs = 1000;
    config.heartbeatSafetyBudgetMs = 5000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);

    expect(controller.enableServo(rb::ServoId::Servo1),
           "APC220 command should be sent before heartbeat coalescing test");
    const qsizetype commandWriteCount = transport.writes().size();
    waitForMs(30);
    expect(transport.writes().size() == commandWriteCount,
           "heartbeat ticks must coalesce instead of writing beside an in-flight command");

    acknowledgeLast(transport);
    expect(transport.writes().size() == commandWriteCount + 1,
           "one coalesced heartbeat should dispatch when the command ACK frees the slot");
    expect(lastPacket(transport).type == rb::MessageType::Heartbeat,
           "coalesced heartbeat should have priority when the slot becomes free");
    waitForMs(30);
    expect(transport.writes().size() == commandWriteCount + 1,
           "additional heartbeat ticks must not create another in-flight heartbeat");
}

void testApc220HeartbeatDuePrecedesCommandRetry()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 5;
    config.ackTimeoutMs = 20;
    config.maxRetries = 1;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);

    expect(controller.enableServo(rb::ServoId::Servo1),
           "APC220 command should be sent before retry priority test");
    const qsizetype commandIndex = transport.writes().size() - 1;
    waitForMs(30);
    expect(transport.writes().size() > commandIndex + 1,
           "a heartbeat should be dispatched once its due intent outranks a command retry");
    expect(packetAt(transport, commandIndex + 1).type == rb::MessageType::Heartbeat,
           "heartbeat due must be dispatched before an ordinary command retry");
}

void testApc220HeartbeatDeadlineWinsRetryBoundary()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10;
    config.ackTimeoutMs = 10;
    config.maxRetries = 1;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);

    expect(controller.enableServo(rb::ServoId::Servo1),
           "APC220 command should be sent before the equal-deadline test");
    const qsizetype commandIndex = transport.writes().size() - 1;
    waitForMs(25);

    expect(transport.writes().size() > commandIndex + 1,
           "equal heartbeat/retry deadlines should produce follow-up traffic");
    if (transport.writes().size() > commandIndex + 1) {
        expect(packetAt(transport, commandIndex + 1).type == rb::MessageType::Heartbeat,
               "heartbeat must win when its deadline equals a command retry deadline");
    }
}

void testApc220HeartbeatDeadlineRefreshesBeforeAckReleasesSlot()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 5;
    config.ackTimeoutMs = 1000;
    config.heartbeatSafetyBudgetMs = 5000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);

    expect(controller.enableServo(rb::ServoId::Servo1),
           "APC220 command should be sent before the ACK deadline refresh test");
    const rb::Packet enable = lastPacket(transport);
    expect(controller.disableAll(),
           "APC220 command should queue behind Enable for the ACK deadline refresh test");
    const qsizetype beforeAck = transport.writes().size();

    // Block the test thread so the heartbeat QTimer cannot run, while the
    // monotonic deadline advances beyond its target.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    acknowledge(transport, enable, rb::AckResult::Ok, rb::MessageType::ServoEnable);

    expect(transport.writes().size() == beforeAck + 1,
           "an ACK at an elapsed heartbeat deadline should release one scheduler action");
    if (transport.writes().size() > beforeAck) {
        expect(lastPacket(transport).type == rb::MessageType::Heartbeat,
               "an elapsed heartbeat deadline must win before a queued command on ACK release");
    }
}

void testApc220CommandQueueHasBoundedCapacity()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);

    expect(controller.enableServo(rb::ServoId::Servo1),
           "first APC220 command should occupy the in-flight slot");
    for (qsizetype index = 0; index < rb::kApc220CommandQueueCapacity; ++index) {
        expect(controller.enableServo(rb::ServoId::Servo1),
               "commands up to the APC220 queue capacity should be accepted");
    }
    expect(controller.queuedCommandCount() == rb::kApc220CommandQueueCapacity,
           "APC220 queue must report its bounded capacity");
    expect(!controller.enableServo(rb::ServoId::Servo1),
           "commands beyond APC220 queue capacity must be rejected");
    expect(controller.queuedCommandCount() == rb::kApc220CommandQueueCapacity,
           "a rejected APC220 command must not grow the queue");
}

void testApc220RetryUsesOriginalSequenceAndFrame()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 1;
    config.maxRetries = 1;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);

    expect(controller.enableServo(rb::ServoId::Servo1),
           "APC220 command should be sent before retry identity test");
    const QByteArray originalFrame = transport.writes().last();
    const qsizetype commandWriteCount = transport.writes().size();
    waitForMs(30);
    expect(transport.writes().size() == commandWriteCount + 1,
           "one APC220 timeout must produce exactly one retry");
    expect(transport.writes().last() == originalFrame,
           "APC220 retry must preserve the original sequence and encoded frame");
}

void testApc220TransportResetClearsSchedulerAndDoesNotAutoEnable()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 5;
    config.ackTimeoutMs = 1000;
    config.heartbeatSafetyBudgetMs = 5000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);

    expect(controller.enableServo(rb::ServoId::Servo1),
           "APC220 enable should be sent before reset test");
    expect(controller.disableAll(),
           "APC220 command should queue before reset test");
    waitForMs(20);
    transport.simulateError(QStringLiteral("link lost"));
    expect(controller.queuedCommandCount() == 0,
           "transport error must clear queued APC220 commands");
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "transport error must clear the logical enabled mask");
    expect(!controller.isServoDisablePending(rb::ServoId::Servo1),
           "transport error must clear Disable pending state");

    const qsizetype beforeReconnect = transport.writes().size();
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();
    expect(transport.writes().size() == beforeReconnect + 1,
           "reconnect must send one fresh APC220 heartbeat, not stale commands");
    expect(lastPacket(transport).type == rb::MessageType::Heartbeat,
           "reconnect must start with a fresh heartbeat");
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "reconnect must not automatically enable a servo");
}

void testApc220RecordsMatchingAckRtt()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);

    expect(controller.enableServo(rb::ServoId::Servo1),
           "APC220 command should be sent before RTT test");
    acknowledgeLast(transport);
    expect(controller.monitor().lastAckRttMs >= 0,
           "matching ACK should record a non-negative RTT in the monitor");
}

void testApc220SynchronousAckDuringInitialDispatchIsHandled()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    installSynchronousAcks(transport, rb::AckResult::Ok, false);

    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();
    expect(transport.writes().size() == 1,
           "synchronous APC220 transport should write the initial heartbeat once");
    expect(controller.queuedCommandCount() == 0,
           "synchronous heartbeat ACK must not leave a stale pending command");

    expect(controller.enableServo(rb::ServoId::Servo1),
           "synchronous ACK should release the APC220 slot for Enable");
    expect(controller.queuedCommandCount() == 0,
           "synchronous Enable ACK must not leave the command queued");
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "synchronous Enable ACK should update the enabled state");
}

void runApc220SynchronousDisableCase(rb::AckResult result,
                                     bool mismatchType,
                                     bool expectedEnabled,
                                     const char *description)
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    installSynchronousAcks(transport, result, mismatchType);
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();
    expect(controller.enableServo(rb::ServoId::Servo1),
           "synchronous Disable case should enable Servo1 first");
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "synchronous Enable ACK should establish the precondition for Disable");

    expect(controller.disableServo(rb::ServoId::Servo1), description);
    expect(!controller.isServoDisablePending(rb::ServoId::Servo1),
           "synchronous Disable ACK, rejection, or mismatch must not leave pending state set");
    expect(controller.isServoEnabled(rb::ServoId::Servo1) == expectedEnabled,
           "synchronous Disable result must preserve the expected logical enabled state");
}

void testApc220SynchronousDisableAckCasesDoNotRelockPending()
{
    runApc220SynchronousDisableCase(
        rb::AckResult::Ok,
        false,
        false,
        "synchronous successful Disable ACK should be accepted");
    runApc220SynchronousDisableCase(
        rb::AckResult::HostNotAlive,
        false,
        true,
        "synchronous rejected Disable ACK should be accepted as a completed response");
    runApc220SynchronousDisableCase(
        rb::AckResult::Ok,
        true,
        true,
        "synchronous Disable ACK type mismatch should be handled without relocking");
}

void testApc220ConnectGatesCommandsUntilHeartbeatAck()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    expect(transport.writes().size() == 1,
           "APC220 connect should immediately send one heartbeat");
    if (!transport.writes().isEmpty()) {
        expect(lastPacket(transport).type == rb::MessageType::Heartbeat,
               "the first APC220 frame must be a heartbeat");
    }
    expect(controller.enableServo(rb::ServoId::Servo1),
           "an early user command should be accepted into the APC220 queue");
    expect(transport.writes().size() == 1,
           "Enable must wait for the first heartbeat ACK");
    expect(controller.queuedCommandCount() == 1,
           "early Enable should remain queued until heartbeat liveness is established");

    acknowledgeLast(transport);
    expect(transport.writes().size() == 2,
           "a successful heartbeat ACK should release the queued Enable");
    expect(lastPacket(transport).type == rb::MessageType::ServoEnable,
           "Enable should dispatch only after heartbeat ACK");
}

void testApc220HeartbeatRejectionKeepsUserCommandsGated()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 5;
    config.ackTimeoutMs = 50;
    config.maxRetries = 0;
    rb::RobotController controller(&transport, config);
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    if (transport.writes().isEmpty()) {
        expect(false, "APC220 connection should provide a heartbeat to reject");
        return;
    }
    const rb::Packet firstHeartbeat = lastPacket(transport);
    expect(controller.enableServo(rb::ServoId::Servo1),
           "Enable should queue while the first heartbeat is in flight");
    acknowledge(transport, firstHeartbeat, rb::AckResult::HostNotAlive,
                rb::MessageType::Heartbeat);
    expect(controller.queuedCommandCount() == 1,
           "rejected heartbeat must leave the user command queued");
    expect(transport.writes().size() == 1,
           "rejected heartbeat must not dispatch Enable as HostNotAlive");

    waitForMs(20);
    expect(transport.writes().size() >= 2,
           "heartbeat rejection should make a later heartbeat due");
    expect(packetAt(transport, 1).type == rb::MessageType::Heartbeat,
           "the retry after heartbeat rejection must still be a heartbeat");
}

void testApc220HeartbeatTimeoutKeepsUserCommandsGated()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 1;
    config.ackTimeoutMs = 20;
    config.maxRetries = 0;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    waitForMs(5);

    if (transport.writes().size() < 2
        || lastPacket(transport).type != rb::MessageType::Heartbeat) {
        expect(false, "APC220 timer should provide a heartbeat for timeout handling");
        return;
    }
    expect(controller.enableServo(rb::ServoId::Servo1),
           "Enable should queue while a later heartbeat times out");
    waitForMs(35);

    expect(controller.queuedCommandCount() == 0,
           "terminal heartbeat timeout must discard queued user commands");
    bool enableSent = false;
    for (qsizetype index = 0; index < transport.writes().size(); ++index) {
        enableSent = enableSent || packetAt(transport, index).type == rb::MessageType::ServoEnable;
    }
    expect(!enableSent,
           "heartbeat timeout must keep the APC220 link gate closed until a fresh heartbeat ACK");
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "terminal heartbeat timeout must clear the logical enabled state");
}

void testApc220HeartbeatTypeMismatchKeepsUserCommandsGated()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 5;
    config.ackTimeoutMs = 50;
    config.maxRetries = 0;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    waitForMs(10);

    if (transport.writes().size() < 2
        || lastPacket(transport).type != rb::MessageType::Heartbeat) {
        expect(false, "APC220 timer should provide a heartbeat for type mismatch handling");
        return;
    }
    const rb::Packet heartbeat = lastPacket(transport);
    const qsizetype beforeMismatch = transport.writes().size();
    expect(controller.enableServo(rb::ServoId::Servo1),
           "Enable should queue while a heartbeat type mismatch is pending");
    acknowledge(transport, heartbeat, rb::AckResult::Ok, rb::MessageType::ServoEnable);
    expect(controller.queuedCommandCount() == 1,
           "heartbeat type mismatch must leave the user command queued");
    expect(transport.writes().size() == beforeMismatch,
           "heartbeat type mismatch must not dispatch the queued Enable");

    waitForMs(10);
    expect(transport.writes().size() > beforeMismatch,
           "heartbeat type mismatch should make a later heartbeat due");
    expect(lastPacket(transport).type == rb::MessageType::Heartbeat,
           "heartbeat type mismatch recovery must send another heartbeat");
}

void testApc220HeartbeatTicksDoNotBurstAndQueueGetsChanceAfterAck()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 20;
    config.ackTimeoutMs = 1000;
    config.heartbeatSafetyBudgetMs = 5000;
    rb::RobotController controller(&transport, config);
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    if (transport.writes().isEmpty()) {
        expect(false, "APC220 connection should provide a heartbeat for coalescing");
        return;
    }
    expect(controller.enableServo(rb::ServoId::Servo1),
           "Enable should queue behind the initial heartbeat");
    waitForMs(25);
    expect(transport.writes().size() == 1,
           "repeated timer ticks must not burst additional heartbeats in flight");
    acknowledgeLast(transport);
    expect(transport.writes().size() == 2,
           "heartbeat ACK should give the queued user command its turn");
    expect(lastPacket(transport).type == rb::MessageType::Heartbeat,
           "an elapsed heartbeat deadline should remain ahead of queued user work");
    acknowledgeLast(transport);
    expect(transport.writes().size() == 3,
           "the queued user command should dispatch after the due heartbeat exchange");
    expect(lastPacket(transport).type == rb::MessageType::ServoEnable,
           "queued user command should dispatch after the due heartbeat ACK");
}

void testApc220SafetyAdmissionProtectsNearTimeoutHeartbeat()
{
    constexpr int kHeartbeatRttMs = 245;
    constexpr qint64 kHardSafetyBudgetMs = 490;

    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.maxRetries = 3;
    expect(config.heartbeatSafetyBudgetMs == kHardSafetyBudgetMs,
           "near-timeout admission test must exercise the APC220 hard safety budget");
    rb::RobotController controller(&transport, config);
    std::vector<qint64> heartbeatSendTimes;
    int heartbeatWriteCount = 0;
    transport.setWriteCallback([&transport, &heartbeatSendTimes, &heartbeatWriteCount](
                                    const QByteArray &wire) {
        const rb::DecodeResult decoded = rb::PacketCodec::decodeWire(
            wire.first(wire.size() - 1));
        if (!decoded.ok()) {
            return;
        }
        const rb::Packet request = decoded.packet;
        const auto scheduleAck = [&transport](const rb::Packet &packet, int delayMs) {
            QTimer::singleShot(delayMs, &transport, [&transport, packet] {
                QByteArray payload;
                payload.append(static_cast<char>(packet.sequence & 0xffU));
                payload.append(static_cast<char>((packet.sequence >> 8U) & 0xffU));
                payload.append(static_cast<char>(packet.type));
                payload.append(static_cast<char>(rb::AckResult::Ok));
                transport.injectBytes(rb::PacketCodec::encodeWire(
                    {rb::MessageType::Ack, 0x8000, payload}));
            });
        };
        if (request.type == rb::MessageType::Heartbeat) {
            heartbeatSendTimes.push_back(monotonicNowMs());
            ++heartbeatWriteCount;
            scheduleAck(request, heartbeatWriteCount == 1 ? kHeartbeatRttMs : 0);
        } else if (request.type == rb::MessageType::ServoEnable) {
            scheduleAck(request, 0);
        }
        // SetServoPwm intentionally receives no ACK in this regression.
    });

    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();
    expect(controller.enableServo(rb::ServoId::Servo1),
           "near-timeout admission test should queue Enable behind the first heartbeat");
    waitForMs(400);
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "near-timeout admission test should establish Servo1 enabled state");
    expect(heartbeatSendTimes.size() >= 2,
           "near-timeout admission test should observe the high-RTT heartbeat and recovery heartbeat");
    if (heartbeatSendTimes.size() < 2 || !controller.isServoEnabled(rb::ServoId::Servo1)) {
        return;
    }

    const qint64 latestHeartbeatTarget = heartbeatSendTimes.back() + config.heartbeatIntervalMs;
    const qint64 sleepBeforeTarget = latestHeartbeatTarget - monotonicNowMs() - 5;
    if (sleepBeforeTarget > 0) {
        // Keep the timer event queued while placing the request immediately
        // before the soft target, where the hard admission budget matters.
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepBeforeTarget));
    }
    const qsizetype beforeLostCommand = transport.writes().size();
    expect(controller.setServoPwm(rb::ServoId::Servo1, 1500),
           "ordinary command should be accepted while its ACK is intentionally lost");
    expect(transport.writes().size() == beforeLostCommand + 1,
           "near-timeout ordinary command should produce one scheduler action");
    if (transport.writes().size() > beforeLostCommand) {
        expect(lastPacket(transport).type == rb::MessageType::Heartbeat,
               "ordinary command must be admitted behind a safety heartbeat, not started near its deadline");
    }

    waitForMs(config.ackTimeoutMs + 400);
    expect(controller.monitor().timeoutCount == 0,
           "a lost ordinary ACK should remain retryable without terminal scheduler timeout");
    int pwmWrites = 0;
    for (qsizetype index = 0; index < transport.writes().size(); ++index) {
        pwmWrites += packetAt(transport, index).type == rb::MessageType::SetServoPwm ? 1 : 0;
    }
    expect(pwmWrites >= 2,
           "the intentionally lost ordinary ACK should exercise the retry path");

    qint64 maxHeartbeatGapMs = 0;
    for (qsizetype index = 1; index < static_cast<qsizetype>(heartbeatSendTimes.size());
         ++index) {
        maxHeartbeatGapMs = qMax(maxHeartbeatGapMs,
                                 heartbeatSendTimes.at(index) - heartbeatSendTimes.at(index - 1));
    }
    const qint64 safetyMarginMs = kHardSafetyBudgetMs - maxHeartbeatGapMs;
    expect(maxHeartbeatGapMs <= kHardSafetyBudgetMs,
           "near-timeout heartbeat plus a lost command must stay inside the hard safety budget");
    expect(safetyMarginMs >= 10,
           "near-timeout heartbeat regression must retain at least 10 ms safety margin");
    std::cout << "APC220 near-timeout admission: heartbeat RTT=" << kHeartbeatRttMs
              << " ms, hard budget=" << kHardSafetyBudgetMs
              << " ms, observed max heartbeat gap=" << maxHeartbeatGapMs
              << " ms, observed safety margin=" << safetyMarginMs << " ms\n";
}

void testApc220FirstHeartbeatTimeoutFailsClosedBeforeRetryExhaustion()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 250;
    config.ackTimeoutMs = 250;
    config.maxRetries = 3;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    expect(controller.enableServo(rb::ServoId::Servo1),
           "first-timeout fail-closed test should enable Servo1 first");
    acknowledgeLast(transport);

    for (int waited = 0; waited < 700
         && (transport.writes().size() < 3
             || lastPacket(transport).type != rb::MessageType::Heartbeat);
         waited += 10) {
        waitForMs(10);
    }
    if (transport.writes().size() < 3
        || lastPacket(transport).type != rb::MessageType::Heartbeat) {
        expect(false, "first-timeout fail-closed test should have a later heartbeat in flight");
        return;
    }
    const qsizetype beforeOutageQueue = transport.writes().size();
    expect(controller.enableServo(rb::ServoId::Servo1),
           "outage-era Enable should be queued before heartbeat loss");
    expect(controller.setServoPwm(rb::ServoId::Servo1, 1500),
           "outage-era PWM should be queued before heartbeat loss");
    expect(controller.setServoAngle(rb::ServoId::Servo1, 100),
           "outage-era Angle should be queued before heartbeat loss");
    expect(controller.neutralServo(rb::ServoId::Servo1),
           "outage-era Neutral should be queued before heartbeat loss");
    expect(controller.queuedCommandCount() >= 4,
           "first-timeout fail-closed test should have stale actuator work queued");

    // The first heartbeat timeout is enough to fail closed.  maxRetries stays
    // at the production value so recovery can be acknowledged before terminal
    // retry exhaustion, after the local outage has crossed the 500 ms budget.
    for (int waited = 0; waited < 600 && controller.isServoEnabled(rb::ServoId::Servo1);
         waited += 10) {
        waitForMs(10);
    }
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "the first heartbeat ACK timeout must clear logical enabled state");
    expect(controller.queuedCommandCount() == 0,
           "the first heartbeat ACK timeout must clear stale queued actuator work");
    expect(controller.monitor().timeoutCount == 0,
           "first heartbeat fail-closed must occur before terminal retry exhaustion");
    if (controller.isServoEnabled(rb::ServoId::Servo1)
        || controller.monitor().timeoutCount != 0) {
        return;
    }

    // Wait until the outage has exceeded Firmware's 500 ms watchdog budget,
    // but remain before the fourth (terminal) heartbeat attempt.
    waitForMs(350);
    expect(!transport.writes().isEmpty()
               && lastPacket(transport).type == rb::MessageType::Heartbeat,
           "heartbeat retry bookkeeping should remain active after local actuator fail-closed");
    acknowledgeLast(transport);

    bool staleCommandSent = false;
    for (qsizetype index = beforeOutageQueue; index < transport.writes().size(); ++index) {
        staleCommandSent = staleCommandSent || packetAt(transport, index).type != rb::MessageType::Heartbeat;
    }
    expect(!staleCommandSent,
           "heartbeat recovery before terminal retry exhaustion must not replay outage-era actions");
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "heartbeat recovery must restore liveness without restoring Servo enabled state");
    expect(!controller.setServoPwm(rb::ServoId::Servo1, 1500),
           "PWM must remain blocked until a fresh post-outage Enable ACK");

    // The recovery ACK can immediately cause a new dispatch because the
    // original soft deadline elapsed during the outage.  ACK that one fresh
    // heartbeat as well, then the new user Enable must get the slot.
    if (!transport.writes().isEmpty()
        && lastPacket(transport).type == rb::MessageType::Heartbeat) {
        acknowledgeLast(transport);
    }
    const qsizetype beforeFreshEnable = transport.writes().size();
    expect(controller.enableServo(rb::ServoId::Servo1),
           "a fresh user Enable must be required after first-timeout fail-closed recovery");
    expect(transport.writes().size() == beforeFreshEnable + 1
               && lastPacket(transport).type == rb::MessageType::ServoEnable,
           "only the fresh Enable may reopen the actuator path");
    acknowledgeLast(transport);
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "matching ACK for the fresh Enable should restore logical control");
}

void testApc220FailClosedRejectsEnableUntilHeartbeatRecovery()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 250;
    config.ackTimeoutMs = 250;
    config.maxRetries = 3;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    expect(controller.enableServo(rb::ServoId::Servo1),
           "Enable gate test should establish Servo1 enabled state");
    acknowledgeLast(transport);
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "Enable ACK should establish the Enable gate precondition");

    for (int waited = 0; waited < 700
         && (transport.writes().size() < 3
             || lastPacket(transport).type != rb::MessageType::Heartbeat);
         waited += 10) {
        waitForMs(10);
    }
    if (transport.writes().size() < 3
        || lastPacket(transport).type != rb::MessageType::Heartbeat) {
        expect(false, "Enable gate test should have a heartbeat in flight");
        return;
    }

    // The first missed heartbeat ACK fail-closes actuators while the
    // production maxRetries=3 retry bookkeeping remains active.
    for (int waited = 0; waited < 600 && controller.isServoEnabled(rb::ServoId::Servo1);
         waited += 10) {
        waitForMs(10);
    }
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "first heartbeat timeout must fail close the logical Servo state");
    expect(!transport.writes().isEmpty()
               && lastPacket(transport).type == rb::MessageType::Heartbeat,
           "first heartbeat timeout must leave a heartbeat retry in flight");

    const qsizetype beforeRejectedEnable = transport.writes().size();
    expect(!controller.enableServo(rb::ServoId::Servo1),
           "Enable must be rejected while APC220 liveness is recovering");
    expect(controller.queuedCommandCount() == 0,
           "rejected recovery Enable must not enter the command queue");
    expect(transport.writes().size() == beforeRejectedEnable,
           "rejected recovery Enable must not write a frame");

    const qsizetype beforeRecoveryAck = transport.writes().size();
    acknowledgeLast(transport);
    // The dispatch-anchored soft deadline may already be due when the retry
    // ACK arrives, causing one fresh heartbeat before user work is admitted.
    if (transport.writes().size() > beforeRecoveryAck
        && lastPacket(transport).type == rb::MessageType::Heartbeat) {
        acknowledgeLast(transport);
    }
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "heartbeat recovery must not implicitly re-enable Servo1");
    expect(controller.queuedCommandCount() == 0,
           "heartbeat recovery must not recreate a rejected Enable");

    const qsizetype beforeFreshEnable = transport.writes().size();
    expect(controller.enableServo(rb::ServoId::Servo1),
           "a fresh Enable must be accepted after heartbeat recovery");
    expect(transport.writes().size() == beforeFreshEnable + 1
               && lastPacket(transport).type == rb::MessageType::ServoEnable,
           "fresh post-recovery Enable must be the next actuator frame");
    acknowledgeLast(transport);
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "matching ACK for fresh Enable must restore Servo1 control");
}

void testApc220HeartbeatAckDoesNotMoveHardDeadline()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 5;
    config.ackTimeoutMs = 1000;
    config.heartbeatSafetyBudgetMs = 5000;
    config.maxRetries = 0;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);

    waitForMs(20);
    if (transport.writes().size() < 2
        || lastPacket(transport).type != rb::MessageType::Heartbeat) {
        expect(false, "APC220 deadline test should have a later heartbeat in flight");
        return;
    }
    const rb::Packet heartbeat = lastPacket(transport);
    expect(controller.enableServo(rb::ServoId::Servo1),
           "Enable should queue behind the heartbeat deadline test exchange");
    const qsizetype beforeAck = transport.writes().size();

    // Let the dispatch-anchored heartbeat deadline elapse while its ACK is
    // still pending.  ACK processing must not move that deadline forward.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    acknowledge(transport, heartbeat, rb::AckResult::Ok, rb::MessageType::Heartbeat);

    expect(transport.writes().size() == beforeAck + 1,
           "an elapsed heartbeat deadline should dispatch exactly one follow-up heartbeat");
    if (transport.writes().size() > beforeAck) {
        expect(lastPacket(transport).type == rb::MessageType::Heartbeat,
               "heartbeat ACK must not postpone the dispatch-anchored hard deadline");
    }
}

void testApc220SustainedLoadPreservesHeartbeatSafetyMargin()
{
    constexpr int kObservedRttMs = 170;
    constexpr qint64 kFirmwareWatchdogMs = 500;
    constexpr qint64 kHeartbeatTargetMs = 250;
    constexpr qint64 kConfiguredHardSafetyBudgetMs = 490;
    constexpr qint64 kConfiguredSafetyMarginMs =
        kFirmwareWatchdogMs - kConfiguredHardSafetyBudgetMs;

    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.maxRetries = 0;
    rb::RobotController controller(&transport, config);
    std::vector<qint64> heartbeatSendTimes;
    transport.setWriteCallback([&transport, &heartbeatSendTimes](const QByteArray &wire) {
        const rb::DecodeResult decoded = rb::PacketCodec::decodeWire(
            wire.first(wire.size() - 1));
        if (!decoded.ok()) {
            return;
        }
        const rb::Packet request = decoded.packet;
        if (request.type == rb::MessageType::Heartbeat) {
            heartbeatSendTimes.push_back(monotonicNowMs());
        }
        QTimer::singleShot(kObservedRttMs, &transport, [&transport, request] {
            QByteArray payload;
            payload.append(static_cast<char>(request.sequence & 0xffU));
            payload.append(static_cast<char>((request.sequence >> 8U) & 0xffU));
            payload.append(static_cast<char>(request.type));
            payload.append(static_cast<char>(rb::AckResult::Ok));
            transport.injectBytes(rb::PacketCodec::encodeWire(
                {rb::MessageType::Ack, 0x8000, payload}));
        });
    });

    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();
    waitForMs(kObservedRttMs + 50);
    expect(controller.isServoEnabled(rb::ServoId::Servo1) == false,
           "sustained-load timing test must begin with Servo1 disabled");
    expect(controller.enableServo(rb::ServoId::Servo1),
           "sustained-load timing test should send a fresh Enable");
    // The admission guard may insert a safety heartbeat before this first
    // ordinary exchange; allow that heartbeat and the Enable ACK to finish.
    waitForMs(450);
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "scheduled Enable ACK should establish the sustained-load precondition");

    for (int index = 0; index < 7; ++index) {
        expect(controller.setServoPwm(rb::ServoId::Servo1,
                                      static_cast<quint16>(1500 + index)),
               "multiple user commands should be accepted into the APC220 bounded queue");
    }
    waitForMs(3500);

    expect(heartbeatSendTimes.size() >= 3,
           "sustained load should provide several heartbeat exchanges to measure");
    qint64 maxHeartbeatGapMs = 0;
    for (qsizetype index = 1; index < static_cast<qsizetype>(heartbeatSendTimes.size());
         ++index) {
        maxHeartbeatGapMs = qMax(maxHeartbeatGapMs,
                                 heartbeatSendTimes.at(index) - heartbeatSendTimes.at(index - 1));
    }

    qsizetype ordinarySinceHeartbeat = 0;
    for (qsizetype index = 0; index < transport.writes().size(); ++index) {
        const rb::Packet packet = packetAt(transport, index);
        if (packet.type == rb::MessageType::Heartbeat) {
            expect(ordinarySinceHeartbeat <= 1,
                   "APC220 sustained load must not put two ordinary commands between heartbeats");
            ordinarySinceHeartbeat = 0;
        } else {
            ++ordinarySinceHeartbeat;
        }
    }
    expect(ordinarySinceHeartbeat <= 1,
           "APC220 sustained load must not leave multiple ordinary commands after the last heartbeat");
    expect(maxHeartbeatGapMs <= kConfiguredHardSafetyBudgetMs,
           "APC220 sustained load must keep heartbeat wire gaps inside the watchdog budget");
    expect(controller.monitor().timeoutCount == 0,
           "scheduled 170 ms APC220 ACKs must not consume timeout budget");
    std::cout << "APC220 sustained load: RTT=" << kObservedRttMs
              << " ms, watchdog=" << kFirmwareWatchdogMs
              << " ms, illustrative nominal target+RTT="
              << (kHeartbeatTargetMs + kObservedRttMs)
              << " ms, configured hard budget=" << kConfiguredHardSafetyBudgetMs
              << " ms, configured margin=" << kConfiguredSafetyMarginMs
              << " ms, observed max heartbeat gap=" << maxHeartbeatGapMs << " ms\n";
}

void testApc220HeartbeatTerminalLossFailsClosedAndRequiresFreshEnable()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 100;
    config.ackTimeoutMs = 20;
    config.maxRetries = 0;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    expect(controller.enableServo(rb::ServoId::Servo1),
           "liveness convergence test should establish Servo1 enabled state");
    acknowledgeLast(transport);
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "Enable ACK should establish the liveness convergence precondition");

    waitForMs(110);
    if (transport.writes().size() < 2
        || lastPacket(transport).type != rb::MessageType::Heartbeat) {
        expect(false, "liveness convergence test should have a heartbeat in flight");
        return;
    }
    const qsizetype beforeLoss = transport.writes().size();
    expect(controller.enableServo(rb::ServoId::Servo1),
           "an outage-era Enable must be queued before terminal heartbeat loss");
    expect(controller.setServoPwm(rb::ServoId::Servo1, 1500),
           "an outage-era PWM must be queued before terminal heartbeat loss");
    expect(controller.setServoAngle(rb::ServoId::Servo1, 100),
           "an outage-era angle must be queued before terminal heartbeat loss");
    expect(controller.neutralServo(rb::ServoId::Servo1),
           "an outage-era Neutral must be queued before terminal heartbeat loss");
    expect(controller.queuedCommandCount() >= 4,
           "liveness convergence test should have stale actuator work to discard");

    waitForMs(45);
    expect(controller.monitor().timeoutCount >= 1,
           "terminal heartbeat loss should be observable as a timeout");
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "terminal heartbeat loss must clear Console logical enabled state");
    expect(controller.queuedCommandCount() == 0,
           "terminal heartbeat loss must discard stale queued actuator commands");
    expect(!transport.writes().isEmpty()
               && lastPacket(transport).type == rb::MessageType::Heartbeat,
           "liveness recovery should keep a fresh heartbeat exchange in flight");

    acknowledgeLast(transport);
    bool staleCommandSent = false;
    for (qsizetype index = beforeLoss; index < transport.writes().size(); ++index) {
        staleCommandSent = staleCommandSent || packetAt(transport, index).type != rb::MessageType::Heartbeat;
    }
    expect(!staleCommandSent,
           "heartbeat recovery must not replay outage-era Enable/PWM/Angle/Neutral commands");
    expect(controller.queuedCommandCount() == 0,
           "heartbeat recovery must not recreate stale queued work");
    expect(controller.setServoPwm(rb::ServoId::Servo1, 1500) == false,
           "PWM must remain blocked until a new user Enable ACK after liveness loss");
    const qsizetype beforeFreshEnable = transport.writes().size();
    expect(controller.enableServo(rb::ServoId::Servo1),
           "a new explicit Enable should be accepted after heartbeat recovery");
    expect(transport.writes().size() == beforeFreshEnable + 1
               && lastPacket(transport).type == rb::MessageType::ServoEnable,
           "only the new explicit Enable may reopen actuator control");
    acknowledgeLast(transport);
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "matching ACK for the new Enable should restore logical control");
}

void testApc220HeartbeatLossClearsDeferredRetry()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 30;
    config.ackTimeoutMs = 10;
    config.maxRetries = 1;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    expect(controller.enableServo(rb::ServoId::Servo1),
           "deferred-retry liveness test should enable Servo1 first");
    acknowledgeLast(transport);
    expect(controller.setServoPwm(rb::ServoId::Servo1, 1500),
           "deferred-retry liveness test should start an ordinary exchange");
    const qsizetype beforeMotion = transport.writes().size() - 1;

    // The ordinary PWM times out once before the heartbeat deadline, then its
    // next retry is deferred behind a heartbeat.  The terminal heartbeat loss
    // must clear that deferred retry rather than replaying it after recovery.
    waitForMs(90);
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "terminal heartbeat loss must clear enabled state with a deferred retry present");
    expect(controller.queuedCommandCount() == 0,
           "terminal heartbeat loss must clear deferred and queued actuator work");
    expect(!transport.writes().isEmpty()
               && lastPacket(transport).type == rb::MessageType::Heartbeat,
           "deferred-retry liveness loss should leave a fresh heartbeat in flight");
    acknowledgeLast(transport);

    int pwmWritesAfterStart = 0;
    for (qsizetype index = beforeMotion; index < transport.writes().size(); ++index) {
        pwmWritesAfterStart += packetAt(transport, index).type == rb::MessageType::SetServoPwm
            ? 1
            : 0;
    }
    expect(pwmWritesAfterStart <= 2,
           "deferred retry must not replay the stale PWM after heartbeat recovery");
    expect(!controller.setServoPwm(rb::ServoId::Servo1, 1500),
           "stale PWM must remain blocked until a fresh Enable ACK");
}

void testApc220DisableAllPrioritizesAndClearsStaleMotionQueue()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 1000;
    config.heartbeatSafetyBudgetMs = 5000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    expect(controller.enableServo(rb::ServoId::Servo1),
           "Disable priority test should enable Servo1 first");
    acknowledgeLast(transport);
    expect(controller.setServoPwm(rb::ServoId::Servo1, 1500),
           "first motion command should occupy the APC220 in-flight slot");
    const rb::Packet inFlightPwm = lastPacket(transport);
    expect(controller.setServoPwm(rb::ServoId::Servo1, 1510),
           "second PWM should enter the ordinary queue");
    expect(controller.setServoAngle(rb::ServoId::Servo1, 100),
           "Set Angle should enter the ordinary queue before Disable All");
    expect(controller.neutralServo(rb::ServoId::Servo1),
           "Neutral should enter the ordinary queue before Disable All");
    expect(controller.disableAll(),
           "Disable All should be accepted as a safety-priority request");
    expect(controller.queuedCommandCount() == 1,
           "Disable All should evict stale queued motion commands and retain only itself");
    expect(controller.isServoDisablePending(rb::ServoId::Servo1),
           "Disable All should mark Servo1 pending before priority dispatch");

    acknowledge(transport, inFlightPwm, rb::AckResult::Ok, rb::MessageType::SetServoPwm);
    expect(!transport.writes().isEmpty()
               && lastPacket(transport).type == rb::MessageType::ServoDisable,
           "Disable All must dispatch before stale queued motion after the current exchange");
    expect(controller.queuedCommandCount() == 0,
           "priority Disable All should leave no stale motion commands queued");
    acknowledgeLast(transport);
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "successful priority Disable All should clear logical enabled state");
    expect(!controller.isServoDisablePending(rb::ServoId::Servo1),
           "successful priority Disable All should clear pending state");
    for (qsizetype index = 0; index < transport.writes().size(); ++index) {
        const rb::MessageType type = packetAt(transport, index).type;
        expect(type != rb::MessageType::SetServoAngle && type != rb::MessageType::Neutral,
               "stale queued Angle/Neutral must not reach the wire after Disable All");
    }
}

void testApc220DisablePriorityPrecedesOrdinaryRetry()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 20;
    config.maxRetries = 1;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    expect(controller.enableServo(rb::ServoId::Servo1),
           "Disable retry-priority test should enable Servo1 first");
    acknowledgeLast(transport);
    expect(controller.setServoPwm(rb::ServoId::Servo1, 1500),
           "ordinary PWM should be in flight before Disable retry-priority test");
    expect(controller.disableAll(),
           "Disable All should queue behind the uncancellable PWM exchange");
    waitForMs(35);

    int pwmWrites = 0;
    for (qsizetype index = 0; index < transport.writes().size(); ++index) {
        pwmWrites += packetAt(transport, index).type == rb::MessageType::SetServoPwm ? 1 : 0;
    }
    expect(pwmWrites == 1,
           "a safety Disable must not be delayed by an ordinary PWM retry");
    expect(!transport.writes().isEmpty()
               && lastPacket(transport).type == rb::MessageType::ServoDisable,
           "Disable All must dispatch after the timed-out exchange before ordinary retry");
    acknowledgeLast(transport);
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "successful retry-priority Disable should leave Servo1 disabled");
}

void testApc220ErrorTypeMismatchKeepsPendingDisable()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    expect(controller.enableServo(rb::ServoId::Servo1),
           "Enable should be sent before Error type validation test");
    acknowledgeLast(transport);
    expect(controller.disableServo(rb::ServoId::Servo1),
           "Disable should be sent before Error type validation test");
    const rb::Packet disable = lastPacket(transport);

    QByteArray payload;
    payload.append(static_cast<char>(disable.sequence & 0xff));
    payload.append(static_cast<char>((disable.sequence >> 8) & 0xff));
    payload.append(static_cast<char>(rb::MessageType::ServoEnable));
    payload.append(static_cast<char>(0x34));
    payload.append(static_cast<char>(0x12));
    transport.injectBytes(rb::PacketCodec::encodeWire(
        {rb::MessageType::Error, 0x8000, payload}));

    expect(controller.isServoDisablePending(rb::ServoId::Servo1),
           "Error with mismatched request type must keep Disable pending");
    acknowledge(transport, disable, rb::AckResult::Ok, rb::MessageType::ServoDisable);
    expect(!controller.isServoDisablePending(rb::ServoId::Servo1),
           "matching Disable ACK should release pending after mismatched Error");
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "matching Disable ACK should still disable Servo1");
}

void testApc220ErrorOnlySignalResetsScheduler()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    expect(controller.enableServo(rb::ServoId::Servo1),
           "Enable should be active before error-only reset test");
    acknowledgeLast(transport);
    expect(controller.disableAll(),
           "Disable All should be queued before error-only reset test");

    emit transport.errorOccurred(QStringLiteral("error without state transition"));
    expect(!controller.isConnected(),
           "APC220 error-only signal should put the controller in Error state");
    expect(controller.queuedCommandCount() == 0,
           "APC220 error-only signal must clear queued user commands");
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "APC220 error-only signal must clear logical enabled state");
    expect(!controller.isServoDisablePending(rb::ServoId::Servo1),
           "APC220 error-only signal must clear Disable pending state");
}

void testApc220RetryWriteFailureConsumesRetryBudget()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 1;
    config.maxRetries = 1;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    expect(controller.enableServo(rb::ServoId::Servo1),
           "Enable should be active before retry write failure test");
    const QByteArray originalFrame = transport.writes().last();
    const qsizetype commandWriteCount = transport.writes().size();
    transport.setWriteErrorSignals(false);
    transport.setWriteSucceeds(false);
    waitForMs(35);

    expect(transport.writes().size() == commandWriteCount,
           "a failed APC220 retry must not report a successful wire write");
    expect(transport.writes().last() == originalFrame,
           "successful APC220 retry attempt must preserve sequence and frame");
    expect(controller.monitor().timeoutCount == 1,
           "a failed APC220 retry attempt must consume budget and reach terminal timeout");
    transport.setWriteSucceeds(true);
    expect(controller.enableServo(rb::ServoId::Servo1),
           "APC220 slot should be reusable after failed retry reaches timeout");
}

void testApc220QueuedWriteFailureDropsCommandAndClearsDisablePending()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 1000;
    config.heartbeatSafetyBudgetMs = 5000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    expect(controller.enableServo(rb::ServoId::Servo1),
           "Enable should be active before queued write failure test");
    acknowledgeLast(transport);
    expect(controller.setServoPwm(rb::ServoId::Servo1, 1500),
           "PWM should occupy the APC220 slot before queueing Disable");
    expect(controller.disableServo(rb::ServoId::Servo1),
           "Disable should queue behind PWM");
    expect(controller.isServoDisablePending(rb::ServoId::Servo1),
           "queued Disable should mark pending before dispatch");

    transport.setWriteErrorSignals(false);
    transport.setWriteSucceeds(false);
    const rb::Packet pwm = lastPacket(transport);
    acknowledge(transport, pwm, rb::AckResult::Ok, rb::MessageType::SetServoPwm);
    expect(controller.queuedCommandCount() == 0,
           "failed queued dispatch must not requeue forever");
    expect(!controller.isServoDisablePending(rb::ServoId::Servo1),
           "failed queued Disable dispatch must clear pending state");
}

void testApc220WriteErrorResetsWithoutInvalidatingRetryState()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 1;
    config.maxRetries = 1;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    expect(controller.enableServo(rb::ServoId::Servo1),
           "Enable should be active before synchronous write error test");
    transport.setWriteSucceeds(false);
    waitForMs(30);
    expect(!controller.isConnected(),
           "APC220 write error should reset transport state to Error");
    expect(controller.queuedCommandCount() == 0,
           "APC220 write error should clear scheduler state");
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    testNoAutomaticEnableAndPwmRequiresEnable();
    testLeakStatusMappingAndPendingAckIsolation();
    testLeakStatusStaleAndDisconnectTransitions();
    testImuSnapshotDoesNotTouchAckOrLeakState();
    testDepthSnapshotDoesNotTouchAckOrLeakState();
    testApcHeartbeatLossClearsLeakState();
    testAngleDegreesConvertToCentidegrees();
    testSetAngleBlockedDuringDisableRequest();
    testApc220DisablePendingBlocksAllMotionAndErrorReleasesNoStaleWork();
    testApc220DisableTimeoutReleasesNoStaleMotion();
    testDisableAckClearsPendingWhenAlreadyDisabled();
    testDisablePendingWriteFailureDoesNotLockAngle();
    testDisablePendingAckRejectedRestoresAngle();
    testDisablePendingAckTypeMismatchRestoresAngle();
    testDisablePendingErrorRestoresAngle();
    testDisablePendingTimeoutRestoresAngle();
    testDisablePendingDisconnectClearsState();
    testDisablePendingSuccessDisablesAngle();
    testPwmCalibrationAndBounds();
    testSetAngleEncodingAndBounds();
    testDisconnectAttemptsDisableAll();
    testNeutralEncodingAndAck();
    testFrontAxisUsesCalibratedPwmAndAngles();
    testSemanticServoCommandBoundaries();
    testAckRejectionAndMatching();
    testRetryReusesIdenticalSequenceAndFrame();
    testUnexpectedTransportLossRecordsDisableFailure();
    testApc220ProfileUsesHalfDuplexTiming();
    testDirectUartRetainsMultiplePendingRequests();
    testApc220AllowsOnlyOneAckRequiringFrameInFlight();
    testApc220HeartbeatTicksCoalesceWhileCommandIsInFlight();
    testApc220HeartbeatDuePrecedesCommandRetry();
    testApc220HeartbeatDeadlineWinsRetryBoundary();
    testApc220HeartbeatDeadlineRefreshesBeforeAckReleasesSlot();
    testApc220CommandQueueHasBoundedCapacity();
    testApc220RetryUsesOriginalSequenceAndFrame();
    testApc220TransportResetClearsSchedulerAndDoesNotAutoEnable();
    testApc220RecordsMatchingAckRtt();
    testApc220SynchronousAckDuringInitialDispatchIsHandled();
    testApc220SynchronousDisableAckCasesDoNotRelockPending();
    testApc220ConnectGatesCommandsUntilHeartbeatAck();
    testApc220HeartbeatRejectionKeepsUserCommandsGated();
    testApc220HeartbeatTimeoutKeepsUserCommandsGated();
    testApc220HeartbeatTypeMismatchKeepsUserCommandsGated();
    testApc220HeartbeatTicksDoNotBurstAndQueueGetsChanceAfterAck();
    testApc220SafetyAdmissionProtectsNearTimeoutHeartbeat();
    testApc220FirstHeartbeatTimeoutFailsClosedBeforeRetryExhaustion();
    testApc220FailClosedRejectsEnableUntilHeartbeatRecovery();
    testApc220HeartbeatAckDoesNotMoveHardDeadline();
    testApc220SustainedLoadPreservesHeartbeatSafetyMargin();
    testApc220HeartbeatTerminalLossFailsClosedAndRequiresFreshEnable();
    testApc220HeartbeatLossClearsDeferredRetry();
    testApc220DisableAllPrioritizesAndClearsStaleMotionQueue();
    testApc220DisablePriorityPrecedesOrdinaryRetry();
    testApc220ErrorTypeMismatchKeepsPendingDisable();
    testApc220ErrorOnlySignalResetsScheduler();
    testApc220RetryWriteFailureConsumesRetryBudget();
    testApc220QueuedWriteFailureDropsCommandAndClearsDisablePending();
    testApc220WriteErrorResetsWithoutInvalidatingRetryState();
    if (failures == 0) {
        std::cout << "All robot controller tests passed\n";
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
