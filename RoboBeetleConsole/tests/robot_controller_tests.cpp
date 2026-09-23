#include <QCoreApplication>
#include <QByteArray>
#include <QByteArrayView>
#include <QEventLoop>
#include <QHash>
#include <QObject>
#include <QQueue>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QVector>
#include <QtGlobal>

#include <cstdlib>
#include <chrono>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include "protocol/PacketCodec.h"
#include "protocol/StreamDecoder.h"
#include "robot/DepthMonitor.h"
#include "robot/DepthSnapshot.h"
#include "robot/ImuMonitor.h"
#include "robot/ImuSnapshot.h"
#include "robot/LeakStatus.h"
#include "robot/RobotCommand.h"
#include "robot/ServoDescriptor.h"
#include "transport/FakeTransport.h"
#include "transport/ITransport.h"

// The Motion STOP eviction regression needs to construct a queue-full fixture
// that the public API cannot create after Motion owns the actuator commands.
// Keep this test-only access local to the test translation unit; production
// visibility and behavior are unchanged.
#define private public
#include "robot/RobotController.h"
#undef private

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

void acknowledgeSequence(rb::FakeTransport &transport,
                          quint16 sequence,
                          rb::AckResult result,
                          rb::MessageType acknowledgedType)
{
    QByteArray payload;
    payload.append(static_cast<char>(sequence & 0xffU));
    payload.append(static_cast<char>((sequence >> 8U) & 0xffU));
    payload.append(static_cast<char>(acknowledgedType));
    payload.append(static_cast<char>(result));
    transport.injectBytes(rb::PacketCodec::encodeWire(
        {rb::MessageType::Ack, 0x8000, payload}));
}

void injectError(rb::FakeTransport &transport,
                 quint16 sequence,
                 rb::MessageType requestType,
                 quint16 errorCode = 0x0001)
{
    QByteArray payload;
    payload.append(static_cast<char>(sequence & 0xffU));
    payload.append(static_cast<char>((sequence >> 8U) & 0xffU));
    payload.append(static_cast<char>(requestType));
    payload.append(static_cast<char>(errorCode & 0xffU));
    payload.append(static_cast<char>((errorCode >> 8U) & 0xffU));
    transport.injectBytes(rb::PacketCodec::encodeWire(
        {rb::MessageType::Error, 0x8000, payload}));
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
            || requestType == rb::MessageType::Neutral
            || requestType == rb::MessageType::SetMotionMode;
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

void testDirectHeartbeatLossIgnoresLateEnableAck()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10;
    config.ackTimeoutMs = 100;
    config.maxRetries = 0;
    rb::RobotController controller(&transport, config);
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    waitForMs(40);
    expect(controller.enableServo(rb::ServoId::Servo1),
           "DirectUart late-ACK setup should send Enable after a heartbeat is in flight");
    const rb::Packet enable = lastPacket(transport);

    bool forcedHeartbeatTimeout = false;
    for (auto it = controller.pending_.begin(); it != controller.pending_.end(); ++it) {
        if (it->type == rb::MessageType::Heartbeat) {
            it->sentAtMs = rb::RobotController::nowMs() - config.ackTimeoutMs - 1;
            forcedHeartbeatTimeout = true;
            break;
        }
    }
    expect(forcedHeartbeatTimeout,
           "DirectUart late-ACK setup should have a heartbeat request in flight");
    controller.checkTimeouts();

    const auto enablePending = controller.pending_.find(enable.sequence);
    expect(enablePending != controller.pending_.end() && enablePending->cancelled,
           "DirectUart heartbeat loss must cancel the in-flight Enable before its late ACK");
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "DirectUart heartbeat loss should clear state before a late Enable ACK");

    acknowledge(transport, enable, rb::AckResult::Ok,
                rb::MessageType::ServoEnable);
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "late DirectUart Enable ACK must not re-arm fail-closed state");
}

void testDirectHeartbeatLossClearsEnabledState()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 5;
    config.ackTimeoutMs = 5;
    config.maxRetries = 0;
    rb::RobotController controller(&transport, config);
    connectAndEnableServo1(transport, controller);

    waitForMs(40);
    expect(!controller.isServoEnabled(rb::ServoId::Servo1),
           "DirectUart heartbeat loss must clear the logical enabled mask");
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
    const rb::ServoDescriptor *frontLeft = rb::servoDescriptor(rb::ServoId::FrontLeft);
    expect(descriptor != nullptr, "FrontRight descriptor must exist");
    expect(frontLeft != nullptr, "FrontLeft descriptor must exist");
    expect(descriptor->commandMinPwmUs == 1160, "FrontRight command minimum must be 1160 us");
    expect(descriptor->neutralPwmUs == 1450, "FrontRight neutral must be 1450 us");
    expect(descriptor->commandMaxPwmUs == 1900, "FrontRight command maximum must be 1900 us");
    expect(frontLeft->commandMinPwmUs == 1140, "FrontLeft command minimum must be 1140 us");
    expect(frontLeft->commandMaxPwmUs == 1860, "FrontLeft command maximum must be 1860 us");
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();
    controller.enableServo(rb::ServoId::Servo1);
    acknowledgeLast(transport);
    const qsizetype before = transport.writes().size();

    expect(!controller.setServoPwm(rb::ServoId::Servo1, 1159),
           "FrontRight PWM below command range must be rejected");
    expect(!controller.setServoPwm(rb::ServoId::Servo1, 1901),
           "FrontRight PWM above command range must be rejected");
    expect(transport.writes().size() == before, "rejected commands must not write frames");

    expect(controller.setServoPwm(rb::ServoId::Servo1, 1160),
           "FrontRight PWM minimum boundary must be accepted");
    expect(lastPacket(transport).payload == QByteArray::fromHex("01008804"),
           "FrontRight PWM minimum must be encoded as count, FrontRight, uint16 LE");
    expect(controller.setServoPwm(rb::ServoId::Servo1, 1900),
           "FrontRight PWM maximum boundary must be accepted");
    expect(lastPacket(transport).payload == QByteArray::fromHex("01006c07"),
           "FrontRight PWM maximum must be encoded as count, FrontRight, uint16 LE");

    expect(controller.enableServo(rb::ServoId::FrontLeft),
           "FrontLeft enable should be sent for its raw command-bound test");
    acknowledgeLast(transport);
    const qsizetype frontLeftBefore = transport.writes().size();
    expect(!controller.setServoPwm(rb::ServoId::FrontLeft, 1139),
           "FrontLeft PWM below command range must be rejected");
    expect(!controller.setServoPwm(rb::ServoId::FrontLeft, 1861),
           "FrontLeft PWM above command range must be rejected");
    expect(transport.writes().size() == frontLeftBefore,
           "FrontLeft rejected commands must not write frames");
    expect(controller.setServoPwm(rb::ServoId::FrontLeft, 1140),
           "FrontLeft PWM minimum boundary must be accepted");
    expect(lastPacket(transport).payload == QByteArray::fromHex("01017404"),
           "FrontLeft PWM minimum must be encoded as count, FrontLeft, uint16 LE");
    expect(controller.setServoPwm(rb::ServoId::FrontLeft, 1860),
           "FrontLeft PWM maximum boundary must be accepted");
    expect(lastPacket(transport).payload == QByteArray::fromHex("01014407"),
           "FrontLeft PWM maximum must be encoded as count, FrontLeft, uint16 LE");
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
        {rb::ServoId::FrontRight, 1160, 1900, -4500, 4500, true},
        {rb::ServoId::FrontLeft, 1140, 1860, -4500, 4500, true},
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

void enablePaddlesOnly(rb::FakeTransport &transport,
                       rb::RobotController &controller)
{
    const rb::ServoId paddles[] = {
        rb::ServoId::FrontRight,
        rb::ServoId::FrontLeft,
        rb::ServoId::RearRight,
        rb::ServoId::RearLeft,
    };
    for (const rb::ServoId id : paddles) {
        expect(controller.enableServo(id),
               "Motion setup should send each paddle Enable");
        acknowledgeLast(transport);
    }
}

void connectAndEnablePaddles(rb::FakeTransport &transport,
                             rb::RobotController &controller)
{
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();
    enablePaddlesOnly(transport, controller);
}

void testMotionStartStopStateAndWireContract()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    connectAndEnablePaddles(transport, controller);

    expect(controller.motionState() == rb::MotionState::Stopped,
           "Motion should start in Stopped state");
    expect(controller.startMotion(rb::MotionMode::Forward),
           "Forward Motion START should be sent");
    const rb::Packet start = lastPacket(transport);
    QByteArray expectedStart;
    expectedStart.append(static_cast<char>(1));
    expectedStart.append(static_cast<char>(rb::MotionMode::Forward));
    expectedStart.append(static_cast<char>(rb::MotionAction::Start));
    expect(start.type == rb::MessageType::SetMotionMode &&
               start.payload == expectedStart,
           "Motion START should use exact schema/mode/action payload");
    acknowledgeLast(transport);
    expect(controller.motionState() == rb::MotionState::Running,
           "successful START ACK should enter Running");

    expect(controller.stopMotion(),
           "ordinary Motion STOP should be sent");
    const rb::Packet stop = lastPacket(transport);
    QByteArray expectedStop;
    expectedStop.append(static_cast<char>(1));
    expectedStop.append(static_cast<char>(rb::MotionMode::Stop));
    expectedStop.append(static_cast<char>(rb::MotionAction::Stop));
    expect(stop.type == rb::MessageType::SetMotionMode &&
               stop.payload == expectedStop,
           "Motion STOP should use canonical STOP payload");
    acknowledgeLast(transport);
    expect(controller.motionState() == rb::MotionState::Stopping,
           "STOP ACK should enter Stopping before the provisional ramp ends");
    waitForMs(800);
    expect(controller.motionState() == rb::MotionState::Stopped,
           "the provisional UI transition should finish at Stopped");
}

void testBackwardRemainsProtocolCompatibleButIsNotBenchStartable()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    connectAndEnablePaddles(transport, controller);

    const qsizetype writesBefore = transport.writes().size();
    expect(!controller.isMotionReady(rb::MotionMode::Backward),
           "BACKWARD must remain unavailable for the bench UI");
    expect(!controller.startMotion(rb::MotionMode::Backward),
           "BACKWARD must not send a fake sign-inverted START");
    expect(transport.writes().size() == writesBefore,
           "rejected BACKWARD START must not write a protocol frame");
}

void testMotionStartSerializesDirectModeChange()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    connectAndEnablePaddles(transport, controller);
    expect(controller.enableServo(rb::ServoId::FrontAxis),
           "Direct Motion serialization setup should enable FrontAxis");
    acknowledgeLast(transport);

    expect(controller.startMotion(rb::MotionMode::Ascend),
           "Direct Motion serialization setup should start Ascend");
    acknowledgeLast(transport);
    expect(controller.startMotion(rb::MotionMode::Forward),
           "Direct Motion serialization setup should start Forward transition");
    const rb::Packet firstModeChange = lastPacket(transport);
    const qsizetype writesBeforeDuplicate = transport.writes().size();

    const bool duplicateAccepted = controller.startMotion(rb::MotionMode::Forward);
    const bool duplicateWasWritten = transport.writes().size() > writesBeforeDuplicate;
    const rb::Packet duplicate = lastPacket(transport);
    expect(!duplicateAccepted,
           "DirectUart must reject a duplicate Motion START before the first ACK");
    expect(!duplicateWasWritten,
           "DirectUart duplicate Motion START must not allocate another sequence");

    acknowledge(transport, firstModeChange, rb::AckResult::Ok,
                rb::MessageType::SetMotionMode);
    if (duplicateWasWritten) {
        acknowledge(transport, duplicate, rb::AckResult::Ok,
                    rb::MessageType::SetMotionMode);
    }
    expect(controller.disableServo(rb::ServoId::FrontAxis),
           "FrontAxis Disable should remain a safety takeover during the transition");
    expect(controller.motionState() == rb::MotionState::Faulted,
           "duplicate START handling must not release old-mode ownership early");
}

void testMotionStartSerializesApc220ModeChange()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 1000;
    config.heartbeatSafetyBudgetMs = 5000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    enablePaddlesOnly(transport, controller);
    expect(controller.enableServo(rb::ServoId::FrontAxis),
           "APC220 Motion serialization setup should enable FrontAxis");
    acknowledgeLast(transport);

    expect(controller.startMotion(rb::MotionMode::Ascend),
           "APC220 Motion serialization setup should start Ascend");
    const rb::Packet ascend = lastPacket(transport);
    acknowledge(transport, ascend, rb::AckResult::Ok,
                rb::MessageType::SetMotionMode);
    expect(controller.startMotion(rb::MotionMode::Forward),
           "APC220 Motion serialization setup should start Forward transition");
    const rb::Packet firstModeChange = lastPacket(transport);
    const qsizetype writesBeforeDuplicate = transport.writes().size();

    const bool duplicateAccepted = controller.startMotion(rb::MotionMode::Forward);
    const bool duplicateWasWritten = transport.writes().size() > writesBeforeDuplicate;
    expect(!duplicateAccepted,
           "APC220 must reject a duplicate Motion START before the first ACK");
    expect(!duplicateWasWritten && controller.queuedCommandCount() == 0,
           "APC220 duplicate Motion START must not enter its command queue");

    const qsizetype writesBeforeFirstAck = transport.writes().size();
    acknowledge(transport, firstModeChange, rb::AckResult::Ok,
                rb::MessageType::SetMotionMode);
    const bool duplicateWasDispatched = transport.writes().size() > writesBeforeFirstAck;
    const rb::Packet duplicate = lastPacket(transport);
    if (duplicateWasDispatched) {
        acknowledge(transport, duplicate, rb::AckResult::Ok,
                    rb::MessageType::SetMotionMode);
    }
    expect(controller.disableServo(rb::ServoId::FrontAxis),
           "APC220 FrontAxis Disable should remain a safety takeover during the transition");
    expect(controller.motionState() == rb::MotionState::Faulted,
           "APC220 duplicate START handling must not release old-mode ownership early");
}

void testMotionStopSupersedesInFlightDirectStart()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    connectAndEnablePaddles(transport, controller);

    expect(controller.startMotion(rb::MotionMode::Forward),
           "Direct STOP race setup should send START");
    const rb::Packet start = lastPacket(transport);
    const qsizetype writesBeforeDuplicate = transport.writes().size();
    expect(!controller.startMotion(rb::MotionMode::Forward),
           "Direct STOP race should reject a duplicate unresolved START");
    expect(transport.writes().size() == writesBeforeDuplicate,
           "Direct STOP race duplicate START must not write a frame");
    const qsizetype writesBeforeStop = transport.writes().size();

    expect(controller.stopMotion(),
           "STOP should be accepted while START is still awaiting ACK");
    expect(transport.writes().size() == writesBeforeStop + 1,
           "Direct STOP should be sent after cancelling stale START handling");
    const rb::Packet stop = lastPacket(transport);
    QByteArray expectedStop;
    expectedStop.append(static_cast<char>(1));
    expectedStop.append(static_cast<char>(rb::MotionMode::Stop));
    expectedStop.append(static_cast<char>(rb::MotionAction::Stop));
    expect(stop.type == rb::MessageType::SetMotionMode &&
               stop.payload == expectedStop,
           "Direct STOP should use the canonical STOP payload");

    acknowledge(transport, start, rb::AckResult::Ok,
                rb::MessageType::SetMotionMode);
    expect(controller.motionState() != rb::MotionState::Running,
           "a late Direct START ACK must not resurrect Motion");
    acknowledge(transport, stop, rb::AckResult::Ok,
                rb::MessageType::SetMotionMode);
    expect(controller.motionState() == rb::MotionState::Stopping,
           "STOP ACK should mark request acceptance and enter Stopping");
}

void testMotionStopSupersedesApcInFlightAndQueuedMotion()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 1000;
    config.heartbeatSafetyBudgetMs = 5000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    enablePaddlesOnly(transport, controller);

    expect(controller.startMotion(rb::MotionMode::Forward),
           "APC STOP race setup should send START");
    const rb::Packet start = lastPacket(transport);
    expect(!controller.startMotion(rb::MotionMode::TurnLeft),
           "APC STOP race should reject a second unresolved mode change");
    expect(!controller.startMotion(rb::MotionMode::TurnRight),
           "APC STOP race should reject another unresolved mode change");
    expect(controller.queuedCommandCount() == 0,
           "APC STOP race setup should not queue duplicate Motion changes");

    expect(controller.stopMotion(),
           "APC STOP should supersede in-flight and queued Motion work");
    expect(controller.queuedCommandCount() == 1,
           "APC STOP should discard stale queued Motion and retain one STOP");

    acknowledge(transport, start, rb::AckResult::Ok,
                rb::MessageType::SetMotionMode);
    expect(lastPacket(transport).type == rb::MessageType::SetMotionMode,
           "APC STOP should dispatch after the cancelled in-flight START slot clears");
    const rb::Packet stop = lastPacket(transport);
    QByteArray expectedStop;
    expectedStop.append(static_cast<char>(1));
    expectedStop.append(static_cast<char>(rb::MotionMode::Stop));
    expectedStop.append(static_cast<char>(rb::MotionAction::Stop));
    expect(stop.payload == expectedStop,
           "APC queued STOP should use the canonical STOP payload");
    acknowledge(transport, stop, rb::AckResult::Ok,
                rb::MessageType::SetMotionMode);
    expect(controller.motionState() == rb::MotionState::Stopping,
           "APC STOP ACK should enter Stopping after stale START ACK");
}

void testApcMotionStopCancelsDeferredMotionRetry()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 20;
    config.maxRetries = 1;
    config.heartbeatSafetyBudgetMs = 5000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);
    enablePaddlesOnly(transport, controller);

    expect(controller.startMotion(rb::MotionMode::Forward),
           "deferred STOP setup should send START");
    const rb::Packet start = lastPacket(transport);
    expect(controller.disableServo(rb::ServoId::FrontAxis),
           "an unrelated safety Disable should create APC priority work");
    waitForMs(35);
    expect(lastPacket(transport).type == rb::MessageType::ServoDisable,
           "START timeout should dispatch the unrelated priority Disable");
    const rb::Packet disable = lastPacket(transport);

    expect(controller.stopMotion(),
           "STOP should supersede a deferred Motion retry");
    expect(controller.queuedCommandCount() == 1,
           "deferred Motion retry should be replaced by one queued STOP");
    acknowledge(transport, start, rb::AckResult::Ok,
                rb::MessageType::SetMotionMode);
    acknowledge(transport, disable, rb::AckResult::Ok,
                rb::MessageType::ServoDisable);
    expect(lastPacket(transport).type == rb::MessageType::SetMotionMode,
           "deferred retry must not dispatch ahead of STOP");
    const rb::Packet stop = lastPacket(transport);
    acknowledge(transport, stop, rb::AckResult::Ok,
                rb::MessageType::SetMotionMode);
    expect(controller.motionState() == rb::MotionState::Stopping,
           "STOP ACK should complete deferred-retry arbitration");
}

void testMotionStopIsIdempotentWhenAlreadyStopped()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    connectAndEnablePaddles(transport, controller);
    const qsizetype writesBeforeStop = transport.writes().size();

    expect(controller.stopMotion(),
           "STOP while already Stopped should be idempotent");
    expect(transport.writes().size() == writesBeforeStop,
           "idempotent STOP should not emit a new wire command");
    expect(controller.motionState() == rb::MotionState::Stopped,
           "idempotent STOP should preserve Stopped state");
}

void testMotionManualArbitrationBusyAndDisableAllPreemption()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    connectAndEnablePaddles(transport, controller);
    expect(controller.startMotion(rb::MotionMode::Forward),
           "manual arbitration setup should start Motion");
    acknowledgeLast(transport);
    expect(controller.motionState() == rb::MotionState::Running,
           "manual arbitration setup should reach Running");

    const qsizetype writesBeforeManual = transport.writes().size();
    expect(!controller.setServoAngle(rb::ServoId::FrontRight, 0),
           "manual SetAngle should be rejected while Motion is Running");
    expect(!controller.setServoPwm(rb::ServoId::FrontRight, 1450),
           "manual SetPWM should be rejected while Motion is Running");
    expect(!controller.neutralServo(rb::ServoId::FrontRight),
           "manual Neutral should be rejected while Motion is Running");
    expect(transport.writes().size() == writesBeforeManual,
           "locally rejected manual commands must not write frames");

    expect(controller.stopMotion(),
           "manual arbitration setup should request STOP");
    acknowledgeLast(transport);
    expect(controller.motionState() == rb::MotionState::Stopping,
           "manual arbitration setup should reach Stopping");
    expect(!controller.setServoAngle(rb::ServoId::FrontRight, 0),
           "manual SetAngle should be rejected while Motion is Stopping");

    expect(controller.disableAll(),
           "Disable All must remain available during Motion STOPPING");
    expect(lastPacket(transport).type == rb::MessageType::ServoDisable,
           "Disable All must retain the Servo Disable safety path");
    expect(controller.motionState() == rb::MotionState::Faulted,
           "Disable All should immediately fault/terminate local Motion state");
}

void testMotionUnrelatedServoDisablePreservesOwnership()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    connectAndEnablePaddles(transport, controller);

    expect(controller.enableServo(rb::ServoId::FrontAxis),
           "unrelated Disable setup should enable FrontAxis");
    acknowledgeLast(transport);
    expect(controller.startMotion(rb::MotionMode::Forward),
           "unrelated Disable setup should start Forward Motion");
    acknowledgeLast(transport);
    expect(controller.motionState() == rb::MotionState::Running,
           "unrelated Disable setup should reach Running");

    expect(controller.disableServo(rb::ServoId::FrontAxis),
           "a non-owned FrontAxis Disable should remain allowed during Forward");
    const rb::Packet disable = lastPacket(transport);
    expect(controller.motionState() == rb::MotionState::Running,
           "non-owned Servo Disable must not fault local Motion");
    acknowledge(transport, disable, rb::AckResult::Ok,
                rb::MessageType::ServoDisable);
    expect(!controller.isServoEnabled(rb::ServoId::FrontAxis),
           "successful non-owned Servo Disable should clear that channel");
    expect(controller.motionState() == rb::MotionState::Running,
           "non-owned Servo Disable ACK must preserve Motion state");
    expect(!controller.setServoAngle(rb::ServoId::FrontRight, 0),
           "owned Servo SetAngle must remain blocked after unrelated Disable");
}

void testMotionTransitionRetainsOldOwnershipAfterAck()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    connectAndEnablePaddles(transport, controller);

    expect(controller.enableServo(rb::ServoId::FrontAxis),
           "transition ownership setup should enable FrontAxis");
    acknowledgeLast(transport);
    expect(controller.startMotion(rb::MotionMode::Ascend),
           "transition ownership setup should start Ascend Motion");
    acknowledgeLast(transport);
    expect(controller.motionState() == rb::MotionState::Running,
           "transition ownership setup should reach Running Ascend");

    expect(controller.startMotion(rb::MotionMode::Forward),
           "Ascend to Forward transition should be sent");
    const rb::Packet modeChange = lastPacket(transport);
    acknowledge(transport, modeChange, rb::AckResult::Ok,
                rb::MessageType::SetMotionMode);
    expect(controller.motionState() == rb::MotionState::Running,
           "mode-change ACK should expose the new Running mode");

    expect(controller.disableServo(rb::ServoId::FrontAxis),
           "Disable during the acknowledged mode transition should be accepted as safety takeover");
    expect(controller.motionState() == rb::MotionState::Faulted,
           "old-mode ownership must remain fail-closed until the transition completes");

    {
        rb::FakeTransport completedTransport;
        rb::RobotController completedController(&completedTransport, config);
        connectAndEnablePaddles(completedTransport, completedController);
        expect(completedController.enableServo(rb::ServoId::FrontAxis),
               "completed transition setup should enable FrontAxis");
        acknowledgeLast(completedTransport);
        expect(completedController.startMotion(rb::MotionMode::Ascend),
               "completed transition setup should start Ascend Motion");
        acknowledgeLast(completedTransport);
        expect(completedController.startMotion(rb::MotionMode::Forward),
               "completed Ascend to Forward transition should be sent");
        const rb::Packet completedModeChange = lastPacket(completedTransport);
        acknowledge(completedTransport, completedModeChange, rb::AckResult::Ok,
                    rb::MessageType::SetMotionMode);

        waitForMs(rb::kMotionTransitionDurationMs + 100);
        expect(completedController.disableServo(rb::ServoId::FrontAxis),
               "Disable after the mode transition should remain non-intersecting");
        expect(completedController.motionState() == rb::MotionState::Running,
               "old-mode ownership should be released after the mode transition completes");
    }
}

void testApcMotionDisableOwnershipAndQueuedPreemption()
{
    {
        rb::FakeTransport transport;
        rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
        config.heartbeatIntervalMs = 10000;
        config.ackTimeoutMs = 1000;
        config.heartbeatSafetyBudgetMs = 5000;
        rb::RobotController controller(&transport, config);
        connectApcAndAcknowledgeHeartbeat(transport, controller);
        enablePaddlesOnly(transport, controller);
        expect(controller.enableServo(rb::ServoId::FrontAxis),
               "APC unrelated Disable setup should enable FrontAxis");
        acknowledgeLast(transport);
        expect(controller.startMotion(rb::MotionMode::Forward),
               "APC unrelated Disable setup should start Forward Motion");
        acknowledgeLast(transport);

        expect(controller.disableServo(rb::ServoId::FrontAxis),
               "APC non-owned FrontAxis Disable should be accepted");
        const rb::Packet disable = lastPacket(transport);
        expect(controller.motionState() == rb::MotionState::Running,
               "APC non-owned Disable must preserve local Motion state");
        acknowledge(transport, disable, rb::AckResult::Ok,
                    rb::MessageType::ServoDisable);
        expect(controller.motionState() == rb::MotionState::Running,
               "APC non-owned Disable ACK must preserve local Motion state");
    }

    {
        rb::FakeTransport transport;
        rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
        config.heartbeatIntervalMs = 10000;
        config.ackTimeoutMs = 1000;
        config.heartbeatSafetyBudgetMs = 5000;
        rb::RobotController controller(&transport, config);
        connectApcAndAcknowledgeHeartbeat(transport, controller);
        enablePaddlesOnly(transport, controller);
        expect(controller.enableServo(rb::ServoId::FrontAxis),
               "APC transition Disable setup should enable FrontAxis");
        acknowledgeLast(transport);
        expect(controller.startMotion(rb::MotionMode::Forward),
               "APC transition Disable setup should start Forward Motion");
        acknowledgeLast(transport);

        expect(controller.startMotion(rb::MotionMode::Ascend),
               "APC transition Disable setup should send Ascend mode change");
        const rb::Packet modeChange = lastPacket(transport);
        expect(controller.disableServo(rb::ServoId::FrontAxis),
               "APC transition-intersecting Disable should be queued as safety priority");
        expect(controller.motionState() == rb::MotionState::Faulted,
               "APC transition-intersecting Disable should fault local Motion");
        expect(controller.queuedCommandCount() == 1,
               "APC transition-intersecting Disable should occupy the priority queue");

        acknowledge(transport, modeChange, rb::AckResult::Ok,
                    rb::MessageType::SetMotionMode);
        expect(controller.motionState() == rb::MotionState::Faulted,
               "late APC transition ACK must not resurrect fail-closed Motion");
        expect(lastPacket(transport).type == rb::MessageType::ServoDisable,
               "APC transition-intersecting Disable must dispatch after stale ACK");
    }
}

void testMotionBusyAckAndReconnectDoesNotResume()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    connectAndEnablePaddles(transport, controller);
    expect(controller.startMotion(rb::MotionMode::Forward),
           "BUSY ACK setup should start Forward Motion");
    acknowledgeLast(transport);

    expect(controller.startMotion(rb::MotionMode::TurnLeft),
           "a mode change should be sent while Running");
    const rb::Packet modeChange = lastPacket(transport);
    acknowledge(transport, modeChange, rb::AckResult::Busy,
                rb::MessageType::SetMotionMode);
    expect(controller.motionState() == rb::MotionState::Running,
           "BUSY mode-change ACK should preserve the prior Running state");
    expect(controller.monitor().ackStatus.contains(QStringLiteral("Busy")),
           "BUSY ACK should be visible in the protocol monitor");

    expect(controller.stopMotion(),
           "reconnect test should request graceful STOP");
    acknowledgeLast(transport);
    expect(controller.motionState() == rb::MotionState::Stopping,
           "reconnect test should enter Stopping before link loss");
    const qsizetype writesBeforeLinkLoss = transport.writes().size();
    transport.simulateError(QStringLiteral("link lost"));
    expect(controller.motionState() == rb::MotionState::Faulted,
           "link loss should immediately fault/clear local Motion state");
    transport.simulateConnected();
    expect(!controller.isMotionActive(),
           "reconnect must not auto-resume the interrupted Motion");
    expect(transport.writes().size() == writesBeforeLinkLoss,
           "reconnect must not emit an automatic Motion command");
}

void testMotionFailClosedIgnoresLateMotionAck()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    connectAndEnablePaddles(transport, controller);

    expect(controller.startMotion(rb::MotionMode::Forward),
           "late-ACK setup should start Forward Motion");
    const rb::Packet start = lastPacket(transport);
    acknowledge(transport, start, rb::AckResult::Ok,
                rb::MessageType::SetMotionMode);
    expect(controller.motionState() == rb::MotionState::Running,
           "late-ACK setup should reach Running");

    expect(controller.stopMotion(),
           "late-ACK setup should send a graceful STOP");
    const rb::Packet stop = lastPacket(transport);
    expect(controller.disableAll(),
           "Disable All should fail-close while STOP is in flight");
    expect(controller.motionState() == rb::MotionState::Faulted,
           "Disable All should fault local Motion before a late ACK");

    acknowledge(transport, stop, rb::AckResult::Ok,
                rb::MessageType::SetMotionMode);
    expect(controller.motionState() == rb::MotionState::Faulted,
           "a late successful Motion ACK must not resurrect Faulted Motion");
}

void testGaitBackendAckCorrelationAndLifecycle()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 5;
    config.maxRetries = 0;
    rb::RobotController controller(&transport, config);
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    expect(!controller.confirmedGaitBackend().has_value(),
           "gait backend must start UNKNOWN before an ACK-confirmed selection");
    expect(controller.setGaitBackend(rb::GaitBackend::SimpleGait),
           "the first gait backend selector request should be sent");
    const rb::Packet first = lastPacket(transport);
    expect(first.type == rb::MessageType::SetGaitBackend,
           "gait backend selection must use SetGaitBackend");
    expect(first.payload.size() == 1
               && static_cast<quint8>(first.payload.front()) == 0U,
           "SimpleGait selector payload must be exactly one byte with value zero");
    expect(controller.isGaitBackendChangePending(),
           "selector request must remain pending until its matching ACK");
    expect(controller.requestedGaitBackend().has_value()
               && *controller.requestedGaitBackend() == rb::GaitBackend::SimpleGait,
           "pending selector must expose the requested backend");

    expect(!controller.setGaitBackend(rb::GaitBackend::CPG),
           "a second selector request must be rejected while one is pending");
    acknowledgeSequence(transport, static_cast<quint16>(first.sequence + 1U),
                        rb::AckResult::Ok, rb::MessageType::SetGaitBackend);
    expect(controller.isGaitBackendChangePending(),
           "an ACK with an unrelated sequence must not clear selector pending state");
    expect(!controller.confirmedGaitBackend().has_value(),
           "an unrelated selector ACK must not confirm a backend");

    acknowledgeSequence(transport, first.sequence, rb::AckResult::Ok,
                        rb::MessageType::SetMotionMode);
    expect(!controller.isGaitBackendChangePending(),
           "a matching-sequence wrong-type ACK must clear selector pending state");
    expect(!controller.confirmedGaitBackend().has_value(),
           "a wrong-type ACK must not confirm the requested backend");

    expect(controller.setGaitBackend(rb::GaitBackend::SimpleGait),
           "selector should be reusable after a matching-sequence mismatch");
    const rb::Packet simpleRequest = lastPacket(transport);
    acknowledgeSequence(transport, simpleRequest.sequence, rb::AckResult::Ok,
                        rb::MessageType::SetGaitBackend);
    expect(!controller.isGaitBackendChangePending(),
           "matching successful selector ACK must clear pending state");
    expect(controller.confirmedGaitBackend().has_value()
               && *controller.confirmedGaitBackend() == rb::GaitBackend::SimpleGait,
           "matching successful selector ACK must confirm SimpleGait");

    expect(controller.setGaitBackend(rb::GaitBackend::CPG),
           "CPG selector request should be sent after SimpleGait confirmation");
    const rb::Packet busyRequest = lastPacket(transport);
    acknowledgeSequence(transport, busyRequest.sequence, rb::AckResult::Busy,
                        rb::MessageType::SetGaitBackend);
    expect(!controller.isGaitBackendChangePending(),
           "BUSY selector ACK must clear pending state");
    expect(controller.confirmedGaitBackend().has_value()
               && *controller.confirmedGaitBackend() == rb::GaitBackend::SimpleGait,
           "BUSY selector ACK must preserve the prior confirmed backend");

    expect(controller.setGaitBackend(rb::GaitBackend::CPG),
           "selector error case should send a new CPG request");
    const rb::Packet errorRequest = lastPacket(transport);
    injectError(transport, errorRequest.sequence, rb::MessageType::SetGaitBackend);
    expect(!controller.isGaitBackendChangePending(),
           "matching selector Error must clear pending state");
    expect(controller.confirmedGaitBackend().has_value()
               && *controller.confirmedGaitBackend() == rb::GaitBackend::SimpleGait,
           "selector Error must preserve the prior confirmed backend");

    expect(controller.setGaitBackend(rb::GaitBackend::CPG),
           "selector transport-error case should send a new CPG request");
    emit transport.errorOccurred(QStringLiteral("selector transport error"));
    expect(!controller.isGaitBackendChangePending(),
           "selector transport error must clear pending state");
    expect(controller.confirmedGaitBackend().has_value()
               && *controller.confirmedGaitBackend() == rb::GaitBackend::SimpleGait,
           "selector transport error must preserve the prior confirmed backend");

    transport.setWriteErrorSignals(false);
    transport.setWriteSucceeds(false);
    expect(!controller.setGaitBackend(rb::GaitBackend::CPG),
           "selector write failure must be reported");
    expect(!controller.isGaitBackendChangePending(),
           "selector write failure must clear pending state");
    expect(controller.confirmedGaitBackend().has_value()
               && *controller.confirmedGaitBackend() == rb::GaitBackend::SimpleGait,
           "selector write failure must preserve the prior confirmed backend");

    transport.setWriteSucceeds(true);
    expect(controller.setGaitBackend(rb::GaitBackend::CPG),
           "selector timeout case should send a new CPG request");
    waitForMs(40);
    expect(!controller.isGaitBackendChangePending(),
           "terminal selector timeout must clear pending state");
    expect(controller.confirmedGaitBackend().has_value()
               && *controller.confirmedGaitBackend() == rb::GaitBackend::SimpleGait,
           "selector timeout must preserve the prior confirmed backend");

    expect(controller.setGaitBackend(rb::GaitBackend::CPG),
           "selector should be reusable after timeout");
    const rb::Packet confirmedCpgRequest = lastPacket(transport);
    acknowledgeSequence(transport, confirmedCpgRequest.sequence, rb::AckResult::Ok,
                        rb::MessageType::SetGaitBackend);
    expect(controller.confirmedGaitBackend().has_value()
               && *controller.confirmedGaitBackend() == rb::GaitBackend::CPG,
           "matching selector ACK must confirm CPG");
    const qsizetype writesBeforeDisconnect = transport.writes().size();
    transport.simulateError(QStringLiteral("link lost"));
    expect(!controller.confirmedGaitBackend().has_value(),
           "disconnect must reset confirmed gait backend to UNKNOWN");
    expect(!controller.isGaitBackendChangePending(),
           "disconnect must clear selector pending state");
    transport.simulateConnected();
    expect(!controller.confirmedGaitBackend().has_value(),
           "reconnect must keep gait backend UNKNOWN until a new ACK");
    expect(transport.writes().size() == writesBeforeDisconnect,
           "reconnect must not emit an automatic gait backend selector");
}

void testFrontRearCoordinationAckAndMotionGate()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    config.heartbeatIntervalMs = 10000;
    rb::RobotController controller(&transport, config);
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    expect(controller.setFrontRearCoordination(
               rb::FrontRearCoordination::OppositeDirection),
           "front/rear coordination should submit while Motion is stopped");
    const rb::Packet request = lastPacket(transport);
    expect(request.type == rb::MessageType::SetFrontRearCoordination,
           "front/rear selection must use Protocol V2 0x17");
    expect(request.payload.size() == 1
               && static_cast<quint8>(request.payload.front()) == 1U,
           "OppositeDirection must be the one-byte value one");
    expect(controller.isFrontRearCoordinationChangePending()
               && controller.requestedFrontRearCoordination().has_value()
               && *controller.requestedFrontRearCoordination()
                   == rb::FrontRearCoordination::OppositeDirection,
           "coordination request remains pending and exposes the requested value");
    expect(!controller.confirmedFrontRearCoordination().has_value(),
           "coordination remains Unknown before its matching ACK");
    expect(!controller.setGaitBackend(rb::GaitBackend::ExperimentalFlex),
           "gait selection must be blocked while coordination is pending");

    acknowledgeSequence(transport, static_cast<quint16>(request.sequence + 1U),
                        rb::AckResult::Ok,
                        rb::MessageType::SetFrontRearCoordination);
    expect(controller.isFrontRearCoordinationChangePending()
               && !controller.confirmedFrontRearCoordination().has_value(),
           "unrelated sequence must not settle the coordination request");
    acknowledgeSequence(transport, request.sequence, rb::AckResult::Ok,
                        rb::MessageType::SetFrontRearCoordination);
    expect(!controller.isFrontRearCoordinationChangePending()
               && controller.confirmedFrontRearCoordination().has_value()
               && *controller.confirmedFrontRearCoordination()
                   == rb::FrontRearCoordination::OppositeDirection,
           "matching ACK confirms the requested coordination");

    expect(controller.setGaitBackend(rb::GaitBackend::ExperimentalFlex),
           "ExperimentalFlex selection should use the existing gait command");
    const rb::Packet flexRequest = lastPacket(transport);
    expect(flexRequest.type == rb::MessageType::SetGaitBackend
               && flexRequest.payload.size() == 1
               && static_cast<quint8>(flexRequest.payload.front()) == 2U,
           "ExperimentalFlex is encoded as backend value two");
    acknowledgeSequence(transport, flexRequest.sequence, rb::AckResult::Ok,
                        rb::MessageType::SetGaitBackend);

    const rb::ServoId motionServos[] = {
        rb::ServoId::FrontRight, rb::ServoId::FrontLeft,
        rb::ServoId::RearRight, rb::ServoId::RearLeft,
    };
    for (const rb::ServoId servo : motionServos) {
        expect(controller.enableServo(servo),
               "motion gate fixture must enable all four paddles");
        acknowledgeLast(transport);
    }
    expect(controller.startMotion(rb::MotionMode::Forward),
           "motion gate fixture must start a supported gait mode");
    acknowledgeLast(transport);
    expect(controller.isMotionActive(),
           "motion gate fixture must reach active Motion");
    const qsizetype writesBeforeRejectedSelectors = transport.writes().size();
    expect(!controller.setGaitBackend(rb::GaitBackend::CPG),
           "gait selector must be rejected during active Motion");
    expect(!controller.setFrontRearCoordination(
               rb::FrontRearCoordination::SameDirection),
           "coordination selector must be rejected during active Motion");
    expect(transport.writes().size() == writesBeforeRejectedSelectors,
           "active-Motion selector rejection must not write robot commands");
}

void testApc220MotionCommandsUseTheExistingBoundedScheduler()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 1000;
    config.heartbeatSafetyBudgetMs = 5000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);

    const rb::ServoId paddles[] = {
        rb::ServoId::FrontRight,
        rb::ServoId::FrontLeft,
        rb::ServoId::RearRight,
        rb::ServoId::RearLeft,
    };
    for (const rb::ServoId id : paddles) {
        expect(controller.enableServo(id),
               "APC220 Motion setup should queue/send each paddle Enable");
        acknowledgeLast(transport);
    }

    expect(controller.startMotion(rb::MotionMode::Forward),
           "APC220 Motion START should occupy the one-flight slot");
    const rb::Packet start = lastPacket(transport);
    expect(!controller.startMotion(rb::MotionMode::TurnLeft),
           "a second unresolved Motion START should be rejected at the Controller boundary");
    expect(controller.queuedCommandCount() == 0,
           "APC220 should not queue a second unresolved Motion command");

    acknowledge(transport, start, rb::AckResult::Ok,
                rb::MessageType::SetMotionMode);
    expect(controller.motionState() == rb::MotionState::Running,
           "START ACK should make the serialized-Motion fixture Running");

    expect(controller.startMotion(rb::MotionMode::TurnLeft),
           "a new Motion mode change should be allowed after START ACK");

    const rb::Packet modeChange = lastPacket(transport);
    expect(controller.stopMotion(),
           "STOP should queue behind an in-flight Motion mode change");
    expect(controller.queuedCommandCount() == 1,
           "STOP should be represented by one queued Motion request");
    expect(controller.disableAll(),
           "Disable All should preempt queued Motion commands");
    expect(controller.queuedCommandCount() == 1,
           "Disable All should retain only its safety-priority request");
    expect(controller.motionState() == rb::MotionState::Faulted,
           "Disable All should immediately fail-close local Motion state");

    acknowledge(transport, modeChange, rb::AckResult::Ok,
                rb::MessageType::SetMotionMode);
    expect(controller.motionState() == rb::MotionState::Faulted,
           "late APC220 Motion ACK must not resurrect Disable All fail-closed state");
    expect(lastPacket(transport).type == rb::MessageType::ServoDisable,
           "Disable All should dispatch before stale queued Motion work");
}

void testApc220GaitBackendSelectorIsSerialized()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 1000;
    config.heartbeatSafetyBudgetMs = 5000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);

    expect(controller.enableServo(rb::ServoId::FrontRight),
           "APC selector serialization setup should occupy the ACK slot");
    const rb::Packet enable = lastPacket(transport);
    expect(controller.setGaitBackend(rb::GaitBackend::CPG),
           "APC selector should be accepted into the bounded command queue");
    expect(controller.isGaitBackendChangePending(),
           "queued APC selector must remain pending before dispatch");
    expect(controller.queuedCommandCount() == 1,
           "queued APC selector must occupy exactly one command entry");
    expect(lastPacket(transport).sequence == enable.sequence
               && lastPacket(transport).type == rb::MessageType::ServoEnable,
           "queued selector must not overtake the in-flight command");
    expect(!controller.setGaitBackend(rb::GaitBackend::SimpleGait),
           "APC selector must reject a second request while the first is queued");

    acknowledge(transport, enable, rb::AckResult::Ok,
                rb::MessageType::ServoEnable);
    expect(!transport.writes().isEmpty()
               && lastPacket(transport).type == rb::MessageType::SetGaitBackend,
           "APC selector must dispatch after the prior ACK releases the slot");
    const rb::Packet selector = lastPacket(transport);
    acknowledge(transport, selector, rb::AckResult::Ok,
                rb::MessageType::SetGaitBackend);
    expect(!controller.isGaitBackendChangePending(),
           "APC selector ACK must clear the serialized pending state");
    expect(controller.confirmedGaitBackend().has_value()
               && *controller.confirmedGaitBackend() == rb::GaitBackend::CPG,
           "APC selector ACK must confirm the requested CPG backend");
}

void testApc220SelectorEvictionBySafetyDisableClearsLifecycle()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 1000;
    config.heartbeatSafetyBudgetMs = 5000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);

    expect(controller.setGaitBackend(rb::GaitBackend::CPG),
           "safety-eviction setup should confirm CPG first");
    const rb::Packet cpgRequest = lastPacket(transport);
    acknowledge(transport, cpgRequest, rb::AckResult::Ok,
                rb::MessageType::SetGaitBackend);
    expect(controller.confirmedGaitBackend().has_value()
               && *controller.confirmedGaitBackend() == rb::GaitBackend::CPG,
           "safety-eviction setup should retain CPG as the confirmed backend");

    expect(controller.enableServo(rb::ServoId::FrontRight),
           "safety-eviction setup should occupy the APC220 in-flight slot");
    const rb::Packet inFlight = lastPacket(transport);
    expect(controller.setGaitBackend(rb::GaitBackend::SimpleGait),
           "selector should be queued before safety eviction");
    expect(controller.isGaitBackendChangePending(),
           "queued selector should be pending before safety eviction");

    for (qsizetype index = 0; index < rb::kApc220CommandQueueCapacity - 1; ++index) {
        expect(controller.enableServo(rb::ServoId::FrontRight),
               "ordinary APC220 filler should reach the bounded queue capacity");
    }
    expect(controller.queuedCommandCount() == rb::kApc220CommandQueueCapacity,
           "safety-eviction fixture should fill the ordinary queue");

    expect(controller.disableServo(rb::ServoId::FrontAxis),
           "unrelated safety Disable should evict ordinary work when the queue is full");
    expect(!controller.isGaitBackendChangePending(),
           "safety Disable eviction must clear dropped selector pending state");
    expect(controller.confirmedGaitBackend().has_value()
               && *controller.confirmedGaitBackend() == rb::GaitBackend::CPG,
           "safety Disable eviction must preserve the confirmed backend");

    acknowledge(transport, inFlight, rb::AckResult::Ok, rb::MessageType::ServoEnable);
    expect(!transport.writes().isEmpty()
               && lastPacket(transport).type == rb::MessageType::ServoDisable,
           "safety Disable should dispatch after the in-flight command ACK");
    acknowledgeLast(transport);
    for (qsizetype index = 0; index < rb::kApc220CommandQueueCapacity - 1; ++index) {
        expect(lastPacket(transport).type == rb::MessageType::ServoEnable,
               "only ordinary filler should remain after the dropped selector");
        acknowledgeLast(transport);
    }
    expect(controller.queuedCommandCount() == 0 && controller.pending_.isEmpty(),
           "all ordinary filler should be fully acknowledged before retrying the selector");

    expect(controller.setGaitBackend(rb::GaitBackend::SimpleGait),
           "a selector should be requestable again after safety eviction cleanup");
    const rb::Packet retry = lastPacket(transport);
    expect(retry.type == rb::MessageType::SetGaitBackend,
           "the re-requested selector should be sent as SetGaitBackend");
    acknowledge(transport, retry, rb::AckResult::Ok,
                rb::MessageType::SetGaitBackend);
    expect(controller.confirmedGaitBackend().has_value()
               && *controller.confirmedGaitBackend() == rb::GaitBackend::SimpleGait,
           "the re-requested selector should confirm normally");
}

void testApc220SelectorEvictionByMotionStopClearsLifecycle()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 10000;
    config.ackTimeoutMs = 1000;
    config.heartbeatSafetyBudgetMs = 5000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);

    expect(controller.setGaitBackend(rb::GaitBackend::CPG),
           "Motion STOP eviction setup should confirm CPG first");
    const rb::Packet cpgRequest = lastPacket(transport);
    acknowledge(transport, cpgRequest, rb::AckResult::Ok,
                rb::MessageType::SetGaitBackend);
    expect(controller.enableServo(rb::ServoId::FrontRight),
           "Motion STOP eviction setup should occupy the APC220 in-flight slot");
    const rb::Packet inFlight = lastPacket(transport);
    expect(controller.setGaitBackend(rb::GaitBackend::SimpleGait),
           "selector should be queued before Motion STOP eviction");

    const rb::RobotController::QueuedCommand filler{
        rb::MessageType::Heartbeat, QByteArray(), 0, std::nullopt, std::nullopt};
    while (controller.commandQueue_.size() < rb::kApc220CommandQueueCapacity) {
        controller.commandQueue_.enqueue(filler);
    }
    controller.motionState_ = rb::MotionState::Running;
    controller.motionMode_ = rb::MotionMode::Forward;
    expect(controller.queuedCommandCount() == rb::kApc220CommandQueueCapacity,
           "Motion STOP eviction fixture should fill the bounded queue");

    expect(controller.stopMotion(),
           "Motion STOP should be accepted while the queue is full");
    expect(!controller.isGaitBackendChangePending(),
           "Motion STOP eviction must clear dropped selector pending state");
    expect(controller.confirmedGaitBackend().has_value()
               && *controller.confirmedGaitBackend() == rb::GaitBackend::CPG,
           "Motion STOP eviction must preserve the confirmed backend");
    expect(controller.queuedCommandCount() == rb::kApc220CommandQueueCapacity,
           "Motion STOP should replace the evicted selector within the queue bound");

    acknowledge(transport, inFlight, rb::AckResult::Ok, rb::MessageType::ServoEnable);
    expect(!transport.writes().isEmpty()
               && lastPacket(transport).type == rb::MessageType::SetMotionMode,
           "Motion STOP should dispatch after the in-flight command ACK");
    acknowledgeLast(transport);
    while (controller.queuedCommandCount() > 0) {
        expect(lastPacket(transport).type == rb::MessageType::Heartbeat,
               "Motion STOP should release the remaining non-selector fixture work");
        acknowledgeLast(transport);
    }
    for (qsizetype index = 0; index < transport.writes().size(); ++index) {
        if (index >= 2) {
            expect(packetAt(transport, index).type != rb::MessageType::SetGaitBackend,
                   "a Motion STOP eviction must not replay the dropped selector");
        }
    }
}

void testApc220SelectorLivenessFailClosedClearsLifecycle()
{
    rb::FakeTransport transport;
    rb::RobotControllerConfig config = rb::RobotControllerConfig::apc220Provisional();
    config.heartbeatIntervalMs = 5;
    config.ackTimeoutMs = 5;
    config.maxRetries = 0;
    config.heartbeatSafetyBudgetMs = 5000;
    rb::RobotController controller(&transport, config);
    connectApcAndAcknowledgeHeartbeat(transport, controller);

    expect(controller.setGaitBackend(rb::GaitBackend::CPG),
           "liveness fail-close setup should confirm CPG first");
    const rb::Packet cpgRequest = lastPacket(transport);
    acknowledge(transport, cpgRequest, rb::AckResult::Ok,
                rb::MessageType::SetGaitBackend);
    expect(controller.enableServo(rb::ServoId::FrontRight),
           "liveness fail-close setup should occupy the APC220 in-flight slot");
    expect(controller.setGaitBackend(rb::GaitBackend::SimpleGait),
           "selector should be queued before heartbeat liveness fail-close");
    const qsizetype selectorSetupWrites = transport.writes().size();

    waitForMs(80);

    expect(!controller.isGaitBackendChangePending(),
           "heartbeat liveness fail-close must clear queued or deferred selector state");
    expect(controller.confirmedGaitBackend().has_value()
               && *controller.confirmedGaitBackend() == rb::GaitBackend::CPG,
           "heartbeat liveness fail-close must preserve the confirmed backend");
    for (qsizetype index = selectorSetupWrites; index < transport.writes().size(); ++index) {
        expect(packetAt(transport, index).type != rb::MessageType::SetGaitBackend,
               "liveness recovery must not dispatch a stale selector");
    }

    const rb::Packet recoveryHeartbeat = lastPacket(transport);
    expect(recoveryHeartbeat.type == rb::MessageType::Heartbeat,
           "liveness recovery fixture should leave a heartbeat exchange in flight");
    acknowledge(transport, recoveryHeartbeat, rb::AckResult::Ok,
                rb::MessageType::Heartbeat);
    const qsizetype writesAfterRecovery = transport.writes().size();
    waitForMs(2);
    for (qsizetype index = writesAfterRecovery; index < transport.writes().size(); ++index) {
        expect(packetAt(transport, index).type != rb::MessageType::SetGaitBackend,
               "heartbeat recovery must not replay the stale selector");
    }
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
    testDirectHeartbeatLossIgnoresLateEnableAck();
    testDirectHeartbeatLossClearsEnabledState();
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
    testMotionStartStopStateAndWireContract();
    testBackwardRemainsProtocolCompatibleButIsNotBenchStartable();
    testMotionStartSerializesDirectModeChange();
    testMotionStartSerializesApc220ModeChange();
    testMotionStopSupersedesInFlightDirectStart();
    testMotionStopSupersedesApcInFlightAndQueuedMotion();
    testApcMotionStopCancelsDeferredMotionRetry();
    testMotionStopIsIdempotentWhenAlreadyStopped();
    testMotionManualArbitrationBusyAndDisableAllPreemption();
    testMotionUnrelatedServoDisablePreservesOwnership();
    testMotionTransitionRetainsOldOwnershipAfterAck();
    testApcMotionDisableOwnershipAndQueuedPreemption();
    testMotionBusyAckAndReconnectDoesNotResume();
    testMotionFailClosedIgnoresLateMotionAck();
    testGaitBackendAckCorrelationAndLifecycle();
    testFrontRearCoordinationAckAndMotionGate();
    testApc220MotionCommandsUseTheExistingBoundedScheduler();
    testApc220GaitBackendSelectorIsSerialized();
    testApc220SelectorEvictionBySafetyDisableClearsLifecycle();
    testApc220SelectorEvictionByMotionStopClearsLifecycle();
    testApc220SelectorLivenessFailClosedClearsLifecycle();
    if (failures == 0) {
        std::cout << "All robot controller tests passed\n";
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
