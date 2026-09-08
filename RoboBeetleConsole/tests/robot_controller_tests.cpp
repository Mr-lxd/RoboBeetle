#include "protocol/PacketCodec.h"
#include "robot/RobotController.h"
#include "transport/FakeTransport.h"

#include <QCoreApplication>
#include <QEventLoop>
#include <QStringList>
#include <QTimer>

#include <cstdlib>
#include <iostream>
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
    expect(pending.events == std::vector<std::pair<int, bool>>{{0, true}, {0, false}},
           "Disable All should emit pending true then false even when Servo1 was already disabled");

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

void testProvisionalPwmCalibrationAndBounds()
{
    rb::FakeTransport transport;
    const rb::RobotControllerConfig config = rb::RobotControllerConfig::bringUpProvisional();
    rb::RobotController controller(&transport, config);
    expect(config.provisionalPwmMinUs == 520, "provisional PWM minimum must be 520 us");
    expect(config.provisionalNeutralUs == 1520, "provisional PWM neutral must be 1520 us");
    expect(config.provisionalPwmMaxUs == 2520, "provisional PWM maximum must be 2520 us");
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();
    controller.enableServo(rb::ServoId::Servo1);
    acknowledgeLast(transport);
    const qsizetype before = transport.writes().size();

    expect(!controller.setServoPwm(rb::ServoId::Servo1, 519),
           "PWM below provisional range must be rejected");
    expect(!controller.setServoPwm(rb::ServoId::Servo1, 2521),
           "PWM above provisional range must be rejected");
    expect(transport.writes().size() == before, "rejected commands must not write frames");

    expect(controller.setServoPwm(rb::ServoId::Servo1, 520),
           "PWM minimum boundary must be accepted");
    expect(lastPacket(transport).payload == QByteArray::fromHex("01000802"),
           "PWM minimum must be encoded as count, Servo1, uint16 LE");
    expect(controller.setServoPwm(rb::ServoId::Servo1, 2520),
           "PWM maximum boundary must be accepted");
    expect(lastPacket(transport).payload == QByteArray::fromHex("0100d809"),
           "PWM maximum must be encoded as count, Servo1, uint16 LE");
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
    expect(!controller.setServoAngle(rb::ServoId::Servo1, -9001),
           "angle below -90 degrees must be rejected");
    expect(!controller.setServoAngle(rb::ServoId::Servo1, 9001),
           "angle above +90 degrees must be rejected");
    expect(transport.writes().size() == before, "out-of-range angles must not write frames");

    expect(controller.setServoAngle(rb::ServoId::Servo1, -9000),
           "-90 degrees must be accepted");
    expect(lastPacket(transport).type == rb::MessageType::SetServoAngle,
           "angle must use SetServoAngle message");
    expect(lastPacket(transport).payload == QByteArray::fromHex("0100d8dc"),
           "-9000 cdeg must be encoded as int16 LE");

    expect(controller.setServoAngle(rb::ServoId::Servo1, 0),
           "zero degrees must be accepted");
    expect(lastPacket(transport).payload == QByteArray::fromHex("01000000"),
           "zero cdeg must be encoded as int16 LE");

    expect(controller.setServoAngle(rb::ServoId::Servo1, -4500),
           "-45 degrees must be accepted");
    expect(lastPacket(transport).payload == QByteArray::fromHex("01006cee"),
           "-4500 cdeg must be encoded as int16 LE");

    expect(controller.setServoAngle(rb::ServoId::Servo1, 4500),
           "+45 degrees must be accepted");
    expect(lastPacket(transport).payload == QByteArray::fromHex("01009411"),
           "+4500 cdeg must be encoded as int16 LE");

    expect(controller.setServoAngle(rb::ServoId::Servo1, 9000),
           "+90 degrees must be accepted");
    expect(lastPacket(transport).payload == QByteArray::fromHex("01002823"),
           "+9000 cdeg must be encoded as int16 LE");
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
    expect(lastPacket(transport).payload == QByteArray::fromHex("0100"),
           "disconnect must disable only the currently supported Servo1 mask");
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

void testUnsupportedServoIsRejectedLocally()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    expect(!controller.isServoSupported(rb::ServoId::Servo2),
           "Servo2 must be marked unsupported in Phase 1");
    expect(!controller.enableServo(rb::ServoId::Servo2),
           "unsupported Servo2 enable must be rejected");
    expect(!controller.disableServo(rb::ServoId::Servo2),
           "unsupported Servo2 disable must be rejected");
    expect(!controller.neutralServo(rb::ServoId::Servo2),
           "unsupported Servo2 neutral must be rejected");
    expect(!controller.setServoPwm(rb::ServoId::Servo2, 1520),
           "unsupported Servo2 PWM must be rejected");
    expect(!controller.setServoAngle(rb::ServoId::Servo2, 0),
           "unsupported Servo2 angle must be rejected");
    expect(transport.writes().isEmpty(), "unsupported Servo2 commands must not write frames");
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
    transport.setWriteCallback([&transport](const QByteArray &wire) {
        const rb::DecodeResult decoded = rb::PacketCodec::decodeWire(
            wire.first(wire.size() - 1));
        if (!decoded.ok()) {
            return;
        }
        const bool expectsAck = decoded.packet.type == rb::MessageType::ServoDisable
            || decoded.packet.type == rb::MessageType::Heartbeat
            || decoded.packet.type == rb::MessageType::ServoEnable
            || decoded.packet.type == rb::MessageType::SetServoPwm
            || decoded.packet.type == rb::MessageType::SetServoAngle
            || decoded.packet.type == rb::MessageType::Neutral;
        if (!expectsAck) {
            return;
        }
        QByteArray payload;
        payload.append(static_cast<char>(decoded.packet.sequence & 0xffU));
        payload.append(static_cast<char>((decoded.packet.sequence >> 8U) & 0xffU));
        payload.append(static_cast<char>(decoded.packet.type));
        payload.append(static_cast<char>(rb::AckResult::Ok));
        transport.injectBytes(rb::PacketCodec::encodeWire(
            {rb::MessageType::Ack, 0x8000, payload}));
    });

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

    expect(controller.queuedCommandCount() == 1,
           "heartbeat timeout must not release queued user commands");
    bool enableSent = false;
    for (qsizetype index = 0; index < transport.writes().size(); ++index) {
        enableSent = enableSent || packetAt(transport, index).type == rb::MessageType::ServoEnable;
    }
    expect(!enableSent,
           "heartbeat timeout must keep the APC220 link gate closed until a heartbeat ACK");
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
    config.heartbeatIntervalMs = 1;
    config.ackTimeoutMs = 1000;
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
    expect(lastPacket(transport).type == rb::MessageType::ServoEnable,
           "queued user command should be dispatched after heartbeat ACK");
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
    testAngleDegreesConvertToCentidegrees();
    testSetAngleBlockedDuringDisableRequest();
    testDisableAckClearsPendingWhenAlreadyDisabled();
    testDisablePendingWriteFailureDoesNotLockAngle();
    testDisablePendingAckRejectedRestoresAngle();
    testDisablePendingAckTypeMismatchRestoresAngle();
    testDisablePendingErrorRestoresAngle();
    testDisablePendingTimeoutRestoresAngle();
    testDisablePendingDisconnectClearsState();
    testDisablePendingSuccessDisablesAngle();
    testProvisionalPwmCalibrationAndBounds();
    testSetAngleEncodingAndBounds();
    testDisconnectAttemptsDisableAll();
    testNeutralEncodingAndAck();
    testUnsupportedServoIsRejectedLocally();
    testAckRejectionAndMatching();
    testRetryReusesIdenticalSequenceAndFrame();
    testUnexpectedTransportLossRecordsDisableFailure();
    testApc220ProfileUsesHalfDuplexTiming();
    testDirectUartRetainsMultiplePendingRequests();
    testApc220AllowsOnlyOneAckRequiringFrameInFlight();
    testApc220HeartbeatTicksCoalesceWhileCommandIsInFlight();
    testApc220HeartbeatDuePrecedesCommandRetry();
    testApc220HeartbeatDeadlineWinsRetryBoundary();
    testApc220CommandQueueHasBoundedCapacity();
    testApc220RetryUsesOriginalSequenceAndFrame();
    testApc220TransportResetClearsSchedulerAndDoesNotAutoEnable();
    testApc220RecordsMatchingAckRtt();
    testApc220SynchronousAckDuringInitialDispatchIsHandled();
    testApc220ConnectGatesCommandsUntilHeartbeatAck();
    testApc220HeartbeatRejectionKeepsUserCommandsGated();
    testApc220HeartbeatTimeoutKeepsUserCommandsGated();
    testApc220HeartbeatTypeMismatchKeepsUserCommandsGated();
    testApc220HeartbeatTicksDoNotBurstAndQueueGetsChanceAfterAck();
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
