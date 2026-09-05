#include "protocol/PacketCodec.h"
#include "robot/RobotController.h"
#include "transport/FakeTransport.h"

#include <QCoreApplication>
#include <QEventLoop>
#include <QStringList>
#include <QTimer>

#include <cstdlib>
#include <iostream>

namespace {

int failures = 0;

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

void testDisableAckNotifiesWhenAlreadyDisabled()
{
    rb::FakeTransport transport;
    rb::RobotController controller(&transport, rb::RobotControllerConfig::bringUpProvisional());
    controller.connectTransport({"COM_TEST", 9600});
    transport.simulateConnected();

    int disableAckNotifications = 0;
    int notifiedServo = -1;
    QObject::connect(&controller, &rb::RobotController::servoDisableAcknowledged,
                     [&disableAckNotifications, &notifiedServo](int index) {
                         ++disableAckNotifications;
                         notifiedServo = index;
                     });

    expect(controller.disableAll(), "Disable All should be sent while connected");
    const rb::Packet request = lastPacket(transport);
    expect(request.type == rb::MessageType::ServoDisable,
           "Disable All must use ServoDisable message");
    acknowledge(transport, request, rb::AckResult::Ok, rb::MessageType::ServoDisable);
    expect(disableAckNotifications == 1 && notifiedServo == 0,
           "successful Disable All ACK must notify Servo1 even when it was already disabled");

    expect(controller.enableServo(rb::ServoId::Servo1),
           "Servo1 should remain re-enableable after an already-disabled Disable All ACK");
    acknowledgeLast(transport);
    expect(controller.isServoEnabled(rb::ServoId::Servo1),
           "Servo1 should be enabled after the subsequent ACK");
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

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    testNoAutomaticEnableAndPwmRequiresEnable();
    testAngleDegreesConvertToCentidegrees();
    testSetAngleBlockedDuringDisableRequest();
    testDisableAckNotifiesWhenAlreadyDisabled();
    testProvisionalPwmCalibrationAndBounds();
    testSetAngleEncodingAndBounds();
    testDisconnectAttemptsDisableAll();
    testNeutralEncodingAndAck();
    testUnsupportedServoIsRejectedLocally();
    testAckRejectionAndMatching();
    testRetryReusesIdenticalSequenceAndFrame();
    testUnexpectedTransportLossRecordsDisableFailure();
    if (failures == 0) {
        std::cout << "All robot controller tests passed\n";
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
