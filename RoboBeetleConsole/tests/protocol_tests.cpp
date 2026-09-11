#include "protocol/Crc16.h"
#include "protocol/PacketCodec.h"
#include "protocol/StreamDecoder.h"

#include <QByteArray>
#include <QCoreApplication>

#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

int failures = 0;

void expect(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

QByteArray hex(const char *value)
{
    return QByteArray::fromHex(value);
}

void expectGoldenVector(rb::MessageType type,
                        quint16 sequence,
                        const QByteArray &payload,
                        const char *expectedHex)
{
    const rb::Packet packet{type, sequence, payload};
    const QByteArray encoded = rb::PacketCodec::encodeWire(packet);
    expect(encoded == hex(expectedHex), "golden vector differs");

    const rb::DecodeResult decoded = rb::PacketCodec::decodeWire(encoded.first(encoded.size() - 1));
    expect(decoded.ok(), "golden vector does not decode");
    expect(decoded.packet == packet, "golden vector round-trip differs");
}

QByteArray invalidFrame(QByteArray logical)
{
    logical.chop(2);
    const quint16 crc = rb::crc16CcittFalse(logical);
    logical.append(static_cast<char>(crc & 0xff));
    logical.append(static_cast<char>((crc >> 8) & 0xff));
    QByteArray wire = rb::PacketCodec::cobsEncode(logical);
    wire.append('\0');
    return wire;
}

void testGoldenVectors()
{
    static_assert(static_cast<quint8>(rb::AckResult::Ok) == 0);
    static_assert(static_cast<quint8>(rb::AckResult::InvalidPayload) == 1);
    static_assert(static_cast<quint8>(rb::AckResult::HostNotAlive) == 2);
    static_assert(static_cast<quint8>(rb::AckResult::UnsupportedServo) == 3);
    static_assert(static_cast<quint8>(rb::AckResult::ServoNotEnabled) == 4);
    static_assert(static_cast<quint8>(rb::AckResult::OutOfRange) == 5);
    static_assert(static_cast<quint8>(rb::AckResult::HardwareFailure) == 6);
    static_assert(static_cast<quint8>(rb::MessageType::LeakStatus) == 0x20);
    static_assert(static_cast<quint8>(rb::MessageType::DepthSnapshot) == 0x22);
    expect(rb::isKnownMessageType(0x22), "DepthSnapshot must be a known message type");
    expect(!rb::isKnownMessageType(0x23), "unassigned message type must remain unknown");

    expectGoldenVector(rb::MessageType::Heartbeat, 1, hex("78563412"),
                       "06524202010102040778563412442800");
    expectGoldenVector(rb::MessageType::ServoEnable, 2, hex("0100"),
                       "065242021002020202010345ad00");
    expectGoldenVector(rb::MessageType::ServoDisable, 3, hex("0100"),
                       "0652420211030202020103845000");
    expectGoldenVector(rb::MessageType::SetServoPwm, 4, hex("0200dc05014006"),
                       "0652420212040207020208dc0501400621db00");
    expectGoldenVector(rb::MessageType::Neutral, 5, hex("0300"),
                       "0652420214050202020303a0c200");
    expectGoldenVector(rb::MessageType::SetServoAngle, 6, hex("0100d8dc"),
                       "0652420213060204020105d8dcf44a00");
    expectGoldenVector(rb::MessageType::SetServoAngle, 7, hex("01000000"),
                       "06524202130702040201010103589b00");
    expectGoldenVector(rb::MessageType::SetServoAngle, 8, hex("01002823"),
                       "06524202130802040201052823d4d900");
    expectGoldenVector(rb::MessageType::LeakStatus, 9, hex("00"),
                       "06524202200902010103e36100");
    expectGoldenVector(rb::MessageType::LeakStatus, 10, hex("01"),
                       "06524202200a02010401109f00");
    expectGoldenVector(rb::MessageType::LeakStatus, 11, hex("02"),
                       "06524202200b02010402220500");
}

void testCrcReferenceValue()
{
    expect(rb::crc16CcittFalse(QByteArray("123456789")) == 0x29b1,
           "CRC-16/CCITT-FALSE check value must be 0x29B1");
}

void testSplitAndStickyFrames()
{
    const QByteArray first = rb::PacketCodec::encodeWire(
        {rb::MessageType::Heartbeat, 7, hex("01000000")});
    const QByteArray second = rb::PacketCodec::encodeWire(
        {rb::MessageType::Neutral, 8, hex("0300")});

    rb::StreamDecoder decoder;
    expect(decoder.feed(first.first(3)).empty(), "partial frame must not emit a packet");
    const auto splitResult = decoder.feed(first.sliced(3));
    expect(splitResult.size() == 1 && splitResult.front().ok(), "split frame must decode after completion");

    const auto stickyResult = decoder.feed(first + second);
    expect(stickyResult.size() == 2, "sticky frames must produce two results");
    expect(stickyResult[0].ok() && stickyResult[1].ok(), "sticky frames must both decode");
}

void testValidationErrorsAndRecovery()
{
    const QByteArray valid = rb::PacketCodec::encodeWire(
        {rb::MessageType::Heartbeat, 9, hex("02000000")});
    QByteArray logical = rb::PacketCodec::encodeLogical(
        {rb::MessageType::Heartbeat, 9, hex("02000000")});

    rb::StreamDecoder decoder;

    QByteArray badCrc = valid;
    badCrc[badCrc.size() - 2] = static_cast<char>(badCrc[badCrc.size() - 2] ^ 0x01);
    auto events = decoder.feed(badCrc + valid);
    expect(events.size() == 2, "bad CRC followed by valid frame must yield two events");
    expect(events[0].error == rb::DecodeError::CrcMismatch, "CRC corruption must be reported");
    expect(events[1].ok(), "decoder must recover after CRC error");

    QByteArray badMagic = logical;
    badMagic[0] = 'X';
    events = decoder.feed(invalidFrame(badMagic));
    expect(events.size() == 1 && events[0].error == rb::DecodeError::InvalidMagic,
           "invalid magic must be reported");

    QByteArray badVersion = logical;
    badVersion[2] = 3;
    events = decoder.feed(invalidFrame(badVersion));
    expect(events.size() == 1 && events[0].error == rb::DecodeError::InvalidVersion,
           "invalid version must be reported");

    QByteArray badLength = logical;
    badLength[6] = 5;
    events = decoder.feed(invalidFrame(badLength));
    expect(events.size() == 1 && events[0].error == rb::DecodeError::InvalidLength,
           "invalid payload length must be reported");

    QByteArray unknownType = logical;
    unknownType[3] = static_cast<char>(0x7f);
    events = decoder.feed(invalidFrame(unknownType));
    expect(events.size() == 1 && events[0].error == rb::DecodeError::UnknownMessageType,
           "unknown message type must be reported");

    events = decoder.feed(hex("03ff00"));
    expect(events.size() == 1 && events[0].error == rb::DecodeError::CobsDecodeFailed,
           "invalid COBS must be reported");
}

void testInterruptedFrameReset()
{
    const QByteArray valid = rb::PacketCodec::encodeWire(
        {rb::MessageType::Neutral, 10, hex("0300")});
    rb::StreamDecoder decoder;
    decoder.feed(valid.first(4));
    decoder.reset();
    const auto events = decoder.feed(valid);
    expect(events.size() == 1 && events.front().ok(),
           "reset must discard an interrupted frame and resynchronize");
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    testCrcReferenceValue();
    testGoldenVectors();
    testSplitAndStickyFrames();
    testValidationErrorsAndRecovery();
    testInterruptedFrameReset();
    if (failures == 0) {
        std::cout << "All protocol tests passed\n";
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
