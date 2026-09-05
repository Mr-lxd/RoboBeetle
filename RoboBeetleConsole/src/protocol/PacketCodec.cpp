#include "protocol/PacketCodec.h"

#include "protocol/Crc16.h"

namespace rb {
namespace {

void appendLe16(QByteArray &data, quint16 value)
{
    data.append(static_cast<char>(value & 0xffU));
    data.append(static_cast<char>((value >> 8U) & 0xffU));
}

quint16 readLe16(QByteArrayView data, qsizetype offset)
{
    const auto low = static_cast<quint8>(data[offset]);
    const auto high = static_cast<quint8>(data[offset + 1]);
    return static_cast<quint16>(low | (static_cast<quint16>(high) << 8U));
}

DecodeResult failure(DecodeError error, const QString &detail)
{
    DecodeResult result;
    result.error = error;
    result.detail = detail;
    return result;
}

} // namespace

QByteArray PacketCodec::encodeLogical(const Packet &packet)
{
    if (packet.payload.size() > MaxPayloadSize) {
        return {};
    }

    QByteArray logical;
    logical.reserve(10 + packet.payload.size());
    logical.append(static_cast<char>(Magic0));
    logical.append(static_cast<char>(Magic1));
    logical.append(static_cast<char>(Version));
    logical.append(static_cast<char>(packet.type));
    appendLe16(logical, packet.sequence);
    appendLe16(logical, static_cast<quint16>(packet.payload.size()));
    logical.append(packet.payload);
    appendLe16(logical, crc16CcittFalse(logical));
    return logical;
}

QByteArray PacketCodec::encodeWire(const Packet &packet)
{
    const QByteArray logical = encodeLogical(packet);
    if (logical.isEmpty()) {
        return {};
    }
    QByteArray wire = cobsEncode(logical);
    wire.append('\0');
    return wire;
}

DecodeResult PacketCodec::decodeLogical(QByteArrayView logicalFrame)
{
    constexpr qsizetype fixedSize = 10;
    if (logicalFrame.size() < fixedSize) {
        return failure(DecodeError::InvalidLength, QStringLiteral("Logical frame is shorter than 10 bytes"));
    }
    if (static_cast<quint8>(logicalFrame[0]) != Magic0
        || static_cast<quint8>(logicalFrame[1]) != Magic1) {
        return failure(DecodeError::InvalidMagic, QStringLiteral("Magic must be 0x52 0x42"));
    }
    if (static_cast<quint8>(logicalFrame[2]) != Version) {
        return failure(DecodeError::InvalidVersion, QStringLiteral("Protocol version must be 0x02"));
    }

    const quint16 payloadLength = readLe16(logicalFrame, 6);
    if (payloadLength > MaxPayloadSize
        || logicalFrame.size() != fixedSize + static_cast<qsizetype>(payloadLength)) {
        return failure(DecodeError::InvalidLength, QStringLiteral("Payload length does not match frame size"));
    }

    const quint8 rawType = static_cast<quint8>(logicalFrame[3]);
    if (!isKnownMessageType(rawType)) {
        return failure(DecodeError::UnknownMessageType, QStringLiteral("Message type is not defined in Protocol V2"));
    }

    const quint16 expectedCrc = readLe16(logicalFrame, logicalFrame.size() - 2);
    const quint16 actualCrc = crc16CcittFalse(logicalFrame.first(logicalFrame.size() - 2));
    if (actualCrc != expectedCrc) {
        return failure(DecodeError::CrcMismatch, QStringLiteral("CRC-16/CCITT-FALSE mismatch"));
    }

    DecodeResult result;
    result.packet.type = static_cast<MessageType>(rawType);
    result.packet.sequence = readLe16(logicalFrame, 4);
    result.packet.payload = QByteArray(logicalFrame.data() + 8, payloadLength);
    return result;
}

DecodeResult PacketCodec::decodeWire(QByteArrayView encodedFrame)
{
    QByteArray logical;
    if (!cobsDecode(encodedFrame, logical)) {
        return failure(DecodeError::CobsDecodeFailed, QStringLiteral("COBS frame is malformed"));
    }
    return decodeLogical(logical);
}

QByteArray PacketCodec::cobsEncode(QByteArrayView data)
{
    QByteArray encoded;
    encoded.reserve(data.size() + data.size() / 254 + 1);
    encoded.append('\0');
    qsizetype codeIndex = 0;
    quint8 code = 1;

    for (const char value : data) {
        if (value == '\0') {
            encoded[codeIndex] = static_cast<char>(code);
            codeIndex = encoded.size();
            encoded.append('\0');
            code = 1;
        } else {
            encoded.append(value);
            ++code;
            if (code == 0xffU) {
                encoded[codeIndex] = static_cast<char>(code);
                codeIndex = encoded.size();
                encoded.append('\0');
                code = 1;
            }
        }
    }
    encoded[codeIndex] = static_cast<char>(code);
    return encoded;
}

bool PacketCodec::cobsDecode(QByteArrayView encoded, QByteArray &decoded)
{
    decoded.clear();
    if (encoded.isEmpty()) {
        return false;
    }

    qsizetype index = 0;
    while (index < encoded.size()) {
        const quint8 code = static_cast<quint8>(encoded[index++]);
        if (code == 0U) {
            return false;
        }
        const qsizetype copyCount = static_cast<qsizetype>(code) - 1;
        if (index + copyCount > encoded.size()) {
            return false;
        }
        decoded.append(encoded.data() + index, copyCount);
        index += copyCount;
        if (code != 0xffU && index < encoded.size()) {
            decoded.append('\0');
        }
    }
    return true;
}

QString decodeErrorText(DecodeError error)
{
    switch (error) {
    case DecodeError::None: return QStringLiteral("None");
    case DecodeError::CobsDecodeFailed: return QStringLiteral("COBS decode failed");
    case DecodeError::InvalidMagic: return QStringLiteral("Invalid magic");
    case DecodeError::InvalidVersion: return QStringLiteral("Invalid version");
    case DecodeError::InvalidLength: return QStringLiteral("Invalid length");
    case DecodeError::UnknownMessageType: return QStringLiteral("Unknown message type");
    case DecodeError::CrcMismatch: return QStringLiteral("CRC mismatch");
    }
    return QStringLiteral("Unknown decode error");
}

} // namespace rb
