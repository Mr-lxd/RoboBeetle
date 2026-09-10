#pragma once

#include <QByteArray>
#include <QString>
#include <QtGlobal>

namespace rb {

enum class MessageType : quint8 {
    Heartbeat = 0x01,
    Ack = 0x02,
    Error = 0x03,
    ServoEnable = 0x10,
    ServoDisable = 0x11,
    SetServoPwm = 0x12,
    SetServoAngle = 0x13,
    Neutral = 0x14,
    LeakStatus = 0x20,
    ImuSnapshot = 0x21,
};

enum class AckResult : quint8 {
    Ok = 0,
    InvalidPayload = 1,
    HostNotAlive = 2,
    UnsupportedServo = 3,
    ServoNotEnabled = 4,
    OutOfRange = 5,
    HardwareFailure = 6,
};

constexpr bool isKnownMessageType(quint8 value)
{
    switch (static_cast<MessageType>(value)) {
    case MessageType::Heartbeat:
    case MessageType::Ack:
    case MessageType::Error:
    case MessageType::ServoEnable:
    case MessageType::ServoDisable:
    case MessageType::SetServoPwm:
    case MessageType::SetServoAngle:
    case MessageType::Neutral:
    case MessageType::LeakStatus:
    case MessageType::ImuSnapshot:
        return true;
    }
    return false;
}

struct Packet {
    MessageType type{MessageType::Heartbeat};
    quint16 sequence{0};
    QByteArray payload;

    bool operator==(const Packet &) const = default;
};

enum class DecodeError {
    None,
    CobsDecodeFailed,
    InvalidMagic,
    InvalidVersion,
    InvalidLength,
    UnknownMessageType,
    CrcMismatch,
};

struct DecodeResult {
    Packet packet;
    DecodeError error{DecodeError::None};
    QString detail;

    [[nodiscard]] bool ok() const { return error == DecodeError::None; }
};

} // namespace rb
