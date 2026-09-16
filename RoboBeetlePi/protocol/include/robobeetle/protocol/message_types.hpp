#pragma once

#include <cstdint>

namespace robobeetle::protocol {

enum class MessageType : std::uint8_t {
    Heartbeat = 0x01,
    Ack = 0x02,
    Error = 0x03,
    ServoEnable = 0x10,
    ServoDisable = 0x11,
    SetServoPwm = 0x12,
    SetServoAngle = 0x13,
    Neutral = 0x14,
    SetMotionMode = 0x15,
    SetGaitBackend = 0x16,
    LeakStatus = 0x20,
    ImuSnapshot = 0x21,
    DepthSnapshot = 0x22,
};

enum class AckResult : std::uint8_t {
    Ok = 0,
    InvalidPayload = 1,
    HostNotAlive = 2,
    UnsupportedServo = 3,
    ServoNotEnabled = 4,
    OutOfRange = 5,
    HardwareFailure = 6,
    Busy = 7,
};

constexpr bool is_known_message_type(std::uint8_t value)
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
    case MessageType::SetMotionMode:
    case MessageType::SetGaitBackend:
    case MessageType::LeakStatus:
    case MessageType::ImuSnapshot:
    case MessageType::DepthSnapshot:
        return true;
    }
    return false;
}

} // namespace robobeetle::protocol
