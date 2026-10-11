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
    SetFrontRearCoordination = 0x17,
    SetCpgParameters = 0x18,
    QueryCpgParameters = 0x1a,
    LeakStatus = 0x20,
    ImuSnapshot = 0x21,
    DepthSnapshot = 0x22,
    MotionStateBatch = 0x23,
    CpgParametersSnapshot = 0x24,
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
    case MessageType::SetCpgParameters:
    case MessageType::QueryCpgParameters:
    case MessageType::CpgParametersSnapshot:
    case MessageType::SetFrontRearCoordination:
    case MessageType::LeakStatus:
    case MessageType::ImuSnapshot:
    case MessageType::DepthSnapshot:
    case MessageType::MotionStateBatch:
        return true;
    }
    return false;
}

} // namespace robobeetle::protocol
