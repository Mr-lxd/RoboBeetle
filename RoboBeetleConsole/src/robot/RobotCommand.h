#pragma once

#include "protocol/Packet.h"

#include <QtGlobal>

#include <cmath>

namespace rb {

enum class ServoId : quint8 {
    Servo1 = 0,
    Servo2 = 1,
};

constexpr quint16 servoMask(ServoId id)
{
    return static_cast<quint16>(1U << static_cast<quint8>(id));
}

constexpr quint16 SupportedServoMaskPhase1 = servoMask(ServoId::Servo1);

inline qint16 angleDegreesToCentidegrees(double degrees)
{
    return static_cast<qint16>(std::lround(degrees * 100.0));
}

} // namespace rb
