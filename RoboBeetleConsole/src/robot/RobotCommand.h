#pragma once

#include "protocol/Packet.h"

#include <QtGlobal>

#include <cmath>

namespace rb {

enum class ServoId : quint8 {
    FrontRight = 0,
    FrontLeft = 1,
    FrontAxis = 2,
    RearRight = 3,
    RearLeft = 4,
    // Historical v0.4 source compatibility only. New code must use
    // FrontRight; this alias must not be used for UI or log semantics.
    Servo1
#if !defined(Q_MOC_RUN)
        [[deprecated("Servo1 is a historical alias for FrontRight")]]
#endif
        = FrontRight,
};

inline constexpr qsizetype kServoCount = 5;
inline constexpr quint16 SupportedServoMask = 0x001f;

static_assert(static_cast<quint8>(ServoId::FrontRight) == 0,
              "FrontRight ID must remain zero");
static_assert(static_cast<quint8>(ServoId::FrontLeft) == 1,
              "FrontLeft ID must remain one");
static_assert(static_cast<quint8>(ServoId::FrontAxis) == 2,
              "FrontAxis ID must remain two");
static_assert(static_cast<quint8>(ServoId::RearRight) == 3,
              "RearRight ID must remain three");
static_assert(static_cast<quint8>(ServoId::RearLeft) == 4,
              "RearLeft ID must remain four");
static_assert(SupportedServoMask == 0x001f,
              "all five semantic servo bits must remain supported");

constexpr quint16 servoMask(ServoId id)
{
    return static_cast<quint16>(1U << static_cast<quint8>(id));
}

constexpr quint16 SupportedServoMaskPhase1 = SupportedServoMask;

inline qint16 angleDegreesToCentidegrees(double degrees)
{
    return static_cast<qint16>(std::lround(degrees * 100.0));
}

} // namespace rb
