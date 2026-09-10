#pragma once

#include "robot/RobotCommand.h"

#include <array>

namespace rb {

struct ServoDescriptor {
    ServoId id;
    quint16 mask;
    const char *semanticName;
    const char *displayName;
    const char *hardwareName;
    bool supported;
    bool angleSupported;
    bool calibrationPending;
    quint16 electricalMinPwmUs;
    quint16 neutralPwmUs;
    quint16 electricalMaxPwmUs;
    qint16 electricalMinAngleCdeg;
    qint16 electricalMaxAngleCdeg;
    quint16 commandMinPwmUs;
    quint16 commandMaxPwmUs;
    qint16 commandMinAngleCdeg;
    qint16 commandMaxAngleCdeg;
};

const std::array<ServoDescriptor, kServoCount> &servoDescriptorTable();
const ServoDescriptor *servoDescriptor(ServoId id);
const ServoDescriptor *servoDescriptor(quint8 id);

} // namespace rb
