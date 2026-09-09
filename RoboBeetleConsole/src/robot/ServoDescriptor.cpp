#include "robot/ServoDescriptor.h"

namespace rb {
namespace {

constexpr std::array<ServoDescriptor, kServoCount> kDescriptors{{
    {
        ServoId::FrontRight,
        0x0001,
        "FrontRight",
        "FrontRight / 前足右",
        "SAVOX SW-0250MG+",
        true,
        true,
        false,
        1000,
        1500,
        2000,
        -5000,
        5000,
        1050,
        1950,
        -4500,
        4500,
    },
    {
        ServoId::FrontLeft,
        0x0002,
        "FrontLeft",
        "FrontLeft / 前足左",
        "SAVOX SW-0250MG+",
        true,
        true,
        false,
        1000,
        1500,
        2000,
        -5000,
        5000,
        1050,
        1950,
        -4500,
        4500,
    },
    {
        ServoId::FrontAxis,
        0x0004,
        "FrontAxis",
        "FrontAxis / 升潜前足轴",
        "HDKJ S3150D",
        true,
        false,
        true,
        500,
        1500,
        2500,
        0,
        0,
        1000,
        2000,
        0,
        0,
    },
    {
        ServoId::RearRight,
        0x0008,
        "RearRight",
        "RearRight / 后足右",
        "GDW IPX896HV",
        true,
        true,
        false,
        520,
        1520,
        2520,
        -9000,
        9000,
        1020,
        2020,
        -4500,
        4500,
    },
    {
        ServoId::RearLeft,
        0x0010,
        "RearLeft",
        "RearLeft / 后足左",
        "GDW IPX896HV",
        true,
        true,
        false,
        520,
        1520,
        2520,
        -9000,
        9000,
        1020,
        2020,
        -4500,
        4500,
    },
}};

static_assert(kDescriptors.size() == static_cast<std::size_t>(kServoCount));
static_assert(static_cast<quint8>(kDescriptors[0].id) == 0);
static_assert(static_cast<quint8>(kDescriptors[1].id) == 1);
static_assert(static_cast<quint8>(kDescriptors[2].id) == 2);
static_assert(static_cast<quint8>(kDescriptors[3].id) == 3);
static_assert(static_cast<quint8>(kDescriptors[4].id) == 4);

} // namespace

const std::array<ServoDescriptor, kServoCount> &servoDescriptorTable()
{
    return kDescriptors;
}

const ServoDescriptor *servoDescriptor(ServoId id)
{
    return servoDescriptor(static_cast<quint8>(id));
}

const ServoDescriptor *servoDescriptor(quint8 id)
{
    if (id >= static_cast<quint8>(kServoCount)) {
        return nullptr;
    }
    const ServoDescriptor &descriptor = kDescriptors.at(id);
    return static_cast<quint8>(descriptor.id) == id ? &descriptor : nullptr;
}

} // namespace rb
