#include "robot/ServoDescriptor.h"

#include <array>
#include <cstddef>

namespace rb {
namespace {

constexpr std::array<ServoDescriptor, kServoCount> kDescriptors{{
    {
        ServoId::FrontRight,
        0x0001,
        "FrontRight",
        "FrontRight",
        "SAVOX SW-0250MG+",
        true,
        true,
        false,
        1140,
        1580,
        2020,
        -4500,
        4500,
        1140,
        1860,
        -4500,
        4500,
    },
    {
        ServoId::FrontLeft,
        0x0002,
        "FrontLeft",
        "FrontLeft",
        "SAVOX SW-0250MG+",
        true,
        true,
        false,
        1900,
        1450,
        1000,
        -4500,
        4500,
        1160,
        1900,
        -4500,
        4500,
    },
    {
        ServoId::FrontAxis,
        0x0004,
        "FrontAxis",
        "Depth",
        "HDKJ S3150D",
        true,
        true,
        false,
        2430,
        1745,
        1060,
        -9000,
        9000,
        1060,
        2430,
        -9000,
        9000,
    },
    {
        ServoId::RearRight,
        0x0008,
        "RearRight",
        "RearRight",
        "GDW IPX896HV",
        true,
        true,
        false,
        1110,
        1570,
        2030,
        -4500,
        4500,
        1110,
        2030,
        -4500,
        4500,
    },
    {
        ServoId::RearLeft,
        0x0010,
        "RearLeft",
        "RearLeft",
        "GDW IPX896HV",
        true,
        true,
        false,
        1940,
        1450,
        960,
        -4500,
        4500,
        960,
        1940,
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
