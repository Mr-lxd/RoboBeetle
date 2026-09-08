#include "robot/ServoDescriptor.h"

#include <cstdlib>
#include <iostream>
#include <string_view>

namespace {

int failures = 0;

void expect(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

void expectSavoX(const rb::ServoDescriptor &descriptor)
{
    expect(descriptor.hardwareName == std::string_view("SAVOX SW-0250MG+"),
           "SAVOX hardware name must match");
    expect(descriptor.electricalMinPwmUs == 1000 && descriptor.neutralPwmUs == 1500
               && descriptor.electricalMaxPwmUs == 2000,
           "SAVOX electrical calibration must match");
    expect(descriptor.commandMinPwmUs == 1050 && descriptor.commandMaxPwmUs == 1950,
           "SAVOX command envelope must match");
    expect(descriptor.commandMinAngleCdeg == -4500
               && descriptor.commandMaxAngleCdeg == 4500,
           "SAVOX command angle envelope must match");
}

void expectGdw(const rb::ServoDescriptor &descriptor)
{
    expect(descriptor.hardwareName == std::string_view("GDW IPX896HV"),
           "GDW hardware name must match");
    expect(descriptor.electricalMinPwmUs == 520 && descriptor.neutralPwmUs == 1520
               && descriptor.electricalMaxPwmUs == 2520,
           "GDW electrical calibration must match");
    expect(descriptor.commandMinPwmUs == 1020 && descriptor.commandMaxPwmUs == 2020,
           "GDW command envelope must match");
    expect(descriptor.commandMinAngleCdeg == -4500
               && descriptor.commandMaxAngleCdeg == 4500,
           "GDW command angle envelope must match");
}

} // namespace

int main()
{
    static_assert(static_cast<quint8>(rb::ServoId::FrontRight) == 0);
    static_assert(static_cast<quint8>(rb::ServoId::FrontLeft) == 1);
    static_assert(static_cast<quint8>(rb::ServoId::FrontAxis) == 2);
    static_assert(static_cast<quint8>(rb::ServoId::RearRight) == 3);
    static_assert(static_cast<quint8>(rb::ServoId::RearLeft) == 4);
    static_assert(rb::SupportedServoMask == 0x001f);

    const auto &table = rb::servoDescriptorTable();
    expect(table.size() == rb::kServoCount, "Qt descriptor table must contain five rows");
    expect((table[0].mask | table[1].mask | table[2].mask | table[3].mask | table[4].mask)
               == rb::SupportedServoMask,
           "Qt descriptor mask must cover exactly five supported bits");
    expect(table[0].id == rb::ServoId::FrontRight && table[0].mask == 0x0001,
           "ID 0 must be FrontRight");
    expect(table[1].id == rb::ServoId::FrontLeft && table[1].mask == 0x0002,
           "ID 1 must be FrontLeft");
    expect(table[2].id == rb::ServoId::FrontAxis && table[2].mask == 0x0004,
           "ID 2 must be FrontAxis");
    expect(table[3].id == rb::ServoId::RearRight && table[3].mask == 0x0008,
           "ID 3 must be RearRight");
    expect(table[4].id == rb::ServoId::RearLeft && table[4].mask == 0x0010,
           "ID 4 must be RearLeft");

    expectSavoX(table[0]);
    expectSavoX(table[1]);
    expect(table[2].hardwareName == std::string_view("HDKJ S3150D"),
           "FrontAxis hardware name must match");
    expect(!table[2].angleSupported && table[2].calibrationPending,
           "FrontAxis must remain PWM-only and calibration pending");
    expect(table[2].electricalMinPwmUs == 500 && table[2].neutralPwmUs == 1500
               && table[2].electricalMaxPwmUs == 2500,
           "FrontAxis electrical metadata must remain 500/1500/2500 us");
    expect(table[2].commandMinPwmUs == 1450 && table[2].commandMaxPwmUs == 1550
               && table[2].neutralPwmUs == 1500,
           "FrontAxis PWM bring-up envelope must match");
    expectGdw(table[3]);
    expectGdw(table[4]);

    expect(rb::servoDescriptor(rb::ServoId::FrontRight) == &table[0],
           "semantic descriptor lookup must return ID 0");
    expect(rb::servoDescriptor(static_cast<quint8>(5)) == nullptr,
           "out-of-range descriptor lookup must fail");

    if (failures == 0) {
        std::cout << "All Console Servo descriptor tests passed\n";
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
