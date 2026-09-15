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

void expectSavoX(
    const rb::ServoDescriptor &descriptor,
    int electricalMinPwmUs,
    int neutralPwmUs,
    int electricalMaxPwmUs,
    int commandMinPwmUs,
    int commandMaxPwmUs)
{
    expect(descriptor.hardwareName == std::string_view("SAVOX SW-0250MG+"),
           "SAVOX hardware name must match");
    expect(descriptor.angleSupported && !descriptor.calibrationPending,
           "SAVOX calibration must support angles and be complete");
    expect(descriptor.electricalMinPwmUs == electricalMinPwmUs
               && descriptor.neutralPwmUs == neutralPwmUs
               && descriptor.electricalMaxPwmUs == electricalMaxPwmUs,
           "SAVOX electrical calibration must match");
    expect(descriptor.electricalMinAngleCdeg == -4500
               && descriptor.electricalMaxAngleCdeg == 4500,
           "SAVOX calibration angle envelope must match +/-45 degrees");
    expect(descriptor.commandMinPwmUs == commandMinPwmUs
               && descriptor.commandMaxPwmUs == commandMaxPwmUs,
           "SAVOX command envelope must match");
    expect(descriptor.commandMinAngleCdeg == -4500
               && descriptor.commandMaxAngleCdeg == 4500,
           "SAVOX command angle envelope must match");
}

void expectGdw(
    const rb::ServoDescriptor &descriptor,
    int electricalMinPwmUs,
    int neutralPwmUs,
    int electricalMaxPwmUs,
    int commandMinPwmUs,
    int commandMaxPwmUs)
{
    expect(descriptor.hardwareName == std::string_view("GDW IPX896HV"),
           "GDW hardware name must match");
    expect(descriptor.angleSupported && !descriptor.calibrationPending,
           "GDW calibration must support angles and be complete");
    expect(descriptor.electricalMinPwmUs == electricalMinPwmUs
               && descriptor.neutralPwmUs == neutralPwmUs
               && descriptor.electricalMaxPwmUs == electricalMaxPwmUs,
           "GDW electrical calibration must match");
    expect(descriptor.electricalMinAngleCdeg == -4500
               && descriptor.electricalMaxAngleCdeg == 4500,
           "GDW calibration angle envelope must match +/-45 degrees");
    expect(descriptor.commandMinPwmUs == commandMinPwmUs
               && descriptor.commandMaxPwmUs == commandMaxPwmUs,
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

    expect(table[0].displayName == std::string_view("FrontRight"),
           "FrontRight display name must be ASCII and semantic");
    expect(table[1].displayName == std::string_view("FrontLeft"),
           "FrontLeft display name must be ASCII and semantic");
    expect(table[2].displayName == std::string_view("Depth"),
           "Depth display name must be ASCII");
    expect(table[3].displayName == std::string_view("RearRight"),
           "RearRight display name must be ASCII and semantic");
    expect(table[4].displayName == std::string_view("RearLeft"),
           "RearLeft display name must be ASCII and semantic");

    expectSavoX(table[0], 1140, 1580, 2020, 1140, 1860);
    expectSavoX(table[1], 1900, 1450, 1000, 1160, 1900);
    expect(table[2].hardwareName == std::string_view("HDKJ S3150D"),
           "FrontAxis hardware name must match");
    expect(table[2].angleSupported && !table[2].calibrationPending,
           "FrontAxis must support calibrated angles and have calibration complete");
    expect(table[2].electricalMinPwmUs == 2430 && table[2].neutralPwmUs == 1745
               && table[2].electricalMaxPwmUs == 1060,
           "FrontAxis electrical calibration must match 2430/1745/1060 us");
    expect(table[2].electricalMinAngleCdeg == -9000
               && table[2].electricalMaxAngleCdeg == 9000,
           "FrontAxis calibration angle envelope must match +/-90 degrees");
    expect(table[2].commandMinPwmUs == 1060 && table[2].commandMaxPwmUs == 2430
               && table[2].commandMinAngleCdeg == -9000
               && table[2].commandMaxAngleCdeg == 9000,
           "FrontAxis command envelope must match calibrated limits");
    expectGdw(table[3], 1110, 1570, 2030, 1110, 2030);
    expectGdw(table[4], 1940, 1450, 960, 960, 1940);

    expect(rb::servoDescriptor(rb::ServoId::FrontRight) == &table[0],
           "semantic descriptor lookup must return ID 0");
    expect(rb::servoDescriptor(static_cast<quint8>(5)) == nullptr,
           "out-of-range descriptor lookup must fail");

    if (failures == 0) {
        std::cout << "All Console Servo descriptor tests passed\n";
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
