#include "servo_descriptor.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

_Static_assert(SERVO_ID_FRONT_RIGHT == 0U, "FrontRight ID drifted");
_Static_assert(SERVO_ID_FRONT_LEFT == 1U, "FrontLeft ID drifted");
_Static_assert(SERVO_ID_FRONT_AXIS == 2U, "FrontAxis ID drifted");
_Static_assert(SERVO_ID_REAR_RIGHT == 3U, "RearRight ID drifted");
_Static_assert(SERVO_ID_REAR_LEFT == 4U, "RearLeft ID drifted");
_Static_assert(SERVO_DESCRIPTOR_COUNT == 5U, "servo count drifted");
_Static_assert(SERVO_DESCRIPTOR_SUPPORTED_MASK == 0x001FU,
               "supported mask drifted");

static int failures = 0;

static void expect(bool condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

static void expect_calibration(
    const servo_descriptor_t *descriptor,
    uint16_t min_pulse_us,
    uint16_t neutral_pulse_us,
    uint16_t max_pulse_us,
    int16_t min_angle_cdeg,
    int16_t max_angle_cdeg,
    uint16_t command_min_pulse_us,
    uint16_t command_max_pulse_us,
    int16_t command_min_angle_cdeg,
    int16_t command_max_angle_cdeg)
{
    expect(descriptor->calibration.min_pulse_us == min_pulse_us,
           "electrical minimum pulse differs");
    expect(descriptor->calibration.neutral_pulse_us == neutral_pulse_us,
           "electrical neutral pulse differs");
    expect(descriptor->calibration.max_pulse_us == max_pulse_us,
           "electrical maximum pulse differs");
    expect(descriptor->calibration.min_angle_cdeg == min_angle_cdeg,
           "electrical minimum angle differs");
    expect(descriptor->calibration.max_angle_cdeg == max_angle_cdeg,
           "electrical maximum angle differs");
    expect(descriptor->command_min_pulse_us == command_min_pulse_us,
           "mechanical minimum pulse differs");
    expect(descriptor->command_max_pulse_us == command_max_pulse_us,
           "mechanical maximum pulse differs");
    expect(descriptor->command_min_angle_cdeg == command_min_angle_cdeg,
           "mechanical minimum angle differs");
    expect(descriptor->command_max_angle_cdeg == command_max_angle_cdeg,
           "mechanical maximum angle differs");
}

static void test_descriptor_table(void)
{
    const servo_descriptor_t *table = servo_descriptor_table();

    expect(table != NULL, "descriptor table must exist");
    expect(servo_descriptor_count() == SERVO_DESCRIPTOR_COUNT,
           "descriptor count must be five");
    expect(servo_descriptor_supported_mask() == 0x001FU,
           "descriptor supported mask must be 0x001f");

    expect(table[SERVO_ID_FRONT_RIGHT].id == SERVO_ID_FRONT_RIGHT,
           "FrontRight row ID differs");
    expect(table[SERVO_ID_FRONT_RIGHT].mask == 0x0001U,
           "FrontRight mask differs");
    expect(table[SERVO_ID_FRONT_RIGHT].supported,
           "FrontRight must be supported");
    expect(table[SERVO_ID_FRONT_RIGHT].angle_supported,
           "FrontRight angle must be supported");
    expect(table[SERVO_ID_FRONT_RIGHT].timer == SERVO_TIMER_TIM3,
           "FrontRight timer differs");
    expect(table[SERVO_ID_FRONT_RIGHT].channel == SERVO_CHANNEL_1,
           "FrontRight channel differs");
    expect_calibration(&table[SERVO_ID_FRONT_RIGHT],
                       1000U, 1500U, 2000U, -5000, 5000,
                       1050U, 1950U, -4500, 4500);

    expect(table[SERVO_ID_FRONT_LEFT].id == SERVO_ID_FRONT_LEFT,
           "FrontLeft row ID differs");
    expect(table[SERVO_ID_FRONT_LEFT].mask == 0x0002U,
           "FrontLeft mask differs");
    expect(table[SERVO_ID_FRONT_LEFT].supported,
           "FrontLeft must be supported");
    expect(table[SERVO_ID_FRONT_LEFT].angle_supported,
           "FrontLeft angle must be supported");
    expect(table[SERVO_ID_FRONT_LEFT].timer == SERVO_TIMER_TIM3,
           "FrontLeft timer differs");
    expect(table[SERVO_ID_FRONT_LEFT].channel == SERVO_CHANNEL_2,
           "FrontLeft channel differs");
    expect_calibration(&table[SERVO_ID_FRONT_LEFT],
                       1000U, 1500U, 2000U, -5000, 5000,
                       1050U, 1950U, -4500, 4500);

    expect(table[SERVO_ID_FRONT_AXIS].id == SERVO_ID_FRONT_AXIS,
           "FrontAxis row ID differs");
    expect(table[SERVO_ID_FRONT_AXIS].mask == 0x0004U,
           "FrontAxis mask differs");
    expect(table[SERVO_ID_FRONT_AXIS].supported,
           "FrontAxis must be supported for PWM bring-up");
    expect(!table[SERVO_ID_FRONT_AXIS].angle_supported,
           "FrontAxis angle must remain unsupported");
    expect(table[SERVO_ID_FRONT_AXIS].timer == SERVO_TIMER_TIM3,
           "FrontAxis timer differs");
    expect(table[SERVO_ID_FRONT_AXIS].channel == SERVO_CHANNEL_3,
           "FrontAxis channel differs");
    expect_calibration(&table[SERVO_ID_FRONT_AXIS],
                       500U, 1500U, 2500U, 0, 0,
                       1450U, 1550U, 0, 0);

    expect(table[SERVO_ID_REAR_RIGHT].id == SERVO_ID_REAR_RIGHT,
           "RearRight row ID differs");
    expect(table[SERVO_ID_REAR_RIGHT].mask == 0x0008U,
           "RearRight mask differs");
    expect(table[SERVO_ID_REAR_RIGHT].supported,
           "RearRight must be supported");
    expect(table[SERVO_ID_REAR_RIGHT].angle_supported,
           "RearRight angle must be supported");
    expect(table[SERVO_ID_REAR_RIGHT].timer == SERVO_TIMER_TIM4,
           "RearRight timer differs");
    expect(table[SERVO_ID_REAR_RIGHT].channel == SERVO_CHANNEL_1,
           "RearRight channel differs");
    expect_calibration(&table[SERVO_ID_REAR_RIGHT],
                       520U, 1520U, 2520U, -9000, 9000,
                       1020U, 2020U, -4500, 4500);

    expect(table[SERVO_ID_REAR_LEFT].id == SERVO_ID_REAR_LEFT,
           "RearLeft row ID differs");
    expect(table[SERVO_ID_REAR_LEFT].mask == 0x0010U,
           "RearLeft mask differs");
    expect(table[SERVO_ID_REAR_LEFT].supported,
           "RearLeft must be supported");
    expect(table[SERVO_ID_REAR_LEFT].angle_supported,
           "RearLeft angle must be supported");
    expect(table[SERVO_ID_REAR_LEFT].timer == SERVO_TIMER_TIM4,
           "RearLeft timer differs");
    expect(table[SERVO_ID_REAR_LEFT].channel == SERVO_CHANNEL_2,
           "RearLeft channel differs");
    expect_calibration(&table[SERVO_ID_REAR_LEFT],
                       520U, 1520U, 2520U, -9000, 9000,
                       1020U, 2020U, -4500, 4500);

    expect(servo_descriptor_for_id(SERVO_ID_FRONT_RIGHT) ==
               &table[SERVO_ID_FRONT_RIGHT],
           "FrontRight lookup differs");
    expect(servo_descriptor_for_id(SERVO_ID_REAR_LEFT) ==
               &table[SERVO_ID_REAR_LEFT],
           "RearLeft lookup differs");
    expect(servo_descriptor_for_id(5U) == NULL,
           "out-of-range ID must not resolve");
    expect(servo_descriptor_for_id(0xffU) == NULL,
           "invalid ID must not resolve");
}

int main(void)
{
    test_descriptor_table();

    if (failures == 0)
    {
        (void)puts("All firmware Servo descriptor tests passed");
    }

    return failures == 0 ? 0 : 1;
}
