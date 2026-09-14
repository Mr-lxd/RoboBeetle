#include "servo_calibration.h"
#include "servo_descriptor.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

static int failures = 0;

static void expect(int condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

typedef struct
{
    uint8_t id;
    uint16_t min_pulse_us;
    uint16_t neutral_pulse_us;
    uint16_t max_pulse_us;
    uint16_t negative_mid_pulse_us;
    uint16_t positive_mid_pulse_us;
} paddle_case_t;

static void expect_paddle_mapping(const paddle_case_t *test_case)
{
    const servo_descriptor_t *table = servo_descriptor_table();
    const servo_calibration_t *calibration =
        &table[test_case->id].calibration;

    expect(calibration != NULL, "paddle calibration must exist");
    expect(calibration->min_pulse_us == test_case->min_pulse_us,
           "paddle minimum pulse calibration differs");
    expect(calibration->neutral_pulse_us == test_case->neutral_pulse_us,
           "paddle neutral pulse calibration differs");
    expect(calibration->max_pulse_us == test_case->max_pulse_us,
           "paddle maximum pulse calibration differs");
    expect(calibration->min_angle_cdeg == -4500,
           "paddle minimum angle calibration differs");
    expect(calibration->max_angle_cdeg == 4500,
           "paddle maximum angle calibration differs");

    expect(servo_calibration_angle_to_pulse(calibration, -4500) ==
               test_case->min_pulse_us,
           "paddle -45 degree mapping differs");
    expect(servo_calibration_angle_to_pulse(calibration, -2250) ==
               test_case->negative_mid_pulse_us,
           "paddle -22.5 degree mapping differs");
    expect(servo_calibration_angle_to_pulse(calibration, 0) ==
               test_case->neutral_pulse_us,
           "paddle zero degree mapping differs");
    expect(servo_calibration_angle_to_pulse(calibration, 2250) ==
               test_case->positive_mid_pulse_us,
           "paddle +22.5 degree mapping differs");
    expect(servo_calibration_angle_to_pulse(calibration, 4500) ==
               test_case->max_pulse_us,
           "paddle +45 degree mapping differs");
}

static void test_front_axis_mapping(void)
{
    const servo_descriptor_t *table = servo_descriptor_table();
    const servo_calibration_t *calibration =
        &table[SERVO_ID_FRONT_AXIS].calibration;

    expect(calibration->min_pulse_us == 2430U,
           "FrontAxis minimum pulse calibration differs");
    expect(calibration->neutral_pulse_us == 1745U,
           "FrontAxis neutral pulse calibration differs");
    expect(calibration->max_pulse_us == 1060U,
           "FrontAxis maximum pulse calibration differs");
    expect(calibration->min_angle_cdeg == -9000,
           "FrontAxis minimum angle calibration differs");
    expect(calibration->max_angle_cdeg == 9000,
           "FrontAxis maximum angle calibration differs");
    expect(servo_calibration_angle_to_pulse(calibration, -9000) == 2430U,
           "FrontAxis -90 degree mapping differs");
    expect(servo_calibration_angle_to_pulse(calibration, -4500) == 2087U,
           "FrontAxis -45 degree mapping differs");
    expect(servo_calibration_angle_to_pulse(calibration, 0) == 1745U,
           "FrontAxis zero degree mapping differs");
    expect(servo_calibration_angle_to_pulse(calibration, 4500) == 1403U,
           "FrontAxis +45 degree mapping differs");
    expect(servo_calibration_angle_to_pulse(calibration, 9000) == 1060U,
           "FrontAxis +90 degree mapping differs");
}

int main(void)
{
    static const paddle_case_t paddle_cases[] = {
        {SERVO_ID_FRONT_RIGHT, 2020U, 1580U, 1140U, 1800U, 1360U},
        {SERVO_ID_FRONT_LEFT, 1000U, 1450U, 1900U, 1225U, 1675U},
        {SERVO_ID_REAR_RIGHT, 1110U, 1570U, 2030U, 1340U, 1800U},
        {SERVO_ID_REAR_LEFT, 1940U, 1450U, 960U, 1695U, 1205U},
    };

    for (size_t index = 0U;
         index < sizeof(paddle_cases) / sizeof(paddle_cases[0]);
         ++index)
    {
        expect_paddle_mapping(&paddle_cases[index]);
    }
    test_front_axis_mapping();

    if (failures == 0)
    {
        (void)puts("All firmware Servo calibration tests passed");
    }

    return failures == 0 ? 0 : 1;
}
