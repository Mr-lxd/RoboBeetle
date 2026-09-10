#include "servo_calibration.h"
#include "servo_descriptor.h"

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

int main(void)
{
    const servo_descriptor_t *table = servo_descriptor_table();
    const servo_calibration_t *calibration =
        &table[SERVO_ID_REAR_RIGHT].calibration;

    expect(calibration != NULL, "RearRight calibration must exist");
    expect(calibration->min_pulse_us == 520U,
           "RearRight minimum pulse calibration differs");
    expect(calibration->neutral_pulse_us == 1520U,
           "RearRight neutral pulse calibration differs");
    expect(calibration->max_pulse_us == 2520U,
           "RearRight maximum pulse calibration differs");
    expect(calibration->min_angle_cdeg == -9000,
           "RearRight minimum angle calibration differs");
    expect(calibration->max_angle_cdeg == 9000,
           "RearRight maximum angle calibration differs");

    expect(servo_calibration_angle_to_pulse(calibration, -9000) == 520U,
           "-9000 cdeg mapping differs");
    expect(servo_calibration_angle_to_pulse(calibration, -4500) == 1020U,
           "-4500 cdeg mapping differs");
    expect(servo_calibration_angle_to_pulse(calibration, 0) == 1520U,
           "zero cdeg mapping differs");
    expect(servo_calibration_angle_to_pulse(calibration, 4500) == 2020U,
           "+4500 cdeg mapping differs");
    expect(servo_calibration_angle_to_pulse(calibration, 9000) == 2520U,
           "+9000 cdeg mapping differs");
    expect(servo_calibration_angle_to_pulse(calibration, -8999) == 521U,
           "negative integer truncation differs");
    expect(servo_calibration_angle_to_pulse(calibration, 8999) == 2519U,
           "positive integer truncation differs");

    calibration = &table[SERVO_ID_FRONT_RIGHT].calibration;
    expect(servo_calibration_angle_to_pulse(calibration, -4500) == 1050U,
           "SAVOX -4500 cdeg mapping differs");
    expect(servo_calibration_angle_to_pulse(calibration, 0) == 1500U,
           "SAVOX zero cdeg mapping differs");
    expect(servo_calibration_angle_to_pulse(calibration, 4500) == 1950U,
           "SAVOX +4500 cdeg mapping differs");

    if (failures == 0)
    {
        (void)puts("All firmware Servo calibration tests passed");
    }

    return failures == 0 ? 0 : 1;
}
