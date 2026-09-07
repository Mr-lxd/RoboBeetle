#include "servo_calibration.h"

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
    const servo_calibration_t *calibration =
        servo_calibration_servo1();

    expect(calibration != NULL, "Servo1 calibration must exist");
    expect(calibration->min_pulse_us == 520U,
           "minimum pulse calibration differs");
    expect(calibration->neutral_pulse_us == 1520U,
           "neutral pulse calibration differs");
    expect(calibration->max_pulse_us == 2520U,
           "maximum pulse calibration differs");
    expect(calibration->min_angle_cdeg == -9000,
           "minimum angle calibration differs");
    expect(calibration->max_angle_cdeg == 9000,
           "maximum angle calibration differs");

    expect(servo_calibration_angle_to_pulse(-9000) == 520U,
           "-9000 cdeg mapping differs");
    expect(servo_calibration_angle_to_pulse(-4500) == 1020U,
           "-4500 cdeg mapping differs");
    expect(servo_calibration_angle_to_pulse(0) == 1520U,
           "zero cdeg mapping differs");
    expect(servo_calibration_angle_to_pulse(4500) == 2020U,
           "+4500 cdeg mapping differs");
    expect(servo_calibration_angle_to_pulse(9000) == 2520U,
           "+9000 cdeg mapping differs");
    expect(servo_calibration_angle_to_pulse(-8999) == 521U,
           "negative integer truncation differs");
    expect(servo_calibration_angle_to_pulse(8999) == 2519U,
           "positive integer truncation differs");

    if (failures == 0)
    {
        (void)puts("All firmware Servo calibration tests passed");
    }

    return failures == 0 ? 0 : 1;
}
