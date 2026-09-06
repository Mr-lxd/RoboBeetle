#include "servo_calibration.h"

static const servo_calibration_t servo1_calibration = {
    .min_pulse_us = 520U,
    .neutral_pulse_us = 1520U,
    .max_pulse_us = 2520U,
    .min_angle_cdeg = -9000,
    .max_angle_cdeg = 9000,
};

const servo_calibration_t *servo_calibration_servo1(void)
{
    return &servo1_calibration;
}

uint16_t servo_calibration_angle_to_pulse(int16_t angle_cdeg)
{
    if (angle_cdeg < 0)
    {
        int32_t pulse =
            (int32_t)servo1_calibration.neutral_pulse_us +
            ((int32_t)angle_cdeg *
             ((int32_t)servo1_calibration.neutral_pulse_us -
              (int32_t)servo1_calibration.min_pulse_us))
            / 9000;

        return (uint16_t)pulse;
    }
    else
    {
        int32_t pulse =
            (int32_t)servo1_calibration.neutral_pulse_us +
            ((int32_t)angle_cdeg *
             ((int32_t)servo1_calibration.max_pulse_us -
              (int32_t)servo1_calibration.neutral_pulse_us))
            / 9000;

        return (uint16_t)pulse;
    }
}
