#include "servo_calibration.h"

#include <stddef.h>
#include <stdint.h>

uint16_t servo_calibration_angle_to_pulse(
    const servo_calibration_t *calibration,
    int16_t angle_cdeg)
{
    if (calibration == NULL)
    {
        return 0U;
    }

    if (angle_cdeg < 0)
    {
        if (calibration->min_angle_cdeg >= 0)
        {
            return calibration->neutral_pulse_us;
        }

        int32_t pulse =
            (int32_t)calibration->neutral_pulse_us +
            ((int32_t)angle_cdeg *
             ((int32_t)calibration->neutral_pulse_us -
              (int32_t)calibration->min_pulse_us))
            / (-(int32_t)calibration->min_angle_cdeg);

        return (uint16_t)pulse;
    }

    if (calibration->max_angle_cdeg <= 0)
    {
        return calibration->neutral_pulse_us;
    }

    int32_t pulse =
        (int32_t)calibration->neutral_pulse_us +
        ((int32_t)angle_cdeg *
         ((int32_t)calibration->max_pulse_us -
          (int32_t)calibration->neutral_pulse_us))
        / (int32_t)calibration->max_angle_cdeg;

    return (uint16_t)pulse;
}
