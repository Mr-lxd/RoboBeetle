#ifndef SERVO_CALIBRATION_H
#define SERVO_CALIBRATION_H

#include <stdint.h>

typedef struct
{
    uint16_t min_pulse_us;
    uint16_t neutral_pulse_us;
    uint16_t max_pulse_us;
    int16_t min_angle_cdeg;
    int16_t max_angle_cdeg;
} servo_calibration_t;

const servo_calibration_t *servo_calibration_servo1(void);
uint16_t servo_calibration_angle_to_pulse(int16_t angle_cdeg);

#endif /* SERVO_CALIBRATION_H */
