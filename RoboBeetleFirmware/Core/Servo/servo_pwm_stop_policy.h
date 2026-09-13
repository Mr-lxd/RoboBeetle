#ifndef SERVO_PWM_STOP_POLICY_H
#define SERVO_PWM_STOP_POLICY_H

#include <stdbool.h>

/* A deferred channel stop may wait for at most the next PWM frame. */
#define SERVO_PWM_SAFE_STOP_MAX_EXTRA_FRAMES 1U

/*
 * With PWM preload/shadow state, readable CNT/CCR values cannot prove the
 * output level of the currently active pulse.  A running active channel must
 * therefore always wait for the next hardware compare edge.
 */
typedef enum
{
    SERVO_PWM_STOP_IMMEDIATE = 0,
    SERVO_PWM_STOP_DEFER_TO_COMPARE
} servo_pwm_stop_decision_t;

servo_pwm_stop_decision_t servo_pwm_stop_policy_decide(
    bool timer_running,
    bool channel_active);

#endif /* SERVO_PWM_STOP_POLICY_H */
