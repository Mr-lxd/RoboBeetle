#ifndef SERVO_PWM_STOP_POLICY_H
#define SERVO_PWM_STOP_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/* A deferred channel stop may wait for at most the next PWM frame. */
#define SERVO_PWM_SAFE_STOP_MAX_EXTRA_FRAMES 1U

/*
 * TIM3/TIM4 use PWM mode 1, active-high, up-counting output compare.  A
 * channel may therefore be disabled immediately only after its compare edge.
 */
typedef enum
{
    SERVO_PWM_STOP_IMMEDIATE = 0,
    SERVO_PWM_STOP_DEFER_TO_COMPARE
} servo_pwm_stop_decision_t;

servo_pwm_stop_decision_t servo_pwm_stop_policy_decide(
    uint32_t counter,
    uint32_t compare);

bool servo_pwm_stop_policy_is_safe_low(
    uint32_t counter,
    uint32_t compare);

#endif /* SERVO_PWM_STOP_POLICY_H */
