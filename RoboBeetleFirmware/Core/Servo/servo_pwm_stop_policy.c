#include "servo_pwm_stop_policy.h"

bool servo_pwm_stop_policy_is_safe_low(
    uint32_t counter,
    uint32_t compare)
{
    return (compare == 0U) || (counter >= compare);
}

servo_pwm_stop_decision_t servo_pwm_stop_policy_decide(
    uint32_t counter,
    uint32_t compare)
{
    if (servo_pwm_stop_policy_is_safe_low(counter, compare))
    {
        return SERVO_PWM_STOP_IMMEDIATE;
    }

    return SERVO_PWM_STOP_DEFER_TO_COMPARE;
}
