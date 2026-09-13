#include "servo_pwm_stop_policy.h"

servo_pwm_stop_decision_t servo_pwm_stop_policy_decide(
    bool timer_running,
    bool channel_active)
{
    if (!timer_running || !channel_active)
    {
        return SERVO_PWM_STOP_IMMEDIATE;
    }

    return SERVO_PWM_STOP_DEFER_TO_COMPARE;
}
