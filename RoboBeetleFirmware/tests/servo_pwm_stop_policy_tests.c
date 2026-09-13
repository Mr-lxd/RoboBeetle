#include "servo_pwm_stop_policy.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static int failures = 0;

static void expect(bool condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

static void test_running_active_pwm_always_defers(void)
{
    expect(servo_pwm_stop_policy_decide(true, true) ==
               SERVO_PWM_STOP_DEFER_TO_COMPARE,
           "running active PWM must always defer to a real compare edge");
}

static void test_inactive_state_is_immediate(void)
{
    expect(servo_pwm_stop_policy_decide(false, true) ==
               SERVO_PWM_STOP_IMMEDIATE,
           "a stopped timer may finalize immediately");
    expect(servo_pwm_stop_policy_decide(true, false) ==
               SERVO_PWM_STOP_IMMEDIATE,
           "an inactive channel may finalize immediately");
}

static void test_deferred_stop_has_one_frame_bound(void)
{
    /* TIM3/TIM4 use ARR=3002, so one complete frame is ARR+1 ticks. */
    const uint32_t current_frame_ticks = 3002U + 1U;

    expect(SERVO_PWM_SAFE_STOP_MAX_EXTRA_FRAMES == 1U,
           "safe-stop policy must allow at most one extra PWM frame");
    expect((SERVO_PWM_SAFE_STOP_MAX_EXTRA_FRAMES * current_frame_ticks) ==
               3003U,
           "the current safe-stop frame bound must be 3003 timer ticks");
}

int main(void)
{
    test_running_active_pwm_always_defers();
    test_inactive_state_is_immediate();
    test_deferred_stop_has_one_frame_bound();

    if (failures == 0)
    {
        (void)puts("All PWM safe-stop policy tests passed");
    }

    return failures == 0 ? 0 : 1;
}
