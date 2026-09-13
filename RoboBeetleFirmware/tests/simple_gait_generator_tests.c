#include "motion_config.h"
#include "motion_types.h"
#include "simple_gait_generator.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void expect(bool condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

static bool targets_equal(
    const joint_targets_t *left,
    const joint_targets_t *right)
{
    return left->front_right_cdeg == right->front_right_cdeg &&
           left->front_left_cdeg == right->front_left_cdeg &&
           left->front_axis_cdeg == right->front_axis_cdeg &&
           left->rear_right_cdeg == right->rear_right_cdeg &&
           left->rear_left_cdeg == right->rear_left_cdeg;
}

static void expect_targets_zero(const joint_targets_t *targets)
{
    expect(targets->front_right_cdeg == 0,
           "FrontRight STOP target should be neutral");
    expect(targets->front_left_cdeg == 0,
           "FrontLeft STOP target should be neutral");
    expect(targets->front_axis_cdeg == 0,
           "FrontAxis STOP target should be neutral");
    expect(targets->rear_right_cdeg == 0,
           "RearRight STOP target should be neutral");
    expect(targets->rear_left_cdeg == 0,
           "RearLeft STOP target should be neutral");
}

static void test_modes_and_stop(void)
{
    simple_gait_generator_t generator;
    joint_targets_t targets;

    simple_gait_generator_init(&generator);

    for (int mode = MOTION_STOP; mode < MOTION_COUNT; ++mode)
    {
        expect(simple_gait_generator_is_mode_valid(
                   (motion_mode_t)mode),
               "every documented Motion mode should be valid");
    }

    (void)memset(&targets, 0xA5, sizeof(targets));
    expect(simple_gait_generator_sample(
               &generator,
               MOTION_STOP,
               1.0F,
               1.0F,
               &targets),
           "STOP should be a valid generator sample");
    expect_targets_zero(&targets);
}

static void test_forward_phase_relation_and_determinism(void)
{
    simple_gait_generator_t generator;
    joint_targets_t first;
    joint_targets_t second;

    simple_gait_generator_init(&generator);
    simple_gait_generator_advance(&generator, 500U);
    expect(simple_gait_generator_sample(
               &generator,
               MOTION_FORWARD,
               1.0F,
               1.0F,
               &first),
           "FORWARD should sample successfully");
    expect(first.front_right_cdeg == MOTION_PROFILE_PADDLE_AMPLITUDE_CDEG &&
               first.front_left_cdeg ==
                   MOTION_PROFILE_PADDLE_AMPLITUDE_CDEG,
           "FORWARD front paddles should share the positive logical stroke");
    expect(first.rear_right_cdeg == -MOTION_PROFILE_PADDLE_AMPLITUDE_CDEG &&
               first.rear_left_cdeg ==
                   -MOTION_PROFILE_PADDLE_AMPLITUDE_CDEG,
           "FORWARD rear paddles should be pi out of phase");

    expect(simple_gait_generator_sample(
               &generator,
               MOTION_FORWARD,
               1.0F,
               1.0F,
               &second),
           "repeated FORWARD sample should succeed");
    expect(targets_equal(&first, &second),
           "same phase and profile should produce deterministic targets");
}

static void test_backward_is_logical_stroke_inversion(void)
{
    simple_gait_generator_t generator;
    joint_targets_t forward;
    joint_targets_t backward;

    simple_gait_generator_init(&generator);
    simple_gait_generator_advance(&generator, 500U);
    (void)simple_gait_generator_sample(
        &generator,
        MOTION_FORWARD,
        1.0F,
        1.0F,
        &forward);
    (void)simple_gait_generator_sample(
        &generator,
        MOTION_BACKWARD,
        1.0F,
        1.0F,
        &backward);

    expect(backward.front_right_cdeg == -forward.front_right_cdeg &&
               backward.front_left_cdeg == -forward.front_left_cdeg &&
               backward.rear_right_cdeg == -forward.rear_right_cdeg &&
               backward.rear_left_cdeg == -forward.rear_left_cdeg,
           "BACKWARD should invert the logical paddle stroke candidate");
}

static void test_turn_scales_and_axis_bias(void)
{
    simple_gait_generator_t generator;
    joint_targets_t turn_left;
    joint_targets_t turn_right;
    joint_targets_t ascend;
    joint_targets_t descend;

    simple_gait_generator_init(&generator);
    simple_gait_generator_advance(&generator, 500U);
    (void)simple_gait_generator_sample(
        &generator,
        MOTION_TURN_LEFT,
        1.0F,
        1.0F,
        &turn_left);
    (void)simple_gait_generator_sample(
        &generator,
        MOTION_TURN_RIGHT,
        1.0F,
        1.0F,
        &turn_right);

    expect(turn_left.front_right_cdeg ==
               MOTION_PROFILE_PADDLE_AMPLITUDE_CDEG &&
               turn_left.front_left_cdeg ==
                   (MOTION_PROFILE_PADDLE_AMPLITUDE_CDEG / 2),
           "TURN_LEFT should reduce the left side to 50 percent");
    expect(turn_right.front_right_cdeg ==
               (MOTION_PROFILE_PADDLE_AMPLITUDE_CDEG / 2) &&
               turn_right.front_left_cdeg ==
                   MOTION_PROFILE_PADDLE_AMPLITUDE_CDEG,
           "TURN_RIGHT should reduce the right side to 50 percent");

    (void)simple_gait_generator_sample(
        &generator,
        MOTION_ASCEND,
        1.0F,
        1.0F,
        &ascend);
    (void)simple_gait_generator_sample(
        &generator,
        MOTION_DESCEND,
        1.0F,
        1.0F,
        &descend);
    expect(ascend.front_axis_cdeg ==
               MOTION_PROFILE_ASCEND_FRONT_AXIS_BIAS_CDEG,
           "ASCEND should use the provisional positive FrontAxis bias");
    expect(descend.front_axis_cdeg ==
               MOTION_PROFILE_DESCEND_FRONT_AXIS_BIAS_CDEG,
           "DESCEND should use the provisional negative FrontAxis bias");
}

static void test_phase_step(void)
{
    simple_gait_generator_t generator;
    const float expected_phase =
        2.0F * MOTION_PI_F * MOTION_PROFILE_FREQUENCY_HZ * 0.010F;

    simple_gait_generator_init(&generator);
    simple_gait_generator_advance(&generator, MOTION_GAIT_TICK_MS);
    expect(fabsf(simple_gait_generator_phase(&generator) - expected_phase) <
               0.000001F,
           "10 ms should advance the deterministic 0.5 Hz phase exactly");
}

static void test_operational_rear_clamp_and_diagnostics(void)
{
    joint_targets_t targets = {
        .front_right_cdeg = 0,
        .front_left_cdeg = 0,
        .front_axis_cdeg = 0,
        .rear_right_cdeg = -3001,
        .rear_left_cdeg = 4501,
    };
    uint32_t clamp_count = 0U;

    simple_gait_generator_clamp_targets(&targets, &clamp_count);
    expect(targets.rear_right_cdeg == MOTION_REAR_MIN_CDEG,
           "RearRight below the operational minimum should clamp");
    expect(targets.rear_left_cdeg == MOTION_REAR_MAX_CDEG,
           "RearLeft above the operational maximum should clamp");
    expect(clamp_count == 2U,
           "both rear boundary violations should be diagnosed");

    targets.rear_right_cdeg = MOTION_REAR_MIN_CDEG;
    targets.rear_left_cdeg = MOTION_REAR_MAX_CDEG;
    simple_gait_generator_clamp_targets(&targets, &clamp_count);
    expect(clamp_count == 2U,
           "valid rear operational boundaries should not increment diagnostics");
}

static void test_sample_reports_a_clamp(void)
{
    simple_gait_generator_t generator;
    joint_targets_t targets;

    simple_gait_generator_init(&generator);
    simple_gait_generator_advance(&generator, 500U);
    expect(simple_gait_generator_sample(
               &generator,
               MOTION_FORWARD,
               5.0F,
               1.0F,
               &targets),
           "large deterministic sample should still return targets");
    expect(simple_gait_generator_operational_clamp_count(&generator) > 0U,
           "sampled rear clamp should be visible in diagnostics");
}

int main(void)
{
    test_modes_and_stop();
    test_forward_phase_relation_and_determinism();
    test_backward_is_logical_stroke_inversion();
    test_turn_scales_and_axis_bias();
    test_phase_step();
    test_operational_rear_clamp_and_diagnostics();
    test_sample_reports_a_clamp();

    if (failures == 0)
    {
        (void)puts("All firmware SimpleGaitGenerator tests passed");
    }

    return failures == 0 ? 0 : 1;
}
