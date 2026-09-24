#include "experimental_flex_gait_generator.h"

#include <stdbool.h>
#include <stddef.h>
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

static joint_targets_t sample(
    experimental_flex_gait_generator_t *generator,
    motion_mode_t mode)
{
    joint_targets_t targets = {0};
    expect(experimental_flex_gait_generator_sample(
               generator, mode, 1.0F, 1.0F, &targets),
           "supported mode should sample successfully");
    return targets;
}

static void expect_flex_mapping(const joint_targets_t *targets)
{
    expect(targets->front_right_cdeg == targets->front_left_cdeg,
           "both raw Front targets should equal q");
    expect(targets->rear_right_cdeg == -targets->front_right_cdeg &&
               targets->rear_left_cdeg == -targets->front_right_cdeg,
           "both raw Rear targets should equal -q");
    expect(targets->front_right_cdeg >= -1000 &&
               targets->front_right_cdeg <= 1000,
           "Flex target should stay within +/-1000 cdeg");
}

static void test_waveform_reset_and_segment_boundaries(void)
{
    experimental_flex_gait_generator_t generator;
    joint_targets_t targets;

    experimental_flex_gait_generator_init(&generator);
    targets = sample(&generator, MOTION_FORWARD);
    expect(targets.front_right_cdeg == 0,
           "reset phase should sample the power-stroke midpoint q=0");
    expect_flex_mapping(&targets);

    experimental_flex_gait_generator_advance(&generator, 1350U);
    targets = sample(&generator, MOTION_FORWARD);
    expect(targets.front_right_cdeg == -1000,
           "1350 ms after reset should wrap to power-stroke start");

    experimental_flex_gait_generator_advance(&generator, 1299U);
    const joint_targets_t before_boundary = sample(&generator, MOTION_FORWARD);
    experimental_flex_gait_generator_advance(&generator, 1U);
    const joint_targets_t at_boundary = sample(&generator, MOTION_FORWARD);
    expect(before_boundary.front_right_cdeg == 1000 &&
               at_boundary.front_right_cdeg == 1000,
           "power and recovery half-cosines should meet continuously at +1000");

    experimental_flex_gait_generator_advance(&generator, 700U);
    targets = sample(&generator, MOTION_FORWARD);
    expect(targets.front_right_cdeg == -1000,
           "recovery endpoint should return to -1000 cdeg");
    expect_flex_mapping(&targets);
}

static void test_modes_stop_axis_and_backward(void)
{
    experimental_flex_gait_generator_t generator;
    joint_targets_t targets;

    experimental_flex_gait_generator_init(&generator);
    targets = sample(&generator, MOTION_STOP);
    expect(targets.front_right_cdeg == 0 && targets.front_left_cdeg == 0 &&
               targets.front_axis_cdeg == 0 &&
               targets.rear_right_cdeg == 0 && targets.rear_left_cdeg == 0,
           "STOP should emit five neutral targets");

    experimental_flex_gait_generator_advance(&generator, 1350U);
    const motion_mode_t supported_modes[] = {
        MOTION_FORWARD,
        MOTION_TURN_LEFT,
        MOTION_TURN_RIGHT,
    };
    for (size_t index = 0U;
         index < sizeof(supported_modes) / sizeof(supported_modes[0]);
         ++index)
    {
        targets = sample(&generator, supported_modes[index]);
        expect_flex_mapping(&targets);
    }

    targets = sample(&generator, MOTION_ASCEND);
    expect(targets.front_axis_cdeg == 1000,
           "ASCEND should use +1000 cdeg FrontAxis bias");
    targets = sample(&generator, MOTION_DESCEND);
    expect(targets.front_axis_cdeg == -1000,
           "DESCEND should use -1000 cdeg FrontAxis bias");

    expect(!experimental_flex_gait_generator_is_mode_valid(MOTION_BACKWARD),
           "BACKWARD should remain unsupported by Experimental Flex");
    expect(!experimental_flex_gait_generator_sample(
               &generator, MOTION_BACKWARD, 1.0F, 1.0F, &targets),
           "BACKWARD should be rejected without a target");
}

int main(void)
{
    test_waveform_reset_and_segment_boundaries();
    test_modes_stop_axis_and_backward();

    if (failures == 0)
    {
        (void)puts("All ExperimentalFlexGaitGenerator tests passed");
    }

    return failures == 0 ? 0 : 1;
}
