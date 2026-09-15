#include "cpg_gait_generator.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void set_raw_output(
    cpg_gait_generator_t *generator,
    double front_right,
    double front_left,
    double rear_right,
    double rear_left)
{
    generator->core.raw_output[0] = front_right;
    generator->core.raw_output[3] = front_left;
    generator->core.raw_output[1] = rear_right;
    generator->core.raw_output[2] = rear_left;
}

static void assert_targets(
    const joint_targets_t *targets,
    int32_t front_right,
    int32_t front_left,
    int32_t front_axis,
    int32_t rear_right,
    int32_t rear_left)
{
    assert(targets->front_right_cdeg == front_right);
    assert(targets->front_left_cdeg == front_left);
    assert(targets->front_axis_cdeg == front_axis);
    assert(targets->rear_right_cdeg == rear_right);
    assert(targets->rear_left_cdeg == rear_left);
}

static void assert_target_amplitudes(
    const cpg_gait_generator_t *generator,
    double node0,
    double node1,
    double node2,
    double node3)
{
    assert(generator->core.params.target_amplitude[0] == node0);
    assert(generator->core.params.target_amplitude[1] == node1);
    assert(generator->core.params.target_amplitude[2] == node2);
    assert(generator->core.params.target_amplitude[3] == node3);
}

static void test_production_profile_and_initialization(void)
{
    cpg_gait_generator_t generator;

    cpg_gait_generator_init(&generator);
    assert(generator.profile.front_amplitude_deg == 10.0);
    assert(generator.profile.rear_amplitude_deg == 10.0);
    assert(generator.profile.nominal_period_s == 2.0);
    assert(generator.profile.turn_reduced_side_scale == 0.5);
    assert(generator.profile.front_axis_bias_cdeg[MOTION_ASCEND] == 1000.0);
    assert(generator.profile.front_axis_bias_cdeg[MOTION_DESCEND] == -1000.0);
    assert(generator.core.params.period_s[0] == 2.0);
    assert(generator.core.params.period_s[1] == 2.0);
    assert(generator.core.params.period_s[2] == 2.0);
    assert(generator.core.params.period_s[3] == 2.0);
    assert(generator.core.params.target_amplitude[0] == -10.0);
    assert(generator.core.params.target_amplitude[1] == 10.0);
    assert(generator.core.params.target_amplitude[2] == 10.0);
    assert(generator.core.params.target_amplitude[3] == -10.0);
}

static void test_forward_mapping_and_phase_topology(void)
{
    cpg_gait_generator_t generator;
    gait_generator_t interface;
    joint_targets_t targets;

    cpg_gait_generator_init(&generator);
    interface = cpg_gait_generator_interface(&generator);
    assert(interface.ops != NULL);
    assert(interface.context == &generator);
    set_raw_output(&generator, -1.25, -1.25, 2.5, 2.5);

    assert(interface.ops->sample(
               interface.context,
               MOTION_FORWARD,
               1.0F,
               1.0F,
               &targets));
    assert_targets(&targets, -125, -125, 0, 250, 250);
}

static void test_turn_installs_target_amplitudes_without_output_jump(void)
{
    cpg_gait_generator_t generator;
    joint_targets_t targets;

    cpg_gait_generator_init(&generator);
    set_raw_output(&generator, -1.25, -1.25, 2.5, 2.5);

    assert(cpg_gait_generator_sample(
        &generator, MOTION_FORWARD, 1.0F, 1.0F, &targets));
    assert_targets(&targets, -125, -125, 0, 250, 250);
    assert_target_amplitudes(&generator, -10.0, 10.0, 10.0, -10.0);

    assert(cpg_gait_generator_sample(
        &generator, MOTION_TURN_LEFT, 1.0F, 1.0F, &targets));
    assert_targets(&targets, -125, -125, 0, 250, 250);
    assert_target_amplitudes(&generator, -10.0, 10.0, 5.0, -5.0);

    assert(cpg_gait_generator_sample(
        &generator, MOTION_TURN_RIGHT, 1.0F, 1.0F, &targets));
    assert_targets(&targets, -125, -125, 0, 250, 250);
    assert_target_amplitudes(&generator, -5.0, 5.0, 10.0, -10.0);

    assert(cpg_gait_generator_sample(
        &generator, MOTION_ASCEND, 1.0F, 1.0F, &targets));
    assert(targets.front_axis_cdeg == 1000);
    assert_target_amplitudes(&generator, -10.0, 10.0, 10.0, -10.0);
    assert(cpg_gait_generator_sample(
        &generator, MOTION_DESCEND, 1.0F, 1.0F, &targets));
    assert(targets.front_axis_cdeg == -1000);
    assert_target_amplitudes(&generator, -10.0, 10.0, 10.0, -10.0);
}

static void test_turn_target_drives_core_amplitude_dynamics(void)
{
    cpg_gait_generator_t generator;
    joint_targets_t targets;
    double initial_rear_distance;

    cpg_gait_generator_init(&generator);
    generator.core.amplitude[0] = -10.0;
    generator.core.amplitude[1] = 10.0;
    generator.core.amplitude[2] = 10.0;
    generator.core.amplitude[3] = -10.0;

    assert(cpg_gait_generator_sample(
        &generator, MOTION_TURN_LEFT, 1.0F, 1.0F, &targets));
    initial_rear_distance = fabs(
        generator.core.amplitude[2] -
        generator.core.params.target_amplitude[2]);

    cpg_gait_generator_advance(&generator, 100U);

    assert(generator.core.amplitude[2] < 10.0);
    assert(generator.core.amplitude[2] > 5.0);
    assert(generator.core.amplitude[3] > -10.0);
    assert(generator.core.amplitude[3] < -5.0);
    assert(fabs(
               generator.core.amplitude[2] -
               generator.core.params.target_amplitude[2]) <
           initial_rear_distance);
}

static void test_dynamic_production_profile_topology_and_envelope(void)
{
    cpg_gait_generator_t generator;
    joint_targets_t targets;
    double max_front_pair_error = 0.0;
    double max_rear_pair_error = 0.0;
    double max_front_rear_phase_error = 0.0;
    double max_front_rear_output_sum = 0.0;
    int32_t max_abs_target = 0;
    uint32_t step;

    cpg_gait_generator_init(&generator);
    for (step = 0U; step < 5000U; ++step)
    {
        cpg_gait_generator_advance(&generator, CPG_CORE_STEP_MS);
        assert(cpg_gait_generator_sample(
            &generator, MOTION_FORWARD, 1.0F, 1.0F, &targets));
        assert(targets.front_axis_cdeg == 0);
        assert(abs(targets.front_right_cdeg) <= 1500);
        assert(abs(targets.front_left_cdeg) <= 1500);
        assert(abs(targets.rear_right_cdeg) <= 1500);
        assert(abs(targets.rear_left_cdeg) <= 1500);

        if (abs(targets.front_right_cdeg) > max_abs_target)
        {
            max_abs_target = abs(targets.front_right_cdeg);
        }
        if (abs(targets.front_left_cdeg) > max_abs_target)
        {
            max_abs_target = abs(targets.front_left_cdeg);
        }
        if (abs(targets.rear_right_cdeg) > max_abs_target)
        {
            max_abs_target = abs(targets.rear_right_cdeg);
        }
        if (abs(targets.rear_left_cdeg) > max_abs_target)
        {
            max_abs_target = abs(targets.rear_left_cdeg);
        }

        if ((step >= 500U) &&
            (fabs(generator.core.raw_output[0] -
                  generator.core.raw_output[3]) > max_front_pair_error))
        {
            max_front_pair_error = fabs(
                generator.core.raw_output[0] -
                generator.core.raw_output[3]);
        }
        if ((step >= 500U) &&
            (fabs(generator.core.raw_output[1] -
                  generator.core.raw_output[2]) > max_rear_pair_error))
        {
            max_rear_pair_error = fabs(
                generator.core.raw_output[1] -
                generator.core.raw_output[2]);
        }
        if ((step >= 500U) &&
            (fabs(generator.core.raw_output[0] +
                  generator.core.raw_output[1]) > max_front_rear_output_sum))
        {
            max_front_rear_output_sum = fabs(
                generator.core.raw_output[0] +
                generator.core.raw_output[1]);
        }
        if ((step >= 500U) &&
            (fabs(generator.core.raw_output[3] +
                  generator.core.raw_output[2]) > max_front_rear_output_sum))
        {
            max_front_rear_output_sum = fabs(
                generator.core.raw_output[3] +
                generator.core.raw_output[2]);
        }
        if (step >= 500U)
        {
            const double phase_errors[] = {
                fabs(generator.core.phase[1] - generator.core.phase[0]),
                fabs(generator.core.phase[2] - generator.core.phase[0]),
                fabs(generator.core.phase[1] - generator.core.phase[3]),
                fabs(generator.core.phase[2] - generator.core.phase[3]),
            };

            for (size_t index = 0U; index < 4U; ++index)
            {
                if (phase_errors[index] > max_front_rear_phase_error)
                {
                    max_front_rear_phase_error = phase_errors[index];
                }
            }
        }
    }

    assert(max_front_pair_error <= 0.000000000001);
    assert(max_rear_pair_error <= 0.000000000001);
    assert(max_front_rear_phase_error <= 0.75);
    assert(max_abs_target <= 1500);
    (void)printf(
        "dynamic_profile max_front_pair_error=%.9f "
        "max_rear_pair_error=%.9f max_front_rear_phase_error=%.9f "
        "max_front_rear_output_sum=%.9f "
        "max_abs_target_cdeg=%d\n",
        max_front_pair_error,
        max_rear_pair_error,
        max_front_rear_phase_error,
        max_front_rear_output_sum,
        max_abs_target);
}

static void test_backward_is_disabled_and_stop_is_safe(void)
{
    cpg_gait_generator_t generator;
    joint_targets_t targets = {
        .front_right_cdeg = 11,
        .front_left_cdeg = 22,
        .front_axis_cdeg = 33,
        .rear_right_cdeg = 44,
        .rear_left_cdeg = 55,
    };

    cpg_gait_generator_init(&generator);
    set_raw_output(&generator, -1.0, -1.0, 1.0, 1.0);
    assert(!cpg_gait_generator_is_mode_valid(MOTION_BACKWARD));
    assert(!cpg_gait_generator_sample(
        &generator, MOTION_BACKWARD, 1.0F, 1.0F, &targets));
    assert_targets(&targets, 11, 22, 33, 44, 55);

    assert(cpg_gait_generator_sample(
        &generator, MOTION_STOP, 1.0F, 1.0F, &targets));
    assert_targets(&targets, 0, 0, 0, 0, 0);
}

static void test_adapter_does_not_apply_rear_clamp(void)
{
    cpg_gait_generator_t generator;
    joint_targets_t targets;

    cpg_gait_generator_init(&generator);
    set_raw_output(&generator, -1.0, -1.0, 100.0, 100.0);
    assert(cpg_gait_generator_sample(
        &generator, MOTION_FORWARD, 1.0F, 1.0F, &targets));
    assert(targets.rear_right_cdeg == 10000);
    assert(targets.rear_left_cdeg == 10000);
}

static void test_double_rounding_and_advance(void)
{
    cpg_gait_generator_t generator;
    joint_targets_t targets;

    cpg_gait_generator_init(&generator);
    set_raw_output(&generator, 0.005, -0.005, 0.015, -0.015);
    assert(cpg_gait_generator_sample(
        &generator, MOTION_FORWARD, 1.0F, 1.0F, &targets));
    assert(targets.front_right_cdeg == 1);
    assert(targets.front_left_cdeg == -1);
    assert(targets.rear_right_cdeg == 2);
    assert(targets.rear_left_cdeg == -2);

    cpg_gait_generator_advance(&generator, 20U);
    assert(cpg_core_executed_step_count(&generator.core) == 2U);
}

static void test_reset_matches_fresh_profile_after_turn(void)
{
    cpg_gait_profile_t profile;
    cpg_gait_generator_t fresh;
    cpg_gait_generator_t exercised;
    joint_targets_t targets;

    cpg_gait_profile_production_default(&profile);
    profile.nominal_period_s = 1.25;
    profile.turn_reduced_side_scale = 0.35;
    profile.front_axis_bias_cdeg[MOTION_TURN_LEFT] = 321.0;

    cpg_gait_generator_init_with_profile(&fresh, &profile);
    cpg_gait_generator_init_with_profile(&exercised, &profile);
    cpg_gait_generator_advance(&exercised, 37U);
    assert(cpg_gait_generator_sample(
        &exercised,
        MOTION_TURN_LEFT,
        1.0F,
        1.0F,
        &targets));
    cpg_gait_generator_advance(&exercised, 113U);

    cpg_gait_generator_reset(&exercised);

    assert(memcmp(&exercised.profile, &fresh.profile, sizeof(profile)) == 0);
    assert(memcmp(&exercised.core, &fresh.core,
                  sizeof(exercised.core)) == 0);
}

int main(void)
{
    test_production_profile_and_initialization();
    test_forward_mapping_and_phase_topology();
    test_turn_installs_target_amplitudes_without_output_jump();
    test_turn_target_drives_core_amplitude_dynamics();
    test_dynamic_production_profile_topology_and_envelope();
    test_backward_is_disabled_and_stop_is_safe();
    test_adapter_does_not_apply_rear_clamp();
    test_double_rounding_and_advance();
    test_reset_matches_fresh_profile_after_turn();
    return 0;
}
