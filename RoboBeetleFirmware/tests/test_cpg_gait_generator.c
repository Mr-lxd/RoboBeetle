#include "cpg_gait_generator.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>

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

static void test_turn_and_front_axis_profiles(void)
{
    cpg_gait_generator_t generator;
    joint_targets_t targets;

    cpg_gait_generator_init(&generator);
    set_raw_output(&generator, -1.25, -1.25, 2.5, 2.5);

    assert(cpg_gait_generator_sample(
        &generator, MOTION_TURN_LEFT, 1.0F, 1.0F, &targets));
    assert_targets(&targets, -125, -63, 0, 250, 125);

    assert(cpg_gait_generator_sample(
        &generator, MOTION_TURN_RIGHT, 1.0F, 1.0F, &targets));
    assert_targets(&targets, -63, -125, 0, 125, 250);

    assert(cpg_gait_generator_sample(
        &generator, MOTION_ASCEND, 1.0F, 1.0F, &targets));
    assert(targets.front_axis_cdeg == 1000);
    assert(cpg_gait_generator_sample(
        &generator, MOTION_DESCEND, 1.0F, 1.0F, &targets));
    assert(targets.front_axis_cdeg == -1000);
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

int main(void)
{
    test_production_profile_and_initialization();
    test_forward_mapping_and_phase_topology();
    test_turn_and_front_axis_profiles();
    test_backward_is_disabled_and_stop_is_safe();
    test_adapter_does_not_apply_rear_clamp();
    test_double_rounding_and_advance();
    return 0;
}
