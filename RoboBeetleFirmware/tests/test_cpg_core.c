#include "cpg_core.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define LEGACY_PI 3.14159265358979323846
#define LEGACY_TWO_PI 6.2831853071795862
#define LEGACY_STEP_S 0.01
#define LEGACY_OUTPUT_SCALE 100.0

static const double legacy_edge_source[CPG_CORE_EDGE_COUNT] = {
    0.0, 0.0, 0.0, 1.0, 1.0, 1.0,
    2.0, 2.0, 2.0, 3.0, 3.0, 3.0,
};

static const double legacy_edge_target[CPG_CORE_EDGE_COUNT] = {
    1.0, 2.0, 3.0, 0.0, 2.0, 3.0,
    0.0, 1.0, 3.0, 0.0, 1.0, 2.0,
};

static const double legacy_weight[CPG_CORE_EDGE_COUNT] = {
    2.0, 2.0, 0.0, 2.0, 0.0, 2.0,
    2.0, 0.0, 2.0, 0.0, 2.0, 2.0,
};

static const double legacy_phase_target_gain[CPG_CORE_EDGE_COUNT] = {
    20.0, 20.0, 0.0, 20.0, 0.0, 20.0,
    20.0, 0.0, 20.0, 0.0, 20.0, 20.0,
};

static const double legacy_target_amplitude[CPG_CORE_NODE_COUNT] = {
    -30.0, 30.0, 30.0, -30.0,
};

typedef struct
{
    double phase[CPG_CORE_NODE_COUNT];
    double phase_rate_memory[CPG_CORE_NODE_COUNT];
    double amplitude[CPG_CORE_NODE_COUNT];
    double amplitude_dot[CPG_CORE_NODE_COUNT];
    double amplitude_accel_memory[CPG_CORE_NODE_COUNT];
    double offset[CPG_CORE_NODE_COUNT];
    double offset_dot[CPG_CORE_NODE_COUNT];
    double offset_accel_memory[CPG_CORE_NODE_COUNT];
    double phase_target[CPG_CORE_EDGE_COUNT];
    double phase_target_dot[CPG_CORE_EDGE_COUNT];
    double phase_target_accel_memory[CPG_CORE_EDGE_COUNT];
    double output_memory[CPG_CORE_NODE_COUNT];
    double raw_output[CPG_CORE_NODE_COUNT];
    double unit_delay[CPG_CORE_NODE_COUNT];
    double theta_dot[CPG_CORE_NODE_COUNT];
} legacy_oracle_t;

static void assert_scalar_close(
    double actual,
    double expected,
    double tolerance)
{
    const double error = fabs(actual - expected);
    const double scale = 1.0 + fabs(expected);

    assert(error <= tolerance * scale);
}

static void assert_vector_close(
    const double *actual,
    const double *expected,
    size_t count,
    double tolerance)
{
    size_t i;

    for (i = 0U; i < count; ++i)
    {
        assert_scalar_close(actual[i], expected[i], tolerance);
    }
}

static void legacy_oracle_init(legacy_oracle_t *oracle)
{
    (void)memset(oracle, 0, sizeof(*oracle));
}

static double legacy_coupling_sum(
    const legacy_oracle_t *oracle,
    size_t node)
{
    double sum = 0.0;
    size_t edge;

    for (edge = 0U; edge < CPG_CORE_EDGE_COUNT; ++edge)
    {
        if ((size_t)legacy_edge_source[edge] == node)
        {
            const size_t target = (size_t)legacy_edge_target[edge];
            const double phase_difference =
                (oracle->phase[target] - oracle->phase[node]) -
                oracle->phase_target[edge];

            sum += sin(phase_difference) * legacy_weight[edge];
        }
    }

    return sum;
}

/*
 * This is an independent test-only transcription of the generated
 * CPG_RoboBeetle_stm.c schedule. In particular, the source Memory* values
 * are consumed one step before their newly computed derivative values.
 */
static void legacy_oracle_step(legacy_oracle_t *oracle)
{
    legacy_oracle_t next = *oracle;
    double theta_dot[CPG_CORE_NODE_COUNT];
    double phase_rate[CPG_CORE_NODE_COUNT];
    double amplitude_accel[CPG_CORE_NODE_COUNT];
    double offset_accel[CPG_CORE_NODE_COUNT];
    double phase_target_accel[CPG_CORE_EDGE_COUNT];
    double coupling_sum[CPG_CORE_NODE_COUNT];
    size_t i;
    size_t edge;

    for (i = 0U; i < CPG_CORE_NODE_COUNT; ++i)
    {
        next.raw_output[i] = oracle->output_memory[i];
        theta_dot[i] =
            (LEGACY_OUTPUT_SCALE * oracle->output_memory[i]) -
            oracle->unit_delay[i];
        coupling_sum[i] = legacy_coupling_sum(oracle, i);
    }

    for (i = 0U; i < CPG_CORE_NODE_COUNT; ++i)
    {
        const double beta = 0.75;
        const double velocity_gain = 1.0;
        const double period_s = 1.0;
        const double nu =
            (((2.0 * beta - 1.0) /
              (2.0 * beta * (1.0 - beta) *
               (exp(-(velocity_gain * theta_dot[i])) + 1.0)) +
              1.0 / (2.0 * beta)) /
             period_s * LEGACY_TWO_PI);

        phase_rate[i] = nu + coupling_sum[i];
        amplitude_accel[i] =
            ((legacy_target_amplitude[i] - oracle->amplitude[i]) * 5.0 -
             oracle->amplitude_dot[i]) * 20.0;
        offset_accel[i] =
            ((0.0 - oracle->offset[i]) * 5.0 -
             oracle->offset_dot[i]) * 20.0;
    }

    for (edge = 0U; edge < CPG_CORE_EDGE_COUNT; ++edge)
    {
        phase_target_accel[edge] =
            ((legacy_phase_target_gain[edge] / 4.0 *
              (0.0 - oracle->phase_target[edge]) -
              oracle->phase_target_dot[edge]) *
             legacy_phase_target_gain[edge]);
    }

    /*
     * The generated source writes Memory7, Memory11, Memory15, then
     * Memory19. All right-hand sides below use pre-Euler oscillator states.
     */
    next.output_memory[1] =
        oracle->amplitude[1] * sin(oracle->phase[1]) +
        oracle->offset[1];
    next.output_memory[2] =
        oracle->amplitude[2] * sin(oracle->phase[2]) +
        oracle->offset[2];
    next.output_memory[3] =
        oracle->amplitude[3] * sin(oracle->phase[3]) +
        oracle->offset[3];
    next.output_memory[0] =
        oracle->amplitude[0] * sin(oracle->phase[0]) +
        oracle->offset[0];

    for (i = 0U; i < CPG_CORE_NODE_COUNT; ++i)
    {
        next.unit_delay[i] =
            LEGACY_OUTPUT_SCALE * oracle->output_memory[i];
        next.theta_dot[i] = theta_dot[i];
        next.phase[i] =
            oracle->phase[i] +
            LEGACY_STEP_S * oracle->phase_rate_memory[i];
        next.amplitude[i] =
            oracle->amplitude[i] +
            LEGACY_STEP_S * oracle->amplitude_dot[i];
        next.amplitude_dot[i] =
            oracle->amplitude_dot[i] +
            LEGACY_STEP_S * oracle->amplitude_accel_memory[i];
        next.offset[i] =
            oracle->offset[i] +
            LEGACY_STEP_S * oracle->offset_dot[i];
        next.offset_dot[i] =
            oracle->offset_dot[i] +
            LEGACY_STEP_S * oracle->offset_accel_memory[i];
    }

    for (edge = 0U; edge < CPG_CORE_EDGE_COUNT; ++edge)
    {
        next.phase_target[edge] =
            oracle->phase_target[edge] +
            LEGACY_STEP_S * oracle->phase_target_dot[edge];
        next.phase_target_dot[edge] =
            oracle->phase_target_dot[edge] +
            LEGACY_STEP_S * oracle->phase_target_accel_memory[edge];
    }

    /*
     * These assignments correspond to the generated Memory* update block at
     * the bottom of the source step function.
     */
    for (i = 0U; i < CPG_CORE_NODE_COUNT; ++i)
    {
        next.phase_rate_memory[i] = phase_rate[i];
        next.amplitude_accel_memory[i] = amplitude_accel[i];
        next.offset_accel_memory[i] = offset_accel[i];
    }
    for (edge = 0U; edge < CPG_CORE_EDGE_COUNT; ++edge)
    {
        next.phase_target_accel_memory[edge] = phase_target_accel[edge];
    }

    *oracle = next;
}

static void assert_core_matches_oracle(
    const cpg_core_snapshot_t *actual,
    const legacy_oracle_t *expected)
{
    const double tolerance = 1.0e-11;

    assert_vector_close(actual->phase, expected->phase,
                        CPG_CORE_NODE_COUNT, tolerance);
    assert_vector_close(actual->phase_rate_memory,
                        expected->phase_rate_memory,
                        CPG_CORE_NODE_COUNT, tolerance);
    assert_vector_close(actual->amplitude, expected->amplitude,
                        CPG_CORE_NODE_COUNT, tolerance);
    assert_vector_close(actual->amplitude_dot,
                        expected->amplitude_dot,
                        CPG_CORE_NODE_COUNT, tolerance);
    assert_vector_close(actual->amplitude_accel_memory,
                        expected->amplitude_accel_memory,
                        CPG_CORE_NODE_COUNT, tolerance);
    assert_vector_close(actual->offset, expected->offset,
                        CPG_CORE_NODE_COUNT, tolerance);
    assert_vector_close(actual->offset_dot, expected->offset_dot,
                        CPG_CORE_NODE_COUNT, tolerance);
    assert_vector_close(actual->offset_accel_memory,
                        expected->offset_accel_memory,
                        CPG_CORE_NODE_COUNT, tolerance);
    assert_vector_close(actual->phase_target,
                        expected->phase_target,
                        CPG_CORE_EDGE_COUNT, tolerance);
    assert_vector_close(actual->phase_target_dot,
                        expected->phase_target_dot,
                        CPG_CORE_EDGE_COUNT, tolerance);
    assert_vector_close(actual->phase_target_accel_memory,
                        expected->phase_target_accel_memory,
                        CPG_CORE_EDGE_COUNT, tolerance);
    assert_vector_close(actual->output_memory,
                        expected->output_memory,
                        CPG_CORE_NODE_COUNT, tolerance);
    assert_vector_close(actual->raw_output, expected->raw_output,
                        CPG_CORE_NODE_COUNT, tolerance);
    assert_vector_close(actual->unit_delay, expected->unit_delay,
                        CPG_CORE_NODE_COUNT, tolerance);
    assert_vector_close(actual->theta_dot, expected->theta_dot,
                        CPG_CORE_NODE_COUNT, tolerance);
}

static void assert_source_golden(
    const cpg_core_snapshot_t *actual,
    unsigned int step)
{
    static const double zero_nodes[CPG_CORE_NODE_COUNT] = {
        0.0, 0.0, 0.0, 0.0,
    };
    static const double zero_edges[CPG_CORE_EDGE_COUNT] = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    };
    static const double step_five_raw[CPG_CORE_NODE_COUNT] = {
        -0.050030624014830674, 0.050030624014830674,
        0.050030624014830674, -0.050030624014830674,
    };
    static const double step_five_memory[CPG_CORE_NODE_COUNT] = {
        -0.22382089844836925, 0.22382089844836925,
        0.22382089844836925, -0.22382089844836925,
    };
    static const double step_five_theta_dot[CPG_CORE_NODE_COUNT] = {
        -5.0030624014830671, 5.0030624014830671,
        5.0030624014830671, -5.0030624014830671,
    };
    static const double step_five_phase[CPG_CORE_NODE_COUNT] = {
        0.33510321638291124, 0.33510321638291124,
        0.33510321638291124, 0.33510321638291124,
    };
    static const double step_five_phase_rate[CPG_CORE_NODE_COUNT] = {
        4.2446897996673805, 12.510471019478182,
        12.510471019478182, 4.2446897996673805,
    };
    static const double step_five_amplitude[CPG_CORE_NODE_COUNT] = {
        -1.7399999999999998, 1.7399999999999998,
        1.7399999999999998, -1.7399999999999998,
    };
    static const double step_five_amplitude_dot[CPG_CORE_NODE_COUNT] = {
        -101.7, 101.7, 101.7, -101.7,
    };
    static const double step_five_amplitude_accel[CPG_CORE_NODE_COUNT] = {
        -1230.0, 1230.0, 1230.0, -1230.0,
    };
    static const double step_ten_raw[CPG_CORE_NODE_COUNT] = {
        -2.2925215814060897, 3.3280500734141136,
        3.3280500734141136, -2.2925215814060897,
    };
    static const double step_ten_memory[CPG_CORE_NODE_COUNT] = {
        -3.1319726686436438, 4.6961042189305502,
        4.6961042189305502, -3.1319726686436438,
    };
    static const double step_ten_theta_dot[CPG_CORE_NODE_COUNT] = {
        -70.547904862531027, 117.32979922037242,
        117.32979922037242, -70.547904862531027,
    };
    static const double step_ten_unit_delay[CPG_CORE_NODE_COUNT] = {
        -229.25215814060897, 332.80500734141134,
        332.80500734141134, -229.25215814060897,
    };
    static const double step_ten_phase[CPG_CORE_NODE_COUNT] = {
        0.56467900928863124, 0.94328546443446926,
        0.94328546443446926, 0.56467900928863124,
    };
    static const double step_ten_phase_rate[CPG_CORE_NODE_COUNT] = {
        5.4247347442199194, 11.330426074925644,
        11.330426074925644, 5.4247347442199194,
    };
    static const double step_ten_amplitude[CPG_CORE_NODE_COUNT] = {
        -7.6606500000000004, 7.6606500000000004,
        7.6606500000000004, -7.6606500000000004,
    };
    static const double step_ten_amplitude_dot[CPG_CORE_NODE_COUNT] = {
        -127.69319999999999, 127.69319999999999,
        127.69319999999999, -127.69319999999999,
    };
    static const double step_ten_amplitude_accel[CPG_CORE_NODE_COUNT] = {
        199.62299999999971, -199.62299999999971,
        -199.62299999999971, 199.62299999999971,
    };

    assert_vector_close(actual->offset, zero_nodes,
                        CPG_CORE_NODE_COUNT, 0.0);
    assert_vector_close(actual->offset_dot, zero_nodes,
                        CPG_CORE_NODE_COUNT, 0.0);
    assert_vector_close(actual->offset_accel_memory, zero_nodes,
                        CPG_CORE_NODE_COUNT, 0.0);
    assert_vector_close(actual->phase_target, zero_edges,
                        CPG_CORE_EDGE_COUNT, 0.0);
    assert_vector_close(actual->phase_target_dot, zero_edges,
                        CPG_CORE_EDGE_COUNT, 0.0);
    assert_vector_close(actual->phase_target_accel_memory, zero_edges,
                        CPG_CORE_EDGE_COUNT, 0.0);

    if (step == 5U)
    {
        assert_vector_close(actual->raw_output, step_five_raw,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
        assert_vector_close(actual->output_memory, step_five_memory,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
        assert_vector_close(actual->theta_dot, step_five_theta_dot,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
        assert_vector_close(actual->unit_delay, step_five_theta_dot,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
        assert_vector_close(actual->phase, step_five_phase,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
        assert_vector_close(actual->phase_rate_memory, step_five_phase_rate,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
        assert_vector_close(actual->amplitude, step_five_amplitude,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
        assert_vector_close(actual->amplitude_dot, step_five_amplitude_dot,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
        assert_vector_close(actual->amplitude_accel_memory,
                            step_five_amplitude_accel,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
    }
    else
    {
        assert(step == 10U);
        assert_vector_close(actual->raw_output, step_ten_raw,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
        assert_vector_close(actual->output_memory, step_ten_memory,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
        assert_vector_close(actual->theta_dot, step_ten_theta_dot,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
        assert_vector_close(actual->unit_delay, step_ten_unit_delay,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
        assert_vector_close(actual->phase, step_ten_phase,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
        assert_vector_close(actual->phase_rate_memory, step_ten_phase_rate,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
        assert_vector_close(actual->amplitude, step_ten_amplitude,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
        assert_vector_close(actual->amplitude_dot, step_ten_amplitude_dot,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
        assert_vector_close(actual->amplitude_accel_memory,
                            step_ten_amplitude_accel,
                            CPG_CORE_NODE_COUNT, 1.0e-12);
    }
}

static void test_initial_state_and_source_order(void)
{
    cpg_core_t core;
    cpg_core_snapshot_t snapshot;
    legacy_oracle_t oracle;
    static const double zero_nodes[CPG_CORE_NODE_COUNT] = {
        0.0, 0.0, 0.0, 0.0,
    };
    static const double first_phase_rate[CPG_CORE_NODE_COUNT] = {
        8.3775804095727811, 8.3775804095727811,
        8.3775804095727811, 8.3775804095727811,
    };
    static const double first_accel[CPG_CORE_NODE_COUNT] = {
        -3000.0, 3000.0, 3000.0, -3000.0,
    };

    cpg_core_init(&core, NULL);
    legacy_oracle_init(&oracle);
    cpg_core_snapshot(&core, &snapshot);

    assert_vector_close(snapshot.theta_dot, zero_nodes,
                        CPG_CORE_NODE_COUNT, 0.0);
    assert_vector_close(snapshot.unit_delay, zero_nodes,
                        CPG_CORE_NODE_COUNT, 0.0);
    assert_vector_close(snapshot.output_memory, zero_nodes,
                        CPG_CORE_NODE_COUNT, 0.0);
    assert(snapshot.params.step_s == LEGACY_STEP_S);
    assert(snapshot.params.beta[0] == 0.75);
    assert(snapshot.params.period_s[0] == 1.0);
    assert(snapshot.params.target_amplitude[0] == -30.0);

    cpg_core_step(&core);
    legacy_oracle_step(&oracle);
    cpg_core_snapshot(&core, &snapshot);
    assert_core_matches_oracle(&snapshot, &oracle);
    assert(snapshot.executed_step_count == 1U);
    assert_vector_close(snapshot.phase, zero_nodes,
                        CPG_CORE_NODE_COUNT, 0.0);
    assert_vector_close(snapshot.phase_rate_memory, first_phase_rate,
                        CPG_CORE_NODE_COUNT, 1.0e-12);
    assert_vector_close(snapshot.amplitude_dot, zero_nodes,
                        CPG_CORE_NODE_COUNT, 0.0);
    assert_vector_close(snapshot.amplitude_accel_memory, first_accel,
                        CPG_CORE_NODE_COUNT, 0.0);
}

static void test_literal_golden_vectors_and_theta_dot(void)
{
    cpg_core_t core;
    cpg_core_snapshot_t snapshot;
    legacy_oracle_t oracle;
    unsigned int step;

    cpg_core_init(&core, NULL);
    legacy_oracle_init(&oracle);

    for (step = 1U; step <= 10U; ++step)
    {
        cpg_core_step(&core);
        legacy_oracle_step(&oracle);
        cpg_core_snapshot(&core, &snapshot);
        assert_core_matches_oracle(&snapshot, &oracle);
        if ((step == 5U) || (step == 10U))
        {
            assert_source_golden(&snapshot, step);
        }
    }

    assert(fabs(snapshot.theta_dot[0]) > 1.0);
}

static void test_phase_target_memory_is_one_step_delayed(void)
{
    cpg_core_t core;
    cpg_core_snapshot_t snapshot;
    legacy_oracle_t oracle;

    cpg_core_init(&core, NULL);
    legacy_oracle_init(&oracle);
    core.phase_target[0] = 0.5;
    core.phase_target_dot[0] = 0.25;
    core.phase_target_accel_memory[0] = 1.0;
    oracle.phase_target[0] = 0.5;
    oracle.phase_target_dot[0] = 0.25;
    oracle.phase_target_accel_memory[0] = 1.0;

    cpg_core_step(&core);
    legacy_oracle_step(&oracle);
    cpg_core_snapshot(&core, &snapshot);
    assert_core_matches_oracle(&snapshot, &oracle);
    assert_scalar_close(snapshot.phase_target[0], 0.5025, 0.0);
    assert_scalar_close(snapshot.phase_target_dot[0], 0.26, 0.0);
    assert_scalar_close(snapshot.phase_target_accel_memory[0], -55.0, 0.0);
}

static void test_elapsed_catch_up_is_bounded(void)
{
    cpg_core_t core;

    cpg_core_init(&core, NULL);
    assert(cpg_core_advance_elapsed_ms(&core, 9U) == 0U);
    assert(core.elapsed_remainder_ms == 9U);
    assert(cpg_core_advance_elapsed_ms(&core, 1U) == 1U);
    assert(core.elapsed_remainder_ms == 0U);
    assert(cpg_core_executed_step_count(&core) == 1U);

    assert(cpg_core_advance_elapsed_ms(&core, 700U) == 70U);
    assert(cpg_core_executed_step_count(&core) == 71U);
    assert(cpg_core_discarded_catch_up_count(&core) == 0U);

    assert(cpg_core_advance_elapsed_ms(&core, 5U) == 0U);
    assert(core.elapsed_remainder_ms == 5U);
    assert(cpg_core_advance_elapsed_ms(&core, 1095U) ==
           CPG_CORE_MAX_CATCH_UP_STEPS);
    assert(core.executed_step_count <=
           CPG_CORE_MAX_CATCH_UP_STEPS + 71U);
    assert(cpg_core_discarded_catch_up_count(&core) == 10U);
    assert(core.elapsed_remainder_ms == 0U);
}

int main(void)
{
    test_initial_state_and_source_order();
    test_literal_golden_vectors_and_theta_dot();
    test_phase_target_memory_is_one_step_delayed();
    test_elapsed_catch_up_is_bounded();
    return 0;
}
