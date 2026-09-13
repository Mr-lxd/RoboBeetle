#include "cpg_core.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define CPG_SOURCE_OUTPUT_SCALE 100.0
#define CPG_SOURCE_PI 3.14159265358979323846

static const uint8_t edge_source[CPG_CORE_EDGE_COUNT] = {
    0U, 0U, 0U, 1U, 1U, 1U,
    2U, 2U, 2U, 3U, 3U, 3U,
};

static const uint8_t edge_target[CPG_CORE_EDGE_COUNT] = {
    1U, 2U, 3U, 0U, 2U, 3U,
    0U, 1U, 3U, 0U, 1U, 2U,
};

static const double source_phase_target_gain[CPG_CORE_EDGE_COUNT] = {
    20.0, 20.0, 0.0, 20.0, 0.0, 20.0,
    20.0, 0.0, 20.0, 0.0, 20.0, 20.0,
};

static const double source_coupling_weight[CPG_CORE_EDGE_COUNT] = {
    2.0, 2.0, 0.0, 2.0, 0.0, 2.0,
    2.0, 0.0, 2.0, 0.0, 2.0, 2.0,
};

static const double source_target_amplitude[CPG_CORE_NODE_COUNT] = {
    -30.0, 30.0, 30.0, -30.0,
};

static int cpg_valid_core(const cpg_core_t *core)
{
    return core != NULL;
}

void cpg_legacy_source_compatible_default_params(
    cpg_model_params_t *params)
{
    size_t i;

    if (params == NULL)
    {
        return;
    }

    (void)memset(params, 0, sizeof(*params));
    params->step_s = 0.01;

    for (i = 0U; i < CPG_CORE_NODE_COUNT; ++i)
    {
        params->beta[i] = 0.75;
        params->period_s[i] = 1.0;
        params->velocity_gain[i] = 1.0;
        params->amplitude_gain[i] = 20.0;
        params->offset_gain[i] = 20.0;
        params->target_amplitude[i] = source_target_amplitude[i];
        params->target_offset[i] = 0.0;
    }

    for (i = 0U; i < CPG_CORE_EDGE_COUNT; ++i)
    {
        params->phase_target_gain[i] = source_phase_target_gain[i];
        params->coupling_weight[i] = source_coupling_weight[i];
        params->desired_phase[i] = 0.0;
    }
}

void cpg_core_init(
    cpg_core_t *core,
    const cpg_model_params_t *params)
{
    cpg_model_params_t default_params;

    if (core == NULL)
    {
        return;
    }

    (void)memset(core, 0, sizeof(*core));
    if (params != NULL)
    {
        core->params = *params;
    }
    else
    {
        cpg_legacy_source_compatible_default_params(&default_params);
        core->params = default_params;
    }
}

void cpg_core_reset(cpg_core_t *core)
{
    cpg_model_params_t params;

    if (core == NULL)
    {
        return;
    }

    params = core->params;
    (void)memset(core, 0, sizeof(*core));
    core->params = params;
}

void cpg_core_set_target_amplitudes(
    cpg_core_t *core,
    const double target_amplitude[CPG_CORE_NODE_COUNT])
{
    size_t i;

    if ((core == NULL) || (target_amplitude == NULL))
    {
        return;
    }

    for (i = 0U; i < CPG_CORE_NODE_COUNT; ++i)
    {
        core->params.target_amplitude[i] = target_amplitude[i];
    }
}

void cpg_core_set_periods(
    cpg_core_t *core,
    const double period_s[CPG_CORE_NODE_COUNT])
{
    size_t i;

    if ((core == NULL) || (period_s == NULL))
    {
        return;
    }

    for (i = 0U; i < CPG_CORE_NODE_COUNT; ++i)
    {
        core->params.period_s[i] = period_s[i];
    }
}

void cpg_core_step(cpg_core_t *core)
{
    cpg_core_t next;
    double theta_dot[CPG_CORE_NODE_COUNT];
    double phase_rate[CPG_CORE_NODE_COUNT];
    double amplitude_accel[CPG_CORE_NODE_COUNT];
    double offset_accel[CPG_CORE_NODE_COUNT];
    double phase_target_accel[CPG_CORE_EDGE_COUNT];
    double coupling_sum[CPG_CORE_NODE_COUNT];
    size_t i;
    size_t edge;

    if (!cpg_valid_core(core))
    {
        return;
    }

    next = *core;

    /*
     * This is the generated source's first read phase. raw_output is the
     * delayed Memory* value exported by the source step, not a newly
     * evaluated post-Euler oscillator expression.
     */
    for (i = 0U; i < CPG_CORE_NODE_COUNT; ++i)
    {
        next.raw_output[i] = core->output_memory[i];
        theta_dot[i] =
            (CPG_SOURCE_OUTPUT_SCALE * core->output_memory[i]) -
            core->unit_delay[i];
        next.theta_dot[i] = theta_dot[i];
        coupling_sum[i] = 0.0;
    }

    /*
     * Source edge order is three outgoing edges per node. The phase target
     * state is one state per directed edge, not one state per oscillator.
     */
    for (edge = 0U; edge < CPG_CORE_EDGE_COUNT; ++edge)
    {
        const size_t source = edge_source[edge];
        const size_t target = edge_target[edge];
        const double phase_difference =
            (core->phase[target] - core->phase[source]) -
            core->phase_target[edge];

        coupling_sum[source] +=
            sin(phase_difference) * core->params.coupling_weight[edge];
    }

    /*
     * All derivative expressions below consume only the state at step entry.
     * In particular, nu_i consumes the pre-Euler theta_dot_i expression.
     */
    for (i = 0U; i < CPG_CORE_NODE_COUNT; ++i)
    {
        const double beta = core->params.beta[i];
        const double denominator =
            2.0 * beta * (1.0 - beta) *
            (exp(-(core->params.velocity_gain[i] * theta_dot[i])) + 1.0);
        const double nu =
            (((2.0 * beta - 1.0) / denominator) +
             1.0 / (2.0 * beta)) /
            core->params.period_s[i] *
            2.0 * CPG_SOURCE_PI;

        phase_rate[i] = nu + coupling_sum[i];
        amplitude_accel[i] =
            ((core->params.target_amplitude[i] -
              core->amplitude[i]) *
             (core->params.amplitude_gain[i] / 4.0) -
             core->amplitude_dot[i]) *
            core->params.amplitude_gain[i];
        offset_accel[i] =
            ((core->params.target_offset[i] -
              core->offset[i]) *
             (core->params.offset_gain[i] / 4.0) -
             core->offset_dot[i]) *
            core->params.offset_gain[i];
    }

    for (edge = 0U; edge < CPG_CORE_EDGE_COUNT; ++edge)
    {
        phase_target_accel[edge] =
            ((core->params.phase_target_gain[edge] / 4.0 *
              (core->params.desired_phase[edge] -
               core->phase_target[edge]) -
              core->phase_target_dot[edge]) *
             core->params.phase_target_gain[edge]);
    }

    /*
     * Generated source output-memory update order is Memory7, Memory11,
     * Memory15, then Memory19. Each right-hand side is pre-Euler.
     */
    next.output_memory[1] =
        core->amplitude[1] * sin(core->phase[1]) +
        core->offset[1];
    next.output_memory[2] =
        core->amplitude[2] * sin(core->phase[2]) +
        core->offset[2];
    next.output_memory[3] =
        core->amplitude[3] * sin(core->phase[3]) +
        core->offset[3];
    next.output_memory[0] =
        core->amplitude[0] * sin(core->phase[0]) +
        core->offset[0];

    /*
     * These assignments are the source node 0 -> node 3 interleaved
     * UnitDelay and Euler writes. Derivative memories are intentionally read
     * from core and written only in the final memory-update block below.
     */
    for (i = 0U; i < CPG_CORE_NODE_COUNT; ++i)
    {
        next.unit_delay[i] =
            CPG_SOURCE_OUTPUT_SCALE * core->output_memory[i];
        next.phase[i] =
            core->phase[i] +
            core->params.step_s * core->phase_rate_memory[i];
        next.amplitude[i] =
            core->amplitude[i] +
            core->params.step_s * core->amplitude_dot[i];
        next.amplitude_dot[i] =
            core->amplitude_dot[i] +
            core->params.step_s * core->amplitude_accel_memory[i];
        next.offset[i] =
            core->offset[i] +
            core->params.step_s * core->offset_dot[i];
        next.offset_dot[i] =
            core->offset_dot[i] +
            core->params.step_s * core->offset_accel_memory[i];
    }

    for (edge = 0U; edge < CPG_CORE_EDGE_COUNT; ++edge)
    {
        next.phase_target[edge] =
            core->phase_target[edge] +
            core->params.step_s * core->phase_target_dot[edge];
        next.phase_target_dot[edge] =
            core->phase_target_dot[edge] +
            core->params.step_s *
            core->phase_target_accel_memory[edge];
    }

    /*
     * Generated source Memory* blocks receive the current derivative values
     * after all state updates. This preserves the one-step source delay.
     */
    for (i = 0U; i < CPG_CORE_NODE_COUNT; ++i)
    {
        next.phase_rate_memory[i] = phase_rate[i];
        next.amplitude_accel_memory[i] = amplitude_accel[i];
        next.offset_accel_memory[i] = offset_accel[i];
    }
    for (edge = 0U; edge < CPG_CORE_EDGE_COUNT; ++edge)
    {
        next.phase_target_accel_memory[edge] =
            phase_target_accel[edge];
    }

    next.executed_step_count = core->executed_step_count + 1U;
    *core = next;
}

uint32_t cpg_core_advance_elapsed_ms(
    cpg_core_t *core,
    uint32_t elapsed_ms)
{
    uint64_t available_steps;
    uint32_t steps_to_execute;
    uint32_t discarded_steps;
    uint32_t step;

    if (core == NULL)
    {
        return 0U;
    }

    core->elapsed_remainder_ms += (uint64_t)elapsed_ms;
    available_steps =
        core->elapsed_remainder_ms / (uint64_t)CPG_CORE_STEP_MS;
    core->elapsed_remainder_ms %=
        (uint64_t)CPG_CORE_STEP_MS;

    if (available_steps > (uint64_t)CPG_CORE_MAX_CATCH_UP_STEPS)
    {
        discarded_steps = (uint32_t)(
            available_steps - (uint64_t)CPG_CORE_MAX_CATCH_UP_STEPS);
        core->discarded_catch_up_count += discarded_steps;
        steps_to_execute = CPG_CORE_MAX_CATCH_UP_STEPS;
    }
    else
    {
        steps_to_execute = (uint32_t)available_steps;
    }

    for (step = 0U; step < steps_to_execute; ++step)
    {
        cpg_core_step(core);
    }

    return steps_to_execute;
}

void cpg_core_snapshot(
    const cpg_core_t *core,
    cpg_core_snapshot_t *snapshot)
{
    if ((core == NULL) || (snapshot == NULL))
    {
        return;
    }

    *snapshot = *core;
}

uint32_t cpg_core_executed_step_count(
    const cpg_core_t *core)
{
    return core == NULL ? 0U : core->executed_step_count;
}

uint32_t cpg_core_discarded_catch_up_count(
    const cpg_core_t *core)
{
    return core == NULL ? 0U : core->discarded_catch_up_count;
}
