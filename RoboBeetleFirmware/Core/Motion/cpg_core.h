#ifndef ROBOBEETLE_CPG_CORE_H
#define ROBOBEETLE_CPG_CORE_H

#include <stdint.h>

#define CPG_CORE_NODE_COUNT 4U
#define CPG_CORE_EDGE_COUNT 12U
#define CPG_CORE_STEP_MS 10U
#define CPG_CORE_MAX_CATCH_UP_STEPS 100U

typedef struct
{
    double beta[CPG_CORE_NODE_COUNT];
    double period_s[CPG_CORE_NODE_COUNT];
    double velocity_gain[CPG_CORE_NODE_COUNT];
    double amplitude_gain[CPG_CORE_NODE_COUNT];
    double offset_gain[CPG_CORE_NODE_COUNT];
    double phase_target_gain[CPG_CORE_EDGE_COUNT];
    double coupling_weight[CPG_CORE_EDGE_COUNT];
    double desired_phase[CPG_CORE_EDGE_COUNT];
    double target_amplitude[CPG_CORE_NODE_COUNT];
    double target_offset[CPG_CORE_NODE_COUNT];
    double step_s;
} cpg_model_params_t;

typedef struct
{
    cpg_model_params_t params;
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
    uint64_t elapsed_remainder_ms;
    uint32_t executed_step_count;
    uint32_t discarded_catch_up_count;
} cpg_core_t;

typedef cpg_core_t cpg_core_snapshot_t;

void cpg_legacy_source_compatible_default_params(
    cpg_model_params_t *params);

void cpg_core_init(
    cpg_core_t *core,
    const cpg_model_params_t *params);

void cpg_core_reset(
    cpg_core_t *core);

void cpg_core_set_target_amplitudes(
    cpg_core_t *core,
    const double target_amplitude[CPG_CORE_NODE_COUNT]);

void cpg_core_set_periods(
    cpg_core_t *core,
    const double period_s[CPG_CORE_NODE_COUNT]);

void cpg_core_step(
    cpg_core_t *core);

uint32_t cpg_core_advance_elapsed_ms(
    cpg_core_t *core,
    uint32_t elapsed_ms);

void cpg_core_snapshot(
    const cpg_core_t *core,
    cpg_core_snapshot_t *snapshot);

uint32_t cpg_core_executed_step_count(
    const cpg_core_t *core);

uint32_t cpg_core_discarded_catch_up_count(
    const cpg_core_t *core);

#endif /* ROBOBEETLE_CPG_CORE_H */
