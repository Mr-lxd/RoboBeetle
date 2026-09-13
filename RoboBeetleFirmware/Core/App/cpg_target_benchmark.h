#ifndef ROBOBEETLE_CPG_TARGET_BENCHMARK_H
#define ROBOBEETLE_CPG_TARGET_BENCHMARK_H

#include "cpg_gait_generator.h"

#include <stdint.h>

typedef struct
{
    uint32_t enabled;
    uint32_t system_core_clock_hz;
    uint32_t repetitions;
    uint32_t nominal_min_cycles;
    uint32_t nominal_median_cycles;
    uint32_t nominal_max_cycles;
    uint32_t nominal_min_us;
    uint32_t nominal_median_us;
    uint32_t nominal_max_us;
    uint32_t catch_up_20_min_cycles;
    uint32_t catch_up_20_median_cycles;
    uint32_t catch_up_20_max_cycles;
    uint32_t catch_up_20_min_us;
    uint32_t catch_up_20_median_us;
    uint32_t catch_up_20_max_us;
    uint32_t catch_up_70_min_cycles;
    uint32_t catch_up_70_median_cycles;
    uint32_t catch_up_70_max_cycles;
    uint32_t catch_up_70_min_us;
    uint32_t catch_up_70_median_us;
    uint32_t catch_up_70_max_us;
    uint32_t catch_up_100_min_cycles;
    uint32_t catch_up_100_median_cycles;
    uint32_t catch_up_100_max_cycles;
    uint32_t catch_up_100_min_us;
    uint32_t catch_up_100_median_us;
    uint32_t catch_up_100_max_us;
    uint32_t bounded_min_cycles;
    uint32_t bounded_median_cycles;
    uint32_t bounded_max_cycles;
    uint32_t bounded_min_us;
    uint32_t bounded_median_us;
    uint32_t bounded_max_us;
} cpg_target_benchmark_report_t;

extern volatile cpg_target_benchmark_report_t
    cpg_target_benchmark_report;

void cpg_target_benchmark_run(
    cpg_gait_generator_t *generator);

#endif /* ROBOBEETLE_CPG_TARGET_BENCHMARK_H */
