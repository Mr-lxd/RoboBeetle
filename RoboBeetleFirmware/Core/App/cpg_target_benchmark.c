#include "cpg_target_benchmark.h"

#include "stm32f4xx_hal.h"
#include "system_stm32f4xx.h"

#include <stddef.h>
#include <stdint.h>

#define CPG_TARGET_BENCHMARK_REPETITIONS 32U

volatile cpg_target_benchmark_report_t cpg_target_benchmark_report;

typedef void (*benchmark_operation_t)(
    cpg_gait_generator_t *generator);

typedef struct
{
    uint32_t min_cycles;
    uint32_t median_cycles;
    uint32_t max_cycles;
} benchmark_stats_t;

static void benchmark_barrier(void)
{
#if defined(ROBOBEETLE_CPG_HOST_COMPILE_CONTRACT)
    /* The host contract checks C syntax and MCU header integration only. */
    return;
#else
    __DSB();
    __ISB();
#endif
}

static void benchmark_nominal(
    cpg_gait_generator_t *generator)
{
    cpg_gait_generator_advance(generator, 10U);
}

static void benchmark_catch_up_20(
    cpg_gait_generator_t *generator)
{
    cpg_gait_generator_advance(generator, 20U);
}

static void benchmark_catch_up_70(
    cpg_gait_generator_t *generator)
{
    cpg_gait_generator_advance(generator, 70U);
}

static void benchmark_catch_up_100(
    cpg_gait_generator_t *generator)
{
    cpg_gait_generator_advance(generator, 100U);
}

static void benchmark_bounded_catch_up(
    cpg_gait_generator_t *generator)
{
    cpg_gait_generator_advance(generator, 1000U);
}

static void benchmark_enable_counter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    benchmark_barrier();
}

static uint32_t benchmark_measure_once(
    cpg_gait_generator_t *generator,
    benchmark_operation_t operation)
{
    uint32_t start;
    uint32_t finish;

    benchmark_barrier();
    start = DWT->CYCCNT;
    operation(generator);
    benchmark_barrier();
    finish = DWT->CYCCNT;
    return finish - start;
}

static benchmark_stats_t benchmark_measure(
    const cpg_gait_generator_t *initial,
    cpg_gait_generator_t *working,
    benchmark_operation_t operation)
{
    uint32_t samples[CPG_TARGET_BENCHMARK_REPETITIONS];
    uint32_t i;
    benchmark_stats_t stats = {
        .min_cycles = UINT32_MAX,
        .median_cycles = 0U,
        .max_cycles = 0U,
    };

    *working = *initial;
    operation(working);

    for (i = 0U; i < CPG_TARGET_BENCHMARK_REPETITIONS; ++i)
    {
        uint32_t j;

        *working = *initial;
        samples[i] = benchmark_measure_once(working, operation);
        if (samples[i] < stats.min_cycles)
        {
            stats.min_cycles = samples[i];
        }
        if (samples[i] > stats.max_cycles)
        {
            stats.max_cycles = samples[i];
        }
        for (j = i; j > 0U; --j)
        {
            if (samples[j - 1U] <= samples[j])
            {
                break;
            }
            {
                const uint32_t swap = samples[j - 1U];
                samples[j - 1U] = samples[j];
                samples[j] = swap;
            }
        }
    }

    stats.median_cycles =
        samples[CPG_TARGET_BENCHMARK_REPETITIONS / 2U];
    return stats;
}

static uint32_t cycles_to_microseconds(uint32_t cycles)
{
    return SystemCoreClock == 0U ?
        0U :
        (uint32_t)(((uint64_t)cycles * 1000000ULL) /
                   (uint64_t)SystemCoreClock);
}

static void write_stats(
    volatile uint32_t *min_cycles,
    volatile uint32_t *median_cycles,
    volatile uint32_t *max_cycles,
    volatile uint32_t *min_us,
    volatile uint32_t *median_us,
    volatile uint32_t *max_us,
    benchmark_stats_t stats)
{
    *min_cycles = stats.min_cycles;
    *median_cycles = stats.median_cycles;
    *max_cycles = stats.max_cycles;
    *min_us = cycles_to_microseconds(stats.min_cycles);
    *median_us = cycles_to_microseconds(stats.median_cycles);
    *max_us = cycles_to_microseconds(stats.max_cycles);
}

void cpg_target_benchmark_run(
    cpg_gait_generator_t *generator)
{
    const cpg_gait_generator_t initial =
        generator == NULL ? (cpg_gait_generator_t){0} : *generator;
    cpg_gait_generator_t working;
    benchmark_stats_t stats;

    {
        volatile uint8_t *report_bytes =
            (volatile uint8_t *)&cpg_target_benchmark_report;
        size_t i;

        for (i = 0U; i < sizeof(cpg_target_benchmark_report); ++i)
        {
            report_bytes[i] = 0U;
        }
    }
    if (generator == NULL)
    {
        return;
    }

    benchmark_enable_counter();
    cpg_gait_generator_init(&working);
    benchmark_nominal(&working);
    *generator = initial;

    stats = benchmark_measure(
        &initial,
        &working,
        benchmark_nominal);
    write_stats(
        &cpg_target_benchmark_report.nominal_min_cycles,
        &cpg_target_benchmark_report.nominal_median_cycles,
        &cpg_target_benchmark_report.nominal_max_cycles,
        &cpg_target_benchmark_report.nominal_min_us,
        &cpg_target_benchmark_report.nominal_median_us,
        &cpg_target_benchmark_report.nominal_max_us,
        stats);

    stats = benchmark_measure(
        &initial,
        &working,
        benchmark_catch_up_20);
    write_stats(
        &cpg_target_benchmark_report.catch_up_20_min_cycles,
        &cpg_target_benchmark_report.catch_up_20_median_cycles,
        &cpg_target_benchmark_report.catch_up_20_max_cycles,
        &cpg_target_benchmark_report.catch_up_20_min_us,
        &cpg_target_benchmark_report.catch_up_20_median_us,
        &cpg_target_benchmark_report.catch_up_20_max_us,
        stats);

    stats = benchmark_measure(
        &initial,
        &working,
        benchmark_catch_up_70);
    write_stats(
        &cpg_target_benchmark_report.catch_up_70_min_cycles,
        &cpg_target_benchmark_report.catch_up_70_median_cycles,
        &cpg_target_benchmark_report.catch_up_70_max_cycles,
        &cpg_target_benchmark_report.catch_up_70_min_us,
        &cpg_target_benchmark_report.catch_up_70_median_us,
        &cpg_target_benchmark_report.catch_up_70_max_us,
        stats);

    stats = benchmark_measure(
        &initial,
        &working,
        benchmark_catch_up_100);
    write_stats(
        &cpg_target_benchmark_report.catch_up_100_min_cycles,
        &cpg_target_benchmark_report.catch_up_100_median_cycles,
        &cpg_target_benchmark_report.catch_up_100_max_cycles,
        &cpg_target_benchmark_report.catch_up_100_min_us,
        &cpg_target_benchmark_report.catch_up_100_median_us,
        &cpg_target_benchmark_report.catch_up_100_max_us,
        stats);

    stats = benchmark_measure(
        &initial,
        &working,
        benchmark_bounded_catch_up);
    write_stats(
        &cpg_target_benchmark_report.bounded_min_cycles,
        &cpg_target_benchmark_report.bounded_median_cycles,
        &cpg_target_benchmark_report.bounded_max_cycles,
        &cpg_target_benchmark_report.bounded_min_us,
        &cpg_target_benchmark_report.bounded_median_us,
        &cpg_target_benchmark_report.bounded_max_us,
        stats);

    cpg_target_benchmark_report.enabled = 1U;
    cpg_target_benchmark_report.system_core_clock_hz =
        SystemCoreClock;
    cpg_target_benchmark_report.repetitions =
        CPG_TARGET_BENCHMARK_REPETITIONS;
    *generator = initial;
}
