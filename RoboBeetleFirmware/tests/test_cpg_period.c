#include "cpg_gait_generator.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define PERIOD_PI 3.14159265358979323846
#define PERIOD_TWO_PI 6.28318530717958647692
#define PERIOD_STEP_MS 10U
#define PERIOD_TRANSIENT_STEPS 500U
#define PERIOD_RUN_STEPS 5000U
#define PERIOD_MAX_CROSSINGS 64U

typedef struct
{
    double nominal_period_s;
    double measured_period_s;
    double measured_frequency_hz;
    double ratio;
    uint32_t cycles;
    uint32_t transient_exclusion_ms;
} period_report_t;

static period_report_t measure_period(void)
{
    cpg_gait_generator_t generator;
    uint32_t crossing_ms[PERIOD_MAX_CROSSINGS];
    uint32_t crossing_count = 0U;
    uint32_t step;
    double next_threshold = PERIOD_TWO_PI;
    period_report_t report = {0};

    cpg_gait_generator_init(&generator);
    report.nominal_period_s = generator.profile.nominal_period_s;

    for (step = 1U; step <= PERIOD_RUN_STEPS; ++step)
    {
        cpg_gait_generator_advance(
            &generator,
            PERIOD_STEP_MS);

        if (step <= PERIOD_TRANSIENT_STEPS)
        {
            continue;
        }

        while ((generator.core.phase[1] >= next_threshold) &&
               (crossing_count < PERIOD_MAX_CROSSINGS))
        {
            crossing_ms[crossing_count] = step * PERIOD_STEP_MS;
            ++crossing_count;
            next_threshold += PERIOD_TWO_PI;
        }
    }

    assert(crossing_count >= 4U);
    report.cycles = crossing_count - 1U;
    report.transient_exclusion_ms =
        PERIOD_TRANSIENT_STEPS * PERIOD_STEP_MS;
    report.measured_period_s =
        ((double)(crossing_ms[crossing_count - 1U] -
                  crossing_ms[0U]) /
         1000.0) /
        (double)report.cycles;
    report.measured_frequency_hz =
        1.0 / report.measured_period_s;
    report.ratio =
        report.measured_period_s / report.nominal_period_s;
    assert(isfinite(report.measured_period_s));
    assert(isfinite(report.measured_frequency_hz));
    assert(isfinite(report.ratio));
    assert(report.measured_period_s > 0.0);
    assert(report.measured_frequency_hz > 0.0);
    return report;
}

int main(void)
{
    const period_report_t first = measure_period();
    const period_report_t second = measure_period();

    assert(first.nominal_period_s == 2.0);
    assert(first.cycles >= 3U);
    assert(fabs(first.measured_period_s -
                second.measured_period_s) <= 0.000000001);
    assert(fabs(first.measured_frequency_hz -
                second.measured_frequency_hz) <= 0.000000001);
    assert(fabs(first.ratio - second.ratio) <= 0.000000001);
    (void)printf(
        "nominal_period_s=%.9f measured_period_s=%.9f "
        "measured_frequency_hz=%.9f ratio=%.9f cycles=%u "
        "transient_exclusion_ms=%u\n",
        first.nominal_period_s,
        first.measured_period_s,
        first.measured_frequency_hz,
        first.ratio,
        (unsigned)first.cycles,
        (unsigned)first.transient_exclusion_ms);
    return 0;
}
