#include "motion_timing_diagnostics.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static int failures;

static void expect(bool condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

_Static_assert(sizeof(motion_timing_u64_t) == 8U,
               "fixed ABI u64 pair must be eight bytes");
_Static_assert(sizeof(motion_timing_distribution_t) == 60U,
               "distribution ABI size changed");
_Static_assert(sizeof(motion_timing_drain_report_t) == 72U,
               "drain ABI size changed");
_Static_assert(sizeof(motion_timing_tx_report_t) == 84U,
               "TX ABI size changed");
_Static_assert(sizeof(motion_timing_motion_report_t) == 456U,
               "Motion ABI size changed");
_Static_assert(offsetof(motion_timing_report_t, magic) == 0U,
               "magic ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, abi_version) == 4U,
               "version ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, report_size) == 8U,
               "size ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, system_core_clock_hz) == 12U,
               "clock ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, diagnostic_flags) == 16U,
               "flags ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, runtime_backend) == 20U,
               "backend ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, run_marker) == 24U,
               "run marker ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, reset_marker) == 28U,
               "reset marker ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, app_loop_body) == 32U,
               "first counter ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, rx_drain) == 152U,
               "RX ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, tx) == 368U,
               "TX ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, motion) == 704U,
               "Motion ABI offset changed");
_Static_assert(offsetof(motion_timing_motion_report_t,
                        gap_gt_10_ms_count) == 188U,
               "Motion gap ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t,
                        diagnostic_saturation_count) == 1160U,
               "saturation ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t,
                        diagnostic_counter_wrap_count) == 1164U,
               "counter-wrap ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t,
                        diagnostic_invalid_count) == 1168U,
               "invalid ABI offset changed");
_Static_assert(sizeof(motion_timing_report_t) == 1172U,
               "report ABI size changed");

static void test_wrap_delta(void)
{
    expect(motion_timing_dwt_delta(0xFFFFFFF0U, 0x00000020U) == 0x30U,
           "DWT delta must be unsigned and wrap-safe");
}

static void test_distribution_accumulation(void)
{
    motion_timing_distribution_t stats = {0};

    motion_timing_distribution_record(&stats, 40U);
    motion_timing_distribution_record(&stats, 10U);
    motion_timing_distribution_record(&stats, 90U);

    expect(stats.count == 3U, "distribution count must accumulate");
    expect(stats.min_value == 10U, "distribution minimum must accumulate");
    expect(stats.max_value == 90U, "distribution maximum must accumulate");
    expect(stats.total_value.lo == 140U && stats.total_value.hi == 0U,
           "distribution total must accumulate");
    expect(stats.worst_interval_value == 90U,
           "distribution worst interval must accumulate");
    expect(motion_timing_distribution_histogram_total(&stats) == 3U,
           "distribution histogram must count every sample");
}

static void test_motion_gap_buckets_are_strict(void)
{
    motion_timing_report_t report = {0};

    motion_timing_report_initialize(
        &report,
        168000000U,
        MOTION_TIMING_DIAGNOSTIC_FLAG_ENABLED,
        MOTION_GAIT_BACKEND_CPG_VALUE,
        1U);
    motion_timing_record_motion_interval_ms(&report, 10U);
    motion_timing_record_motion_interval_ms(&report, 12U);
    motion_timing_record_motion_interval_ms(&report, 15U);
    motion_timing_record_motion_interval_ms(&report, 20U);
    motion_timing_record_motion_interval_ms(&report, 30U);
    motion_timing_record_motion_interval_ms(&report, 31U);

    expect(report.motion.gap_gt_10_ms_count == 5U,
           "10 ms equality must not enter >10 ms bucket");
    expect(report.motion.gap_gt_12_ms_count == 4U,
           "12 ms equality must not enter >12 ms bucket");
    expect(report.motion.gap_gt_15_ms_count == 3U,
           "15 ms equality must not enter >15 ms bucket");
    expect(report.motion.gap_gt_20_ms_count == 2U,
           "20 ms equality must not enter >20 ms bucket");
    expect(report.motion.gap_gt_30_ms_count == 1U,
           "30 ms equality must not enter >30 ms bucket");
    expect(report.motion.actual_interval_ms.worst_interval_value == 31U,
           "Motion worst interval must be retained");
}

static void test_tx_class_accounting(void)
{
    motion_timing_report_t report = {0};

    motion_timing_report_initialize(
        &report,
        168000000U,
        MOTION_TIMING_DIAGNOSTIC_FLAG_ENABLED,
        MOTION_GAIT_BACKEND_CPG_VALUE,
        2U);
    motion_timing_record_tx_sample(
        &report,
        MOTION_TIMING_TX_ACK,
        17U,
        MOTION_TIMING_STATUS_OK,
        100U);
    motion_timing_record_tx_sample(
        &report,
        MOTION_TIMING_TX_LEAK,
        14U,
        MOTION_TIMING_STATUS_ERROR,
        200U);
    motion_timing_record_tx_sample(
        &report,
        MOTION_TIMING_TX_IMU,
        71U,
        MOTION_TIMING_STATUS_TIMEOUT,
        300U);
    motion_timing_record_tx_sample(
        &report,
        MOTION_TIMING_TX_DEPTH,
        52U,
        MOTION_TIMING_STATUS_OK,
        400U);

    expect(report.tx[MOTION_TIMING_TX_ACK].call_count == 1U,
           "ACK TX call must be counted");
    expect(report.tx[MOTION_TIMING_TX_ACK].byte_count == 17U,
           "ACK TX bytes must be counted");
    expect(report.tx[MOTION_TIMING_TX_ACK].ok_count == 1U,
           "ACK TX status must be counted");
    expect(report.tx[MOTION_TIMING_TX_LEAK].error_count == 1U,
           "Leak TX error must be counted");
    expect(report.tx[MOTION_TIMING_TX_IMU].timeout_count == 1U,
           "IMU TX timeout must be counted");
    expect(report.tx[MOTION_TIMING_TX_DEPTH].duration.max_value == 400U,
           "Depth TX duration must be counted");
}

static void test_zero_clock_does_not_invent_gap_evidence(void)
{
    motion_timing_report_t report = {0};

    motion_timing_report_initialize(
        &report,
        0U,
        MOTION_TIMING_DIAGNOSTIC_FLAG_ENABLED,
        MOTION_GAIT_BACKEND_CPG_VALUE,
        4U);
    motion_timing_record_motion_interval_cycles(
        &report,
        100U,
        0U);

    expect(report.diagnostic_invalid_count == 1U,
           "zero SystemCoreClock must be visible as an invalid diagnostic");
    expect(report.motion.gap_gt_10_ms_count == 0U &&
               report.motion.gap_gt_12_ms_count == 0U &&
               report.motion.gap_gt_15_ms_count == 0U &&
               report.motion.gap_gt_20_ms_count == 0U &&
               report.motion.gap_gt_30_ms_count == 0U,
           "zero SystemCoreClock must not invent Motion gap evidence");
}

static void test_report_validation(void)
{
    motion_timing_report_t report = {0};

    motion_timing_report_initialize(
        &report,
        168000000U,
        MOTION_TIMING_DIAGNOSTIC_FLAG_ENABLED,
        MOTION_GAIT_BACKEND_CPG_VALUE,
        3U);
    expect(motion_timing_report_is_valid(&report),
           "fresh diagnostic report must validate");

    report.magic ^= 1U;
    expect(!motion_timing_report_is_valid(&report),
           "wrong report magic must fail validation");
    report.magic = MOTION_TIMING_REPORT_MAGIC;
    report.abi_version += 1U;
    expect(!motion_timing_report_is_valid(&report),
           "wrong report version must fail validation");
    report.abi_version = MOTION_TIMING_REPORT_ABI_VERSION;
    report.report_size -= 4U;
    expect(!motion_timing_report_is_valid(&report),
           "wrong report size must fail validation");
}

int main(void)
{
    test_wrap_delta();
    test_distribution_accumulation();
    test_motion_gap_buckets_are_strict();
    test_tx_class_accounting();
    test_zero_clock_does_not_invent_gap_evidence();
    test_report_validation();

    if (failures == 0)
    {
        (void)puts("All motion timing diagnostics tests passed");
    }

    return failures == 0 ? 0 : 1;
}
