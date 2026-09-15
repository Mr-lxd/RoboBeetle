#include "motion_timing_diagnostics.h"

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

int main(void)
{
    motion_timing_mark_t mark;

    motion_timing_diagnostics_init(
        MOTION_GAIT_BACKEND_CPG_VALUE);
    expect(motion_timing_report_is_valid(&motion_timing_report),
           "diagnostic init must publish a valid report ABI");
    expect(motion_timing_report.runtime_backend ==
               MOTION_GAIT_BACKEND_CPG_VALUE,
           "diagnostic init must publish the runtime backend");
    motion_timing_diagnostics_set_runtime_backend(
        MOTION_GAIT_BACKEND_SIMPLE_GAIT_VALUE);
    expect(motion_timing_report.runtime_backend ==
               MOTION_GAIT_BACKEND_SIMPLE_GAIT_VALUE,
           "runtime selector must update the report backend label");
    expect((motion_timing_report.diagnostic_flags &
            MOTION_TIMING_DIAGNOSTIC_FLAG_ENABLED) != 0U,
           "diagnostic init must publish enabled flag");

    mark = motion_timing_diagnostics_loop_begin();
    motion_timing_diagnostics_record_rx(
        MOTION_TIMING_RX_HOST,
        3U,
        1U,
        mark);
    motion_timing_diagnostics_record_tx(
        MOTION_TIMING_TX_ACK,
        17U,
        MOTION_TIMING_STATUS_OK,
        mark);
    motion_timing_diagnostics_loop_end(mark);

    expect(motion_timing_report.app_loop_body.count == 1U,
           "loop hook must record one body sample");
    expect(motion_timing_report.rx_drain[MOTION_TIMING_RX_HOST].call_count == 1U,
           "RX hook must record one drain call");
    expect(motion_timing_report.rx_drain[MOTION_TIMING_RX_HOST].byte_count == 3U,
           "RX hook must record popped bytes");
    expect(motion_timing_report.tx[MOTION_TIMING_TX_ACK].call_count == 1U,
           "TX hook must record one ACK call");

    motion_timing_diagnostics_motion_tick_begin(10U);
    motion_timing_diagnostics_motion_tick_end(
        MOTION_TIMING_STATUS_OK);
    motion_timing_diagnostics_motion_tick_begin(30U);
    motion_timing_diagnostics_motion_tick_end(
        MOTION_TIMING_STATUS_ERROR);

    expect(motion_timing_report.motion.accepted_tick_count == 2U,
           "Motion hook must count accepted ticks");
    expect(motion_timing_report.motion.requested_elapsed_ms.count == 2U,
           "Motion hook must record requested elapsed time");
    expect(motion_timing_report.motion.actual_interval_cycles.count == 1U,
           "Motion hook must record intervals after the first tick");
    expect(motion_timing_report.motion.result_ok_count == 1U,
           "Motion hook must record successful tick results");
    expect(motion_timing_report.motion.result_error_count == 1U,
           "Motion hook must record failed tick results");

    if (failures == 0)
    {
        (void)puts("All motion timing diagnostics hook tests passed");
    }
    return failures == 0 ? 0 : 1;
}
