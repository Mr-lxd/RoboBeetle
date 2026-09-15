#include "motion_timing_diagnostics.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures;

static void expect(bool condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

static void test_trial_lifecycle_and_freeze(void)
{
    motion_timing_mark_t mark;
    motion_timing_report_t frozen_report;
    uint32_t first_run_marker;

    motion_timing_diagnostics_init(MOTION_GAIT_BACKEND_CPG_VALUE);
    motion_timing_diagnostics_record_uart_completed(
        MOTION_TIMING_TX_ACK);
    expect(motion_timing_report.uart_transport.completed_count[
               MOTION_TIMING_TX_ACK] == 0U,
           "UART events before begin_run must not enter the report");
    motion_timing_diagnostics_begin_run(MOTION_GAIT_BACKEND_CPG_VALUE);
    first_run_marker = motion_timing_report.run_marker;
    expect(motion_timing_report.run_state == MOTION_TIMING_RUN_STATE_RUNNING,
           "begin_run must make the report recordable");

    mark = motion_timing_diagnostics_mark();
    motion_timing_diagnostics_record_rx(
        MOTION_TIMING_RX_HOST,
        7U,
        1U,
        mark);
    motion_timing_diagnostics_motion_tick_begin(10U);
    motion_timing_diagnostics_motion_tick_end(MOTION_TIMING_STATUS_OK);
    motion_timing_diagnostics_record_uart_enqueue(
        MOTION_TIMING_TX_ACK,
        MOTION_TIMING_UART_EVENT_ENQUEUED);
    motion_timing_diagnostics_record_uart_completed(
        MOTION_TIMING_TX_ACK);
    expect(motion_timing_report.motion.accepted_tick_count == 1U,
           "first trial must collect Motion counters");

    motion_timing_diagnostics_freeze(
        MOTION_TIMING_TERMINATION_NORMAL_STOP);
    expect(motion_timing_report.run_state == MOTION_TIMING_RUN_STATE_FROZEN,
           "normal stop must freeze the report");
    expect(motion_timing_report.termination_reason ==
               MOTION_TIMING_TERMINATION_NORMAL_STOP,
           "normal stop reason must be retained");
    frozen_report = motion_timing_report;

    motion_timing_diagnostics_record_uart_enqueue(
        MOTION_TIMING_TX_IMU,
        MOTION_TIMING_UART_EVENT_ENQUEUED);
    motion_timing_diagnostics_record_uart_completed(
        MOTION_TIMING_TX_IMU);
    motion_timing_diagnostics_record_uart_dropped(
        MOTION_TIMING_TX_IMU);
    motion_timing_diagnostics_record_uart_start_busy();
    motion_timing_diagnostics_record_uart_start_error();
    motion_timing_diagnostics_record_uart_error(true);
    motion_timing_diagnostics_record_uart_rearm(
        MOTION_TIMING_UART_REARM_ATTEMPT);
    motion_timing_diagnostics_record_uart_unexpected_callback();
    motion_timing_diagnostics_record_uart_high_water(9U);
    motion_timing_diagnostics_record_uart_busy_recovery();
    motion_timing_diagnostics_record_uart_reinitialization();

    mark = motion_timing_diagnostics_loop_begin();
    motion_timing_diagnostics_record_rx(
        MOTION_TIMING_RX_HOST,
        99U,
        1U,
        mark);
    motion_timing_diagnostics_motion_tick_begin(30U);
    motion_timing_diagnostics_motion_tick_end(MOTION_TIMING_STATUS_ERROR);
    motion_timing_diagnostics_set_runtime_backend(
        MOTION_GAIT_BACKEND_SIMPLE_GAIT_VALUE);
    motion_timing_report_t after_freeze = motion_timing_report;
    expect(memcmp(&frozen_report, &after_freeze, sizeof frozen_report) == 0,
           "frozen evidence must not change in later loops or ACK traffic");

    motion_timing_diagnostics_begin_run(
        MOTION_GAIT_BACKEND_SIMPLE_GAIT_VALUE);
    expect(motion_timing_report.run_state == MOTION_TIMING_RUN_STATE_RUNNING,
           "a new trial must become recordable");
    expect(motion_timing_report.run_marker != first_run_marker,
           "a new trial must advance the run marker");
    expect(motion_timing_report.runtime_backend ==
               MOTION_GAIT_BACKEND_SIMPLE_GAIT_VALUE,
           "a new trial must publish its selected backend");
    expect(motion_timing_report.motion.accepted_tick_count == 0U &&
               motion_timing_report.rx_drain[MOTION_TIMING_RX_HOST].byte_count ==
                   0U &&
               motion_timing_report.uart_transport.completed_count[
                   MOTION_TIMING_TX_ACK] == 0U &&
               motion_timing_report.uart_transport.enqueued_count[
                   MOTION_TIMING_TX_IMU] == 0U,
           "a new trial must clear the previous trial counters");

    motion_timing_diagnostics_motion_tick_begin(20U);
    motion_timing_diagnostics_motion_tick_end(MOTION_TIMING_STATUS_OK);
    expect(motion_timing_report.motion.accepted_tick_count == 1U &&
               motion_timing_report.runtime_backend ==
                   MOTION_GAIT_BACKEND_SIMPLE_GAIT_VALUE,
           "the second trial must contain only SimpleGait data");
}

static void test_worst_gap_context_is_interval_local(void)
{
    motion_timing_mark_t mark;

    motion_timing_diagnostics_init(MOTION_GAIT_BACKEND_CPG_VALUE);
    motion_timing_diagnostics_begin_run(MOTION_GAIT_BACKEND_CPG_VALUE);
    /* The host test supplies a deterministic clock for cycle-to-ms conversion. */
    motion_timing_report.system_core_clock_hz = 1000U;

    motion_timing_diagnostics_host_set_cycles(0U);
    motion_timing_diagnostics_motion_tick_begin(10U);
    motion_timing_diagnostics_host_set_cycles(10U);
    motion_timing_diagnostics_motion_tick_end(MOTION_TIMING_STATUS_OK);

    mark = motion_timing_diagnostics_mark();
    motion_timing_diagnostics_host_set_cycles(15U);
    motion_timing_diagnostics_record_rx(
        MOTION_TIMING_RX_HOST,
        7U,
        1U,
        mark);
    mark = motion_timing_diagnostics_mark();
    motion_timing_diagnostics_host_set_cycles(25U);
    motion_timing_diagnostics_record_tx(
        MOTION_TIMING_TX_ACK,
        2U,
        MOTION_TIMING_STATUS_OK,
        mark);

    motion_timing_diagnostics_host_set_cycles(100U);
    motion_timing_diagnostics_motion_tick_begin(20U);
    expect(motion_timing_report.worst_gap_context.interval_cycles == 100U &&
               motion_timing_report.worst_gap_context.interval_ms == 100U,
           "worst-gap context must capture interval cycles and milliseconds");
    expect(motion_timing_report.worst_gap_context.requested_elapsed_ms == 20U,
           "worst-gap context must capture requested elapsed time");
    expect(motion_timing_report.worst_gap_context.rx_byte_count[
                MOTION_TIMING_RX_HOST] == 7U &&
               motion_timing_report.worst_gap_context.tx_call_count[
                MOTION_TIMING_TX_ACK] == 1U &&
               motion_timing_report.worst_gap_context.tx_byte_count[
                MOTION_TIMING_TX_ACK] == 2U,
           "first worst-gap context must contain only its preceding work");

    motion_timing_diagnostics_host_set_cycles(120U);
    motion_timing_diagnostics_motion_tick_end(MOTION_TIMING_STATUS_OK);
    mark = motion_timing_diagnostics_mark();
    motion_timing_diagnostics_host_set_cycles(130U);
    motion_timing_diagnostics_record_rx(
        MOTION_TIMING_RX_JY901S,
        11U,
        1U,
        mark);
    mark = motion_timing_diagnostics_mark();
    motion_timing_diagnostics_host_set_cycles(150U);
    motion_timing_diagnostics_record_tx(
        MOTION_TIMING_TX_LEAK,
        3U,
        MOTION_TIMING_STATUS_OK,
        mark);

    motion_timing_diagnostics_host_set_cycles(250U);
    motion_timing_diagnostics_motion_tick_begin(30U);
    expect(motion_timing_report.worst_gap_context.interval_cycles == 150U &&
               motion_timing_report.worst_gap_context.requested_elapsed_ms == 30U,
           "a new worst gap must replace the previous snapshot");
    expect(motion_timing_report.worst_gap_context.rx_byte_count[
                MOTION_TIMING_RX_HOST] == 0U &&
               motion_timing_report.worst_gap_context.rx_byte_count[
                MOTION_TIMING_RX_JY901S] == 11U &&
               motion_timing_report.worst_gap_context.tx_call_count[
                MOTION_TIMING_TX_ACK] == 0U &&
               motion_timing_report.worst_gap_context.tx_call_count[
                MOTION_TIMING_TX_LEAK] == 1U,
           "worst-gap context must reset at each accepted tick");
    expect(motion_timing_report.worst_gap_context.previous_tick_duration_cycles ==
               20U,
           "worst-gap context must retain the preceding tick duration");
}

int main(void)
{
    test_trial_lifecycle_and_freeze();
    test_worst_gap_context_is_interval_local();

    if (failures == 0)
    {
        (void)puts("All motion timing diagnostics lifecycle tests passed");
    }
    return failures == 0 ? 0 : 1;
}
