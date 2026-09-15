#include "motion_timing_diagnostics.h"

#if ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS && \
    !defined(ROBOBEETLE_MOTION_TIMING_HOST_TEST)
#include "stm32f4xx_hal.h"
#include "system_stm32f4xx.h"
#endif

#include <limits.h>

volatile motion_timing_report_t motion_timing_report;

#if ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS
static uint32_t motion_timing_run_marker;
static uint32_t motion_timing_reset_marker;

#if defined(ROBOBEETLE_MOTION_TIMING_HOST_TEST)
static motion_timing_mark_t motion_timing_host_cycles;

static motion_timing_mark_t motion_timing_read_cycles(void)
{
    return motion_timing_host_cycles;
}
#else
static motion_timing_mark_t motion_timing_read_cycles(void)
{
    return DWT->CYCCNT;
}

static void motion_timing_barrier(void)
{
    __DSB();
    __ISB();
}
#endif
#endif

static void motion_timing_increment_saturated(volatile uint32_t *value)
{
    if (*value < UINT32_MAX)
    {
        ++(*value);
    }
}

static void motion_timing_add_u64(
    volatile motion_timing_u64_t *total,
    uint32_t value,
    volatile uint32_t *saturated_count)
{
    const uint32_t old_lo = total->lo;
    const uint32_t new_lo = old_lo + value;
    const uint32_t carry = (new_lo < old_lo) ? 1U : 0U;

    if ((carry != 0U) && (total->hi == UINT32_MAX))
    {
        total->lo = UINT32_MAX;
        total->hi = UINT32_MAX;
        motion_timing_increment_saturated(saturated_count);
        return;
    }

    total->lo = new_lo;
    if (carry != 0U)
    {
        ++total->hi;
    }
}

static uint32_t motion_timing_histogram_bucket(uint32_t value)
{
    if (value <= MOTION_TIMING_HISTOGRAM_BUCKET_0_MAX)
    {
        return 0U;
    }
    if (value <= MOTION_TIMING_HISTOGRAM_BUCKET_1_MAX)
    {
        return 1U;
    }
    if (value <= MOTION_TIMING_HISTOGRAM_BUCKET_2_MAX)
    {
        return 2U;
    }
    if (value <= MOTION_TIMING_HISTOGRAM_BUCKET_3_MAX)
    {
        return 3U;
    }
    if (value <= MOTION_TIMING_HISTOGRAM_BUCKET_4_MAX)
    {
        return 4U;
    }
    if (value <= MOTION_TIMING_HISTOGRAM_BUCKET_5_MAX)
    {
        return 5U;
    }
    if (value <= MOTION_TIMING_HISTOGRAM_BUCKET_6_MAX)
    {
        return 6U;
    }
    return 7U;
}

uint32_t motion_timing_dwt_delta(
    uint32_t start,
    uint32_t finish)
{
    return finish - start;
}

void motion_timing_report_initialize(
    volatile motion_timing_report_t *report,
    uint32_t system_core_clock_hz,
    uint32_t diagnostic_flags,
    uint32_t runtime_backend,
    uint32_t reset_marker)
{
    volatile uint8_t *report_bytes;
    size_t index;

    if (report == NULL)
    {
        return;
    }

    report_bytes = (volatile uint8_t *)report;
    for (index = 0U; index < sizeof(*report); ++index)
    {
        report_bytes[index] = 0U;
    }

    report->magic = MOTION_TIMING_REPORT_MAGIC;
    report->abi_version = MOTION_TIMING_REPORT_ABI_VERSION;
    report->report_size = (uint32_t)sizeof(*report);
    report->system_core_clock_hz = system_core_clock_hz;
    report->diagnostic_flags = diagnostic_flags;
    report->runtime_backend = runtime_backend;
    report->run_marker = 0U;
    report->reset_marker = reset_marker;
    report->run_state = MOTION_TIMING_RUN_STATE_IDLE;
    report->termination_reason = MOTION_TIMING_TERMINATION_NONE;
}

bool motion_timing_report_is_valid(
    const volatile motion_timing_report_t *report)
{
    return (report != NULL) &&
           (report->magic == MOTION_TIMING_REPORT_MAGIC) &&
           (report->abi_version == MOTION_TIMING_REPORT_ABI_VERSION) &&
           (report->report_size == MOTION_TIMING_REPORT_SIZE);
}

static void motion_timing_distribution_record_internal(
    volatile motion_timing_distribution_t *distribution,
    uint32_t value,
    bool include_histogram)
{
    if (distribution == NULL)
    {
        return;
    }

    if (distribution->count == UINT32_MAX)
    {
        motion_timing_increment_saturated(&distribution->saturated_count);
    }
    else
    {
        ++distribution->count;
    }

    if ((distribution->count == 1U) ||
        (value < distribution->min_value))
    {
        distribution->min_value = value;
    }
    if (value > distribution->max_value)
    {
        distribution->max_value = value;
    }
    if (value > distribution->worst_interval_value)
    {
        distribution->worst_interval_value = value;
    }

    motion_timing_add_u64(
        &distribution->total_value,
        value,
        &distribution->saturated_count);

    if (include_histogram)
    {
        const uint32_t bucket = motion_timing_histogram_bucket(value);

        if (distribution->histogram[bucket] == UINT32_MAX)
        {
            motion_timing_increment_saturated(&distribution->saturated_count);
        }
        else
        {
            ++distribution->histogram[bucket];
        }
    }
}

void motion_timing_distribution_record(
    volatile motion_timing_distribution_t *distribution,
    uint32_t value)
{
    motion_timing_distribution_record_internal(
        distribution,
        value,
        true);
}

void motion_timing_distribution_record_cycles(
    volatile motion_timing_distribution_t *distribution,
    uint32_t value)
{
    motion_timing_distribution_record_internal(
        distribution,
        value,
        false);
}

uint32_t motion_timing_distribution_histogram_total(
    const volatile motion_timing_distribution_t *distribution)
{
    uint32_t total = 0U;
    uint32_t index;

    if (distribution == NULL)
    {
        return 0U;
    }

    for (index = 0U;
         index < MOTION_TIMING_HISTOGRAM_BUCKET_COUNT;
         ++index)
    {
        if (UINT32_MAX - total < distribution->histogram[index])
        {
            return UINT32_MAX;
        }
        total += distribution->histogram[index];
    }
    return total;
}

static void motion_timing_record_bytes(
    volatile uint32_t *total,
    uint32_t value,
    volatile uint32_t *saturated_count)
{
    if (UINT32_MAX - *total < value)
    {
        *total = UINT32_MAX;
        motion_timing_increment_saturated(saturated_count);
    }
    else
    {
        *total += value;
    }
}

void motion_timing_record_rx_sample(
    volatile motion_timing_report_t *report,
    motion_timing_rx_kind_t kind,
    uint32_t byte_count,
    uint32_t nonempty,
    uint32_t duration_cycles)
{
    volatile motion_timing_drain_report_t *drain;

    if ((report == NULL) || (kind >= MOTION_TIMING_RX_COUNT))
    {
        return;
    }

    drain = &report->rx_drain[kind];
    motion_timing_increment_saturated(&drain->call_count);
    if (nonempty != 0U)
    {
        motion_timing_increment_saturated(&drain->nonempty_count);
    }
    motion_timing_record_bytes(
        &drain->byte_count,
        byte_count,
        &report->diagnostic_saturation_count);
    motion_timing_distribution_record_cycles(
        &drain->duration,
        duration_cycles);
}

void motion_timing_record_tx_sample(
    volatile motion_timing_report_t *report,
    motion_timing_tx_kind_t kind,
    uint32_t byte_count,
    uint32_t status,
    uint32_t duration_cycles)
{
    volatile motion_timing_tx_report_t *tx;

    if ((report == NULL) || (kind >= MOTION_TIMING_TX_COUNT))
    {
        return;
    }

    tx = &report->tx[kind];
    motion_timing_increment_saturated(&tx->call_count);
    motion_timing_record_bytes(
        &tx->byte_count,
        byte_count,
        &report->diagnostic_saturation_count);
    tx->last_status = status;
    if (status == MOTION_TIMING_STATUS_OK)
    {
        motion_timing_increment_saturated(&tx->ok_count);
    }
    else
    {
        motion_timing_increment_saturated(&tx->error_count);
    }
    if (status == MOTION_TIMING_STATUS_TIMEOUT)
    {
        motion_timing_increment_saturated(&tx->timeout_count);
    }
    motion_timing_distribution_record_cycles(
        &tx->duration,
        duration_cycles);
}

static void motion_timing_record_gap_counts(
    volatile motion_timing_motion_report_t *motion,
    bool gt_10,
    bool gt_12,
    bool gt_15,
    bool gt_20,
    bool gt_30)
{
    if (gt_10)
    {
        motion_timing_increment_saturated(
            &motion->gap_gt_10_ms_count);
    }
    if (gt_12)
    {
        motion_timing_increment_saturated(
            &motion->gap_gt_12_ms_count);
    }
    if (gt_15)
    {
        motion_timing_increment_saturated(
            &motion->gap_gt_15_ms_count);
    }
    if (gt_20)
    {
        motion_timing_increment_saturated(
            &motion->gap_gt_20_ms_count);
    }
    if (gt_30)
    {
        motion_timing_increment_saturated(
            &motion->gap_gt_30_ms_count);
    }
}

void motion_timing_record_motion_interval_ms(
    volatile motion_timing_report_t *report,
    uint32_t interval_ms)
{
    if (report == NULL)
    {
        return;
    }

    motion_timing_distribution_record(
        &report->motion.actual_interval_ms,
        interval_ms);
    motion_timing_record_gap_counts(
        &report->motion,
        interval_ms > 10U,
        interval_ms > 12U,
        interval_ms > 15U,
        interval_ms > 20U,
        interval_ms > 30U);
}

void motion_timing_record_motion_interval_cycles(
    volatile motion_timing_report_t *report,
    uint32_t interval_cycles,
    uint32_t system_core_clock_hz)
{
    uint32_t interval_ms = 0U;
    const uint64_t clock = (uint64_t)system_core_clock_hz;
    const uint64_t cycles = (uint64_t)interval_cycles;

    if (report == NULL)
    {
        return;
    }

    if (system_core_clock_hz == 0U)
    {
        motion_timing_increment_saturated(
            &report->diagnostic_invalid_count);
        motion_timing_distribution_record_cycles(
            &report->motion.actual_interval_cycles,
            interval_cycles);
        motion_timing_distribution_record(
            &report->motion.actual_interval_ms,
            0U);
        return;
    }

    interval_ms = (uint32_t)((cycles * 1000ULL) / clock);
    motion_timing_distribution_record_cycles(
        &report->motion.actual_interval_cycles,
        interval_cycles);
    motion_timing_distribution_record(
        &report->motion.actual_interval_ms,
        interval_ms);
    motion_timing_record_gap_counts(
        &report->motion,
        (cycles * 1000ULL) > (clock * 10ULL),
        (cycles * 1000ULL) > (clock * 12ULL),
        (cycles * 1000ULL) > (clock * 15ULL),
        (cycles * 1000ULL) > (clock * 20ULL),
        (cycles * 1000ULL) > (clock * 30ULL));
}

#if ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS

static bool have_loop_entry;
static motion_timing_mark_t loop_entry_mark;
static bool have_motion_tick;
static motion_timing_mark_t motion_tick_mark;
static motion_timing_mark_t motion_tick_start_mark;
static uint32_t previous_tick_duration_cycles;
static motion_timing_gap_context_t gap_context;
static uint32_t configured_clock_hz;
static uint32_t configured_flags;

typedef enum
{
    MOTION_TIMING_GATE_RESETTING = 0,
    MOTION_TIMING_GATE_RUNNING,
    MOTION_TIMING_GATE_FROZEN
} motion_timing_recording_gate_t;

static volatile motion_timing_recording_gate_t recording_gate =
    MOTION_TIMING_GATE_FROZEN;

#if defined(ROBOBEETLE_MOTION_TIMING_HOST_TEST)
static uint32_t motion_timing_irq_save(void)
{
    return 0U;
}

static void motion_timing_irq_restore(uint32_t primask)
{
    (void)primask;
}
#else
static uint32_t motion_timing_irq_save(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void motion_timing_irq_restore(uint32_t primask)
{
    __set_PRIMASK(primask);
}
#endif

static bool motion_timing_is_recording(void)
{
    return recording_gate == MOTION_TIMING_GATE_RUNNING;
}

static void motion_timing_clear_gap_context(void)
{
    uint8_t *context_bytes = (uint8_t *)&gap_context;
    size_t index;

    for (index = 0U; index < sizeof(gap_context); ++index)
    {
        context_bytes[index] = 0U;
    }
}

static void motion_timing_copy_gap_context(
    volatile motion_timing_gap_context_t *destination,
    const motion_timing_gap_context_t *source)
{
    const uint8_t *source_bytes = (const uint8_t *)source;
    volatile uint8_t *destination_bytes = (volatile uint8_t *)destination;
    size_t index;

    for (index = 0U; index < sizeof(*destination); ++index)
    {
        destination_bytes[index] = source_bytes[index];
    }
}

static uint32_t motion_timing_cycles_to_ms(
    uint32_t cycles,
    uint32_t system_core_clock_hz)
{
    if (system_core_clock_hz == 0U)
    {
        return 0U;
    }
    return (uint32_t)(((uint64_t)cycles * 1000ULL) /
                      (uint64_t)system_core_clock_hz);
}

static void motion_timing_gap_context_record_rx(
    motion_timing_rx_kind_t kind,
    uint32_t byte_count,
    uint32_t duration_cycles)
{
    if (kind >= MOTION_TIMING_RX_COUNT)
    {
        return;
    }

    motion_timing_record_bytes(
        &gap_context.rx_byte_count[kind],
        byte_count,
        &motion_timing_report.diagnostic_saturation_count);
    motion_timing_add_u64(
        &gap_context.rx_cycles[kind],
        duration_cycles,
        &motion_timing_report.diagnostic_saturation_count);
}

static void motion_timing_gap_context_record_tx(
    motion_timing_tx_kind_t kind,
    uint32_t byte_count,
    uint32_t duration_cycles)
{
    if (kind >= MOTION_TIMING_TX_COUNT)
    {
        return;
    }

    motion_timing_increment_saturated(
        &gap_context.tx_call_count[kind]);
    motion_timing_record_bytes(
        &gap_context.tx_byte_count[kind],
        byte_count,
        &motion_timing_report.diagnostic_saturation_count);
    motion_timing_add_u64(
        &gap_context.tx_cycles[kind],
        duration_cycles,
        &motion_timing_report.diagnostic_saturation_count);
}

static void motion_timing_capture_worst_gap_context(
    uint32_t interval_cycles,
    uint32_t requested_elapsed_ms)
{
    motion_timing_copy_gap_context(
        &motion_timing_report.worst_gap_context,
        &gap_context);
    motion_timing_report.worst_gap_context.interval_cycles =
        interval_cycles;
    motion_timing_report.worst_gap_context.interval_ms =
        motion_timing_cycles_to_ms(
            interval_cycles,
            motion_timing_report.system_core_clock_hz);
    motion_timing_report.worst_gap_context.requested_elapsed_ms =
        requested_elapsed_ms;
    motion_timing_report.worst_gap_context.runtime_backend =
        motion_timing_report.runtime_backend;
    motion_timing_report.worst_gap_context.previous_tick_duration_cycles =
        previous_tick_duration_cycles;
}

static uint32_t motion_timing_measure_delta(
    motion_timing_mark_t start)
{
    const motion_timing_mark_t finish = motion_timing_read_cycles();

    if (finish < start)
    {
        motion_timing_increment_saturated(
            &motion_timing_report.diagnostic_counter_wrap_count);
    }
    return motion_timing_dwt_delta(start, finish);
}

void motion_timing_diagnostics_init(uint32_t runtime_backend)
{
    uint32_t primask = motion_timing_irq_save();

    recording_gate = MOTION_TIMING_GATE_RESETTING;
    motion_timing_irq_restore(primask);

    configured_flags = MOTION_TIMING_DIAGNOSTIC_FLAG_ENABLED;

#if ROBOBEETLE_MOTION_TIMING_REDUCED_TELEMETRY
    configured_flags |= MOTION_TIMING_DIAGNOSTIC_FLAG_REDUCED_TELEMETRY;
#endif

#if !defined(ROBOBEETLE_MOTION_TIMING_HOST_TEST)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    motion_timing_barrier();
    configured_clock_hz = SystemCoreClock;
#else
    configured_clock_hz = 0U;
#endif

    ++motion_timing_reset_marker;
    motion_timing_report_initialize(
        &motion_timing_report,
        configured_clock_hz,
        configured_flags,
        runtime_backend,
        motion_timing_reset_marker);
    have_loop_entry = false;
    have_motion_tick = false;
    loop_entry_mark = 0U;
    motion_tick_mark = 0U;
    motion_tick_start_mark = 0U;
    previous_tick_duration_cycles = 0U;
    motion_timing_clear_gap_context();

    primask = motion_timing_irq_save();
    recording_gate = MOTION_TIMING_GATE_FROZEN;
    motion_timing_irq_restore(primask);
}

void motion_timing_diagnostics_begin_run(uint32_t runtime_backend)
{
    uint32_t primask = motion_timing_irq_save();

    recording_gate = MOTION_TIMING_GATE_RESETTING;
    motion_timing_irq_restore(primask);

    ++motion_timing_run_marker;
    ++motion_timing_reset_marker;
    motion_timing_report_initialize(
        &motion_timing_report,
        configured_clock_hz,
        configured_flags,
        runtime_backend,
        motion_timing_reset_marker);
    motion_timing_report.run_marker = motion_timing_run_marker;
    motion_timing_report.run_state = MOTION_TIMING_RUN_STATE_RUNNING;
    motion_timing_report.termination_reason =
        MOTION_TIMING_TERMINATION_NONE;
    have_loop_entry = false;
    have_motion_tick = false;
    loop_entry_mark = 0U;
    motion_tick_mark = 0U;
    motion_tick_start_mark = 0U;
    previous_tick_duration_cycles = 0U;
    motion_timing_clear_gap_context();

    primask = motion_timing_irq_save();
    recording_gate = MOTION_TIMING_GATE_RUNNING;
    motion_timing_irq_restore(primask);
}

void motion_timing_diagnostics_freeze(uint32_t termination_reason)
{
    uint32_t primask = motion_timing_irq_save();

    if (!motion_timing_is_recording())
    {
        motion_timing_irq_restore(primask);
        return;
    }

    recording_gate = MOTION_TIMING_GATE_FROZEN;
    motion_timing_report.run_state = MOTION_TIMING_RUN_STATE_FROZEN;
    motion_timing_report.termination_reason = termination_reason;
    motion_timing_irq_restore(primask);

    have_loop_entry = false;
    have_motion_tick = false;
    loop_entry_mark = 0U;
    motion_tick_mark = 0U;
    motion_tick_start_mark = 0U;
    motion_timing_clear_gap_context();
}

motion_timing_mark_t motion_timing_diagnostics_mark(void)
{
    return motion_timing_read_cycles();
}

void motion_timing_diagnostics_set_runtime_backend(uint32_t runtime_backend)
{
    if (motion_timing_report.run_state != MOTION_TIMING_RUN_STATE_FROZEN)
    {
        motion_timing_report.runtime_backend = runtime_backend;
    }
}

motion_timing_mark_t motion_timing_diagnostics_loop_begin(void)
{
    const motion_timing_mark_t mark = motion_timing_read_cycles();

    if (!motion_timing_is_recording())
    {
        have_loop_entry = false;
        return mark;
    }

    if (have_loop_entry)
    {
        motion_timing_distribution_record_cycles(
            &motion_timing_report.app_loop_interval,
            motion_timing_measure_delta(loop_entry_mark));
    }
    loop_entry_mark = mark;
    have_loop_entry = true;
    return mark;
}

void motion_timing_diagnostics_loop_end(motion_timing_mark_t start)
{
    if (motion_timing_is_recording())
    {
        motion_timing_distribution_record_cycles(
            &motion_timing_report.app_loop_body,
            motion_timing_measure_delta(start));
    }
}

void motion_timing_diagnostics_record_rx(
    motion_timing_rx_kind_t kind,
    uint32_t byte_count,
    uint32_t nonempty,
    motion_timing_mark_t start)
{
    uint32_t duration_cycles;

    if (!motion_timing_is_recording())
    {
        return;
    }

    duration_cycles = motion_timing_measure_delta(start);
    motion_timing_record_rx_sample(
        &motion_timing_report,
        kind,
        byte_count,
        nonempty,
        duration_cycles);
    motion_timing_gap_context_record_rx(
        kind,
        byte_count,
        duration_cycles);
}

void motion_timing_diagnostics_record_tx(
    motion_timing_tx_kind_t kind,
    uint32_t byte_count,
    uint32_t status,
    motion_timing_mark_t start)
{
    uint32_t duration_cycles;

    if (!motion_timing_is_recording())
    {
        return;
    }

    duration_cycles = motion_timing_measure_delta(start);
    motion_timing_record_tx_sample(
        &motion_timing_report,
        kind,
        byte_count,
        (status == 0U) || (status == 1U)
            ? MOTION_TIMING_STATUS_OK
            : MOTION_TIMING_STATUS_ERROR,
        duration_cycles);
    /* Preserve the wire-independent enqueue result in the fixed report. */
    if (kind < MOTION_TIMING_TX_COUNT)
    {
        motion_timing_report.tx[kind].last_status = status;
    }
    motion_timing_gap_context_record_tx(
        kind,
        byte_count,
        duration_cycles);
}

static void motion_timing_diagnostics_record_motion_span(
    volatile motion_timing_distribution_t *distribution,
    motion_timing_mark_t start)
{
    if (motion_timing_is_recording())
    {
        motion_timing_distribution_record_cycles(
            distribution,
            motion_timing_measure_delta(start));
    }
}

void motion_timing_diagnostics_record_generator_advance(
    motion_timing_mark_t start)
{
    motion_timing_diagnostics_record_motion_span(
        &motion_timing_report.motion.generator_advance_cycles,
        start);
}

void motion_timing_diagnostics_record_sample(
    motion_timing_mark_t start)
{
    motion_timing_diagnostics_record_motion_span(
        &motion_timing_report.motion.sample_cycles,
        start);
}

void motion_timing_diagnostics_record_apply(
    motion_timing_mark_t start)
{
    motion_timing_diagnostics_record_motion_span(
        &motion_timing_report.motion.apply_cycles,
        start);
}

void motion_timing_diagnostics_motion_tick_begin(uint32_t elapsed_ms)
{
    const motion_timing_mark_t mark = motion_timing_read_cycles();
    uint32_t interval_cycles = 0U;

    if (!motion_timing_is_recording())
    {
        return;
    }

    motion_timing_increment_saturated(
        &motion_timing_report.motion.accepted_tick_count);
    motion_timing_report.motion.last_elapsed_ms = elapsed_ms;
    motion_timing_distribution_record(
        &motion_timing_report.motion.requested_elapsed_ms,
        elapsed_ms);
    if (have_motion_tick)
    {
        interval_cycles = motion_timing_measure_delta(motion_tick_mark);
        if (interval_cycles >
            motion_timing_report.motion.actual_interval_cycles.worst_interval_value)
        {
            motion_timing_capture_worst_gap_context(
                interval_cycles,
                elapsed_ms);
        }
        motion_timing_record_motion_interval_cycles(
            &motion_timing_report,
            interval_cycles,
            motion_timing_report.system_core_clock_hz);
    }
    motion_timing_clear_gap_context();
    motion_tick_mark = mark;
    motion_tick_start_mark = mark;
    have_motion_tick = true;
}

void motion_timing_diagnostics_motion_tick_end(uint32_t result)
{
    uint32_t duration_cycles;

    if (!motion_timing_is_recording() || !have_motion_tick)
    {
        return;
    }

    duration_cycles = motion_timing_measure_delta(motion_tick_start_mark);
    motion_timing_distribution_record_cycles(
        &motion_timing_report.motion.tick_duration_cycles,
        duration_cycles);
    previous_tick_duration_cycles = duration_cycles;
    if (result == MOTION_TIMING_STATUS_OK)
    {
        motion_timing_increment_saturated(
            &motion_timing_report.motion.result_ok_count);
    }
    else
    {
        motion_timing_increment_saturated(
            &motion_timing_report.motion.result_error_count);
    }
}

void motion_timing_diagnostics_record_uart_enqueue(
    motion_timing_tx_kind_t kind,
    uint32_t event)
{
    if (!motion_timing_is_recording() ||
        (kind >= MOTION_TIMING_TX_COUNT))
    {
        return;
    }

    switch (event)
    {
        case MOTION_TIMING_UART_EVENT_ENQUEUED:
            motion_timing_increment_saturated(
                &motion_timing_report.uart_transport.enqueued_count[kind]);
            break;

        case MOTION_TIMING_UART_EVENT_COALESCED:
            motion_timing_increment_saturated(
                &motion_timing_report.uart_transport.coalesced_count[kind]);
            break;

        case MOTION_TIMING_UART_EVENT_REJECTED:
            motion_timing_increment_saturated(
                &motion_timing_report.uart_transport.rejected_count[kind]);
            break;

        default:
            motion_timing_increment_saturated(
                &motion_timing_report.diagnostic_invalid_count);
            break;
    }
}

void motion_timing_diagnostics_record_uart_completed(
    motion_timing_tx_kind_t kind)
{
    if (motion_timing_is_recording() &&
        (kind < MOTION_TIMING_TX_COUNT))
    {
        motion_timing_increment_saturated(
            &motion_timing_report.uart_transport.completed_count[kind]);
    }
}

void motion_timing_diagnostics_record_uart_dropped(
    motion_timing_tx_kind_t kind)
{
    if (motion_timing_is_recording() &&
        (kind < MOTION_TIMING_TX_COUNT))
    {
        motion_timing_increment_saturated(
            &motion_timing_report.uart_transport.dropped_count[kind]);
    }
}

void motion_timing_diagnostics_record_uart_queue_full(
    bool control_queue)
{
    if (!motion_timing_is_recording())
    {
        return;
    }

    if (control_queue)
    {
        motion_timing_increment_saturated(
            &motion_timing_report.uart_transport.control_queue_full_count);
    }
    else
    {
        motion_timing_increment_saturated(
            &motion_timing_report.uart_transport.telemetry_queue_full_count);
    }
}

void motion_timing_diagnostics_record_uart_start_busy(void)
{
    if (motion_timing_is_recording())
    {
        motion_timing_increment_saturated(
            &motion_timing_report.uart_transport.start_busy_count);
    }
}

void motion_timing_diagnostics_record_uart_start_error(void)
{
    if (motion_timing_is_recording())
    {
        motion_timing_increment_saturated(
            &motion_timing_report.uart_transport.start_error_count);
    }
}

void motion_timing_diagnostics_record_uart_error(
    bool rx_rearm_required)
{
    if (!motion_timing_is_recording())
    {
        return;
    }

    motion_timing_increment_saturated(
        &motion_timing_report.uart_transport.uart_error_count);
    if (rx_rearm_required)
    {
        motion_timing_increment_saturated(
            &motion_timing_report.uart_transport.rx_error_count);
    }
}

void motion_timing_diagnostics_record_uart_rearm(
    uint32_t result)
{
    if (!motion_timing_is_recording())
    {
        return;
    }

    switch (result)
    {
        case MOTION_TIMING_UART_REARM_ATTEMPT:
            motion_timing_increment_saturated(
                &motion_timing_report.uart_transport.rx_rearm_attempt_count);
            break;

        case MOTION_TIMING_UART_REARM_BUSY:
            motion_timing_increment_saturated(
                &motion_timing_report.uart_transport.rx_rearm_busy_count);
            break;

        case MOTION_TIMING_UART_REARM_ERROR:
            motion_timing_increment_saturated(
                &motion_timing_report.uart_transport.rx_rearm_error_count);
            break;

        default:
            motion_timing_increment_saturated(
                &motion_timing_report.diagnostic_invalid_count);
            break;
    }
}

void motion_timing_diagnostics_record_uart_unexpected_callback(void)
{
    if (motion_timing_is_recording())
    {
        motion_timing_increment_saturated(
            &motion_timing_report.uart_transport.unexpected_callback_count);
    }
}

void motion_timing_diagnostics_record_uart_high_water(
    uint32_t high_water_mark)
{
    if (motion_timing_is_recording() &&
        (high_water_mark >
         motion_timing_report.uart_transport.high_water_mark))
    {
        motion_timing_report.uart_transport.high_water_mark = high_water_mark;
    }
}

void motion_timing_diagnostics_record_uart_busy_recovery(void)
{
    if (motion_timing_is_recording())
    {
        motion_timing_increment_saturated(
            &motion_timing_report.uart_transport.busy_recovery_count);
    }
}

void motion_timing_diagnostics_record_uart_reinitialization(void)
{
    if (motion_timing_is_recording())
    {
        motion_timing_increment_saturated(
            &motion_timing_report.uart_transport.reinitialization_count);
    }
}

#if defined(ROBOBEETLE_MOTION_TIMING_HOST_TEST)
void motion_timing_diagnostics_host_set_cycles(uint32_t cycles)
{
    motion_timing_host_cycles = cycles;
}
#endif

#endif
