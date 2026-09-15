#ifndef ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS_H
#define ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS
#define ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS 0
#endif

#ifndef ROBOBEETLE_MOTION_TIMING_REDUCED_TELEMETRY
#define ROBOBEETLE_MOTION_TIMING_REDUCED_TELEMETRY 0
#endif

#if ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS
#define MOTION_TIMING_DIAGNOSTICS_ACTIVE 1
#else
#define MOTION_TIMING_DIAGNOSTICS_ACTIVE 0
#endif

#if ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS && \
    ROBOBEETLE_MOTION_TIMING_REDUCED_TELEMETRY
#define MOTION_TIMING_REDUCED_TELEMETRY_ACTIVE 1
#else
#define MOTION_TIMING_REDUCED_TELEMETRY_ACTIVE 0
#endif

#define MOTION_TIMING_HISTOGRAM_BUCKET_COUNT 8U
#define MOTION_TIMING_HISTOGRAM_BUCKET_0_MAX 10U
#define MOTION_TIMING_HISTOGRAM_BUCKET_1_MAX 20U
#define MOTION_TIMING_HISTOGRAM_BUCKET_2_MAX 50U
#define MOTION_TIMING_HISTOGRAM_BUCKET_3_MAX 100U
#define MOTION_TIMING_HISTOGRAM_BUCKET_4_MAX 500U
#define MOTION_TIMING_HISTOGRAM_BUCKET_5_MAX 1000U
#define MOTION_TIMING_HISTOGRAM_BUCKET_6_MAX 5000U
/* Bucket 7 is strictly greater than BUCKET_6_MAX. */
#define MOTION_TIMING_REPORT_MAGIC 0x4D54494DU
#define MOTION_TIMING_REPORT_ABI_VERSION 2U
#define MOTION_TIMING_REPORT_SIZE 1172U

#define MOTION_TIMING_DIAGNOSTIC_FLAG_ENABLED (1U << 0U)
#define MOTION_TIMING_DIAGNOSTIC_FLAG_REDUCED_TELEMETRY (1U << 1U)

#define MOTION_GAIT_BACKEND_SIMPLE_GAIT_VALUE 0U
#define MOTION_GAIT_BACKEND_CPG_VALUE 1U

#define MOTION_TIMING_STATUS_OK 0U
#define MOTION_TIMING_STATUS_ERROR 1U
#define MOTION_TIMING_STATUS_BUSY 2U
#define MOTION_TIMING_STATUS_TIMEOUT 3U

typedef uint32_t motion_timing_mark_t;

typedef struct
{
    uint32_t lo;
    uint32_t hi;
} motion_timing_u64_t;

typedef struct
{
    uint32_t count;
    uint32_t min_value;
    uint32_t max_value;
    motion_timing_u64_t total_value;
    uint32_t worst_interval_value;
    uint32_t histogram[MOTION_TIMING_HISTOGRAM_BUCKET_COUNT];
    uint32_t saturated_count;
} motion_timing_distribution_t;

typedef struct
{
    uint32_t call_count;
    uint32_t nonempty_count;
    uint32_t byte_count;
    motion_timing_distribution_t duration;
} motion_timing_drain_report_t;

typedef struct
{
    uint32_t call_count;
    uint32_t byte_count;
    uint32_t last_status;
    uint32_t ok_count;
    uint32_t error_count;
    uint32_t timeout_count;
    motion_timing_distribution_t duration;
} motion_timing_tx_report_t;

typedef struct
{
    uint32_t accepted_tick_count;
    uint32_t last_elapsed_ms;
    motion_timing_distribution_t requested_elapsed_ms;
    motion_timing_distribution_t actual_interval_cycles;
    motion_timing_distribution_t actual_interval_ms;
    uint32_t gap_gt_10_ms_count;
    uint32_t gap_gt_12_ms_count;
    uint32_t gap_gt_15_ms_count;
    uint32_t gap_gt_20_ms_count;
    uint32_t gap_gt_30_ms_count;
    motion_timing_distribution_t tick_duration_cycles;
    motion_timing_distribution_t generator_advance_cycles;
    motion_timing_distribution_t sample_cycles;
    motion_timing_distribution_t apply_cycles;
    uint32_t result_ok_count;
    uint32_t result_error_count;
} motion_timing_motion_report_t;

typedef enum
{
    MOTION_TIMING_RX_HOST = 0,
    MOTION_TIMING_RX_JY901S,
    MOTION_TIMING_RX_DEPTH,
    MOTION_TIMING_RX_COUNT
} motion_timing_rx_kind_t;

typedef enum
{
    MOTION_TIMING_TX_ACK = 0,
    MOTION_TIMING_TX_LEAK,
    MOTION_TIMING_TX_IMU,
    MOTION_TIMING_TX_DEPTH,
    MOTION_TIMING_TX_COUNT
} motion_timing_tx_kind_t;

typedef struct
{
    uint32_t magic;
    uint32_t abi_version;
    uint32_t report_size;
    uint32_t system_core_clock_hz;
    uint32_t diagnostic_flags;
    uint32_t runtime_backend;
    uint32_t run_marker;
    uint32_t reset_marker;
    motion_timing_distribution_t app_loop_body;
    motion_timing_distribution_t app_loop_interval;
    motion_timing_drain_report_t rx_drain[MOTION_TIMING_RX_COUNT];
    motion_timing_tx_report_t tx[MOTION_TIMING_TX_COUNT];
    motion_timing_motion_report_t motion;
    uint32_t diagnostic_saturation_count;
    uint32_t diagnostic_counter_wrap_count;
    uint32_t diagnostic_invalid_count;
} motion_timing_report_t;

_Static_assert(sizeof(motion_timing_u64_t) == 8U,
               "motion timing u64 ABI size changed");
_Static_assert(sizeof(motion_timing_distribution_t) == 60U,
               "motion timing distribution ABI size changed");
_Static_assert(sizeof(motion_timing_drain_report_t) == 72U,
               "motion timing drain ABI size changed");
_Static_assert(sizeof(motion_timing_tx_report_t) == 84U,
               "motion timing TX ABI size changed");
_Static_assert(sizeof(motion_timing_motion_report_t) == 456U,
               "motion timing Motion ABI size changed");
_Static_assert(offsetof(motion_timing_report_t, magic) == 0U,
               "motion timing magic ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, abi_version) == 4U,
               "motion timing version ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, report_size) == 8U,
               "motion timing size ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, system_core_clock_hz) == 12U,
               "motion timing clock ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, diagnostic_flags) == 16U,
               "motion timing flags ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, runtime_backend) == 20U,
               "motion timing backend ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, run_marker) == 24U,
               "motion timing run marker ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, reset_marker) == 28U,
               "motion timing reset marker ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, app_loop_body) == 32U,
               "motion timing first counter ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, rx_drain) == 152U,
               "motion timing RX ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, tx) == 368U,
               "motion timing TX ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t, motion) == 704U,
               "motion timing Motion ABI offset changed");
_Static_assert(offsetof(motion_timing_motion_report_t,
                        gap_gt_10_ms_count) == 188U,
               "motion timing gap ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t,
                        diagnostic_saturation_count) == 1160U,
               "motion timing saturation ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t,
                        diagnostic_counter_wrap_count) == 1164U,
               "motion timing counter-wrap ABI offset changed");
_Static_assert(offsetof(motion_timing_report_t,
                        diagnostic_invalid_count) == 1168U,
               "motion timing invalid ABI offset changed");
_Static_assert(sizeof(motion_timing_report_t) == MOTION_TIMING_REPORT_SIZE,
               "motion timing report ABI size changed");

extern volatile motion_timing_report_t motion_timing_report;

uint32_t motion_timing_dwt_delta(
    uint32_t start,
    uint32_t finish);

void motion_timing_report_initialize(
    volatile motion_timing_report_t *report,
    uint32_t system_core_clock_hz,
    uint32_t diagnostic_flags,
    uint32_t runtime_backend,
    uint32_t reset_marker);

bool motion_timing_report_is_valid(
    const volatile motion_timing_report_t *report);

void motion_timing_distribution_record(
    volatile motion_timing_distribution_t *distribution,
    uint32_t value);

uint32_t motion_timing_distribution_histogram_total(
    const volatile motion_timing_distribution_t *distribution);

void motion_timing_record_rx_sample(
    volatile motion_timing_report_t *report,
    motion_timing_rx_kind_t kind,
    uint32_t byte_count,
    uint32_t nonempty,
    uint32_t duration_cycles);

void motion_timing_record_tx_sample(
    volatile motion_timing_report_t *report,
    motion_timing_tx_kind_t kind,
    uint32_t byte_count,
    uint32_t status,
    uint32_t duration_cycles);

void motion_timing_record_motion_interval_ms(
    volatile motion_timing_report_t *report,
    uint32_t interval_ms);

void motion_timing_record_motion_interval_cycles(
    volatile motion_timing_report_t *report,
    uint32_t interval_cycles,
    uint32_t system_core_clock_hz);

#if ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS

void motion_timing_diagnostics_init(uint32_t runtime_backend);
void motion_timing_diagnostics_set_runtime_backend(uint32_t runtime_backend);
motion_timing_mark_t motion_timing_diagnostics_mark(void);
motion_timing_mark_t motion_timing_diagnostics_loop_begin(void);
void motion_timing_diagnostics_loop_end(motion_timing_mark_t start);
void motion_timing_diagnostics_record_rx(
    motion_timing_rx_kind_t kind,
    uint32_t byte_count,
    uint32_t nonempty,
    motion_timing_mark_t start);
void motion_timing_diagnostics_record_tx(
    motion_timing_tx_kind_t kind,
    uint32_t byte_count,
    uint32_t status,
    motion_timing_mark_t start);
void motion_timing_diagnostics_record_generator_advance(
    motion_timing_mark_t start);
void motion_timing_diagnostics_record_sample(
    motion_timing_mark_t start);
void motion_timing_diagnostics_record_apply(
    motion_timing_mark_t start);
void motion_timing_diagnostics_motion_tick_begin(uint32_t elapsed_ms);
void motion_timing_diagnostics_motion_tick_end(uint32_t result);

#else

static inline void motion_timing_diagnostics_init(
    uint32_t runtime_backend)
{
    (void)runtime_backend;
}

static inline motion_timing_mark_t motion_timing_diagnostics_mark(void)
{
    return 0U;
}

static inline void motion_timing_diagnostics_set_runtime_backend(
    uint32_t runtime_backend)
{
    (void)runtime_backend;
}

static inline motion_timing_mark_t motion_timing_diagnostics_loop_begin(void)
{
    return 0U;
}

static inline void motion_timing_diagnostics_loop_end(
    motion_timing_mark_t start)
{
    (void)start;
}

static inline void motion_timing_diagnostics_record_rx(
    motion_timing_rx_kind_t kind,
    uint32_t byte_count,
    uint32_t nonempty,
    motion_timing_mark_t start)
{
    (void)kind;
    (void)byte_count;
    (void)nonempty;
    (void)start;
}

static inline void motion_timing_diagnostics_record_tx(
    motion_timing_tx_kind_t kind,
    uint32_t byte_count,
    uint32_t status,
    motion_timing_mark_t start)
{
    (void)kind;
    (void)byte_count;
    (void)status;
    (void)start;
}

static inline void motion_timing_diagnostics_record_generator_advance(
    motion_timing_mark_t start)
{
    (void)start;
}

static inline void motion_timing_diagnostics_record_sample(
    motion_timing_mark_t start)
{
    (void)start;
}

static inline void motion_timing_diagnostics_record_apply(
    motion_timing_mark_t start)
{
    (void)start;
}

static inline void motion_timing_diagnostics_motion_tick_begin(
    uint32_t elapsed_ms)
{
    (void)elapsed_ms;
}

static inline void motion_timing_diagnostics_motion_tick_end(
    uint32_t result)
{
    (void)result;
}

#endif

#endif /* ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS_H */
