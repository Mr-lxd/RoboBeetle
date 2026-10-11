#include "app_main.h"

#include "protocol_dispatcher.h"
#include "rb_protocol_v2.h"
#include "telemetry_scheduler.h"
#include "imu_telemetry_policy.h"
#include "safety_supervisor.h"
#include "leak_sensor.h"
#include "leak_telemetry_policy.h"
#include "leak_sensor_stm32.h"
#include "servo_driver_stm32.h"
#include "servo_service.h"
#include "jy901s_parser.h"
#include "jy901s_telemetry.h"
#include "jy901s_transport_stm32.h"
#include "uart_transport_stm32.h"
#include "depth_parser.h"
#include "depth_telemetry.h"
#include "depth_transport_stm32.h"
#include "motion_manager.h"
#include "motion_state_sampler.h"
#include "cpg_gait_generator.h"
#include "experimental_flex_gait_generator.h"
#include "simple_gait_generator.h"
#include "motion_timing_diagnostics.h"
#if defined(JY901S_ONESHOT_CONFIG_20HZ) && JY901S_ONESHOT_CONFIG_20HZ
#include "jy901s_oneshot_config.h"
#endif
#if defined(ROBOBEETLE_CPG_TARGET_BENCHMARK) && \
    ROBOBEETLE_CPG_TARGET_BENCHMARK
#include "cpg_target_benchmark.h"
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>

#ifndef MOTION_DEFAULT_GAIT_BACKEND_CPG
/* Normal bench image: validate the source-compatible CPG backend. */
#define MOTION_DEFAULT_GAIT_BACKEND_CPG 1
#endif

static uint8_t protocol_wire_buffer[
    RBP2_MAX_WIRE_SIZE];

static uint16_t protocol_wire_length = 0U;

static uint8_t protocol_drop_until_delimiter = 0U;

static uint16_t protocol_tx_sequence = 0U;

/* Telemetry has its own sequence space so existing ACK sequence behavior
 * remains unchanged for every command. */
static uint16_t protocol_telemetry_sequence = 0U;

#define LEAK_TELEMETRY_REFRESH_INTERVAL_MS 500U

/* 下面几个主要用于 bring-up / debug */

static volatile uint32_t protocol_good_frames = 0U;

static volatile uint32_t protocol_bad_frames = 0U;

static volatile uint32_t heartbeat_count = 0U;

static volatile uint32_t last_host_uptime_ms = 0U;

static protocol_dispatcher_t protocol_dispatcher;
static safety_supervisor_t safety_supervisor;
static servo_driver_stm32_t servo_driver;
static servo_service_t servo_service;
static simple_gait_generator_t simple_gait_generator;
static cpg_gait_generator_t cpg_gait_generator;
static experimental_flex_gait_generator_t experimental_flex_gait_generator;
static motion_manager_t motion_manager;
static leak_sensor_t leak_sensor;
static leak_sensor_stm32_t leak_sensor_reader;
static leak_telemetry_policy_t leak_telemetry_policy;
static imu_telemetry_policy_t imu_telemetry_policy;
static depth_telemetry_policy_t depth_telemetry_policy;
static telemetry_scheduler_t telemetry_scheduler;
static jy901s_parser_t jy901s_parser;
static depth_parser_t depth_parser;
static volatile uint32_t jy901s_last_valid_frame_ms;
#if !MOTION_TIMING_REDUCED_TELEMETRY_ACTIVE
static motion_state_sampler_t motion_state_sampler;
static uint32_t jy901s_last_angle_ms;
#endif

static bool protocol_send_cpg_snapshot(uint16_t request_sequence);

static bool protocol_send_ack(
    uint16_t request_sequence,
    uint8_t request_type,
    rbp2_result_t result);

static bool protocol_tx_result_accepted(
    uart_tx_enqueue_result_t result)
{
    return (result == UART_TX_ENQUEUED) ||
           (result == UART_TX_COALESCED);
}

#if !MOTION_TIMING_REDUCED_TELEMETRY_ACTIVE
static bool protocol_send_leak_status(
    leak_sensor_state_t state);

static bool protocol_send_imu_snapshot(void);

static bool protocol_send_depth_snapshot(void);
#endif

static void app_main_apply_safety_stop(void)
{
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    motion_timing_diagnostics_freeze(
        MOTION_TIMING_TERMINATION_SAFETY_STOP);
#endif
    motion_manager_stop_immediate(&motion_manager);
    servo_service_disable_all(&servo_service);
    protocol_dispatcher_invalidate_action_cache(
        &protocol_dispatcher);
}

/*
 * HAL_TIM_IRQHandler clears the CC flag and invokes this callback after the
 * PWM1 compare/falling edge.  Keep the callback at the driver boundary: the
 * ISR performs only the pending-channel finalizer and never enters Motion or
 * Safety state machines.
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    servo_driver_stm32_handle_timer_compare(
        &servo_driver,
        htim,
        htim == NULL ? 0U : htim->Channel);
}

static void protocol_feed_byte(
    uint8_t byte)
{
    /*
     * 0x00 是 COBS frame delimiter
     */
    if (byte == 0U)
    {
        if ((protocol_drop_until_delimiter == 0U) &&
            (protocol_wire_length > 0U))
        {
            rbp2_frame_t frame;

            rbp2_status_t status =
                rbp2_decode_wire(
                    protocol_wire_buffer,
                    protocol_wire_length,
                    &frame);

            if (status == RBP2_OK)
            {
                protocol_dispatcher_outcome_t outcome;
                const uint32_t now_ms = HAL_GetTick();

                ++protocol_good_frames;

                /* Reject stale actuator frames before dispatching them. A
                 * Heartbeat is intentionally allowed to refresh liveness
                 * before this guard is applied to the next command. */
                if ((frame.type != RBP2_MSG_HEARTBEAT) &&
                    safety_supervisor_process(
                        &safety_supervisor,
                        now_ms))
                {
                    app_main_apply_safety_stop();
                    motion_manager.stop_reason = MOTION_STOP_REASON_LINK_LOST;
                }

                outcome = protocol_dispatcher_handle(
                    &protocol_dispatcher,
                    &frame,
                    now_ms);

                if (outcome.count_bad_frame)
                {
                    ++protocol_bad_frames;
                }

                if (outcome.heartbeat_accepted)
                {
                    last_host_uptime_ms =
                        outcome.heartbeat_uptime_ms;
                    ++heartbeat_count;
                }

                const bool ack_sent = protocol_send_ack(
                    frame.sequence,
                    frame.type,
                    outcome.result);

                if (ack_sent && outcome.cpg_snapshot)
                {
                    (void)protocol_send_cpg_snapshot(frame.sequence);
                }
#if MOTION_TIMING_REDUCED_TELEMETRY_ACTIVE
                (void)ack_sent;
#else
                if (ack_sent && outcome.heartbeat_accepted)
                {
                    motion_state_sampler_begin_batch(&motion_state_sampler);
                    const leak_sensor_state_t state =
                        leak_sensor_state(&leak_sensor);

                    const bool leak_due =
                        leak_telemetry_policy_should_publish(
                            &leak_telemetry_policy,
                            state,
                            now_ms,
                            LEAK_TELEMETRY_REFRESH_INTERVAL_MS);
                    const bool imu_due =
                        imu_telemetry_policy_should_publish(
                            &imu_telemetry_policy,
                            now_ms,
                            JY901S_IMU_TELEMETRY_INTERVAL_MS);
                    depth_parser_stats_t depth_stats;

                    depth_parser_get_stats(&depth_parser, &depth_stats);

                    /* Depth is published on its own track: one frame per
                     * accepted Heartbeat when a new sample is available. The
                     * scheduler then picks at most one Leak/IMU frame, so a
                     * Heartbeat is followed by at most two telemetry frames
                     * (the telemetry TX queue holds two). The ACK is already
                     * enqueued on the control queue, which always drains
                     * first. */
                    if (depth_telemetry_policy_is_due(
                            &depth_telemetry_policy,
                            now_ms,
                            depth_stats.valid_line_count) &&
                        protocol_send_depth_snapshot())
                    {
                        depth_telemetry_policy_mark_success(
                            &depth_telemetry_policy,
                            now_ms,
                            depth_stats.valid_line_count);
                    }

                    switch (telemetry_scheduler_select(
                                &telemetry_scheduler,
                                leak_due,
                                imu_due,
                                false))
                    {
                        case TELEMETRY_SLOT_LEAK_STATUS:
                            if (protocol_send_leak_status(state))
                            {
                                leak_telemetry_policy_mark_published(
                                    &leak_telemetry_policy,
                                    state,
                                    now_ms);
                                telemetry_scheduler_mark_success(
                                    &telemetry_scheduler,
                                    TELEMETRY_SLOT_LEAK_STATUS);
                            }
                            break;

                        case TELEMETRY_SLOT_IMU_SNAPSHOT:
                            if (protocol_send_imu_snapshot())
                            {
                                imu_telemetry_policy_mark_published(
                                    &imu_telemetry_policy,
                                    now_ms);
                                telemetry_scheduler_mark_success(
                                    &telemetry_scheduler,
                                    TELEMETRY_SLOT_IMU_SNAPSHOT);
                            }
                            break;

                        case TELEMETRY_SLOT_DEPTH_SNAPSHOT:
                        case TELEMETRY_SLOT_NONE:
                        default:
                            break;
                    }
                }
#endif
            }
            else
            {
                ++protocol_bad_frames;
            }
        }

        /*
         * 收到 delimiter 后重新同步
         */
        protocol_wire_length = 0U;
        protocol_drop_until_delimiter = 0U;

        return;
    }

    /*
     * 如果之前已经溢出，
     * 就一直丢弃到下一个 0x00。
     */
    if (protocol_drop_until_delimiter != 0U)
    {
        return;
    }

    if (protocol_wire_length <
        sizeof(protocol_wire_buffer))
    {
        protocol_wire_buffer[
            protocol_wire_length++] = byte;
    }
    else
    {
        /*
         * 当前帧太长，认为损坏。
         * 不解析尾巴，等待下一个 delimiter。
         */
        protocol_wire_length = 0U;
        protocol_drop_until_delimiter = 1U;

        ++protocol_bad_frames;
    }
}

static bool protocol_send_ack(
    uint16_t request_sequence,
    uint8_t request_type,
    rbp2_result_t result)
{
    uint8_t payload[4];
    uint8_t wire[RBP2_MAX_WIRE_SIZE];
    uart_tx_enqueue_result_t enqueue_result;

    payload[0] =
        (uint8_t)(request_sequence & 0xFFU);

    payload[1] =
        (uint8_t)(
            (request_sequence >> 8U) & 0xFFU);

    payload[2] = request_type;
    payload[3] = (uint8_t)result;

    size_t wire_length =
        rbp2_encode_wire(
            RBP2_MSG_ACK,
            protocol_tx_sequence++,
            payload,
            sizeof(payload),
            wire,
            sizeof(wire));

    if (wire_length > 0U)
    {
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
        const motion_timing_mark_t timing_start =
            motion_timing_diagnostics_mark();
#endif

        enqueue_result = uart_transport_stm32_enqueue(
            wire,
            (uint16_t)wire_length,
            UART_TX_MESSAGE_ACK);
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
        motion_timing_diagnostics_record_tx(
            MOTION_TIMING_TX_ACK,
            (uint32_t)wire_length,
            (uint32_t)enqueue_result,
            timing_start);
#endif
        return protocol_tx_result_accepted(enqueue_result);
    }

    return false;
}

static bool protocol_send_cpg_snapshot(uint16_t request_sequence)
{
    uint8_t payload[CPG_PARAMETERS_SNAPSHOT_SIZE], wire[RBP2_MAX_WIRE_SIZE];
    const size_t payload_size = protocol_dispatcher_cpg_snapshot(
        &protocol_dispatcher, request_sequence, payload, sizeof payload);
    if (payload_size == 0)
        return false;
    const size_t wire_size =
        rbp2_encode_wire(RBP2_MSG_CPG_PARAMETERS_SNAPSHOT, protocol_telemetry_sequence++, payload,
                         (uint16_t)payload_size, wire, sizeof wire);
    return wire_size != 0 && protocol_tx_result_accepted(uart_transport_stm32_enqueue(
                                 wire, (uint16_t)wire_size, UART_TX_MESSAGE_CPG));
}

#if !MOTION_TIMING_REDUCED_TELEMETRY_ACTIVE
static uint16_t motion_state_age(uint32_t now_ms, uint32_t then_ms)
{
    const uint32_t age = now_ms - then_ms;
    return age > 65534U ? 65534U : (uint16_t)age;
}

static int16_t motion_state_fixed(float value, float scale)
{
    const float scaled = value * scale;
    if (scaled >= 32767.0f) return 32767;
    if (scaled <= -32768.0f) return -32768;
    return (int16_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

/* This is a read-only snapshot. No generator advance/sample is performed. */
static void motion_state_capture(uint32_t now_ms)
{
    const jy901s_imu_state_t *imu = &jy901s_parser.state;
    rb_motion_state_sample_t sample = {0};
    const double cycle = 6.28318530717958647692;
    double phase;
    sample.control_mode = MOTION_CONTROL_DISCRETE;
    sample.stop_reason = motion_manager.stop_reason;
    sample.parameter_version = motion_manager.cpg_parameter_version;
    sample.mcu_ms = now_ms;
    sample.gyro_tenth_dps[0] = motion_state_fixed(imu->gyro_x_dps, 10.0f);
    sample.gyro_tenth_dps[1] = motion_state_fixed(imu->gyro_y_dps, 10.0f);
    sample.gyro_tenth_dps[2] = motion_state_fixed(imu->gyro_z_dps, 10.0f);
    sample.gyro_valid = true;
    sample.angle_valid = imu->angle_valid;
    if (sample.angle_valid)
    {
        sample.roll_centidegrees = motion_state_fixed(imu->angle_roll_deg, 100.0f);
        sample.pitch_centidegrees = motion_state_fixed(imu->angle_pitch_deg, 100.0f);
    }
    sample.angle_age_ms = sample.angle_valid
        ? motion_state_age(now_ms, jy901s_last_angle_ms) : 65535U;
    sample.backend = (uint8_t)motion_manager.gait_backend;
    sample.state = (uint8_t)motion_manager.state;
    sample.active_mode = (uint8_t)motion_manager.active_mode;
    sample.target_mode = motion_manager.state == MOTION_STATE_STOPPING
        ? MOTION_STOP : motion_manager.transition != MOTION_MANAGER_TRANSITION_NONE
        ? (uint8_t)motion_manager.transition_mode : sample.active_mode;
    sample.coordination = (uint8_t)motion_manager.front_rear_coordination;
    sample.transition = motion_manager.state == MOTION_STATE_STOPPING ||
        motion_manager.transition != MOTION_MANAGER_TRANSITION_NONE;
    sample.phase_valid = motion_manager.state == MOTION_STATE_RUNNING &&
        motion_manager.phase_tick_valid;
    sample.phase_age_ms = sample.phase_valid
        ? motion_state_age(now_ms, motion_manager.last_tick_ms) : 65535U;
    switch (motion_manager.gait_backend)
    {
        case MOTION_GAIT_BACKEND_CPG:
            phase = cpg_gait_generator.core.phase[3];
            double wrapped = fmod(cpg_gait_generator.core.phase[0], cycle);
            if (wrapped < 0)
                wrapped += cycle;
            sample.fr_phase_u16 = (uint16_t)(wrapped * 65536.0 / cycle);
            wrapped = fmod(cpg_gait_generator.core.phase[1], cycle);
            if (wrapped < 0)
                wrapped += cycle;
            sample.rr_phase_u16 = (uint16_t)(wrapped * 65536.0 / cycle);
            wrapped = fmod(cpg_gait_generator.core.phase[2], cycle);
            if (wrapped < 0)
                wrapped += cycle;
            sample.rl_phase_u16 = (uint16_t)(wrapped * 65536.0 / cycle);
            break;
        case MOTION_GAIT_BACKEND_EXPERIMENTAL_FLEX:
            phase = cycle * experimental_flex_gait_generator.phase_ms / 2000.0;
            break;
        default:
            phase = simple_gait_generator.phase_rad;
            break;
    }
    phase = fmod(phase, cycle);
    if (phase < 0.0) phase += cycle;
    sample.phase_u16 = (uint16_t)(phase * 65536.0 / cycle);
    if (motion_manager.gait_backend != MOTION_GAIT_BACKEND_CPG)
    {
        sample.fr_phase_u16 = sample.phase_u16;
        sample.rr_phase_u16 = sample.phase_u16;
        sample.rl_phase_u16 = sample.phase_u16;
    }
    motion_state_sampler_push(&motion_state_sampler, &sample);
}

static void protocol_send_motion_fragment(void)
{
    rb_motion_state_batch_t batch;
    uint8_t payload[RB_MOTION_STATE_MAX_PAYLOAD];
    uint8_t wire[RBP2_MAX_WIRE_SIZE];
    if (uart_transport_stm32_motion_pending())
    {
        return;
    }
    if (!motion_state_sampler_fragment(&motion_state_sampler, HAL_GetTick(), &batch))
    {
        return;
    }
    const size_t payload_length = rb_motion_state_encode(&batch, payload, sizeof payload);
    const size_t wire_length = rbp2_encode_wire(RBP2_MSG_MOTION_STATE_BATCH,
        protocol_telemetry_sequence, payload, (uint16_t)payload_length, wire, sizeof wire);
    if (wire_length != 0U && uart_transport_stm32_enqueue(wire,
            (uint16_t)wire_length, UART_TX_MESSAGE_MOTION) == UART_TX_ENQUEUED)
    {
        ++protocol_telemetry_sequence;
        motion_state_sampler_accept_fragment(&motion_state_sampler);
    }
}

static bool protocol_send_leak_status(
    leak_sensor_state_t state)
{
    uint8_t payload[1];
    uint8_t wire[RBP2_MAX_WIRE_SIZE];
    uart_tx_enqueue_result_t enqueue_result;

    if (!leak_sensor_state_is_valid(state))
    {
        return false;
    }

    payload[0] = (uint8_t)state;

    size_t wire_length =
        rbp2_encode_wire(
            RBP2_MSG_LEAK_STATUS,
            protocol_telemetry_sequence++,
            payload,
            sizeof(payload),
            wire,
            sizeof(wire));

    if (wire_length == 0U)
    {
        return false;
    }

#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    const motion_timing_mark_t timing_start =
        motion_timing_diagnostics_mark();
#endif

    enqueue_result = uart_transport_stm32_enqueue(
        wire,
        (uint16_t)wire_length,
        UART_TX_MESSAGE_LEAK);
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    motion_timing_diagnostics_record_tx(
        MOTION_TIMING_TX_LEAK,
        (uint32_t)wire_length,
        (uint32_t)enqueue_result,
        timing_start);
#endif
    return protocol_tx_result_accepted(enqueue_result);
}

static bool protocol_send_imu_snapshot(void)
{
    jy901s_imu_state_t state;
    jy901s_parser_stats_t parser_stats;
    jy901s_transport_stm32_diagnostics_t transport_stats;
    jy901s_imu_telemetry_diagnostics_t transport_diagnostics;
    uint8_t payload[JY901S_IMU_TELEMETRY_PAYLOAD_LENGTH];
    uint8_t wire[RBP2_MAX_WIRE_SIZE];
    uart_tx_enqueue_result_t enqueue_result;

    jy901s_parser_get_state(&jy901s_parser, &state);
    jy901s_parser_get_stats(&jy901s_parser, &parser_stats);
    jy901s_transport_stm32_get_diagnostics(&transport_stats);

    transport_diagnostics.rx_byte_count =
        transport_stats.rx_byte_count;
    transport_diagnostics.rx_buffer_overflow_count =
        transport_stats.rx_buffer_overflow_count;
    transport_diagnostics.rx_rearm_failure_count =
        transport_stats.rx_rearm_failure_count;
    transport_diagnostics.uart_error_count =
        transport_stats.uart_error_count;

    if (jy901s_imu_telemetry_encode(
            &state,
            &parser_stats,
            &transport_diagnostics,
            payload,
            sizeof payload) == 0U)
    {
        return false;
    }

    const size_t wire_length =
        rbp2_encode_wire(
            RBP2_MSG_IMU_SNAPSHOT,
            protocol_telemetry_sequence++,
            payload,
            JY901S_IMU_TELEMETRY_PAYLOAD_LENGTH,
            wire,
            sizeof wire);

    if (wire_length == 0U)
    {
        return false;
    }

#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    const motion_timing_mark_t timing_start =
        motion_timing_diagnostics_mark();
#endif

    enqueue_result = uart_transport_stm32_enqueue(
        wire,
        (uint16_t)wire_length,
        UART_TX_MESSAGE_IMU);
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    motion_timing_diagnostics_record_tx(
        MOTION_TIMING_TX_IMU,
        (uint32_t)wire_length,
        (uint32_t)enqueue_result,
        timing_start);
#endif
    return protocol_tx_result_accepted(enqueue_result);
}

static bool protocol_send_depth_snapshot(void)
{
    depth_parser_state_t state;
    depth_parser_stats_t parser_stats;
    depth_transport_stm32_diagnostics_t transport_stats;
    depth_telemetry_source_t source;
    depth_telemetry_diagnostics_t diagnostics;
    uint8_t payload[DEPTH_TELEMETRY_PAYLOAD_LENGTH];
    uint8_t wire[RBP2_MAX_WIRE_SIZE];
    uart_tx_enqueue_result_t enqueue_result;
    const uint32_t now_ms = HAL_GetTick();

    depth_parser_get_state(&depth_parser, &state);
    depth_parser_get_stats(&depth_parser, &parser_stats);
    depth_transport_stm32_get_diagnostics(&transport_stats);

    const bool sample_is_current =
        depth_telemetry_sensor_sample_is_current(
            state.depth_valid,
            state.last_valid_sample_ms,
            now_ms);

    source.depth_valid = state.depth_valid && sample_is_current;
    source.temperature_valid = state.temperature_valid && sample_is_current;
    source.depth_mm = source.depth_valid ? state.depth_mm : 0;
    source.temperature_centi_c =
        source.temperature_valid ? state.temperature_centi_c : 0;
    source.sample_age_ms = state.depth_valid
        ? (uint32_t)(now_ms - state.last_valid_sample_ms)
        : 0U;

    diagnostics.rx_byte_count = transport_stats.rx_byte_count;
    diagnostics.valid_line_count = parser_stats.valid_line_count;
    diagnostics.parse_error_count = parser_stats.parse_error_count;
    diagnostics.overlong_line_count = parser_stats.overlong_line_count;
    diagnostics.rx_buffer_overflow_count =
        transport_stats.rx_buffer_overflow_count;
    diagnostics.hard_rearm_failure_count =
        transport_stats.hard_rearm_failure_count;
    diagnostics.uart_error_count = transport_stats.uart_error_count;

    if (depth_telemetry_encode(
            &source,
            &diagnostics,
            payload,
            sizeof payload) == 0U)
    {
        return false;
    }

    const size_t wire_length =
        rbp2_encode_wire(
            RBP2_MSG_DEPTH_SNAPSHOT,
            protocol_telemetry_sequence++,
            payload,
            DEPTH_TELEMETRY_PAYLOAD_LENGTH,
            wire,
            sizeof wire);

    if (wire_length == 0U)
    {
        return false;
    }

#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    const motion_timing_mark_t timing_start =
        motion_timing_diagnostics_mark();
#endif

    enqueue_result = uart_transport_stm32_enqueue(
        wire,
        (uint16_t)wire_length,
        UART_TX_MESSAGE_DEPTH);
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    motion_timing_diagnostics_record_tx(
        MOTION_TIMING_TX_DEPTH,
        (uint32_t)wire_length,
        (uint32_t)enqueue_result,
        timing_start);
#endif
    return protocol_tx_result_accepted(enqueue_result);
}
#endif

void app_main_init(
    UART_HandleTypeDef *uart,
    UART_HandleTypeDef *jy901s_uart,
    UART_HandleTypeDef *depth_uart,
    TIM_HandleTypeDef *tim3,
    TIM_HandleTypeDef *tim4,
    GPIO_TypeDef *leak_gpio_port,
    uint16_t leak_gpio_pin)
{
    leak_sensor_init(&leak_sensor);
    leak_telemetry_policy_init(&leak_telemetry_policy);
    leak_sensor_stm32_init(
        &leak_sensor_reader,
        leak_gpio_port,
        leak_gpio_pin);
    safety_supervisor_init(&safety_supervisor);
    servo_driver_stm32_init(
        &servo_driver,
        tim3,
        tim4);
    servo_service_init(
        &servo_service,
        servo_driver_stm32_ops(),
        &servo_driver);
    simple_gait_generator_init(&simple_gait_generator);
    cpg_gait_generator_init(&cpg_gait_generator);
    experimental_flex_gait_generator_init(&experimental_flex_gait_generator);
#if defined(ROBOBEETLE_CPG_TARGET_BENCHMARK) && \
    ROBOBEETLE_CPG_TARGET_BENCHMARK
    cpg_target_benchmark_run(&cpg_gait_generator);
#endif
#if MOTION_DEFAULT_GAIT_BACKEND_CPG
    const motion_gait_backend_t initial_backend = MOTION_GAIT_BACKEND_CPG;
#else
    const motion_gait_backend_t initial_backend =
        MOTION_GAIT_BACKEND_SIMPLE_GAIT;
#endif
    motion_manager_init_with_backends(
        &motion_manager,
        &servo_service,
        &safety_supervisor,
        simple_gait_generator_interface(&simple_gait_generator),
        cpg_gait_generator_interface(&cpg_gait_generator),
        experimental_flex_gait_generator_interface(
            &experimental_flex_gait_generator),
        initial_backend);
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    motion_timing_diagnostics_init(
        (uint32_t)initial_backend);
#endif
    protocol_dispatcher_init(
        &protocol_dispatcher,
        &servo_service,
        &safety_supervisor,
        &motion_manager);
#if defined(JY901S_ONESHOT_CONFIG_20HZ) && JY901S_ONESHOT_CONFIG_20HZ
    /* Maintenance image only; finish before any serial RX transport is armed. */
    HAL_Delay(200U);
    for (size_t command = 0U; command < 4U; ++command)
    {
        (void)HAL_UART_Transmit(
            jy901s_uart,
            jy901s_oneshot_config_commands[command],
            sizeof(jy901s_oneshot_config_commands[command]),
            20U);
        if (command < 3U)
        {
            HAL_Delay(100U);
        }
    }
    HAL_Delay(200U);
#endif
    uart_transport_stm32_init(uart);
    jy901s_parser_init(&jy901s_parser);
    imu_telemetry_policy_init(&imu_telemetry_policy);
    depth_parser_init(&depth_parser);
    depth_telemetry_policy_init(&depth_telemetry_policy);
    telemetry_scheduler_init(&telemetry_scheduler);
    jy901s_last_valid_frame_ms = 0U;
#if !MOTION_TIMING_REDUCED_TELEMETRY_ACTIVE
    motion_state_sampler_init(&motion_state_sampler);
    jy901s_last_angle_ms = 0U;
#endif
    jy901s_transport_stm32_init(jy901s_uart);
    depth_transport_stm32_init(depth_uart);
}

void app_main_process(void)
{
    uint8_t byte;
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    const motion_timing_mark_t app_loop_start =
        motion_timing_diagnostics_loop_begin();
#endif

    jy901s_transport_stm32_poll();
    depth_transport_stm32_poll();

    leak_sensor_update_from_gpio_level(
        &leak_sensor,
        leak_sensor_stm32_read_level(
            &leak_sensor_reader));

    {
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
        const motion_timing_mark_t drain_start =
            motion_timing_diagnostics_mark();
        uint32_t byte_count = 0U;
#endif

        while (uart_transport_stm32_pop(&byte))
        {
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
            ++byte_count;
#endif
            protocol_feed_byte(byte);
        }
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
        motion_timing_diagnostics_record_rx(
            MOTION_TIMING_RX_HOST,
            byte_count,
            byte_count != 0U ? 1U : 0U,
            drain_start);
#endif
    }

    {
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
        const motion_timing_mark_t drain_start =
            motion_timing_diagnostics_mark();
        uint32_t byte_count = 0U;
#endif

        while (jy901s_transport_stm32_pop(&byte))
        {
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
            ++byte_count;
#endif
            jy901s_parser_event_t event =
                jy901s_parser_feed_byte(&jy901s_parser, byte);

            if (event != JY901S_PARSER_EVENT_NONE)
            {
                jy901s_last_valid_frame_ms = HAL_GetTick();
#if !MOTION_TIMING_REDUCED_TELEMETRY_ACTIVE
                if (event == JY901S_PARSER_EVENT_ANGLE)
                {
                    jy901s_last_angle_ms = jy901s_last_valid_frame_ms;
                }
                else if (event == JY901S_PARSER_EVENT_GYRO)
                {
                    motion_state_capture(jy901s_last_valid_frame_ms);
                }
#endif
            }
        }
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
        motion_timing_diagnostics_record_rx(
            MOTION_TIMING_RX_JY901S,
            byte_count,
            byte_count != 0U ? 1U : 0U,
            drain_start);
#endif
    }

    {
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
        const motion_timing_mark_t drain_start =
            motion_timing_diagnostics_mark();
        uint32_t byte_count = 0U;
#endif

        while (depth_transport_stm32_pop(&byte))
        {
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
            ++byte_count;
#endif
            (void)depth_parser_feed_byte(
                &depth_parser,
                byte,
                HAL_GetTick());
        }
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
        motion_timing_diagnostics_record_rx(
            MOTION_TIMING_RX_DEPTH,
            byte_count,
            byte_count != 0U ? 1U : 0U,
            drain_start);
#endif
    }

    /* Sample safety and Motion time after this pass's input dispatch. */
    const uint32_t now_ms = HAL_GetTick();
    if (safety_supervisor_process(
            &safety_supervisor,
            now_ms))
    {
        /* Fail-safe: host loss immediately stops all implemented actuators. */
        app_main_apply_safety_stop();
        motion_manager.stop_reason = MOTION_STOP_REASON_LINK_LOST;
    }
    else
    {
        const motion_manager_result_t motion_result =
            motion_manager_process(&motion_manager, now_ms);

        if ((motion_result == MOTION_MANAGER_RESULT_HOST_NOT_ALIVE) ||
            (motion_result == MOTION_MANAGER_RESULT_HARDWARE_FAILURE))
        {
            app_main_apply_safety_stop();
            motion_manager.stop_reason = motion_result == MOTION_MANAGER_RESULT_HOST_NOT_ALIVE
                                             ? MOTION_STOP_REASON_LINK_LOST
                                             : MOTION_STOP_REASON_NONE;
        }
    }
#if !MOTION_TIMING_REDUCED_TELEMETRY_ACTIVE
    protocol_send_motion_fragment();
#endif
    uart_transport_stm32_process();
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    motion_timing_diagnostics_loop_end(app_loop_start);
#endif
}

void app_main_jy901s_get_state(jy901s_imu_state_t *state)
{
    jy901s_parser_get_state(&jy901s_parser, state);
}

void app_main_jy901s_get_parser_stats(jy901s_parser_stats_t *stats)
{
    jy901s_parser_get_stats(&jy901s_parser, stats);
}

void app_main_jy901s_get_transport_diagnostics(
    jy901s_transport_stm32_diagnostics_t *diagnostics)
{
    jy901s_transport_stm32_get_diagnostics(diagnostics);
}

uint32_t app_main_jy901s_last_valid_frame_ms(void)
{
    return jy901s_last_valid_frame_ms;
}

void app_main_depth_get_state(depth_parser_state_t *state)
{
    depth_parser_get_state(&depth_parser, state);
}

void app_main_depth_get_parser_stats(depth_parser_stats_t *stats)
{
    depth_parser_get_stats(&depth_parser, stats);
}

void app_main_depth_get_transport_diagnostics(
    depth_transport_stm32_diagnostics_t *diagnostics)
{
    depth_transport_stm32_get_diagnostics(diagnostics);
}
