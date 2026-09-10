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
static leak_sensor_t leak_sensor;
static leak_sensor_stm32_t leak_sensor_reader;
static leak_telemetry_policy_t leak_telemetry_policy;
static imu_telemetry_policy_t imu_telemetry_policy;
static telemetry_scheduler_t telemetry_scheduler;
static jy901s_parser_t jy901s_parser;
static volatile uint32_t jy901s_last_valid_frame_ms;

static bool protocol_send_ack(
    uint16_t request_sequence,
    uint8_t request_type,
    rbp2_result_t result);

static bool protocol_send_leak_status(
    leak_sensor_state_t state);

static bool protocol_send_imu_snapshot(void);

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
                uint32_t now_ms = 0U;

                ++protocol_good_frames;

                if ((frame.type == RBP2_MSG_HEARTBEAT) &&
                    (frame.payload_length == 4U))
                {
                    now_ms = HAL_GetTick();
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

                if (ack_sent && outcome.heartbeat_accepted)
                {
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

                    switch (telemetry_scheduler_select(
                                &telemetry_scheduler,
                                leak_due,
                                imu_due))
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

                        case TELEMETRY_SLOT_NONE:
                        default:
                            break;
                    }
                }
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
        return uart_transport_stm32_transmit(
            wire,
            (uint16_t)wire_length) == HAL_OK;
    }

    return false;
}

static bool protocol_send_leak_status(
    leak_sensor_state_t state)
{
    uint8_t payload[1];
    uint8_t wire[RBP2_MAX_WIRE_SIZE];

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

    return uart_transport_stm32_transmit(
               wire,
               (uint16_t)wire_length) == HAL_OK;
}

static bool protocol_send_imu_snapshot(void)
{
    jy901s_imu_state_t state;
    jy901s_parser_stats_t parser_stats;
    jy901s_transport_stm32_diagnostics_t transport_stats;
    jy901s_imu_telemetry_diagnostics_t transport_diagnostics;
    uint8_t payload[JY901S_IMU_TELEMETRY_PAYLOAD_LENGTH];
    uint8_t wire[RBP2_MAX_WIRE_SIZE];

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

    return uart_transport_stm32_transmit(
               wire,
               (uint16_t)wire_length) == HAL_OK;
}

void app_main_init(
    UART_HandleTypeDef *uart,
    UART_HandleTypeDef *jy901s_uart,
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
    protocol_dispatcher_init(
        &protocol_dispatcher,
        &servo_service,
        &safety_supervisor);
    uart_transport_stm32_init(uart);
    jy901s_parser_init(&jy901s_parser);
    imu_telemetry_policy_init(&imu_telemetry_policy);
    telemetry_scheduler_init(&telemetry_scheduler);
    jy901s_last_valid_frame_ms = 0U;
    jy901s_transport_stm32_init(jy901s_uart);
}

void app_main_process(void)
{
    uint8_t byte;

    jy901s_transport_stm32_poll();

    leak_sensor_update_from_gpio_level(
        &leak_sensor,
        leak_sensor_stm32_read_level(
            &leak_sensor_reader));

    while (uart_transport_stm32_pop(&byte))
    {
        protocol_feed_byte(byte);
    }

    while (jy901s_transport_stm32_pop(&byte))
    {
        jy901s_parser_event_t event =
            jy901s_parser_feed_byte(&jy901s_parser, byte);

        if (event != JY901S_PARSER_EVENT_NONE)
        {
            jy901s_last_valid_frame_ms = HAL_GetTick();
        }
    }

    if (safety_supervisor_process(
            &safety_supervisor,
            HAL_GetTick()))
    {
        /*
         * Fail-safe:
         * 上位机失联，立即停止所有已实现执行器。
         */
        servo_service_disable_all(&servo_service);

        protocol_dispatcher_invalidate_action_cache(
            &protocol_dispatcher);
    }
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
