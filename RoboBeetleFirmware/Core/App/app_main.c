#include "app_main.h"

#include "protocol_dispatcher.h"
#include "rb_protocol_v2.h"
#include "safety_supervisor.h"
#include "leak_sensor.h"
#include "leak_sensor_stm32.h"
#include "servo_driver_stm32.h"
#include "servo_service.h"
#include "uart_transport_stm32.h"

static uint8_t protocol_wire_buffer[
    RBP2_MAX_WIRE_SIZE];

static uint16_t protocol_wire_length = 0U;

static uint8_t protocol_drop_until_delimiter = 0U;

static uint16_t protocol_tx_sequence = 0U;

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

static void protocol_send_ack(
    uint16_t request_sequence,
    uint8_t request_type,
    rbp2_result_t result);

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

                protocol_send_ack(
                    frame.sequence,
                    frame.type,
                    outcome.result);
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

static void protocol_send_ack(
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
        (void)uart_transport_stm32_transmit(
            wire,
            (uint16_t)wire_length);
    }
}

void app_main_init(
    UART_HandleTypeDef *uart,
    TIM_HandleTypeDef *tim3,
    TIM_HandleTypeDef *tim4,
    GPIO_TypeDef *leak_gpio_port,
    uint16_t leak_gpio_pin)
{
    leak_sensor_init(&leak_sensor);
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
}

void app_main_process(void)
{
    uint8_t byte;

    leak_sensor_update_from_gpio_level(
        &leak_sensor,
        leak_sensor_stm32_read_level(
            &leak_sensor_reader));

    while (uart_transport_stm32_pop(&byte))
    {
        protocol_feed_byte(byte);
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
