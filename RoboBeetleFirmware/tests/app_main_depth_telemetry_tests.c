#include "app_main.h"
#include "depth_telemetry.h"
#include "rb_protocol_v2.h"
#include "stm32f4xx_hal.h"
#include "uart_transport_stm32.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_MAX_TX_FRAMES 24U

static int failures = 0;
static uint32_t test_tick = 0U;
static uint8_t *uart1_receive_destination = NULL;
static uint8_t *uart3_receive_destination = NULL;
static uint8_t *uart6_receive_destination = NULL;
static uint8_t tx_wires[TEST_MAX_TX_FRAMES][RBP2_MAX_WIRE_SIZE];
static uint16_t tx_lengths[TEST_MAX_TX_FRAMES];
static size_t tx_frame_count = 0U;

static void expect(bool condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

static uint8_t **receive_destination_for(
    UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return NULL;
    }

    if (huart->Instance == USART1)
    {
        return &uart1_receive_destination;
    }

    if (huart->Instance == USART3)
    {
        return &uart3_receive_destination;
    }

    if (huart->Instance == USART6)
    {
        return &uart6_receive_destination;
    }

    return NULL;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size)
{
    uint8_t **destination = receive_destination_for(huart);

    if ((destination == NULL) || (data == NULL) || (size != 1U))
    {
        return HAL_ERROR;
    }

    *destination = data;
    huart->RxState = HAL_UART_STATE_BUSY_RX;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit(
    UART_HandleTypeDef *huart,
    const uint8_t *data,
    uint16_t size,
    uint32_t timeout)
{
    (void)timeout;

    if ((huart == NULL) ||
        (huart->Instance != USART1) ||
        (data == NULL) ||
        (size > RBP2_MAX_WIRE_SIZE) ||
        (tx_frame_count >= TEST_MAX_TX_FRAMES))
    {
        return HAL_ERROR;
    }

    memcpy(tx_wires[tx_frame_count], data, size);
    tx_lengths[tx_frame_count] = size;
    ++tx_frame_count;
    return HAL_OK;
}

uint32_t HAL_GetTick(void)
{
    return test_tick;
}

GPIO_PinState HAL_GPIO_ReadPin(
    GPIO_TypeDef *GPIOx,
    uint16_t GPIO_Pin)
{
    (void)GPIOx;
    (void)GPIO_Pin;
    return GPIO_PIN_SET;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(
    TIM_HandleTypeDef *htim,
    uint32_t Channel)
{
    (void)htim;
    (void)Channel;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(
    TIM_HandleTypeDef *htim,
    uint32_t Channel)
{
    (void)htim;
    (void)Channel;
    return HAL_OK;
}

static void inject_byte(
    UART_HandleTypeDef *huart,
    uint8_t byte,
    void (*on_complete)(UART_HandleTypeDef *))
{
    uint8_t **destination = receive_destination_for(huart);

    expect((destination != NULL) && (*destination != NULL),
           "RX staging pointer is missing");
    if ((destination != NULL) && (*destination != NULL))
    {
        **destination = byte;
        huart->RxState = HAL_UART_STATE_READY;
        on_complete(huart);
    }
}

static void inject_depth_line(
    UART_HandleTypeDef *depth_uart,
    const char *line)
{
    for (size_t index = 0U; line[index] != '\0'; ++index)
    {
        inject_byte(
            depth_uart,
            (uint8_t)line[index],
            depth_transport_stm32_on_rx_complete);
    }
}

static void send_heartbeat(
    UART_HandleTypeDef *host_uart,
    uint16_t sequence)
{
    const uint8_t payload[4] = {0U, 0U, 0U, 0U};
    uint8_t wire[RBP2_MAX_WIRE_SIZE];
    size_t wire_length = rbp2_encode_wire(
        RBP2_MSG_HEARTBEAT,
        sequence,
        payload,
        sizeof payload,
        wire,
        sizeof wire);

    expect(wire_length > 0U, "Heartbeat test frame did not encode");
    for (size_t index = 0U; index < wire_length; ++index)
    {
        inject_byte(
            host_uart,
            wire[index],
            uart_transport_stm32_on_rx_complete);
    }

    app_main_process();
}

static bool decode_tx_frame(
    size_t index,
    rbp2_frame_t *frame)
{
    if ((index >= tx_frame_count) || (tx_lengths[index] < 2U))
    {
        return false;
    }

    return rbp2_decode_wire(
               tx_wires[index],
               (size_t)tx_lengths[index] - 1U,
               frame) == RBP2_OK;
}

static void expect_tx_type(
    size_t index,
    uint8_t type,
    uint16_t payload_length)
{
    rbp2_frame_t frame;

    expect(decode_tx_frame(index, &frame),
           "application TX frame did not decode");
    if (decode_tx_frame(index, &frame))
    {
        expect(frame.type == type, "application TX type differs");
        expect(frame.payload_length == payload_length,
               "application TX payload length differs");
    }
}

static bool find_last_tx_type(
    uint8_t type,
    rbp2_frame_t *frame)
{
    for (size_t index = tx_frame_count; index > 0U; --index)
    {
        rbp2_frame_t candidate;

        if (decode_tx_frame(index - 1U, &candidate) &&
            (candidate.type == type))
        {
            if (frame != NULL)
            {
                *frame = candidate;
            }
            return true;
        }
    }

    return false;
}

int main(void)
{
    UART_HandleTypeDef uart1 = {0};
    UART_HandleTypeDef uart3 = {0};
    UART_HandleTypeDef uart6 = {0};
    TIM_HandleTypeDef tim3 = {0};
    TIM_HandleTypeDef tim4 = {0};
    rbp2_frame_t depth_frame;
    depth_telemetry_source_t source;
    depth_telemetry_diagnostics_t diagnostics;

    uart1.Instance = USART1;
    uart3.Instance = USART3;
    uart6.Instance = USART6;
    test_tick = 1000U;

    app_main_init(
        &uart1,
        &uart3,
        &uart6,
        &tim3,
        &tim4,
        NULL,
        0U);

    expect(tx_frame_count == 0U,
           "application initialization transmitted unexpectedly");

    inject_depth_line(
        &uart6,
        "Depth:4.32m Temp:18.75C\r\n");
    app_main_process();

    send_heartbeat(&uart1, 1U);
    send_heartbeat(&uart1, 2U);
    send_heartbeat(&uart1, 3U);

    expect(tx_frame_count == 6U,
           "three Heartbeats should produce ACK plus three telemetry frames");
    expect_tx_type(0U, RBP2_MSG_ACK, 4U);
    expect_tx_type(1U, RBP2_MSG_LEAK_STATUS, 1U);
    expect_tx_type(2U, RBP2_MSG_ACK, 4U);
    expect_tx_type(3U, RBP2_MSG_IMU_SNAPSHOT, 56U);
    expect_tx_type(4U, RBP2_MSG_ACK, 4U);
    expect_tx_type(5U, RBP2_MSG_DEPTH_SNAPSHOT,
                   DEPTH_TELEMETRY_PAYLOAD_LENGTH);

    expect(decode_tx_frame(5U, &depth_frame),
           "DepthSnapshot frame did not decode");
    if (decode_tx_frame(5U, &depth_frame))
    {
        expect(depth_telemetry_decode(
                   depth_frame.payload,
                   depth_frame.payload_length,
                   &source,
                   &diagnostics),
               "DepthSnapshot payload did not decode");
        expect(source.depth_valid, "DepthSnapshot depth is not valid");
        expect(source.temperature_valid,
               "DepthSnapshot temperature is not valid");
        expect(source.depth_mm == 4320, "DepthSnapshot depth differs");
        expect(source.temperature_centi_c == 1875,
               "DepthSnapshot temperature differs");
        expect(source.sample_age_ms == 0U,
               "DepthSnapshot age differs at same test tick");
        expect(diagnostics.rx_byte_count == 25U,
               "DepthSnapshot RX byte diagnostics differ");
        expect(diagnostics.valid_line_count == 1U,
               "DepthSnapshot valid-line diagnostics differ");
        expect(diagnostics.parse_error_count == 0U,
               "DepthSnapshot parse-error diagnostics differ");
        expect(diagnostics.overlong_line_count == 0U,
               "DepthSnapshot overlong diagnostics differ");
        expect(diagnostics.rx_buffer_overflow_count == 0U,
               "DepthSnapshot overflow diagnostics differ");
        expect(diagnostics.hard_rearm_failure_count == 0U,
               "DepthSnapshot re-arm diagnostics differ");
        expect(diagnostics.uart_error_count == 0U,
               "DepthSnapshot UART diagnostics differ");
    }

    test_tick = 4000U;
    send_heartbeat(&uart1, 4U);
    send_heartbeat(&uart1, 5U);
    send_heartbeat(&uart1, 6U);

    expect(find_last_tx_type(RBP2_MSG_DEPTH_SNAPSHOT, &depth_frame),
           "sensor-stop regression did not emit a DepthSnapshot heartbeat");
    if (find_last_tx_type(RBP2_MSG_DEPTH_SNAPSHOT, &depth_frame))
    {
        expect(depth_telemetry_decode(
                   depth_frame.payload,
                   depth_frame.payload_length,
                   &source,
                   &diagnostics),
               "stale DepthSnapshot payload did not decode");
        expect(!source.depth_valid && !source.temperature_valid,
               "stopped sensor must not keep old depth values valid");
        expect(source.depth_mm == 0 && source.temperature_centi_c == 0,
               "stopped sensor must scrub old depth values");
        expect(diagnostics.valid_line_count == 1U,
               "stale DepthSnapshot must retain parser diagnostics");
    }

    test_tick = 5000U;
    inject_depth_line(
        &uart6,
        "Depth:5.67m Temp:19.25C\r\n");
    app_main_process();
    send_heartbeat(&uart1, 7U);
    send_heartbeat(&uart1, 8U);
    send_heartbeat(&uart1, 9U);

    expect(find_last_tx_type(RBP2_MSG_DEPTH_SNAPSHOT, &depth_frame),
           "fresh sensor recovery did not emit a DepthSnapshot");
    if (find_last_tx_type(RBP2_MSG_DEPTH_SNAPSHOT, &depth_frame))
    {
        expect(depth_telemetry_decode(
                   depth_frame.payload,
                   depth_frame.payload_length,
                   &source,
                   &diagnostics),
               "recovered DepthSnapshot payload did not decode");
        expect(source.depth_valid && source.temperature_valid,
               "fresh sensor line must restore valid depth values");
        expect(source.depth_mm == 5670,
               "fresh sensor recovery depth differs");
        expect(source.temperature_centi_c == 1925,
               "fresh sensor recovery temperature differs");
    }

    if (failures == 0)
    {
        (void)puts("All app_main DepthSnapshot integration tests passed");
    }

    return failures == 0 ? 0 : 1;
}
