#include "app_main.h"
#include "motion_timing_diagnostics.h"
#include "rb_protocol_v2.h"
#include "safety_supervisor.h"
#include "stm32f4xx_hal.h"
#include "servo_descriptor.h"
#include "uart_transport_stm32.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_MAX_TX_FRAMES 16U

static int failures;
static uint8_t *host_rx_destination;
static uint32_t test_tick;
static unsigned int tim_stop_call_count;
static uint8_t tx_wires[TEST_MAX_TX_FRAMES][RBP2_MAX_WIRE_SIZE];
static uint16_t tx_lengths[TEST_MAX_TX_FRAMES];
static size_t tx_frame_count;

static void expect(bool condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

HAL_StatusTypeDef HAL_UART_Receive_IT(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size)
{
    if ((huart == NULL) || (data == NULL) || (size != 1U))
    {
        return HAL_ERROR;
    }
    if (huart->Instance == USART1)
    {
        host_rx_destination = data;
    }
    huart->RxState = HAL_UART_STATE_BUSY_RX;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit(
    UART_HandleTypeDef *huart,
    const uint8_t *data,
    uint16_t size,
    uint32_t timeout)
{
    (void)huart;
    (void)timeout;
    if ((data == NULL) ||
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
    ++tim_stop_call_count;
    return HAL_OK;
}

static void inject_frame(
    UART_HandleTypeDef *host_uart,
    uint8_t type,
    uint16_t sequence,
    const uint8_t *payload,
    size_t payload_length)
{
    uint8_t wire[RBP2_MAX_WIRE_SIZE];
    const size_t wire_length = rbp2_encode_wire(
        type,
        sequence,
        payload,
        payload_length,
        wire,
        sizeof wire);

    expect(wire_length > 0U, "reduced telemetry frame must encode");
    for (size_t index = 0U; index < wire_length; ++index)
    {
        expect(host_rx_destination != NULL,
               "reduced telemetry host RX destination must exist");
        if (host_rx_destination != NULL)
        {
            *host_rx_destination = wire[index];
            host_uart->RxState = HAL_UART_STATE_READY;
            uart_transport_stm32_on_rx_complete(host_uart);
        }
    }
}

static void inject_heartbeat(
    UART_HandleTypeDef *host_uart,
    uint16_t sequence)
{
    const uint8_t payload[4] = {0U, 0U, 0U, 0U};

    inject_frame(
        host_uart,
        RBP2_MSG_HEARTBEAT,
        sequence,
        payload,
        sizeof payload);
    app_main_process();
}

static void send_servo_enable(
    UART_HandleTypeDef *host_uart,
    uint16_t sequence)
{
    const uint8_t payload[2] = {
        (uint8_t)(SERVO_DESCRIPTOR_SUPPORTED_MASK & 0xFFU),
        (uint8_t)((SERVO_DESCRIPTOR_SUPPORTED_MASK >> 8U) & 0xFFU),
    };

    inject_frame(
        host_uart,
        RBP2_MSG_SERVO_ENABLE,
        sequence,
        payload,
        sizeof payload);
    app_main_process();
}

static void send_gait_backend(
    UART_HandleTypeDef *host_uart,
    uint16_t sequence,
    uint8_t backend)
{
    inject_frame(
        host_uart,
        RBP2_MSG_SET_GAIT_BACKEND,
        sequence,
        &backend,
        1U);
    app_main_process();
}

static bool ack_matches(
    size_t frame_index,
    uint16_t request_sequence,
    uint8_t request_type,
    uint8_t result)
{
    rbp2_frame_t frame;

    if ((frame_index >= tx_frame_count) ||
        (tx_lengths[frame_index] < 2U) ||
        (rbp2_decode_wire(
             tx_wires[frame_index],
             (size_t)tx_lengths[frame_index] - 1U,
             &frame) != RBP2_OK))
    {
        return false;
    }

    return (frame.type == RBP2_MSG_ACK) &&
           (frame.payload_length == 4U) &&
           ((uint16_t)frame.payload[0] |
            ((uint16_t)frame.payload[1] << 8U)) == request_sequence &&
           (frame.payload[2] == request_type) &&
           (frame.payload[3] == result);
}

int main(void)
{
    UART_HandleTypeDef uart1 = {0};
    UART_HandleTypeDef uart3 = {0};
    UART_HandleTypeDef uart6 = {0};
    TIM_HandleTypeDef tim3 = {0};
    TIM_HandleTypeDef tim4 = {0};
    TIM_TypeDef fake_tim3 = {0};
    TIM_TypeDef fake_tim4 = {0};

    uart1.Instance = USART1;
    uart3.Instance = USART3;
    uart6.Instance = USART6;
    tim3.Instance = &fake_tim3;
    tim4.Instance = &fake_tim4;

    app_main_init(
        &uart1,
        &uart3,
        &uart6,
        &tim3,
        &tim4,
        NULL,
        0U);
    motion_timing_diagnostics_begin_run(
        MOTION_GAIT_BACKEND_CPG_VALUE);
    inject_heartbeat(&uart1, 1U);

    expect(MOTION_TIMING_REDUCED_TELEMETRY_ACTIVE == 1,
           "this test must compile with reduced diagnostics active");
    expect(tx_frame_count == 1U,
           "reduced diagnostics must preserve Heartbeat ACK and suppress optional telemetry");
    expect(ack_matches(
               0U,
               1U,
               RBP2_MSG_HEARTBEAT,
               RBP2_RESULT_OK),
           "reduced diagnostics Heartbeat ACK must remain valid");

    send_servo_enable(&uart1, 2U);
    expect(ack_matches(
               1U,
               2U,
               RBP2_MSG_SERVO_ENABLE,
               RBP2_RESULT_OK),
           "reduced diagnostics must preserve Protocol command dispatch");

    test_tick = SAFETY_SUPERVISOR_HEARTBEAT_TIMEOUT_MS + 1U;
    app_main_process();
    expect(tim_stop_call_count > 0U,
           "reduced diagnostics must preserve host-liveness actuator fail-safe");

    inject_heartbeat(&uart1, 3U);
    send_gait_backend(
        &uart1,
        4U,
        MOTION_GAIT_BACKEND_SIMPLE_GAIT_VALUE);
    expect(ack_matches(
               3U,
               4U,
               RBP2_MSG_SET_GAIT_BACKEND,
               RBP2_RESULT_OK),
           "reduced diagnostics must preserve runtime Protocol selector");
    expect(motion_timing_report.run_state == MOTION_TIMING_RUN_STATE_FROZEN &&
               motion_timing_report.runtime_backend ==
                   MOTION_GAIT_BACKEND_CPG_VALUE,
           "reduced diagnostics must preserve the frozen pre-fail-safe report");
    expect(motion_timing_report.tx[MOTION_TIMING_TX_ACK].call_count == 2U,
           "frozen diagnostics must not classify post-stop ACKs");
    expect(motion_timing_report.tx[MOTION_TIMING_TX_LEAK].call_count == 0U &&
               motion_timing_report.tx[MOTION_TIMING_TX_IMU].call_count == 0U &&
               motion_timing_report.tx[MOTION_TIMING_TX_DEPTH].call_count == 0U,
           "reduced diagnostics must suppress optional telemetry TX");

    if (failures == 0)
    {
        (void)puts("All reduced telemetry diagnostic tests passed");
    }
    return failures == 0 ? 0 : 1;
}
