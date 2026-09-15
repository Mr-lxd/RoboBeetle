#include "app_main.h"
#include "motion_timing_diagnostics.h"
#include "rb_protocol_v2.h"
#include "stm32f4xx_hal.h"
#include "uart_transport_stm32.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static int failures;
static uint8_t *host_rx_destination;
static unsigned int tx_call_count;

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
    (void)data;
    (void)size;
    (void)timeout;
    ++tx_call_count;
    return HAL_OK;
}

uint32_t HAL_GetTick(void)
{
    return 0U;
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

static void inject_heartbeat(
    UART_HandleTypeDef *host_uart,
    uint16_t sequence)
{
    const uint8_t payload[4] = {0U, 0U, 0U, 0U};
    uint8_t wire[RBP2_MAX_WIRE_SIZE];
    const size_t wire_length = rbp2_encode_wire(
        RBP2_MSG_HEARTBEAT,
        sequence,
        payload,
        sizeof payload,
        wire,
        sizeof wire);

    expect(wire_length > 0U, "reduced telemetry heartbeat must encode");
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
    app_main_process();
}

int main(void)
{
    UART_HandleTypeDef uart1 = {0};
    UART_HandleTypeDef uart3 = {0};
    UART_HandleTypeDef uart6 = {0};
    TIM_HandleTypeDef tim3 = {0};
    TIM_HandleTypeDef tim4 = {0};

    uart1.Instance = USART1;
    uart3.Instance = USART3;
    uart6.Instance = USART6;

    app_main_init(
        &uart1,
        &uart3,
        &uart6,
        &tim3,
        &tim4,
        NULL,
        0U);
    inject_heartbeat(&uart1, 1U);

    expect(MOTION_TIMING_REDUCED_TELEMETRY_ACTIVE == 1,
           "this test must compile with reduced diagnostics active");
    expect(tx_call_count == 1U,
           "reduced diagnostics must preserve Heartbeat ACK and suppress optional telemetry");

    if (failures == 0)
    {
        (void)puts("All reduced telemetry diagnostic tests passed");
    }
    return failures == 0 ? 0 : 1;
}
