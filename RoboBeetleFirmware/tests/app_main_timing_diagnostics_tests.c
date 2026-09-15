#include "app_main.h"
#include "motion_timing_diagnostics.h"
#include "stm32f4xx_hal.h"

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

HAL_StatusTypeDef HAL_UART_Receive_IT(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size)
{
    (void)data;
    if ((huart == NULL) || (size != 1U))
    {
        return HAL_ERROR;
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
    expect(motion_timing_report_is_valid(&motion_timing_report),
           "app_main init must initialize timing diagnostics");

    app_main_process();
    app_main_process();

    expect(motion_timing_report.app_loop_body.count == 2U,
           "app_main must record each loop body");
    expect(motion_timing_report.app_loop_interval.count == 1U,
           "app_main must record loop intervals after the first pass");
    expect(motion_timing_report.rx_drain[MOTION_TIMING_RX_HOST].call_count == 2U,
           "host drain hook must observe each pass");
    expect(motion_timing_report.rx_drain[MOTION_TIMING_RX_JY901S].call_count == 2U,
           "JY901S drain hook must observe each pass");
    expect(motion_timing_report.rx_drain[MOTION_TIMING_RX_DEPTH].call_count == 2U,
           "Depth drain hook must observe each pass");
    expect(motion_timing_report.motion.accepted_tick_count == 0U,
           "stopped app must not record a Motion tick");

    if (failures == 0)
    {
        (void)puts("All app_main timing diagnostics tests passed");
    }
    return failures == 0 ? 0 : 1;
}
