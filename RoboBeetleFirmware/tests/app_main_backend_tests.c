#include "app_main.h"
#include "cpg_gait_generator.h"
#include "experimental_flex_gait_generator.h"
#include "gait_generator.h"
#include "motion_manager.h"
#include "simple_gait_generator.h"
#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifndef MOTION_DEFAULT_GAIT_BACKEND_CPG
#define MOTION_DEFAULT_GAIT_BACKEND_CPG 1
#endif

static int failures = 0;
static unsigned int motion_manager_init_calls = 0U;
static gait_generator_t captured_simple_generator;
static gait_generator_t captured_cpg_generator;
static gait_generator_t captured_flex_generator;
static motion_gait_backend_t captured_initial_backend;

static void expect(bool condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

void __wrap_motion_manager_init_with_backends(
    motion_manager_t *manager,
    servo_service_t *servo_service,
    safety_supervisor_t *safety_supervisor,
    gait_generator_t simple,
    gait_generator_t cpg,
    gait_generator_t experimental_flex,
    motion_gait_backend_t initial_backend)
{
    (void)manager;
    (void)servo_service;
    (void)safety_supervisor;
    captured_simple_generator = simple;
    captured_cpg_generator = cpg;
    captured_flex_generator = experimental_flex;
    captured_initial_backend = initial_backend;
    ++motion_manager_init_calls;
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

    huart->RxState = HAL_UART_STATE_BUSY_RX;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(
    UART_HandleTypeDef *huart,
    const uint8_t *data,
    uint16_t size)
{
    (void)data;
    (void)size;
    if (huart == NULL)
    {
        return HAL_ERROR;
    }
    huart->gState = HAL_UART_STATE_BUSY_TX;
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

HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(
    UART_HandleTypeDef *huart)
{
    if (huart != NULL)
    {
        huart->gState = HAL_UART_STATE_READY;
    }
    return HAL_OK;
}

HAL_UART_StateTypeDef HAL_UART_GetState(
    const UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return HAL_UART_STATE_RESET;
    }
    return (HAL_UART_StateTypeDef)(huart->gState | huart->RxState);
}

uint32_t HAL_UART_GetError(
    const UART_HandleTypeDef *huart)
{
    return huart == NULL ? HAL_UART_ERROR_NONE : huart->ErrorCode;
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
    simple_gait_generator_t expected_simple;
    cpg_gait_generator_t expected_cpg;
    experimental_flex_gait_generator_t expected_flex;
    joint_targets_t targets = {0};
    gait_generator_t selected_generator;

    app_main_init(
        &uart1,
        &uart3,
        &uart6,
        &tim3,
        &tim4,
        NULL,
        0U);

    simple_gait_generator_init(&expected_simple);
    const gait_generator_t simple_interface =
        simple_gait_generator_interface(&expected_simple);
    cpg_gait_generator_init(&expected_cpg);
    const gait_generator_t cpg_interface =
        cpg_gait_generator_interface(&expected_cpg);
    experimental_flex_gait_generator_init(&expected_flex);
    const gait_generator_t flex_interface =
        experimental_flex_gait_generator_interface(&expected_flex);

    expect(motion_manager_init_calls == 1U,
           "app_main must initialize MotionManager exactly once");
    expect(captured_simple_generator.ops == simple_interface.ops &&
               captured_simple_generator.context != NULL,
           "app_main must register the SimpleGait generator");
    expect(captured_cpg_generator.ops == cpg_interface.ops &&
               captured_cpg_generator.context != NULL,
           "app_main must register the CPG generator");
    expect(captured_flex_generator.ops == flex_interface.ops &&
               captured_flex_generator.context != NULL,
           "app_main must register the ExperimentalFlex generator");
#if MOTION_DEFAULT_GAIT_BACKEND_CPG
    expect(captured_initial_backend == MOTION_GAIT_BACKEND_CPG,
           "selected app_main initial backend must be CPG");
    selected_generator = captured_cpg_generator;
#else
    expect(captured_initial_backend == MOTION_GAIT_BACKEND_SIMPLE_GAIT,
           "selected app_main initial backend must be SimpleGait");
    selected_generator = captured_simple_generator;
#endif

    if ((selected_generator.ops != NULL) &&
        (selected_generator.context != NULL))
    {
#if MOTION_DEFAULT_GAIT_BACKEND_CPG
        selected_generator.ops->advance(
            selected_generator.context,
            10U);
        expect(selected_generator.ops->sample(
                   selected_generator.context,
                   MOTION_FORWARD,
                   1.0F,
                   1.0F,
                   &targets),
               "selected CPG backend must sample Forward");
#else
        selected_generator.ops->advance(
            selected_generator.context,
            500U);
        expect(selected_generator.ops->sample(
                   selected_generator.context,
                   MOTION_FORWARD,
                   1.0F,
                   1.0F,
                   &targets),
               "selected bench generator must sample Forward");
        expect(targets.front_right_cdeg == 1000 &&
                   targets.front_left_cdeg == 1000 &&
                   targets.rear_right_cdeg == -1000 &&
                   targets.rear_left_cdeg == -1000,
               "selected bench generator must preserve SimpleGait Forward anti-phase");
#endif
    }

    if (failures == 0)
    {
#if MOTION_DEFAULT_GAIT_BACKEND_CPG
        (void)puts("All app_main CPG backend selection tests passed");
#else
        (void)puts("All app_main SimpleGait backend selection tests passed");
#endif
    }

    return failures == 0 ? 0 : 1;
}
