#include "app_main.h"
#include "cpg_gait_generator.h"
#include "gait_generator.h"
#include "motion_manager.h"
#include "simple_gait_generator.h"
#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static int failures = 0;
static unsigned int motion_manager_init_calls = 0U;
static gait_generator_t captured_generator;

static void expect(bool condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

void __wrap_motion_manager_init(
    motion_manager_t *manager,
    servo_service_t *servo_service,
    safety_supervisor_t *safety_supervisor,
    gait_generator_t generator)
{
    (void)manager;
    (void)servo_service;
    (void)safety_supervisor;
    captured_generator = generator;
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
    simple_gait_generator_t expected_simple;
    cpg_gait_generator_t expected_cpg;
    joint_targets_t targets = {0};

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

    expect(motion_manager_init_calls == 1U,
           "app_main must initialize exactly one MotionManager generator");
    expect(captured_generator.ops == simple_interface.ops,
           "default bench app_main backend must be SimpleGait");
    expect(captured_generator.ops != cpg_interface.ops,
           "default bench app_main backend must not be CPG");
    expect(captured_generator.context != NULL,
           "selected SimpleGait context must be non-null");

    if ((captured_generator.ops != NULL) &&
        (captured_generator.context != NULL))
    {
        captured_generator.ops->advance(
            captured_generator.context,
            500U);
        expect(captured_generator.ops->sample(
                   captured_generator.context,
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
    }

    if (failures == 0)
    {
        (void)puts("All app_main SimpleGait backend selection tests passed");
    }

    return failures == 0 ? 0 : 1;
}
