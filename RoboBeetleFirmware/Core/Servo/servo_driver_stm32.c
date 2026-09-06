#include "servo_driver_stm32.h"

static bool servo_driver_supports_id(uint8_t servo_id)
{
    return servo_id == SERVO_SERVICE_SERVO1_ID;
}

static void servo_driver_write_pulse(
    void *context,
    uint8_t servo_id,
    uint16_t pulse_us)
{
    servo_driver_stm32_t *driver =
        (servo_driver_stm32_t *)context;

    if (driver->timer != NULL &&
        servo_driver_supports_id(servo_id))
    {
        __HAL_TIM_SET_COMPARE(
            driver->timer,
            TIM_CHANNEL_1,
            pulse_us);
    }
}

static bool servo_driver_start(
    void *context,
    uint8_t servo_id)
{
    servo_driver_stm32_t *driver =
        (servo_driver_stm32_t *)context;

    if (driver->timer == NULL ||
        !servo_driver_supports_id(servo_id))
    {
        return false;
    }

    return HAL_TIM_PWM_Start(
               driver->timer,
               TIM_CHANNEL_1) == HAL_OK;
}

static void servo_driver_stop(
    void *context,
    uint8_t servo_id)
{
    servo_driver_stm32_t *driver =
        (servo_driver_stm32_t *)context;

    if (driver->timer != NULL &&
        servo_driver_supports_id(servo_id))
    {
        (void)HAL_TIM_PWM_Stop(
            driver->timer,
            TIM_CHANNEL_1);
    }
}

void servo_driver_stm32_init(
    servo_driver_stm32_t *driver,
    TIM_HandleTypeDef *timer)
{
    driver->timer = timer;
}

const servo_service_driver_ops_t *servo_driver_stm32_ops(void)
{
    static const servo_service_driver_ops_t ops = {
        .write_pulse_us = servo_driver_write_pulse,
        .start = servo_driver_start,
        .stop = servo_driver_stop,
    };

    return &ops;
}
