#include "servo_driver_stm32.h"

static TIM_HandleTypeDef *timer_for_descriptor(
    servo_timer_id_t timer_id,
    TIM_HandleTypeDef *tim3,
    TIM_HandleTypeDef *tim4)
{
    switch (timer_id)
    {
        case SERVO_TIMER_TIM3:
            return tim3;

        case SERVO_TIMER_TIM4:
            return tim4;

        default:
            return NULL;
    }
}

static uint32_t hal_channel_for_descriptor(
    servo_channel_id_t channel)
{
    switch (channel)
    {
        case SERVO_CHANNEL_1:
            return TIM_CHANNEL_1;

        case SERVO_CHANNEL_2:
            return TIM_CHANNEL_2;

        case SERVO_CHANNEL_3:
            return TIM_CHANNEL_3;

        default:
            return 0U;
    }
}

const servo_driver_stm32_binding_t *servo_driver_stm32_binding_for_id(
    const servo_driver_stm32_t *driver,
    uint8_t servo_id)
{
    if ((driver == NULL) || (servo_id >= SERVO_DESCRIPTOR_COUNT))
    {
        return NULL;
    }

    if ((driver->bindings[servo_id].descriptor == NULL) ||
        (driver->bindings[servo_id].descriptor->id != servo_id))
    {
        return NULL;
    }

    return &driver->bindings[servo_id];
}

static void servo_driver_write_pulse(
    void *context,
    uint8_t servo_id,
    uint16_t pulse_us)
{
    servo_driver_stm32_t *driver =
        (servo_driver_stm32_t *)context;
    const servo_driver_stm32_binding_t *binding =
        servo_driver_stm32_binding_for_id(driver, servo_id);

    if ((binding != NULL) &&
        binding->descriptor->supported &&
        (binding->timer != NULL) &&
        (binding->hal_channel != 0U))
    {
        __HAL_TIM_SET_COMPARE(
            binding->timer,
            binding->hal_channel,
            pulse_us);
    }
}

static bool servo_driver_start(
    void *context,
    uint8_t servo_id)
{
    servo_driver_stm32_t *driver =
        (servo_driver_stm32_t *)context;
    const servo_driver_stm32_binding_t *binding =
        servo_driver_stm32_binding_for_id(driver, servo_id);

    if ((binding == NULL) ||
        !binding->descriptor->supported ||
        (binding->timer == NULL) ||
        (binding->hal_channel == 0U))
    {
        return false;
    }

    return HAL_TIM_PWM_Start(
               binding->timer,
               binding->hal_channel) == HAL_OK;
}

static void servo_driver_stop(
    void *context,
    uint8_t servo_id)
{
    servo_driver_stm32_t *driver =
        (servo_driver_stm32_t *)context;
    const servo_driver_stm32_binding_t *binding =
        servo_driver_stm32_binding_for_id(driver, servo_id);

    if ((binding != NULL) &&
        binding->descriptor->supported &&
        (binding->timer != NULL) &&
        (binding->hal_channel != 0U))
    {
        (void)HAL_TIM_PWM_Stop(
            binding->timer,
            binding->hal_channel);
    }
}

void servo_driver_stm32_init(
    servo_driver_stm32_t *driver,
    TIM_HandleTypeDef *tim3,
    TIM_HandleTypeDef *tim4)
{
    const servo_descriptor_t *table = servo_descriptor_table();

    for (size_t index = 0U; index < servo_descriptor_count(); ++index)
    {
        driver->bindings[index].descriptor = &table[index];
        driver->bindings[index].timer =
            timer_for_descriptor(table[index].timer, tim3, tim4);
        driver->bindings[index].hal_channel =
            hal_channel_for_descriptor(table[index].channel);
    }
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
