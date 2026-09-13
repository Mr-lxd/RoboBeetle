#include "servo_driver_stm32.h"

#include "servo_pwm_stop_policy.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef uint32_t servo_driver_irq_state_t;

static servo_driver_irq_state_t servo_driver_enter_critical(void)
{
#if defined(__arm__) || defined(__thumb__)
    const uint32_t state = __get_PRIMASK();
    __disable_irq();
    return state;
#else
    return 0U;
#endif
}

static void servo_driver_exit_critical(servo_driver_irq_state_t state)
{
#if defined(__arm__) || defined(__thumb__)
    if ((state & 1U) == 0U)
    {
        __enable_irq();
    }
#else
    (void)state;
#endif
}

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

static bool hal_channel_for_descriptor(
    servo_channel_id_t channel,
    uint32_t *hal_channel)
{
    if (hal_channel == NULL)
    {
        return false;
    }

    switch (channel)
    {
        case SERVO_CHANNEL_1:
            *hal_channel = TIM_CHANNEL_1;
            return true;

        case SERVO_CHANNEL_2:
            *hal_channel = TIM_CHANNEL_2;
            return true;

        case SERVO_CHANNEL_3:
            *hal_channel = TIM_CHANNEL_3;
            return true;

        default:
            return false;
    }
}

static uint32_t cc_interrupt_for_channel(uint32_t hal_channel)
{
    switch (hal_channel)
    {
        case TIM_CHANNEL_1:
            return TIM_IT_CC1;

        case TIM_CHANNEL_2:
            return TIM_IT_CC2;

        case TIM_CHANNEL_3:
            return TIM_IT_CC3;

        case TIM_CHANNEL_4:
            return TIM_IT_CC4;

        default:
            return 0U;
    }
}

static uint32_t cc_flag_for_channel(uint32_t hal_channel)
{
    switch (hal_channel)
    {
        case TIM_CHANNEL_1:
            return TIM_FLAG_CC1;

        case TIM_CHANNEL_2:
            return TIM_FLAG_CC2;

        case TIM_CHANNEL_3:
            return TIM_FLAG_CC3;

        case TIM_CHANNEL_4:
            return TIM_FLAG_CC4;

        default:
            return 0U;
    }
}

static uint16_t channel_mask_for_id(uint8_t servo_id)
{
    return servo_id < SERVO_DESCRIPTOR_COUNT
        ? (uint16_t)(1U << servo_id)
        : 0U;
}

static uint32_t hal_channel_for_active_channel(uint32_t active_channel)
{
    switch (active_channel)
    {
        case HAL_TIM_ACTIVE_CHANNEL_1:
            return TIM_CHANNEL_1;

        case HAL_TIM_ACTIVE_CHANNEL_2:
            return TIM_CHANNEL_2;

        case HAL_TIM_ACTIVE_CHANNEL_3:
            return TIM_CHANNEL_3;

        case HAL_TIM_ACTIVE_CHANNEL_4:
            return TIM_CHANNEL_4;

        default:
            return UINT32_MAX;
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
        binding->channel_valid &&
        ((driver->stop_pending_mask & channel_mask_for_id(servo_id)) == 0U))
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
        !binding->channel_valid)
    {
        return false;
    }

    const uint16_t channel_mask = channel_mask_for_id(servo_id);
    const servo_driver_irq_state_t irq_state =
        servo_driver_enter_critical();
    const bool pending =
        (driver->stop_pending_mask & channel_mask) != 0U;
    bool started = false;

    if (!pending)
    {
        started = HAL_TIM_PWM_Start(
                      binding->timer,
                      binding->hal_channel) == HAL_OK;
        if (started)
        {
            driver->active_mask = (uint16_t)(
                driver->active_mask | channel_mask);
        }
    }

    servo_driver_exit_critical(irq_state);
    return started;
}

static void servo_driver_finalize_stop(
    servo_driver_stm32_t *driver,
    uint8_t servo_id,
    const servo_driver_stm32_binding_t *binding)
{
    const uint16_t channel_mask = channel_mask_for_id(servo_id);
    const uint32_t interrupt = cc_interrupt_for_channel(binding->hal_channel);

    /*
     * Keep the HAL ChannelState and CCER state aligned through the existing
     * HAL stop API.  This HAL's __HAL_TIM_DISABLE() only clears CEN after all
     * CCxE/CCxNE outputs are disabled, so a shared timer keeps running for
     * every other enabled channel.
     */
    (void)HAL_TIM_PWM_Stop(binding->timer, binding->hal_channel);
    if (interrupt != 0U)
    {
        __HAL_TIM_DISABLE_IT(binding->timer, interrupt);
    }
    driver->stop_pending_mask = (uint16_t)(
        driver->stop_pending_mask & (uint16_t)(~channel_mask));
    driver->active_mask = (uint16_t)(
        driver->active_mask & (uint16_t)(~channel_mask));
    ++driver->safe_stop_finalization_count;
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
        binding->channel_valid)
    {
        const uint16_t channel_mask = channel_mask_for_id(servo_id);
        const uint32_t interrupt = cc_interrupt_for_channel(
            binding->hal_channel);
        const uint32_t flag = cc_flag_for_channel(binding->hal_channel);
        const servo_driver_irq_state_t irq_state =
            servo_driver_enter_critical();

        if ((driver->stop_pending_mask & channel_mask) != 0U)
        {
            servo_driver_exit_critical(irq_state);
            return;
        }

        const bool channel_active =
            (driver->active_mask & channel_mask) != 0U;

        /* Repeated Disable after completion is an intentional no-op. */
        if (!channel_active)
        {
            servo_driver_exit_critical(irq_state);
            return;
        }

        const bool timer_running =
            (binding->timer->Instance->CR1 & TIM_CR1_CEN) != 0U;

        if (servo_pwm_stop_policy_decide(
                timer_running,
                channel_active) == SERVO_PWM_STOP_IMMEDIATE)
        {
            servo_driver_finalize_stop(driver, servo_id, binding);
            servo_driver_exit_critical(irq_state);
            return;
        }

        /*
         * The channel is still HIGH.  Clear stale state before publishing
         * pending ownership and arming its own compare interrupt.
         */
        __HAL_TIM_CLEAR_FLAG(binding->timer, flag);
        driver->stop_pending_mask = (uint16_t)(
            driver->stop_pending_mask | channel_mask);
        __HAL_TIM_ENABLE_IT(binding->timer, interrupt);

        /*
         * The compare edge may have happened during clear/arm.  Re-check both
         * the counter and flag so that the race cannot wait a full extra frame
         * when the falling edge is already available.
         */
        /*
         * Re-read CNT/CCR as part of the arm-sequence audit, but do not use
         * their ordering as a level decision: with OCxPE, CCR may be the
         * next preload while the current shadow compare still drives HIGH.
         * CCxIF is the only post-arm evidence accepted for same-edge finalize.
         */
        const uint32_t counter_after_arm = __HAL_TIM_GET_COUNTER(
            binding->timer);
        const uint32_t compare_after_arm = __HAL_TIM_GET_COMPARE(
            binding->timer,
            binding->hal_channel);
        const bool compare_event_after_arm = __HAL_TIM_GET_FLAG(
            binding->timer,
            flag);
        (void)counter_after_arm;
        (void)compare_after_arm;
        if (compare_event_after_arm)
        {
            servo_driver_finalize_stop(driver, servo_id, binding);
        }

        servo_driver_exit_critical(irq_state);
    }
}

static bool servo_driver_is_stop_pending(
    void *context,
    uint8_t servo_id)
{
    const servo_driver_stm32_t *driver =
        (const servo_driver_stm32_t *)context;
    const uint16_t channel_mask = channel_mask_for_id(servo_id);

    return (channel_mask != 0U) &&
           ((driver->stop_pending_mask & channel_mask) != 0U);
}

void servo_driver_stm32_init(
    servo_driver_stm32_t *driver,
    TIM_HandleTypeDef *tim3,
    TIM_HandleTypeDef *tim4)
{
    const servo_descriptor_t *table = servo_descriptor_table();

    driver->active_mask = 0U;
    driver->stop_pending_mask = 0U;
    driver->safe_stop_finalization_count = 0U;

    for (size_t index = 0U; index < servo_descriptor_count(); ++index)
    {
        driver->bindings[index].descriptor = &table[index];
        driver->bindings[index].timer =
            timer_for_descriptor(table[index].timer, tim3, tim4);
        driver->bindings[index].hal_channel = 0U;
        driver->bindings[index].channel_valid =
            hal_channel_for_descriptor(
                table[index].channel,
                &driver->bindings[index].hal_channel);
    }
}

uint16_t servo_driver_stm32_stop_pending_mask(
    const servo_driver_stm32_t *driver)
{
    return driver == NULL ? 0U : driver->stop_pending_mask;
}

uint32_t servo_driver_stm32_safe_stop_finalization_count(
    const servo_driver_stm32_t *driver)
{
    return driver == NULL ? 0U : driver->safe_stop_finalization_count;
}

void servo_driver_stm32_handle_timer_compare(
    servo_driver_stm32_t *driver,
    TIM_HandleTypeDef *timer,
    uint32_t hal_active_channel)
{
    const uint32_t hal_channel = hal_channel_for_active_channel(
        hal_active_channel);

    if ((driver == NULL) || (timer == NULL) || (hal_channel == UINT32_MAX))
    {
        return;
    }

    for (uint8_t servo_id = 0U;
         servo_id < SERVO_DESCRIPTOR_COUNT;
         ++servo_id)
    {
        const servo_driver_stm32_binding_t *binding =
            servo_driver_stm32_binding_for_id(driver, servo_id);

        if ((binding != NULL) &&
            (binding->timer == timer) &&
            binding->channel_valid &&
            (binding->hal_channel == hal_channel) &&
            ((driver->stop_pending_mask & channel_mask_for_id(servo_id)) != 0U))
        {
            const servo_driver_irq_state_t irq_state =
                servo_driver_enter_critical();
            if ((driver->stop_pending_mask & channel_mask_for_id(servo_id)) != 0U)
            {
                servo_driver_finalize_stop(driver, servo_id, binding);
            }
            servo_driver_exit_critical(irq_state);
            return;
        }
    }
}

const servo_service_driver_ops_t *servo_driver_stm32_ops(void)
{
    static const servo_service_driver_ops_t ops = {
        .write_pulse_us = servo_driver_write_pulse,
        .start = servo_driver_start,
        .stop = servo_driver_stop,
        .is_stop_pending = servo_driver_is_stop_pending,
    };

    return &ops;
}
