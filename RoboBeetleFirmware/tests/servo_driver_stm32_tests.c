#include "servo_driver_stm32.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

_Static_assert(TIM_CHANNEL_1 == 0U,
               "STM32 HAL TIM_CHANNEL_1 must remain a legal zero value");

static int failures = 0;
static unsigned int start_calls = 0U;
static unsigned int stop_calls = 0U;
static TIM_HandleTypeDef *last_timer = NULL;
static uint32_t last_channel = 0U;

static void expect(bool condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

static void fake_set_channel_output(
    TIM_TypeDef *timer,
    uint32_t channel,
    bool enabled)
{
    uint32_t output_mask = 0U;

    switch (channel)
    {
        case TIM_CHANNEL_1:
            output_mask = TIM_CCER_CC1E;
            break;

        case TIM_CHANNEL_2:
            output_mask = TIM_CCER_CC2E;
            break;

        case TIM_CHANNEL_3:
            output_mask = TIM_CCER_CC3E;
            break;

        case TIM_CHANNEL_4:
            output_mask = TIM_CCER_CC4E;
            break;

        default:
            return;
    }

    if (enabled)
    {
        timer->CCER |= output_mask;
    }
    else
    {
        timer->CCER &= ~output_mask;
    }
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(
    TIM_HandleTypeDef *htim,
    uint32_t channel)
{
    ++start_calls;
    last_timer = htim;
    last_channel = channel;
    TIM_CHANNEL_STATE_SET(htim, channel, HAL_TIM_CHANNEL_STATE_BUSY);
    fake_set_channel_output(htim->Instance, channel, true);
    __HAL_TIM_ENABLE(htim);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(
    TIM_HandleTypeDef *htim,
    uint32_t channel)
{
    ++stop_calls;
    last_timer = htim;
    last_channel = channel;
    fake_set_channel_output(htim->Instance, channel, false);
    TIM_CHANNEL_STATE_SET(htim, channel, HAL_TIM_CHANNEL_STATE_READY);
    if ((htim->Instance->CCER &
         (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E)) ==
        0U)
    {
        __HAL_TIM_DISABLE(htim);
    }
    return HAL_OK;
}

static void reset_timer_registers(TIM_TypeDef *timer)
{
    (void)memset(timer, 0, sizeof(*timer));
    timer->ARR = 3002U;
}

static void test_running_pwm_always_defers_even_at_readable_compare(void)
{
    TIM_TypeDef timer_registers = {0};
    TIM_HandleTypeDef timer = {0};
    servo_driver_stm32_t driver = {0};
    const servo_service_driver_ops_t *ops = servo_driver_stm32_ops();
    const unsigned int stops_before = stop_calls;

    reset_timer_registers(&timer_registers);
    timer.Instance = &timer_registers;
    servo_driver_stm32_init(&driver, &timer, NULL);

    expect(ops->start(&driver, SERVO_ID_FRONT_LEFT),
           "safe-stop test channel must start");
    timer_registers.CCR1 = 1500U;
    timer_registers.CNT = 1500U;
    timer_registers.SR = TIM_SR_CC1IF;
    ops->stop(&driver, SERVO_ID_FRONT_LEFT);

    expect(stop_calls == stops_before,
           "running PWM must defer even when readable CNT >= CCR");
    expect((timer_registers.SR & TIM_SR_CC1IF) == 0U,
           "deferred running stop must clear a stale CC1 flag");
    expect((timer_registers.CCER & TIM_CCER_CC1E) != 0U,
           "deferred running stop must retain CC1 output");
    expect((timer_registers.DIER & TIM_DIER_CC1IE) != 0U,
           "deferred running stop must arm CC1 interrupt");
    expect(servo_driver_stm32_stop_pending_mask(&driver) ==
               (uint16_t)(1U << SERVO_ID_FRONT_LEFT),
           "deferred running stop must retain pending ownership");
}

static void test_preload_shadow_mismatch_never_truncates_running_pulse(void)
{
    TIM_TypeDef timer_registers = {0};
    TIM_HandleTypeDef timer = {0};
    servo_driver_stm32_t driver = {0};
    const servo_service_driver_ops_t *ops = servo_driver_stm32_ops();
    const unsigned int stops_before = stop_calls;

    reset_timer_registers(&timer_registers);
    timer.Instance = &timer_registers;
    servo_driver_stm32_init(&driver, &timer, NULL);

    expect(ops->start(&driver, SERVO_ID_FRONT_LEFT),
           "preload/shadow regression channel must start");
    /*
     * Host HAL mocks expose one CCR only.  Model the hazardous observation:
     * the readable preload is 1000 while the current shadow pulse is still
     * 1900 and CNT is 1500.  The driver must not infer LOW from that mismatch.
     */
    timer_registers.CCR1 = 1000U;
    timer_registers.CNT = 1500U;
    ops->stop(&driver, SERVO_ID_FRONT_LEFT);

    expect(stop_calls == stops_before,
           "preload/shadow mismatch must not truncate a running pulse");
    expect(servo_driver_stm32_stop_pending_mask(&driver) ==
               (uint16_t)(1U << SERVO_ID_FRONT_LEFT),
           "preload/shadow mismatch must defer to compare");
    expect((timer_registers.CCER & TIM_CCER_CC1E) != 0U,
           "preload/shadow mismatch must retain the active output");
}

static void test_stopped_timer_stop_is_immediate(void)
{
    TIM_TypeDef timer_registers = {0};
    TIM_HandleTypeDef timer = {0};
    servo_driver_stm32_t driver = {0};
    const servo_service_driver_ops_t *ops = servo_driver_stm32_ops();
    const unsigned int stops_before = stop_calls;

    reset_timer_registers(&timer_registers);
    timer.Instance = &timer_registers;
    servo_driver_stm32_init(&driver, &timer, NULL);
    expect(ops->start(&driver, SERVO_ID_FRONT_LEFT),
           "stopped-timer regression channel must start");
    timer_registers.CR1 &= ~TIM_CR1_CEN;
    ops->stop(&driver, SERVO_ID_FRONT_LEFT);

    expect(stop_calls == stops_before + 1U,
           "inactive timer may finalize PWM stop immediately");
    expect(servo_driver_stm32_stop_pending_mask(&driver) == 0U,
           "inactive timer immediate stop must not remain pending");
}

static void test_high_stop_waits_for_compare_and_preserves_hal_state(void)
{
    TIM_TypeDef timer_registers = {0};
    TIM_HandleTypeDef timer = {0};
    servo_driver_stm32_t driver = {0};
    const servo_service_driver_ops_t *ops = servo_driver_stm32_ops();
    const unsigned int stops_before = stop_calls;

    reset_timer_registers(&timer_registers);
    timer.Instance = &timer_registers;
    servo_driver_stm32_init(&driver, &timer, NULL);

    expect(ops->start(&driver, SERVO_ID_FRONT_LEFT),
           "deferred-stop test channel must start");
    timer_registers.CCR1 = 1500U;
    timer_registers.CNT = 1000U;
    timer_registers.SR = TIM_SR_CC1IF; /* stale flag must be cleared while arming */
    ops->stop(&driver, SERVO_ID_FRONT_LEFT);

    expect(stop_calls == stops_before,
           "stop during HIGH must not truncate the active pulse");
    expect((timer_registers.SR & TIM_SR_CC1IF) == 0U,
           "arming must clear a stale CC1 flag");
    expect((timer_registers.CCER & TIM_CCER_CC1E) != 0U,
           "deferred stop must keep CC1E active through the falling edge");
    expect((timer_registers.DIER & TIM_DIER_CC1IE) != 0U,
           "deferred stop must arm CC1 compare interrupt");
    expect(servo_driver_stm32_stop_pending_mask(&driver) ==
               (uint16_t)(1U << SERVO_ID_FRONT_LEFT),
           "deferred stop must retain per-channel pending ownership");

    timer_registers.CNT = timer_registers.CCR1;
    timer_registers.SR |= TIM_SR_CC1IF;
    servo_driver_stm32_handle_timer_compare(
        &driver, &timer, HAL_TIM_ACTIVE_CHANNEL_1);

    expect(stop_calls == stops_before + 1U,
           "compare callback must finalize the deferred stop");
    expect((timer_registers.CCER & TIM_CCER_CC1E) == 0U,
           "compare finalizer must disable CC1 after the falling edge");
    expect((timer_registers.DIER & TIM_DIER_CC1IE) == 0U,
           "compare finalizer must disarm CC1 interrupt");
    expect(timer.ChannelState[0] == HAL_TIM_CHANNEL_STATE_READY,
           "finalizer must preserve HAL channel READY state");
    expect(servo_driver_stm32_stop_pending_mask(&driver) == 0U,
           "compare finalizer must clear pending ownership");
    expect(servo_driver_stm32_safe_stop_finalization_count(&driver) == 1U,
           "compare finalizer must record one finalization");
}

static void test_shared_timer_channels_finalize_independently(void)
{
    TIM_TypeDef timer_registers = {0};
    TIM_HandleTypeDef timer = {0};
    servo_driver_stm32_t driver = {0};
    const servo_service_driver_ops_t *ops = servo_driver_stm32_ops();

    reset_timer_registers(&timer_registers);
    timer.Instance = &timer_registers;
    servo_driver_stm32_init(&driver, &timer, NULL);

    expect(ops->start(&driver, SERVO_ID_FRONT_LEFT),
           "shared timer CC1 must start");
    expect(ops->start(&driver, SERVO_ID_FRONT_RIGHT),
           "shared timer CC2 must start");
    timer_registers.CCR1 = 1200U;
    timer_registers.CCR2 = 1800U;
    timer_registers.CNT = 1000U;

    ops->stop(&driver, SERVO_ID_FRONT_LEFT);
    ops->stop(&driver, SERVO_ID_FRONT_RIGHT);
    expect((timer_registers.CCER & TIM_CCER_CC1E) != 0U &&
               (timer_registers.CCER & TIM_CCER_CC2E) != 0U,
           "stopping CC1 during HIGH must not affect enabled CC2");

    timer_registers.CNT = timer_registers.CCR1;
    servo_driver_stm32_handle_timer_compare(
        &driver, &timer, HAL_TIM_ACTIVE_CHANNEL_1);
    expect((timer_registers.CCER & TIM_CCER_CC1E) == 0U &&
               (timer_registers.CCER & TIM_CCER_CC2E) != 0U,
           "CC1 finalization must leave CC2 output enabled");
    expect((timer_registers.CR1 & TIM_CR1_CEN) != 0U,
           "shared timer counter must remain enabled for CC2");

    timer_registers.CNT = timer_registers.CCR2;
    servo_driver_stm32_handle_timer_compare(
        &driver, &timer, HAL_TIM_ACTIVE_CHANNEL_2);
    expect((timer_registers.CCER & TIM_CCER_CC2E) == 0U &&
               (timer_registers.CR1 & TIM_CR1_CEN) == 0U,
           "timer may stop only after the final shared channel is stopped");
    expect(servo_driver_stm32_safe_stop_finalization_count(&driver) == 2U,
           "shared timer stops must record one finalization per channel");
}

static void test_pending_stop_is_idempotent_and_blocks_reenable(void)
{
    TIM_TypeDef timer_registers = {0};
    TIM_HandleTypeDef timer = {0};
    servo_driver_stm32_t driver = {0};
    servo_service_t service;
    const servo_service_driver_ops_t *ops = servo_driver_stm32_ops();
    const unsigned int stops_before = stop_calls;

    reset_timer_registers(&timer_registers);
    timer.Instance = &timer_registers;
    servo_driver_stm32_init(&driver, &timer, NULL);
    expect(ops->start(&driver, SERVO_ID_FRONT_LEFT),
           "pending ownership test channel must start");
    timer_registers.CCR1 = 1500U;
    timer_registers.CNT = 1000U;
    ops->stop(&driver, SERVO_ID_FRONT_LEFT);
    ops->stop(&driver, SERVO_ID_FRONT_LEFT);

    expect(stop_calls == stops_before,
           "repeated Disable must not call HAL stop before the safe edge");
    expect((timer_registers.DIER & TIM_DIER_CC1IE) != 0U,
           "repeated Disable must retain one pending compare interrupt");

    (void)memset(&service, 0, sizeof(service));
    servo_service_init(&service, ops, &driver);
    expect(servo_service_enable(
               &service,
               (uint16_t)(1U << SERVO_ID_FRONT_LEFT)) ==
               SERVO_SERVICE_RESULT_BUSY,
           "Enable must return BUSY while physical stop is pending");
    expect(timer_registers.CCR1 == 1500U,
           "blocked Enable must not rewrite a pending channel target");
}

static void test_disable_all_arms_each_shared_channel_independently(void)
{
    TIM_TypeDef tim3_registers = {0};
    TIM_TypeDef tim4_registers = {0};
    TIM_HandleTypeDef tim3 = {0};
    TIM_HandleTypeDef tim4 = {0};
    servo_driver_stm32_t driver = {0};
    servo_service_t service;
    const servo_service_driver_ops_t *ops = servo_driver_stm32_ops();

    reset_timer_registers(&tim3_registers);
    reset_timer_registers(&tim4_registers);
    tim3.Instance = &tim3_registers;
    tim4.Instance = &tim4_registers;
    servo_driver_stm32_init(&driver, &tim3, &tim4);
    (void)memset(&service, 0, sizeof(service));
    servo_service_init(&service, ops, &driver);
    expect(servo_service_enable(&service, 0x001FU) ==
               SERVO_SERVICE_RESULT_OK,
           "Disable All safe-stop setup must enable every channel");

    tim3_registers.CCR1 = 1200U;
    tim3_registers.CCR2 = 1300U;
    tim3_registers.CCR3 = 1400U;
    tim4_registers.CCR1 = 1500U;
    tim4_registers.CCR2 = 1600U;
    tim3_registers.CNT = 1000U;
    tim4_registers.CNT = 1000U;
    const uint16_t cc3_before = tim3_registers.CCR3;
    const uint16_t tim4_cc1_before = tim4_registers.CCR1;
    const unsigned int stops_before = stop_calls;

    servo_service_disable_all(&service);
    expect(servo_service_enabled_mask(&service) == 0U,
           "Disable All must clear logical ownership immediately");
    expect(stop_calls == stops_before,
           "Disable All during HIGH must wait for each safe compare edge");
    expect(servo_driver_stm32_stop_pending_mask(&driver) == 0x001FU,
           "Disable All must register every channel in per-channel pending state");
    expect((tim3_registers.CCER &
            (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E)) ==
               (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E),
           "Disable All must retain all active TIM3 outputs until their edges");
    expect((tim4_registers.CCER & (TIM_CCER_CC1E | TIM_CCER_CC2E)) ==
               (TIM_CCER_CC1E | TIM_CCER_CC2E),
           "Disable All must retain all active TIM4 outputs until their edges");
    expect(tim3_registers.CCR3 == cc3_before &&
               tim4_registers.CCR1 == tim4_cc1_before,
           "Disable All must not insert a Neutral write before stopping PWM");
    expect(servo_service_set_pwm(
               &service,
               SERVO_ID_FRONT_RIGHT,
               1500U) == SERVO_SERVICE_RESULT_BUSY,
           "Disable All pending-stop ownership must block ApplyPWM");
    expect(servo_service_set_angle(
               &service,
               SERVO_ID_FRONT_RIGHT,
               0) == SERVO_SERVICE_RESULT_BUSY,
           "Disable All pending-stop ownership must block SetAngle");

    tim3_registers.CNT = tim3_registers.CCR1;
    servo_driver_stm32_handle_timer_compare(
        &driver, &tim3, HAL_TIM_ACTIVE_CHANNEL_1);
    expect((tim3_registers.CCER & TIM_CCER_CC1E) == 0U &&
               (tim3_registers.CCER & TIM_CCER_CC2E) != 0U &&
               (tim3_registers.CCER & TIM_CCER_CC3E) != 0U,
           "TIM3 CC1 finalization must not affect TIM3 CC2/CC3");
    expect((tim3_registers.CR1 & TIM_CR1_CEN) != 0U,
           "TIM3 counter must continue while other channels are pending");

    tim3_registers.CNT = tim3_registers.CCR2;
    servo_driver_stm32_handle_timer_compare(
        &driver, &tim3, HAL_TIM_ACTIVE_CHANNEL_2);
    tim3_registers.CNT = tim3_registers.CCR3;
    servo_driver_stm32_handle_timer_compare(
        &driver, &tim3, HAL_TIM_ACTIVE_CHANNEL_3);
    expect((tim3_registers.CCER &
            (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E)) == 0U &&
               (tim3_registers.CR1 & TIM_CR1_CEN) == 0U,
           "TIM3 counter may stop after its final channel is finalized");

    tim4_registers.CNT = tim4_registers.CCR1;
    servo_driver_stm32_handle_timer_compare(
        &driver, &tim4, HAL_TIM_ACTIVE_CHANNEL_1);
    expect((tim4_registers.CCER & TIM_CCER_CC2E) != 0U &&
               (tim4_registers.CR1 & TIM_CR1_CEN) != 0U,
           "TIM4 CC1 finalization must not affect TIM4 CC2");
    tim4_registers.CNT = tim4_registers.CCR2;
    servo_driver_stm32_handle_timer_compare(
        &driver, &tim4, HAL_TIM_ACTIVE_CHANNEL_2);
    expect(servo_driver_stm32_stop_pending_mask(&driver) == 0U &&
               (tim4_registers.CR1 & TIM_CR1_CEN) == 0U,
           "Disable All pending state must clear after each channel edge");
    expect(servo_driver_stm32_safe_stop_finalization_count(&driver) == 5U,
           "Disable All must finalize every pending channel independently");
}

static void test_descriptor_bindings(
    servo_driver_stm32_t *driver,
    TIM_HandleTypeDef *tim3,
    TIM_HandleTypeDef *tim4)
{
    const servo_driver_stm32_binding_t *front_right =
        servo_driver_stm32_binding_for_id(driver, SERVO_ID_FRONT_RIGHT);
    const servo_driver_stm32_binding_t *front_left =
        servo_driver_stm32_binding_for_id(driver, SERVO_ID_FRONT_LEFT);
    const servo_driver_stm32_binding_t *front_axis =
        servo_driver_stm32_binding_for_id(driver, SERVO_ID_FRONT_AXIS);
    const servo_driver_stm32_binding_t *rear_right =
        servo_driver_stm32_binding_for_id(driver, SERVO_ID_REAR_RIGHT);
    const servo_driver_stm32_binding_t *rear_left =
        servo_driver_stm32_binding_for_id(driver, SERVO_ID_REAR_LEFT);

    expect(front_right != NULL && front_right->timer == tim3,
           "FrontRight must bind to TIM3");
    expect(front_right != NULL && front_right->hal_channel == TIM_CHANNEL_2
               && front_right->channel_valid,
           "FrontRight remap must bind to TIM3_CH2");
    expect(front_left != NULL && front_left->timer == tim3
               && front_left->hal_channel == TIM_CHANNEL_1
               && front_left->hal_channel == 0U
               && front_left->channel_valid,
           "FrontLeft TIM3_CH1 zero HAL value must still be valid");
    expect(front_axis != NULL && front_axis->timer == tim3
               && front_axis->hal_channel == TIM_CHANNEL_3
               && front_axis->channel_valid,
           "FrontAxis TIM3_CH3 mapping must remain valid");
    expect(rear_right != NULL && rear_right->timer == tim4,
           "RearRight must bind to TIM4");
    expect(rear_right != NULL && rear_right->hal_channel == TIM_CHANNEL_1
               && rear_right->hal_channel == 0U
               && rear_right->channel_valid,
           "RearRight TIM4_CH1 zero HAL value must still be valid");
    expect(rear_left != NULL && rear_left->timer == tim4
               && rear_left->hal_channel == TIM_CHANNEL_2
               && rear_left->channel_valid,
           "RearLeft TIM4_CH2 mapping must remain valid");
}

static void test_all_valid_channels_start(
    servo_driver_stm32_t *driver,
    TIM_HandleTypeDef *tim3,
    TIM_HandleTypeDef *tim4)
{
    const servo_service_driver_ops_t *ops = servo_driver_stm32_ops();

    expect(ops->start(driver, SERVO_ID_FRONT_RIGHT),
           "FrontRight ServoEnable must remain valid after remap");
    expect(last_timer == tim3 && last_channel == TIM_CHANNEL_2,
           "FrontRight start must use TIM3_CH2");
    expect(ops->start(driver, SERVO_ID_FRONT_LEFT),
           "FrontLeft start must remain valid");
    expect(last_timer == tim3 && last_channel == TIM_CHANNEL_1,
           "FrontLeft start must use TIM3_CH1");
    expect(ops->start(driver, SERVO_ID_FRONT_AXIS),
           "FrontAxis start must remain valid");
    expect(last_timer == tim3 && last_channel == TIM_CHANNEL_3,
           "FrontAxis start must use TIM3_CH3");
    expect(ops->start(driver, SERVO_ID_REAR_RIGHT),
           "RearRight ServoEnable must accept HAL channel zero");
    expect(last_timer == tim4 && last_channel == TIM_CHANNEL_1,
           "RearRight start must use TIM4_CH1");
    expect(ops->start(driver, SERVO_ID_REAR_LEFT),
           "RearLeft start must remain valid");
    expect(last_timer == tim4 && last_channel == TIM_CHANNEL_2,
           "RearLeft start must use TIM4_CH2");
}

static void test_servo_enable_for_front_bindings(
    servo_driver_stm32_t *driver,
    TIM_TypeDef *tim3_registers,
    TIM_TypeDef *tim4_registers)
{
    servo_service_t service;
    const servo_service_driver_ops_t *ops = servo_driver_stm32_ops();

    (void)memset(&service, 0, sizeof(service));
    servo_service_init(&service, ops, driver);
    expect(servo_service_enable(&service, 0x0002U) ==
               SERVO_SERVICE_RESULT_OK,
           "FrontLeft ServoEnable must not fail for HAL channel zero");
    expect(tim3_registers->CCR1 == 1450U,
           "FrontLeft enable must write TIM3_CH1 neutral pulse");

    (void)memset(&service, 0, sizeof(service));
    servo_service_init(&service, ops, driver);
    expect(servo_service_enable(&service, 0x0001U) ==
               SERVO_SERVICE_RESULT_OK,
           "FrontRight ServoEnable must write its remapped channel");
    expect(tim3_registers->CCR2 == 1580U,
           "FrontRight enable must write TIM3_CH2 neutral pulse");

    (void)memset(&service, 0, sizeof(service));
    servo_service_init(&service, ops, driver);
    expect(servo_service_enable(&service, 0x0008U) ==
               SERVO_SERVICE_RESULT_OK,
           "RearRight ServoEnable must not fail for HAL channel zero");
    expect(tim4_registers->CCR1 == 1570U,
           "RearRight enable must write TIM4_CH1 neutral pulse");
}

static void test_invalid_channel_fails_closed(
    servo_driver_stm32_t *driver,
    TIM_TypeDef *tim3_registers)
{
    servo_driver_stm32_binding_t *binding =
        &driver->bindings[SERVO_ID_FRONT_RIGHT];
    servo_descriptor_t invalid_descriptor = *binding->descriptor;
    const servo_service_driver_ops_t *ops = servo_driver_stm32_ops();
    const unsigned int stops_before = stop_calls;
    const uint32_t compare_before = tim3_registers->CCR2;

    invalid_descriptor.channel = (servo_channel_id_t)0xFFU;
    binding->descriptor = &invalid_descriptor;
    binding->channel_valid = false;

    expect(!ops->start(driver, SERVO_ID_FRONT_RIGHT),
           "invalid symbolic channel must fail closed on start");
    ops->write_pulse_us(driver, SERVO_ID_FRONT_RIGHT, 1600U);
    ops->stop(driver, SERVO_ID_FRONT_RIGHT);
    expect(stop_calls == stops_before,
           "invalid symbolic channel must not call HAL stop");
    expect(tim3_registers->CCR2 == compare_before,
           "invalid symbolic channel must not write a compare value");
}

int main(void)
{
    TIM_TypeDef tim3_registers = {0};
    TIM_TypeDef tim4_registers = {0};
    TIM_HandleTypeDef tim3 = {0};
    TIM_HandleTypeDef tim4 = {0};
    servo_driver_stm32_t driver = {0};

    tim3.Instance = &tim3_registers;
    tim4.Instance = &tim4_registers;
    servo_driver_stm32_init(&driver, &tim3, &tim4);
    test_descriptor_bindings(&driver, &tim3, &tim4);
    test_all_valid_channels_start(&driver, &tim3, &tim4);
    test_servo_enable_for_front_bindings(&driver, &tim3_registers, &tim4_registers);
    test_invalid_channel_fails_closed(&driver, &tim3_registers);
    test_running_pwm_always_defers_even_at_readable_compare();
    test_preload_shadow_mismatch_never_truncates_running_pulse();
    test_stopped_timer_stop_is_immediate();
    test_high_stop_waits_for_compare_and_preserves_hal_state();
    test_shared_timer_channels_finalize_independently();
    test_pending_stop_is_idempotent_and_blocks_reenable();
    test_disable_all_arms_each_shared_channel_independently();

    if (failures == 0)
    {
        (void)puts("All STM32 Servo driver mapping tests passed");
    }

    return failures == 0 ? 0 : 1;
}
