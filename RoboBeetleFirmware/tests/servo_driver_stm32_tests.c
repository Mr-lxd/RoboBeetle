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

HAL_StatusTypeDef HAL_TIM_PWM_Start(
    TIM_HandleTypeDef *htim,
    uint32_t channel)
{
    ++start_calls;
    last_timer = htim;
    last_channel = channel;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(
    TIM_HandleTypeDef *htim,
    uint32_t channel)
{
    ++stop_calls;
    last_timer = htim;
    last_channel = channel;
    return HAL_OK;
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
    expect(front_right != NULL && front_right->hal_channel == TIM_CHANNEL_1
               && front_right->hal_channel == 0U
               && front_right->channel_valid,
           "FrontRight TIM3_CH1 zero HAL value must still be valid");
    expect(front_left != NULL && front_left->timer == tim3
               && front_left->hal_channel == TIM_CHANNEL_2
               && front_left->channel_valid,
           "FrontLeft TIM3_CH2 mapping must remain valid");
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
           "FrontRight ServoEnable must accept HAL channel zero");
    expect(last_timer == tim3 && last_channel == TIM_CHANNEL_1,
           "FrontRight start must use TIM3_CH1");
    expect(ops->start(driver, SERVO_ID_FRONT_LEFT),
           "FrontLeft start must remain valid");
    expect(last_timer == tim3 && last_channel == TIM_CHANNEL_2,
           "FrontLeft start must use TIM3_CH2");
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

static void test_servo_enable_for_ch1_channels(
    servo_driver_stm32_t *driver,
    TIM_TypeDef *tim3_registers,
    TIM_TypeDef *tim4_registers)
{
    servo_service_t service;
    const servo_service_driver_ops_t *ops = servo_driver_stm32_ops();

    (void)memset(&service, 0, sizeof(service));
    servo_service_init(&service, ops, driver);
    expect(servo_service_enable(&service, 0x0001U) ==
               SERVO_SERVICE_RESULT_OK,
           "FrontRight ServoEnable must not fail for HAL channel zero");
    expect(tim3_registers->CCR1 == 1450U,
           "FrontRight enable must write TIM3_CH1 neutral pulse");

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
    const uint32_t compare_before = tim3_registers->CCR1;

    invalid_descriptor.channel = (servo_channel_id_t)0xFFU;
    binding->descriptor = &invalid_descriptor;
    binding->channel_valid = false;

    expect(!ops->start(driver, SERVO_ID_FRONT_RIGHT),
           "invalid symbolic channel must fail closed on start");
    ops->write_pulse_us(driver, SERVO_ID_FRONT_RIGHT, 1600U);
    ops->stop(driver, SERVO_ID_FRONT_RIGHT);
    expect(stop_calls == stops_before,
           "invalid symbolic channel must not call HAL stop");
    expect(tim3_registers->CCR1 == compare_before,
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
    test_servo_enable_for_ch1_channels(&driver, &tim3_registers, &tim4_registers);
    test_invalid_channel_fails_closed(&driver, &tim3_registers);

    if (failures == 0)
    {
        (void)puts("All STM32 Servo driver mapping tests passed");
    }

    return failures == 0 ? 0 : 1;
}
