#include "servo_service.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void expect(int condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

typedef struct
{
    bool start_result;
    unsigned int event_count;
    char events[16];
    uint8_t event_servo[16];
    uint16_t event_pulse[16];
    unsigned int start_calls;
    unsigned int stop_calls;
    unsigned int write_calls;
    uint16_t last_pulse;
} fake_driver_t;

static void record_event(
    fake_driver_t *driver,
    char event,
    uint8_t servo_id,
    uint16_t pulse_us)
{
    if (driver->event_count < 16U)
    {
        driver->events[driver->event_count] = event;
        driver->event_servo[driver->event_count] = servo_id;
        driver->event_pulse[driver->event_count] = pulse_us;
        ++driver->event_count;
    }
}

static void fake_write_pulse(
    void *context,
    uint8_t servo_id,
    uint16_t pulse_us)
{
    fake_driver_t *driver = (fake_driver_t *)context;

    ++driver->write_calls;
    driver->last_pulse = pulse_us;
    record_event(driver, 'W', servo_id, pulse_us);
}

static bool fake_start(
    void *context,
    uint8_t servo_id)
{
    fake_driver_t *driver = (fake_driver_t *)context;

    ++driver->start_calls;
    record_event(driver, 'S', servo_id, 0U);
    return driver->start_result;
}

static void fake_stop(
    void *context,
    uint8_t servo_id)
{
    fake_driver_t *driver = (fake_driver_t *)context;

    ++driver->stop_calls;
    record_event(driver, 'T', servo_id, 0U);
}

static const servo_service_driver_ops_t fake_ops = {
    .write_pulse_us = fake_write_pulse,
    .start = fake_start,
    .stop = fake_stop,
};

static void init_service(
    servo_service_t *service,
    fake_driver_t *driver)
{
    (void)memset(driver, 0, sizeof(*driver));
    driver->start_result = true;
    servo_service_init(service, &fake_ops, driver);
}

static void test_mask_validation(void)
{
    fake_driver_t driver;
    servo_service_t service;

    init_service(&service, &driver);

    expect(servo_service_validate_mask(0U) ==
               SERVO_SERVICE_RESULT_INVALID_PAYLOAD,
           "zero mask should be invalid");
    expect(servo_service_validate_mask(0x0002U) ==
               SERVO_SERVICE_RESULT_UNSUPPORTED_SERVO,
           "Servo2 mask should be unsupported");
    expect(servo_service_validate_mask(0x0001U) ==
               SERVO_SERVICE_RESULT_OK,
           "Servo1 mask should be supported");
    expect(servo_service_disable(&service, 0U) ==
               SERVO_SERVICE_RESULT_INVALID_PAYLOAD,
           "disable zero mask should be invalid");
}

static void test_enable_sequence_and_start_failure(void)
{
    fake_driver_t driver;
    servo_service_t service;

    init_service(&service, &driver);

    expect(servo_service_enable(
               &service,
               SERVO_SERVICE_SERVO1_MASK) == SERVO_SERVICE_RESULT_OK,
           "Servo1 enable should succeed");
    expect(driver.event_count == 2U &&
               driver.events[0] == 'W' &&
               driver.events[1] == 'S',
           "enable must write neutral before start");
    expect(driver.event_pulse[0] == 1520U,
           "enable must write the neutral pulse");
    expect(servo_service_enabled_mask(&service) ==
               SERVO_SERVICE_SERVO1_MASK,
           "successful enable must set state");

    init_service(&service, &driver);
    driver.start_result = false;

    expect(servo_service_enable(
               &service,
               SERVO_SERVICE_SERVO1_MASK) ==
               SERVO_SERVICE_RESULT_HARDWARE_FAILURE,
           "start failure should return hardware failure");
    expect(servo_service_enabled_mask(&service) == 0U,
           "start failure must not set enabled state");
}

static void test_pwm_and_angle_rules(void)
{
    fake_driver_t driver;
    servo_service_t service;

    init_service(&service, &driver);

    expect(servo_service_set_pwm(
               &service,
               SERVO_SERVICE_SERVO1_ID,
               1520U) == SERVO_SERVICE_RESULT_SERVO_NOT_ENABLED,
           "PWM before enable should be rejected");
    expect(servo_service_set_angle(
               &service,
               SERVO_SERVICE_SERVO1_ID,
               0) == SERVO_SERVICE_RESULT_SERVO_NOT_ENABLED,
           "angle before enable should be rejected");

    expect(servo_service_enable(
               &service,
               SERVO_SERVICE_SERVO1_MASK) == SERVO_SERVICE_RESULT_OK,
           "setup enable should succeed");

    expect(servo_service_set_pwm(
               &service,
               SERVO_SERVICE_SERVO1_ID,
               1000U) == SERVO_SERVICE_RESULT_OK,
           "valid PWM should succeed");
    expect(driver.last_pulse == 1000U,
           "valid PWM should reach the driver");
    expect(servo_service_set_pwm(
               &service,
               SERVO_SERVICE_SERVO1_ID,
               520U) == SERVO_SERVICE_RESULT_OK,
           "minimum PWM should be accepted");
    expect(driver.last_pulse == 520U,
           "minimum PWM should reach the driver");
    expect(servo_service_set_pwm(
               &service,
               SERVO_SERVICE_SERVO1_ID,
               2520U) == SERVO_SERVICE_RESULT_OK,
           "maximum PWM should be accepted");
    expect(driver.last_pulse == 2520U,
           "maximum PWM should reach the driver");
    expect(servo_service_set_pwm(
               &service,
               SERVO_SERVICE_SERVO1_ID,
               519U) == SERVO_SERVICE_RESULT_OUT_OF_RANGE,
           "PWM below range should be rejected");
    expect(servo_service_set_pwm(
               &service,
               SERVO_SERVICE_SERVO1_ID,
               2521U) == SERVO_SERVICE_RESULT_OUT_OF_RANGE,
           "PWM above range should be rejected");
    expect(servo_service_set_pwm(
               &service,
               1U,
               1520U) == SERVO_SERVICE_RESULT_UNSUPPORTED_SERVO,
           "Servo2 PWM should be rejected");

    expect(servo_service_set_angle(
               &service,
               SERVO_SERVICE_SERVO1_ID,
               -4500) == SERVO_SERVICE_RESULT_OK,
           "-4500 cdeg should succeed");
    expect(driver.last_pulse == 1020U,
           "-4500 cdeg should write 1020 us");
    expect(servo_service_set_angle(
               &service,
               SERVO_SERVICE_SERVO1_ID,
               4500) == SERVO_SERVICE_RESULT_OK,
           "+4500 cdeg should succeed");
    expect(driver.last_pulse == 2020U,
           "+4500 cdeg should write 2020 us");
    expect(servo_service_set_angle(
               &service,
               SERVO_SERVICE_SERVO1_ID,
               -9000) == SERVO_SERVICE_RESULT_OK,
           "-9000 cdeg should succeed");
    expect(driver.last_pulse == 520U,
           "-9000 cdeg should write 520 us");
    expect(servo_service_set_angle(
               &service,
               SERVO_SERVICE_SERVO1_ID,
               9000) == SERVO_SERVICE_RESULT_OK,
           "+9000 cdeg should succeed");
    expect(driver.last_pulse == 2520U,
           "+9000 cdeg should write 2520 us");
    expect(servo_service_set_angle(
               &service,
               SERVO_SERVICE_SERVO1_ID,
               -9001) == SERVO_SERVICE_RESULT_OUT_OF_RANGE,
           "angle below range should be rejected");
    expect(servo_service_set_angle(
               &service,
               SERVO_SERVICE_SERVO1_ID,
               9001) == SERVO_SERVICE_RESULT_OUT_OF_RANGE,
           "angle above range should be rejected");
}

static void test_neutral_disable_and_disable_all(void)
{
    fake_driver_t driver;
    servo_service_t service;

    init_service(&service, &driver);

    expect(servo_service_neutral(
               &service,
               SERVO_SERVICE_SERVO1_MASK) ==
               SERVO_SERVICE_RESULT_SERVO_NOT_ENABLED,
           "neutral before enable should be rejected");

    expect(servo_service_enable(
               &service,
               SERVO_SERVICE_SERVO1_MASK) == SERVO_SERVICE_RESULT_OK,
           "neutral setup enable should succeed");
    expect(servo_service_neutral(
               &service,
               SERVO_SERVICE_SERVO1_MASK) ==
               SERVO_SERVICE_RESULT_OK,
           "neutral after enable should succeed");
    expect(driver.last_pulse == 1520U,
           "neutral should write 1520 us");
    expect(servo_service_enabled_mask(&service) ==
               SERVO_SERVICE_SERVO1_MASK,
           "neutral must preserve enabled state");

    expect(servo_service_disable(
               &service,
               SERVO_SERVICE_SERVO1_MASK) == SERVO_SERVICE_RESULT_OK,
           "disable should succeed");
    expect(driver.stop_calls == 1U &&
               servo_service_enabled_mask(&service) == 0U,
           "disable should stop and clear state");

    expect(servo_service_enable(
               &service,
               SERVO_SERVICE_SERVO1_MASK) == SERVO_SERVICE_RESULT_OK,
           "disable-all setup enable should succeed");
    servo_service_disable_all(&service);
    expect(driver.stop_calls == 2U &&
               servo_service_enabled_mask(&service) == 0U,
           "disable-all should stop and clear state");
}

int main(void)
{
    test_mask_validation();
    test_enable_sequence_and_start_failure();
    test_pwm_and_angle_rules();
    test_neutral_disable_and_disable_all();

    if (failures == 0)
    {
        (void)puts("All firmware Servo service tests passed");
    }

    return failures == 0 ? 0 : 1;
}
