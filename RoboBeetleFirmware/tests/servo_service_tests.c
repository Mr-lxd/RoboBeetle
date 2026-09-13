#include "servo_descriptor.h"
#include "servo_service.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void expect(bool condition, const char *message)
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
    uint16_t start_fail_mask;
    unsigned int event_count;
    char events[64];
    uint8_t event_servo[64];
    uint16_t event_pulse[64];
    unsigned int start_calls;
    unsigned int stop_calls;
    unsigned int write_calls;
    uint16_t last_pulse;
    uint16_t current_pulse[SERVO_DESCRIPTOR_COUNT];
} fake_driver_t;

static void record_event(
    fake_driver_t *driver,
    char event,
    uint8_t servo_id,
    uint16_t pulse_us)
{
    if (driver->event_count < 64U)
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
    driver->current_pulse[servo_id] = pulse_us;
    record_event(driver, 'W', servo_id, pulse_us);
}

static bool fake_start(
    void *context,
    uint8_t servo_id)
{
    fake_driver_t *driver = (fake_driver_t *)context;

    ++driver->start_calls;
    record_event(driver, 'S', servo_id, 0U);
    if ((driver->start_fail_mask &
         (uint16_t)(1U << servo_id)) != 0U)
    {
        return false;
    }

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

static unsigned int count_event(
    const fake_driver_t *driver,
    char event,
    uint8_t servo_id)
{
    unsigned int count = 0U;

    for (unsigned int index = 0U; index < driver->event_count; ++index)
    {
        if ((driver->events[index] == event) &&
            (driver->event_servo[index] == servo_id))
        {
            ++count;
        }
    }

    return count;
}

static bool last_write_pulse(
    const fake_driver_t *driver,
    uint8_t servo_id,
    uint16_t *pulse_us)
{
    for (unsigned int index = driver->event_count; index > 0U; --index)
    {
        const unsigned int event_index = index - 1U;

        if ((driver->events[event_index] == 'W') &&
            (driver->event_servo[event_index] == servo_id))
        {
            *pulse_us = driver->event_pulse[event_index];
            return true;
        }
    }

    return false;
}

static void test_mask_validation(void)
{
    fake_driver_t driver;
    servo_service_t service;

    init_service(&service, &driver);

    expect(servo_service_validate_mask(0U) ==
               SERVO_SERVICE_RESULT_INVALID_PAYLOAD,
           "zero mask should be invalid");
    expect(servo_service_validate_mask(0x001FU) ==
               SERVO_SERVICE_RESULT_OK,
           "all five servo bits should be supported");
    expect(servo_service_validate_mask(0x0020U) ==
               SERVO_SERVICE_RESULT_UNSUPPORTED_SERVO,
           "bit above the five-servo mask should be unsupported");
    expect(servo_service_validate_mask(0x001BU) ==
               SERVO_SERVICE_RESULT_OK,
           "a subset of the five servo bits should be supported");
}

static void test_enable_each_servo_and_independent_bits(void)
{
    fake_driver_t driver;
    servo_service_t service;
    const servo_descriptor_t *table = servo_descriptor_table();
    static const uint16_t expected_neutral_pulse_us[SERVO_DESCRIPTOR_COUNT] = {
        1450U, 1580U, 1745U, 1570U, 1450U,
    };

    init_service(&service, &driver);

    for (uint8_t id = 0U; id < SERVO_DESCRIPTOR_COUNT; ++id)
    {
        const uint16_t mask = table[id].mask;

        expect(servo_service_enable(&service, mask) ==
                   SERVO_SERVICE_RESULT_OK,
               "each supported servo should enable");
        expect((servo_service_enabled_mask(&service) & mask) != 0U,
               "each successful enable should set only its bit");
        expect(driver.event_servo[driver.event_count - 2U] == id &&
                   driver.events[driver.event_count - 2U] == 'W',
               "enable must write neutral for the requested semantic ID");
        expect(driver.event_pulse[driver.event_count - 2U] ==
                   table[id].calibration.neutral_pulse_us,
               "enable must use the descriptor neutral pulse");
        expect(driver.event_pulse[driver.event_count - 2U] ==
                   expected_neutral_pulse_us[id],
               "enable must use the approved servo neutral pulse");
        expect(driver.event_servo[driver.event_count - 1U] == id &&
                   driver.events[driver.event_count - 1U] == 'S',
               "enable must start the requested semantic ID");
    }

    expect(servo_service_enabled_mask(&service) == 0x001FU,
           "enabling five independent bits should produce 0x001f");

    for (uint8_t id = 0U; id < SERVO_DESCRIPTOR_COUNT; ++id)
    {
        uint16_t enable_pulse_us = 0U;
        expect(last_write_pulse(&driver, id, &enable_pulse_us) &&
                   enable_pulse_us == expected_neutral_pulse_us[id],
               "Enable must write the approved neutral pulse");
    }
}

static void test_multi_enable_rolls_back_on_start_failure(void)
{
    fake_driver_t driver;
    servo_service_t service;
    const uint16_t original_mask = 0x0001U;
    const uint16_t request_mask = 0x0007U;

    init_service(&service, &driver);
    expect(servo_service_enable(&service, original_mask) ==
               SERVO_SERVICE_RESULT_OK,
           "rollback setup servo should enable");
    expect(servo_service_set_pwm(&service, SERVO_ID_FRONT_RIGHT, 1600U) ==
               SERVO_SERVICE_RESULT_OK,
           "rollback setup should place FrontRight away from neutral");
    const unsigned int front_right_writes_before =
        count_event(&driver, 'W', SERVO_ID_FRONT_RIGHT);
    const unsigned int front_right_starts_before =
        count_event(&driver, 'S', SERVO_ID_FRONT_RIGHT);
    const unsigned int front_right_stops_before =
        count_event(&driver, 'T', SERVO_ID_FRONT_RIGHT);
    const unsigned int stops_before = driver.stop_calls;
    driver.start_fail_mask = 0x0004U;

    expect(servo_service_enable(&service, request_mask) ==
               SERVO_SERVICE_RESULT_HARDWARE_FAILURE,
           "multi-bit enable should fail when one channel cannot start");
    expect(servo_service_enabled_mask(&service) == original_mask,
           "failed multi-bit enable must restore the original enabled mask");
    expect(driver.stop_calls == stops_before + 1U,
           "failed multi-bit enable must stop only the newly started channel");
    expect(count_event(&driver, 'W', SERVO_ID_FRONT_RIGHT) ==
               front_right_writes_before,
           "idempotent multi-enable must not rewrite an already-enabled servo");
    expect(count_event(&driver, 'S', SERVO_ID_FRONT_RIGHT) ==
               front_right_starts_before,
           "idempotent multi-enable must not restart an already-enabled servo");
    expect(count_event(&driver, 'T', SERVO_ID_FRONT_RIGHT) ==
               front_right_stops_before,
           "rollback must not stop a servo enabled before the call");
    expect(driver.current_pulse[SERVO_ID_FRONT_RIGHT] == 1600U,
           "rollback must preserve the pre-existing FrontRight pulse");
    expect(count_event(&driver, 'T', SERVO_ID_FRONT_LEFT) == 1U,
           "rollback must stop FrontLeft after it started");
}

static void test_command_ranges_and_capabilities(void)
{
    fake_driver_t driver;
    servo_service_t service;
    static const struct
    {
        uint8_t id;
        uint16_t min_pulse_us;
        uint16_t max_pulse_us;
    } paddle_cases[] = {
        {SERVO_ID_FRONT_RIGHT, 1000U, 1900U},
        {SERVO_ID_FRONT_LEFT, 1140U, 2020U},
        {SERVO_ID_REAR_RIGHT, 1110U, 2030U},
        {SERVO_ID_REAR_LEFT, 960U, 1940U},
    };

    init_service(&service, &driver);
    expect(servo_service_enable(&service, 0x001FU) ==
               SERVO_SERVICE_RESULT_OK,
           "range setup should enable all five servos");

    for (size_t index = 0U;
         index < sizeof(paddle_cases) / sizeof(paddle_cases[0]);
         ++index)
    {
        const uint8_t id = paddle_cases[index].id;
        const uint16_t min_pulse_us = paddle_cases[index].min_pulse_us;
        const uint16_t max_pulse_us = paddle_cases[index].max_pulse_us;

        expect(servo_service_set_pwm(&service, id,
                                     (uint16_t)(min_pulse_us - 1U)) ==
                   SERVO_SERVICE_RESULT_OUT_OF_RANGE,
               "paddle raw PWM below the lower bound must be rejected");
        expect(servo_service_set_pwm(&service, id, min_pulse_us) ==
                   SERVO_SERVICE_RESULT_OK,
               "paddle raw PWM lower bound must be accepted");
        expect(servo_service_set_pwm(&service, id, max_pulse_us) ==
                   SERVO_SERVICE_RESULT_OK,
               "paddle raw PWM upper bound must be accepted");
        expect(servo_service_set_pwm(&service, id,
                                     (uint16_t)(max_pulse_us + 1U)) ==
                   SERVO_SERVICE_RESULT_OUT_OF_RANGE,
               "paddle raw PWM above the upper bound must be rejected");

        expect(servo_service_set_angle(&service, id, -4501) ==
                   SERVO_SERVICE_RESULT_OUT_OF_RANGE,
               "paddle angle below -45 degrees must be rejected");
        expect(servo_service_set_angle(&service, id, -4500) ==
                   SERVO_SERVICE_RESULT_OK,
               "paddle angle -45 degrees must be accepted");
        expect(servo_service_set_angle(&service, id, 4500) ==
                   SERVO_SERVICE_RESULT_OK,
               "paddle angle +45 degrees must be accepted");
        expect(servo_service_set_angle(&service, id, 4501) ==
                   SERVO_SERVICE_RESULT_OUT_OF_RANGE,
               "paddle angle above +45 degrees must be rejected");
    }

    expect(servo_service_set_pwm(&service, SERVO_ID_FRONT_AXIS, 1059U) ==
               SERVO_SERVICE_RESULT_OUT_OF_RANGE,
           "FrontAxis 1059 us must be rejected");
    expect(servo_service_set_pwm(&service, SERVO_ID_FRONT_AXIS, 1060U) ==
               SERVO_SERVICE_RESULT_OK,
           "FrontAxis 1060 us must be accepted");
    expect(servo_service_set_pwm(&service, SERVO_ID_FRONT_AXIS, 1745U) ==
               SERVO_SERVICE_RESULT_OK,
           "FrontAxis 1745 us must be accepted");
    expect(servo_service_set_pwm(&service, SERVO_ID_FRONT_AXIS, 2430U) ==
               SERVO_SERVICE_RESULT_OK,
           "FrontAxis 2430 us must be accepted");
    expect(servo_service_set_pwm(&service, SERVO_ID_FRONT_AXIS, 2431U) ==
               SERVO_SERVICE_RESULT_OUT_OF_RANGE,
           "FrontAxis 2431 us must be rejected");
    expect(servo_service_set_angle(&service, SERVO_ID_FRONT_AXIS, -9001) ==
               SERVO_SERVICE_RESULT_OUT_OF_RANGE,
           "FrontAxis below -90 degrees must be rejected");
    expect(servo_service_set_angle(&service, SERVO_ID_FRONT_AXIS, -9000) ==
               SERVO_SERVICE_RESULT_OK,
           "FrontAxis -90 degrees must be accepted");
    expect(servo_service_set_angle(&service, SERVO_ID_FRONT_AXIS, 0) ==
               SERVO_SERVICE_RESULT_OK,
           "FrontAxis zero degrees must be accepted");
    expect(servo_service_set_angle(&service, SERVO_ID_FRONT_AXIS, 9000) ==
               SERVO_SERVICE_RESULT_OK,
           "FrontAxis +90 degrees must be accepted");
    expect(servo_service_set_angle(&service, SERVO_ID_FRONT_AXIS, 9001) ==
               SERVO_SERVICE_RESULT_OUT_OF_RANGE,
           "FrontAxis above +90 degrees must be rejected");
}

static void test_unenabled_and_invalid_commands(void)
{
    fake_driver_t driver;
    servo_service_t service;
    const unsigned int writes_before = 0U;

    init_service(&service, &driver);

    expect(servo_service_set_pwm(&service, SERVO_ID_FRONT_LEFT, 1500U) ==
               SERVO_SERVICE_RESULT_SERVO_NOT_ENABLED,
           "PWM before Enable must be rejected");
    expect(servo_service_set_angle(&service, SERVO_ID_REAR_LEFT, 0) ==
               SERVO_SERVICE_RESULT_SERVO_NOT_ENABLED,
           "Angle before Enable must be rejected");
    expect(servo_service_set_pwm(&service, 5U, 1500U) ==
               SERVO_SERVICE_RESULT_UNSUPPORTED_SERVO,
           "out-of-range PWM ID must be rejected");
    expect(servo_service_set_angle(&service, 0xffU, 0) ==
               SERVO_SERVICE_RESULT_UNSUPPORTED_SERVO,
           "invalid angle ID must be rejected");
    expect(driver.write_calls == writes_before,
           "invalid/unenabled commands must not reach the driver");
}

static void test_neutral_disable_and_disable_all(void)
{
    fake_driver_t driver;
    servo_service_t service;
    static const uint16_t expected_neutral_pulse_us[SERVO_DESCRIPTOR_COUNT] = {
        1450U, 1580U, 1745U, 1570U, 1450U,
    };

    init_service(&service, &driver);
    expect(servo_service_enable(&service, 0x001FU) ==
               SERVO_SERVICE_RESULT_OK,
           "Neutral setup should enable all five servos");

    expect(servo_service_neutral(&service, 0x001FU) ==
               SERVO_SERVICE_RESULT_OK,
           "Neutral should accept all enabled servos");
    for (uint8_t id = 0U; id < SERVO_DESCRIPTOR_COUNT; ++id)
    {
        uint16_t neutral_pulse_us = 0U;
        expect(last_write_pulse(&driver, id, &neutral_pulse_us) &&
                   neutral_pulse_us == expected_neutral_pulse_us[id],
               "Neutral must write the approved neutral pulse");
    }
    expect(driver.write_calls >= 10U,
           "Neutral should write one pulse for each requested servo");
    expect(servo_service_disable(&service, 0x0002U) ==
               SERVO_SERVICE_RESULT_OK,
           "individual Disable should succeed");
    expect(servo_service_enabled_mask(&service) == 0x001DU,
           "individual Disable should clear only FrontLeft");
    expect(count_event(&driver, 'T', SERVO_ID_FRONT_LEFT) == 1U,
           "individual Disable should stop FrontLeft");

    const unsigned int stops_before = driver.stop_calls;
    servo_service_disable_all(&service);
    expect(servo_service_enabled_mask(&service) == 0U,
           "Disable All should clear every enabled bit");
    expect(driver.stop_calls == stops_before + 4U,
           "Disable All should stop all remaining enabled channels");
    expect(count_event(&driver, 'T', SERVO_ID_FRONT_RIGHT) == 1U,
           "Disable All should stop FrontRight");
    expect(count_event(&driver, 'T', SERVO_ID_FRONT_AXIS) == 1U,
           "Disable All should stop FrontAxis");
    expect(count_event(&driver, 'T', SERVO_ID_REAR_RIGHT) == 1U,
           "Disable All should stop RearRight");
    expect(count_event(&driver, 'T', SERVO_ID_REAR_LEFT) == 1U,
           "Disable All should stop RearLeft");
}

int main(void)
{
    test_mask_validation();
    test_enable_each_servo_and_independent_bits();
    test_multi_enable_rolls_back_on_start_failure();
    test_command_ranges_and_capabilities();
    test_unenabled_and_invalid_commands();
    test_neutral_disable_and_disable_all();

    if (failures == 0)
    {
        (void)puts("All firmware Servo service tests passed");
    }

    return failures == 0 ? 0 : 1;
}
