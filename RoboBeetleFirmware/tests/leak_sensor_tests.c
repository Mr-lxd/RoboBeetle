#include "leak_sensor.h"
#include "leak_telemetry_policy.h"

#include <stdbool.h>
#include <stdio.h>

static int failures = 0;

static void expect(bool condition, const char *message)
{
    if (!condition)
    {
        ++failures;
        (void)fprintf(stderr, "FAIL: %s\n", message);
    }
}

static void test_initial_state_is_unknown(void)
{
    leak_sensor_t sensor;

    leak_sensor_init(&sensor);

    expect(leak_sensor_state(&sensor) ==
               LEAK_SENSOR_STATE_UNKNOWN,
           "initial leak state should be unknown before first sample");
}

static void test_gpio_polarity(void)
{
    expect(leak_sensor_state_from_gpio_level(true) ==
               LEAK_SENSOR_STATE_DRY,
           "GPIO HIGH should map to dry");
    expect(leak_sensor_state_from_gpio_level(false) ==
               LEAK_SENSOR_STATE_WET,
           "GPIO LOW should map to wet");
}

static void test_state_update(void)
{
    leak_sensor_t sensor;

    leak_sensor_init(&sensor);
    leak_sensor_update_from_gpio_level(&sensor, true);
    expect(leak_sensor_state(&sensor) ==
               LEAK_SENSOR_STATE_DRY,
           "HIGH update should store dry state");

    leak_sensor_update_from_gpio_level(&sensor, false);
    expect(leak_sensor_state(&sensor) ==
               LEAK_SENSOR_STATE_WET,
           "LOW update should store wet state");
}

static void test_state_validity(void)
{
    expect(leak_sensor_state_is_valid(LEAK_SENSOR_STATE_UNKNOWN),
           "UNKNOWN leak state should be valid for telemetry");
    expect(leak_sensor_state_is_valid(LEAK_SENSOR_STATE_DRY),
           "DRY leak state should be valid for telemetry");
    expect(leak_sensor_state_is_valid(LEAK_SENSOR_STATE_WET),
           "WET leak state should be valid for telemetry");
    expect(!leak_sensor_state_is_valid((leak_sensor_state_t)3),
           "out-of-range leak state must be rejected");
}

static void test_telemetry_publish_policy(void)
{
    leak_telemetry_policy_t policy;

    leak_telemetry_policy_init(&policy);
    expect(leak_telemetry_policy_should_publish(
               &policy, LEAK_SENSOR_STATE_DRY, 100U, 500U),
           "first valid leak state should publish");
    leak_telemetry_policy_mark_published(
        &policy, LEAK_SENSOR_STATE_DRY, 100U);
    expect(!leak_telemetry_policy_should_publish(
               &policy, LEAK_SENSOR_STATE_DRY, 599U, 500U),
           "unchanged leak state before refresh must not publish");
    expect(leak_telemetry_policy_should_publish(
               &policy, LEAK_SENSOR_STATE_DRY, 600U, 500U),
           "unchanged leak state after refresh should publish");
    expect(leak_telemetry_policy_should_publish(
               &policy, LEAK_SENSOR_STATE_WET, 601U, 500U),
           "leak state change should publish immediately");
    expect(!leak_telemetry_policy_should_publish(
               &policy, (leak_sensor_state_t)3, 602U, 500U),
           "invalid leak state must never publish");
}

int main(void)
{
    test_initial_state_is_unknown();
    test_gpio_polarity();
    test_state_update();
    test_state_validity();
    test_telemetry_publish_policy();

    if (failures != 0)
    {
        return 1;
    }

    (void)puts("All firmware leak sensor tests passed");
    return 0;
}
