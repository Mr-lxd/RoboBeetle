#include "leak_sensor.h"

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

int main(void)
{
    test_initial_state_is_unknown();
    test_gpio_polarity();
    test_state_update();

    if (failures != 0)
    {
        return 1;
    }

    (void)puts("All firmware leak sensor tests passed");
    return 0;
}
