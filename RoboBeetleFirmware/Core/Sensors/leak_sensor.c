#include "leak_sensor.h"

void leak_sensor_init(leak_sensor_t *sensor)
{
    sensor->state = LEAK_SENSOR_STATE_UNKNOWN;
}

leak_sensor_state_t leak_sensor_state_from_gpio_level(bool gpio_high)
{
    return gpio_high
               ? LEAK_SENSOR_STATE_DRY
               : LEAK_SENSOR_STATE_WET;
}

void leak_sensor_update_from_gpio_level(
    leak_sensor_t *sensor,
    bool gpio_high)
{
    sensor->state =
        leak_sensor_state_from_gpio_level(gpio_high);
}

leak_sensor_state_t leak_sensor_state(
    const leak_sensor_t *sensor)
{
    return sensor->state;
}

bool leak_sensor_state_is_valid(
    leak_sensor_state_t state)
{
    return state == LEAK_SENSOR_STATE_UNKNOWN ||
           state == LEAK_SENSOR_STATE_DRY ||
           state == LEAK_SENSOR_STATE_WET;
}
