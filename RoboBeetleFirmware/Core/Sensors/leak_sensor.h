#ifndef ROBOBEETLE_LEAK_SENSOR_H
#define ROBOBEETLE_LEAK_SENSOR_H

#include <stdbool.h>

typedef enum
{
    LEAK_SENSOR_STATE_UNKNOWN = 0,
    LEAK_SENSOR_STATE_DRY,
    LEAK_SENSOR_STATE_WET
} leak_sensor_state_t;

typedef struct
{
    leak_sensor_state_t state;
} leak_sensor_t;

void leak_sensor_init(leak_sensor_t *sensor);

leak_sensor_state_t leak_sensor_state_from_gpio_level(bool gpio_high);

void leak_sensor_update_from_gpio_level(
    leak_sensor_t *sensor,
    bool gpio_high);

leak_sensor_state_t leak_sensor_state(
    const leak_sensor_t *sensor);

#endif /* ROBOBEETLE_LEAK_SENSOR_H */
