#ifndef ROBOBEETLE_LEAK_SENSOR_STM32_H
#define ROBOBEETLE_LEAK_SENSOR_STM32_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} leak_sensor_stm32_t;

void leak_sensor_stm32_init(
    leak_sensor_stm32_t *reader,
    GPIO_TypeDef *port,
    uint16_t pin);

bool leak_sensor_stm32_read_level(
    const leak_sensor_stm32_t *reader);

#endif /* ROBOBEETLE_LEAK_SENSOR_STM32_H */
