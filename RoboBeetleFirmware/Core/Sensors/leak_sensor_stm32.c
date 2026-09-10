#include "leak_sensor_stm32.h"

void leak_sensor_stm32_init(
    leak_sensor_stm32_t *reader,
    GPIO_TypeDef *port,
    uint16_t pin)
{
    reader->port = port;
    reader->pin = pin;
}

bool leak_sensor_stm32_read_level(
    const leak_sensor_stm32_t *reader)
{
    return HAL_GPIO_ReadPin(
               reader->port,
               reader->pin) == GPIO_PIN_SET;
}
