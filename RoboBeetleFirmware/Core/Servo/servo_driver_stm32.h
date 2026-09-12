#ifndef SERVO_DRIVER_STM32_H
#define SERVO_DRIVER_STM32_H

#include "servo_service.h"
#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    const servo_descriptor_t *descriptor;
    TIM_HandleTypeDef *timer;
    uint32_t hal_channel;
    bool channel_valid;
} servo_driver_stm32_binding_t;

typedef struct
{
    servo_driver_stm32_binding_t bindings[SERVO_DESCRIPTOR_COUNT];
} servo_driver_stm32_t;

void servo_driver_stm32_init(
    servo_driver_stm32_t *driver,
    TIM_HandleTypeDef *tim3,
    TIM_HandleTypeDef *tim4);

const servo_driver_stm32_binding_t *servo_driver_stm32_binding_for_id(
    const servo_driver_stm32_t *driver,
    uint8_t servo_id);

const servo_service_driver_ops_t *servo_driver_stm32_ops(void);

#endif /* SERVO_DRIVER_STM32_H */
