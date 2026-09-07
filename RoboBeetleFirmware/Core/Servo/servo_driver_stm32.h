#ifndef SERVO_DRIVER_STM32_H
#define SERVO_DRIVER_STM32_H

#include "servo_service.h"
#include "stm32f4xx_hal.h"

typedef struct
{
    TIM_HandleTypeDef *timer;
} servo_driver_stm32_t;

void servo_driver_stm32_init(
    servo_driver_stm32_t *driver,
    TIM_HandleTypeDef *timer);

const servo_service_driver_ops_t *servo_driver_stm32_ops(void);

#endif /* SERVO_DRIVER_STM32_H */
