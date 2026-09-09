#ifndef ROBOBEETLE_APP_MAIN_H
#define ROBOBEETLE_APP_MAIN_H

#include "stm32f4xx_hal.h"

void app_main_init(
    UART_HandleTypeDef *uart,
    TIM_HandleTypeDef *tim3,
    TIM_HandleTypeDef *tim4,
    GPIO_TypeDef *leak_gpio_port,
    uint16_t leak_gpio_pin);

void app_main_process(void);

#endif /* ROBOBEETLE_APP_MAIN_H */
