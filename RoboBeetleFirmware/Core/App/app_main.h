#ifndef ROBOBEETLE_APP_MAIN_H
#define ROBOBEETLE_APP_MAIN_H

#include "stm32f4xx_hal.h"

void app_main_init(
    UART_HandleTypeDef *uart,
    TIM_HandleTypeDef *servo_timer);

void app_main_process(void);

#endif /* ROBOBEETLE_APP_MAIN_H */
