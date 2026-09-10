#ifndef ROBOBEETLE_APP_MAIN_H
#define ROBOBEETLE_APP_MAIN_H

#include "stm32f4xx_hal.h"
#include "jy901s_parser.h"
#include "jy901s_transport_stm32.h"
#include "depth_parser.h"
#include "depth_transport_stm32.h"

#include <stdint.h>

void app_main_init(
    UART_HandleTypeDef *uart,
    UART_HandleTypeDef *jy901s_uart,
    UART_HandleTypeDef *depth_uart,
    TIM_HandleTypeDef *tim3,
    TIM_HandleTypeDef *tim4,
    GPIO_TypeDef *leak_gpio_port,
    uint16_t leak_gpio_pin);

void app_main_process(void);

void app_main_jy901s_get_state(jy901s_imu_state_t *state);
void app_main_jy901s_get_parser_stats(jy901s_parser_stats_t *stats);
void app_main_jy901s_get_transport_diagnostics(
    jy901s_transport_stm32_diagnostics_t *diagnostics);
uint32_t app_main_jy901s_last_valid_frame_ms(void);

void app_main_depth_get_state(depth_parser_state_t *state);
void app_main_depth_get_parser_stats(depth_parser_stats_t *stats);
void app_main_depth_get_transport_diagnostics(
    depth_transport_stm32_diagnostics_t *diagnostics);

#endif /* ROBOBEETLE_APP_MAIN_H */
