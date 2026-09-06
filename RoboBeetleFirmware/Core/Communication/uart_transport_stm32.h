#ifndef ROBOBEETLE_UART_TRANSPORT_STM32_H
#define ROBOBEETLE_UART_TRANSPORT_STM32_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

void uart_transport_stm32_init(UART_HandleTypeDef *huart);
void uart_transport_stm32_on_rx_complete(UART_HandleTypeDef *huart);
bool uart_transport_stm32_pop(uint8_t *byte);
HAL_StatusTypeDef uart_transport_stm32_transmit(
    const uint8_t *data,
    uint16_t length);

#endif /* ROBOBEETLE_UART_TRANSPORT_STM32_H */
