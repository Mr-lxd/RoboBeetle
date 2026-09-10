#ifndef ROBOBEETLE_DEPTH_TRANSPORT_STM32_H
#define ROBOBEETLE_DEPTH_TRANSPORT_STM32_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define DEPTH_TRANSPORT_RX_BUFFER_STORAGE_SIZE 512U

typedef struct
{
    uint32_t rx_byte_count;
    uint32_t rx_buffer_push_count;
    uint32_t rx_pop_count;
    uint32_t rx_buffer_overflow_count;
    uint32_t hard_rearm_failure_count;
    uint32_t rx_rearm_deferred_count;
    uint32_t uart_error_count;
    uint32_t uart_overrun_error_count;
    uint32_t uart_framing_error_count;
    uint32_t uart_noise_error_count;
    uint32_t uart_parity_error_count;
    uint32_t uart_dma_error_count;
    uint32_t uart_other_error_count;
    bool rx_armed;
    bool rx_needs_rearm;
} depth_transport_stm32_diagnostics_t;

void depth_transport_stm32_init(UART_HandleTypeDef *huart);
void depth_transport_stm32_on_rx_complete(UART_HandleTypeDef *huart);
void depth_transport_stm32_on_error(UART_HandleTypeDef *huart);
void depth_transport_stm32_poll(void);
bool depth_transport_stm32_pop(uint8_t *byte);
void depth_transport_stm32_get_diagnostics(
    depth_transport_stm32_diagnostics_t *diagnostics);

#endif /* ROBOBEETLE_DEPTH_TRANSPORT_STM32_H */
