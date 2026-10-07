#ifndef ROBOBEETLE_UART_TRANSPORT_STM32_H
#define ROBOBEETLE_UART_TRANSPORT_STM32_H

#include "stm32f4xx_hal.h"
#include "uart_tx_queue.h"

#include <stdbool.h>
#include <stdint.h>

#define UART_TX_MAX_CONSECUTIVE_START_BUSY 8U
#define UART_TX_MAX_REINITIALIZE_ABORT_BUSY_RETRIES 2U

typedef enum
{
    UART_TRANSPORT_STATE_UNINITIALIZED = 0,
    UART_TRANSPORT_STATE_IDLE,
    UART_TRANSPORT_STATE_STARTING,
    UART_TRANSPORT_STATE_ACTIVE,
    UART_TRANSPORT_STATE_START_DEFERRED,
    UART_TRANSPORT_STATE_REINITIALIZING,
    UART_TRANSPORT_STATE_RECOVERY_FAILED
} uart_transport_stm32_state_t;

typedef struct
{
    uint32_t enqueued_count[UART_TX_MESSAGE_KIND_COUNT];
    uint32_t coalesced_count[UART_TX_MESSAGE_KIND_COUNT];
    uint32_t rejected_count[UART_TX_MESSAGE_KIND_COUNT];
    uint32_t completed_count[UART_TX_MESSAGE_KIND_COUNT];
    uint32_t dropped_count[UART_TX_MESSAGE_KIND_COUNT];
    uint32_t control_queue_full_count;
    uint32_t telemetry_queue_full_count;
    uint32_t start_busy_count;
    uint32_t start_error_count;
    uint32_t uart_error_count;
    uint32_t unexpected_callback_count;
    uint32_t high_water_mark;
    uint32_t reinitialization_count;
    uint32_t busy_recovery_count;
    uint32_t rx_error_count;
    uint32_t rx_rearm_attempt_count;
    uint32_t rx_rearm_busy_count;
    uint32_t rx_rearm_error_count;
    bool rx_armed;
    bool rx_needs_rearm;
} uart_transport_stm32_diagnostics_t;

void uart_transport_stm32_init(UART_HandleTypeDef *huart);
void uart_transport_stm32_on_rx_complete(UART_HandleTypeDef *huart);
void uart_transport_stm32_on_tx_complete(UART_HandleTypeDef *huart);
void uart_transport_stm32_on_abort_transmit_complete(
    UART_HandleTypeDef *huart);
void uart_transport_stm32_on_error(UART_HandleTypeDef *huart);
void uart_transport_stm32_process(void);

bool uart_transport_stm32_motion_pending(void);

uart_tx_enqueue_result_t uart_transport_stm32_enqueue(
    const uint8_t *data,
    uint16_t length,
    uart_tx_message_kind_t kind);

HAL_StatusTypeDef uart_transport_stm32_reinitialize(void);

bool uart_transport_stm32_pop(uint8_t *byte);

uart_transport_stm32_state_t uart_transport_stm32_get_state(void);

void uart_transport_stm32_get_diagnostics(
    uart_transport_stm32_diagnostics_t *diagnostics);

#if defined(ROBOBEETLE_UART_TRANSPORT_HOST_TEST)
void uart_transport_stm32_host_set_before_abort_hook(
    void (*hook)(void));
#endif

#endif /* ROBOBEETLE_UART_TRANSPORT_STM32_H */
