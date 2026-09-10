#include "jy901s_transport_stm32.h"

#include "ring_buffer.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define JY901S_RX_BUFFER_STORAGE_SIZE 256U

static UART_HandleTypeDef *uart_handle;
static uint8_t rx_byte;
static uint8_t rx_buffer_storage[JY901S_RX_BUFFER_STORAGE_SIZE];
static ring_buffer_t rx_buffer;
static volatile jy901s_transport_stm32_diagnostics_t diagnostics;
static volatile uint32_t rx_rearm_generation;

static void mark_needs_rearm(void)
{
    diagnostics.rx_needs_rearm = true;
    diagnostics.rx_armed = false;
    ++rx_rearm_generation;
}

static void complete_rearm_if_unchanged(uint32_t generation)
{
    if (generation != rx_rearm_generation)
    {
        mark_needs_rearm();
        return;
    }

    diagnostics.rx_needs_rearm = false;
    if (generation != rx_rearm_generation)
    {
        mark_needs_rearm();
        return;
    }

    diagnostics.rx_armed = true;
    if (generation != rx_rearm_generation)
    {
        mark_needs_rearm();
    }
}

static void try_rearm(void)
{
    HAL_StatusTypeDef status;
    uint32_t generation;

    if (uart_handle == NULL)
    {
        mark_needs_rearm();
        ++diagnostics.rx_rearm_failure_count;
        return;
    }

    generation = rx_rearm_generation;

    /* An already-active receive is the successful recovery state. */
    if (uart_handle->RxState == HAL_UART_STATE_BUSY_RX)
    {
        complete_rearm_if_unchanged(generation);
        return;
    }

    mark_needs_rearm();
    generation = rx_rearm_generation;
    status = HAL_UART_Receive_IT(uart_handle, &rx_byte, 1U);

    if (status == HAL_OK)
    {
        complete_rearm_if_unchanged(generation);
        return;
    }

    if (status == HAL_BUSY)
    {
        /* HAL_BUSY is deferred work, not a hard re-arm failure. */
        ++diagnostics.rx_rearm_deferred_count;
        if (uart_handle->RxState == HAL_UART_STATE_BUSY_RX)
        {
            complete_rearm_if_unchanged(generation);
        }
    }
    else
    {
        ++diagnostics.rx_rearm_failure_count;
    }
}

void jy901s_transport_stm32_init(UART_HandleTypeDef *huart)
{
    uart_handle = huart;
    rx_byte = 0U;
    diagnostics =
        (jy901s_transport_stm32_diagnostics_t){0};
    rx_rearm_generation = 0U;
    ring_buffer_init(
        &rx_buffer,
        rx_buffer_storage,
        (uint16_t)sizeof rx_buffer_storage);
    try_rearm();
}

void jy901s_transport_stm32_on_rx_complete(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART3))
    {
        return;
    }

    ++diagnostics.rx_byte_count;

    if (ring_buffer_push(&rx_buffer, rx_byte))
    {
        ++diagnostics.rx_buffer_push_count;
    }
    else
    {
        ++diagnostics.rx_buffer_overflow_count;
    }

    /* HAL completes a one-byte receive before invoking this callback. */
    mark_needs_rearm();
}

void jy901s_transport_stm32_on_error(UART_HandleTypeDef *huart)
{
    const uint32_t known_error_flags =
        HAL_UART_ERROR_PE |
        HAL_UART_ERROR_NE |
        HAL_UART_ERROR_FE |
        HAL_UART_ERROR_ORE |
        HAL_UART_ERROR_DMA;
    const uint32_t blocking_error_flags =
        HAL_UART_ERROR_ORE | HAL_UART_ERROR_DMA;
    uint32_t error_code;

    if ((huart == NULL) || (huart->Instance != USART3))
    {
        return;
    }

    error_code = huart->ErrorCode;
    ++rx_rearm_generation;
    ++diagnostics.uart_error_count;

    if ((error_code & HAL_UART_ERROR_ORE) != 0U)
    {
        ++diagnostics.uart_overrun_error_count;
    }
    if ((error_code & HAL_UART_ERROR_FE) != 0U)
    {
        ++diagnostics.uart_framing_error_count;
    }
    if ((error_code & HAL_UART_ERROR_NE) != 0U)
    {
        ++diagnostics.uart_noise_error_count;
    }
    if ((error_code & HAL_UART_ERROR_PE) != 0U)
    {
        ++diagnostics.uart_parity_error_count;
    }
    if ((error_code & HAL_UART_ERROR_DMA) != 0U)
    {
        ++diagnostics.uart_dma_error_count;
    }
    if ((error_code == HAL_UART_ERROR_NONE) ||
        ((error_code & ~known_error_flags) != 0U))
    {
        ++diagnostics.uart_other_error_count;
    }

    if (((error_code & (blocking_error_flags | ~known_error_flags)) == 0U) &&
        (huart->RxState == HAL_UART_STATE_BUSY_RX))
    {
        diagnostics.rx_armed = true;
        diagnostics.rx_needs_rearm = false;
    }
    else
    {
        mark_needs_rearm();
    }
}

void jy901s_transport_stm32_poll(void)
{
    if (diagnostics.rx_needs_rearm)
    {
        try_rearm();
    }
}

bool jy901s_transport_stm32_pop(uint8_t *byte)
{
    if (ring_buffer_pop(&rx_buffer, byte))
    {
        ++diagnostics.rx_pop_count;
        return true;
    }

    return false;
}

void jy901s_transport_stm32_get_diagnostics(
    jy901s_transport_stm32_diagnostics_t *output)
{
    if (output == NULL)
    {
        return;
    }

    output->rx_byte_count = diagnostics.rx_byte_count;
    output->rx_buffer_push_count = diagnostics.rx_buffer_push_count;
    output->rx_pop_count = diagnostics.rx_pop_count;
    output->rx_buffer_overflow_count =
        diagnostics.rx_buffer_overflow_count;
    output->rx_rearm_failure_count =
        diagnostics.rx_rearm_failure_count;
    output->rx_rearm_deferred_count =
        diagnostics.rx_rearm_deferred_count;
    output->uart_error_count = diagnostics.uart_error_count;
    output->uart_overrun_error_count =
        diagnostics.uart_overrun_error_count;
    output->uart_framing_error_count =
        diagnostics.uart_framing_error_count;
    output->uart_noise_error_count =
        diagnostics.uart_noise_error_count;
    output->uart_parity_error_count =
        diagnostics.uart_parity_error_count;
    output->uart_dma_error_count = diagnostics.uart_dma_error_count;
    output->uart_other_error_count = diagnostics.uart_other_error_count;
    output->rx_armed = diagnostics.rx_armed;
    output->rx_needs_rearm = diagnostics.rx_needs_rearm;
}
