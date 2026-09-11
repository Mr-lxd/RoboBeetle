#include "depth_transport_stm32.h"

#include "ring_buffer.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static UART_HandleTypeDef *uart_handle;
static uint8_t rx_byte;
static uint8_t rx_buffer_storage[
    DEPTH_TRANSPORT_RX_BUFFER_STORAGE_SIZE];
static ring_buffer_t rx_buffer;
static volatile depth_transport_stm32_diagnostics_t diagnostics;
static volatile uint32_t rx_rearm_generation;
static volatile bool rearm_in_progress;

static bool is_target_uart(const UART_HandleTypeDef *huart)
{
    return (huart != NULL) &&
           (uart_handle != NULL) &&
           (huart->Instance == USART6);
}

static void mark_needs_rearm(void)
{
    diagnostics.rx_armed = false;
    diagnostics.rx_needs_rearm = true;
    ++rx_rearm_generation;
}

static bool complete_rearm_if_unchanged(uint32_t generation)
{
    if (generation != rx_rearm_generation)
    {
        return false;
    }

    diagnostics.rx_needs_rearm = false;
    if (generation != rx_rearm_generation)
    {
        mark_needs_rearm();
        return false;
    }

    diagnostics.rx_armed = true;
    if (generation != rx_rearm_generation)
    {
        mark_needs_rearm();
        return false;
    }

    return true;
}

static void try_rearm(void)
{
    HAL_StatusTypeDef status;
    uint32_t generation;

    if ((uart_handle == NULL) || rearm_in_progress)
    {
        return;
    }

    if (uart_handle->RxState == HAL_UART_STATE_BUSY_RX)
    {
        (void)complete_rearm_if_unchanged(rx_rearm_generation);
        return;
    }

    diagnostics.rx_armed = false;
    diagnostics.rx_needs_rearm = true;
    generation = rx_rearm_generation;
    rearm_in_progress = true;
    status = HAL_UART_Receive_IT(uart_handle, &rx_byte, 1U);
    rearm_in_progress = false;

    if (generation != rx_rearm_generation)
    {
        return;
    }

    if (status == HAL_OK)
    {
        (void)complete_rearm_if_unchanged(generation);
    }
    else if (status == HAL_BUSY)
    {
        ++diagnostics.rx_rearm_deferred_count;
        if (uart_handle->RxState == HAL_UART_STATE_BUSY_RX)
        {
            (void)complete_rearm_if_unchanged(generation);
        }
    }
    else
    {
        ++diagnostics.hard_rearm_failure_count;
    }
}

void depth_transport_stm32_init(UART_HandleTypeDef *huart)
{
    uart_handle = NULL;
    rx_byte = 0U;
    diagnostics =
        (depth_transport_stm32_diagnostics_t){0};
    rx_rearm_generation = 0U;
    rearm_in_progress = false;
    ring_buffer_init(
        &rx_buffer,
        rx_buffer_storage,
        (uint16_t)sizeof rx_buffer_storage);

    if ((huart != NULL) && (huart->Instance == USART6))
    {
        uart_handle = huart;
        try_rearm();
    }
}

void depth_transport_stm32_on_rx_complete(UART_HandleTypeDef *huart)
{
    if (!is_target_uart(huart))
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

    mark_needs_rearm();
    if (!rearm_in_progress)
    {
        try_rearm();
    }
}

void depth_transport_stm32_on_error(UART_HandleTypeDef *huart)
{
    const uint32_t known_error_flags =
        HAL_UART_ERROR_PE |
        HAL_UART_ERROR_NE |
        HAL_UART_ERROR_FE |
        HAL_UART_ERROR_ORE |
        HAL_UART_ERROR_DMA;
    uint32_t error_code;

    if (!is_target_uart(huart))
    {
        return;
    }

    error_code = huart->ErrorCode;
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

    mark_needs_rearm();
}

void depth_transport_stm32_poll(void)
{
    if (diagnostics.rx_needs_rearm)
    {
        try_rearm();
    }
}

bool depth_transport_stm32_pop(uint8_t *byte)
{
    if (ring_buffer_pop(&rx_buffer, byte))
    {
        ++diagnostics.rx_pop_count;
        return true;
    }

    return false;
}

void depth_transport_stm32_get_diagnostics(
    depth_transport_stm32_diagnostics_t *output)
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
    output->hard_rearm_failure_count =
        diagnostics.hard_rearm_failure_count;
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
