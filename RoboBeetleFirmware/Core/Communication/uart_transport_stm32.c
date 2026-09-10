#include "uart_transport_stm32.h"

#include "ring_buffer.h"

static UART_HandleTypeDef *uart_handle;
static uint8_t rx_byte;
static uint8_t rx_buffer_storage[RING_BUFFER_STORAGE_SIZE];
static ring_buffer_t rx_buffer;

void uart_transport_stm32_init(UART_HandleTypeDef *huart)
{
    uart_handle = huart;
    ring_buffer_init(
        &rx_buffer,
        rx_buffer_storage,
        (uint16_t)sizeof rx_buffer_storage);

    (void)HAL_UART_Receive_IT(
        uart_handle,
        &rx_byte,
        1U);
}

void uart_transport_stm32_on_rx_complete(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        (void)ring_buffer_push(&rx_buffer, rx_byte);

        (void)HAL_UART_Receive_IT(
            uart_handle,
            &rx_byte,
            1U);
    }
}

bool uart_transport_stm32_pop(uint8_t *byte)
{
    return ring_buffer_pop(&rx_buffer, byte);
}

HAL_StatusTypeDef uart_transport_stm32_transmit(
    const uint8_t *data,
    uint16_t length)
{
    return HAL_UART_Transmit(
        uart_handle,
        data,
        length,
        100U);
}
