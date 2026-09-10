#include "jy901s_transport_stm32.h"

#include <stdint.h>
#include <stdio.h>

static int failures = 0;
static uint8_t *receive_destination = NULL;
static HAL_StatusTypeDef receive_status = HAL_OK;
static uint32_t receive_call_count = 0U;

HAL_StatusTypeDef HAL_UART_Receive_IT(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size)
{
    receive_destination = data;
    ++receive_call_count;

    if ((huart == NULL) ||
        (huart->Instance != USART3) ||
        (size != 1U))
    {
        return HAL_ERROR;
    }

    return receive_status;
}

static void expect(int condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

static void reset_mock(void)
{
    receive_destination = NULL;
    receive_status = HAL_OK;
    receive_call_count = 0U;
}

static jy901s_transport_stm32_diagnostics_t diagnostics(void)
{
    jy901s_transport_stm32_diagnostics_t value;

    jy901s_transport_stm32_get_diagnostics(&value);
    return value;
}

static void deliver_byte(UART_HandleTypeDef *huart, uint8_t byte)
{
    expect(receive_destination != NULL, "RX staging pointer is missing");
    if (receive_destination != NULL)
    {
        *receive_destination = byte;
    }
    jy901s_transport_stm32_on_rx_complete(huart);
}

static void expect_pop(uint8_t expected)
{
    uint8_t actual = 0U;

    expect(
        jy901s_transport_stm32_pop(&actual),
        "expected a queued USART3 byte");
    expect(actual == expected, "USART3 FIFO order differs");
}

static void test_rearm_failure_recovery_and_subsequent_rx(void)
{
    UART_HandleTypeDef uart = {0};
    jy901s_transport_stm32_diagnostics_t value;

    uart.Instance = USART3;
    reset_mock();
    jy901s_transport_stm32_init(&uart);

    value = diagnostics();
    expect(receive_call_count == 1U, "initial RX arm call is missing");
    expect(value.rx_armed, "initial RX should be armed");
    expect(!value.rx_needs_rearm, "initial RX should not need re-arm");

    receive_status = HAL_BUSY;
    deliver_byte(&uart, 0xA1U);
    value = diagnostics();
    expect(value.rx_byte_count == 1U, "RX byte counter did not increment");
    expect(value.rx_buffer_push_count == 1U, "RX push counter did not increment");
    expect(value.rx_rearm_failure_count == 1U,
           "callback re-arm failure counter differs");
    expect(!value.rx_armed, "failed callback re-arm must clear armed state");
    expect(value.rx_needs_rearm, "failed callback re-arm must set pending state");

    deliver_byte(&uart, 0xA2U);
    value = diagnostics();
    expect(value.rx_byte_count == 2U,
           "RX callback should still account for a second delivered byte");
    expect(value.rx_rearm_failure_count == 2U,
           "foreground failed re-arm attempt was not counted");

    receive_status = HAL_OK;
    {
        uint32_t calls_before = receive_call_count;
        jy901s_transport_stm32_poll();
        expect(receive_call_count == calls_before + 1U,
               "foreground recovery did not attempt one re-arm");
    }
    value = diagnostics();
    expect(value.rx_rearm_failure_count == 2U,
           "successful recovery changed failure count");
    expect(value.rx_armed, "foreground recovery did not arm RX");
    expect(!value.rx_needs_rearm,
           "foreground recovery did not clear pending state");

    deliver_byte(&uart, 0xA3U);
    value = diagnostics();
    expect(value.rx_byte_count == 3U,
           "RX did not remain functional after recovery");
    expect(value.rx_buffer_push_count == 3U,
           "post-recovery RX push was not counted");
    expect(value.rx_rearm_failure_count == 2U,
           "post-recovery re-arm should succeed");

    expect_pop(0xA1U);
    expect_pop(0xA2U);
    expect_pop(0xA3U);
}

static void test_uart_error_enters_recovery_path(void)
{
    UART_HandleTypeDef uart = {0};
    jy901s_transport_stm32_diagnostics_t value;

    uart.Instance = USART3;
    uart.ErrorCode = HAL_UART_ERROR_ORE |
                     HAL_UART_ERROR_FE |
                     HAL_UART_ERROR_NE;
    reset_mock();
    jy901s_transport_stm32_init(&uart);
    jy901s_transport_stm32_on_error(&uart);

    value = diagnostics();
    expect(value.uart_error_count == 1U, "UART error count differs");
    expect(value.uart_overrun_error_count == 1U,
           "UART overrun error count differs");
    expect(value.uart_framing_error_count == 1U,
           "UART framing error count differs");
    expect(value.uart_noise_error_count == 1U,
           "UART noise error count differs");
    expect(!value.rx_armed, "UART error must clear armed state");
    expect(value.rx_needs_rearm, "UART error must set pending state");

    jy901s_transport_stm32_poll();
    value = diagnostics();
    expect(value.rx_armed, "UART error recovery did not arm RX");
    expect(!value.rx_needs_rearm,
           "UART error recovery did not clear pending state");

    uart.ErrorCode = HAL_UART_ERROR_DMA | 0x80000000U;
    jy901s_transport_stm32_on_error(&uart);
    value = diagnostics();
    expect(value.uart_error_count == 2U,
           "second UART error was not counted");
    expect(value.uart_dma_error_count == 1U,
           "UART DMA error count differs");
    expect(value.uart_other_error_count == 1U,
           "unknown UART error flag count differs");
}

static void test_ring_overflow_does_not_stop_rearm(void)
{
    UART_HandleTypeDef uart = {0};
    jy901s_transport_stm32_diagnostics_t value;

    uart.Instance = USART3;
    reset_mock();
    jy901s_transport_stm32_init(&uart);

    for (uint16_t index = 0U; index < 256U; ++index)
    {
        deliver_byte(&uart, (uint8_t)index);
    }

    value = diagnostics();
    expect(value.rx_byte_count == 256U,
           "overflow test RX byte count differs");
    expect(value.rx_buffer_push_count == 255U,
           "overflow test successful push count differs");
    expect(value.rx_buffer_overflow_count == 1U,
           "overflow test drop count differs");
    expect(receive_call_count == 257U,
           "RX was not re-armed after a full-buffer drop");
    expect(value.rx_armed, "RX should remain armed after overflow");
    expect(!value.rx_needs_rearm,
           "overflow should not leave RX pending re-arm");

    for (uint16_t index = 0U; index < 255U; ++index)
    {
        expect_pop((uint8_t)index);
    }
    expect(!jy901s_transport_stm32_pop(&(uint8_t){0U}),
           "overflow test ring should be empty after 255 pops");
}

static void test_other_uart_is_ignored(void)
{
    UART_HandleTypeDef uart3 = {0};
    UART_HandleTypeDef uart1 = {0};
    jy901s_transport_stm32_diagnostics_t before;
    jy901s_transport_stm32_diagnostics_t after;

    uart3.Instance = USART3;
    uart1.Instance = USART1;
    uart1.ErrorCode = HAL_UART_ERROR_ORE;
    reset_mock();
    jy901s_transport_stm32_init(&uart3);
    before = diagnostics();
    jy901s_transport_stm32_on_rx_complete(&uart1);
    jy901s_transport_stm32_on_error(&uart1);
    after = diagnostics();

    expect(
        receive_call_count == 1U,
        "USART1 callback should not re-arm the USART3 receiver");
    expect(after.rx_byte_count == before.rx_byte_count,
           "USART1 callback changed USART3 RX count");
    expect(after.uart_error_count == before.uart_error_count,
           "USART1 error changed USART3 error count");
}

int main(void)
{
    test_rearm_failure_recovery_and_subsequent_rx();
    test_uart_error_enters_recovery_path();
    test_ring_overflow_does_not_stop_rearm();
    test_other_uart_is_ignored();

    if (failures == 0)
    {
        (void)puts("All JY901S STM32 transport tests passed");
    }

    return failures == 0 ? 0 : 1;
}
