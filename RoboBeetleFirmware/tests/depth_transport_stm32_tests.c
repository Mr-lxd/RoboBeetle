#include "depth_transport_stm32.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

static int failures = 0;
static uint8_t *receive_destination = NULL;
static HAL_StatusTypeDef receive_default_status = HAL_OK;
static HAL_StatusTypeDef receive_status_sequence[16U];
static size_t receive_status_length = 0U;
static size_t receive_status_index = 0U;
static uint32_t receive_call_count = 0U;

HAL_StatusTypeDef HAL_UART_Receive_IT(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size)
{
    HAL_StatusTypeDef status = receive_default_status;

    receive_destination = data;
    ++receive_call_count;

    if ((huart == NULL) ||
        (huart->Instance != USART6) ||
        (size != 1U))
    {
        return HAL_ERROR;
    }

    if (receive_status_index < receive_status_length)
    {
        status = receive_status_sequence[receive_status_index];
        ++receive_status_index;
    }

    if (status == HAL_OK)
    {
        huart->RxState = HAL_UART_STATE_BUSY_RX;
    }

    return status;
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
    receive_default_status = HAL_OK;
    receive_status_length = 0U;
    receive_status_index = 0U;
    receive_call_count = 0U;
}

static void set_status_sequence(
    const HAL_StatusTypeDef *statuses,
    size_t length)
{
    for (size_t index = 0U; index < length; ++index)
    {
        receive_status_sequence[index] = statuses[index];
    }
    receive_status_length = length;
    receive_status_index = 0U;
}

static depth_transport_stm32_diagnostics_t diagnostics(void)
{
    depth_transport_stm32_diagnostics_t value;

    depth_transport_stm32_get_diagnostics(&value);
    return value;
}

static void deliver_byte(UART_HandleTypeDef *huart, uint8_t byte)
{
    expect(receive_destination != NULL, "USART6 RX staging pointer is missing");
    if (receive_destination != NULL)
    {
        *receive_destination = byte;
    }
    huart->RxState = HAL_UART_STATE_READY;
    depth_transport_stm32_on_rx_complete(huart);
}

static void expect_pop(uint8_t expected)
{
    uint8_t actual = 0U;

    expect(
        depth_transport_stm32_pop(&actual),
        "expected a queued USART6 byte");
    expect(actual == expected, "USART6 FIFO order differs");
}

static void test_initial_and_completion_rearm(void)
{
    UART_HandleTypeDef uart = {0};
    depth_transport_stm32_diagnostics_t value;

    uart.Instance = USART6;
    reset_mock();
    depth_transport_stm32_init(&uart);
    value = diagnostics();

    expect(receive_call_count == 1U, "initial RX arm call is missing");
    expect(value.rx_armed, "initial RX should be armed");
    expect(!value.rx_needs_rearm, "initial RX should not be pending");

    deliver_byte(&uart, 0xA1U);
    value = diagnostics();
    expect(value.rx_byte_count == 1U, "completion byte count differs");
    expect(value.rx_buffer_push_count == 1U, "completion push count differs");
    expect(receive_call_count == 2U, "completion should re-arm once");
    expect(value.rx_armed, "successful completion re-arm should be armed");
    expect(!value.rx_needs_rearm, "successful completion re-arm remains pending");
    expect(value.hard_rearm_failure_count == 0U, "normal re-arm reported failure");
    expect_pop(0xA1U);
}

static void test_initial_busy_and_error_recover_in_foreground(void)
{
    static const HAL_StatusTypeDef busy_statuses[] = {HAL_BUSY, HAL_OK};
    static const HAL_StatusTypeDef error_statuses[] = {HAL_ERROR, HAL_OK};
    UART_HandleTypeDef uart = {0};
    depth_transport_stm32_diagnostics_t value;

    uart.Instance = USART6;

    reset_mock();
    set_status_sequence(
        busy_statuses,
        sizeof busy_statuses / sizeof busy_statuses[0]);
    depth_transport_stm32_init(&uart);
    value = diagnostics();
    expect(receive_call_count == 1U, "initial HAL_BUSY arm call is missing");
    expect(value.rx_rearm_deferred_count == 1U, "initial HAL_BUSY was not deferred");
    expect(value.hard_rearm_failure_count == 0U, "initial HAL_BUSY became hard failure");
    expect(value.rx_needs_rearm, "initial HAL_BUSY should remain pending");
    depth_transport_stm32_poll();
    value = diagnostics();
    expect(value.rx_armed, "initial HAL_BUSY foreground recovery did not arm");
    expect(!value.rx_needs_rearm, "initial HAL_BUSY recovery remains pending");

    uart.RxState = HAL_UART_STATE_READY;
    reset_mock();
    set_status_sequence(
        error_statuses,
        sizeof error_statuses / sizeof error_statuses[0]);
    depth_transport_stm32_init(&uart);
    value = diagnostics();
    expect(receive_call_count == 1U, "initial HAL_ERROR arm call is missing");
    expect(value.hard_rearm_failure_count == 1U, "initial HAL_ERROR was not counted");
    expect(value.rx_rearm_deferred_count == 0U, "initial HAL_ERROR was deferred");
    expect(value.rx_needs_rearm, "initial HAL_ERROR should remain pending");
    depth_transport_stm32_poll();
    value = diagnostics();
    expect(value.rx_armed, "initial HAL_ERROR foreground recovery did not arm");
    expect(!value.rx_needs_rearm, "initial HAL_ERROR recovery remains pending");
}

static void test_busy_completion_defers_to_foreground(void)
{
    static const HAL_StatusTypeDef statuses[] = {HAL_OK, HAL_BUSY, HAL_OK};
    UART_HandleTypeDef uart = {0};
    depth_transport_stm32_diagnostics_t value;

    uart.Instance = USART6;
    reset_mock();
    set_status_sequence(statuses, sizeof statuses / sizeof statuses[0]);
    depth_transport_stm32_init(&uart);
    deliver_byte(&uart, 0xB1U);

    value = diagnostics();
    expect(value.rx_rearm_deferred_count == 1U, "HAL_BUSY was not deferred");
    expect(value.hard_rearm_failure_count == 0U, "HAL_BUSY became hard failure");
    expect(!value.rx_armed, "busy re-arm should remain unarmed");
    expect(value.rx_needs_rearm, "busy re-arm should remain pending");

    depth_transport_stm32_poll();
    value = diagnostics();
    expect(receive_call_count == 3U, "foreground busy recovery call is missing");
    expect(value.rx_armed, "foreground busy recovery did not arm");
    expect(!value.rx_needs_rearm, "foreground busy recovery remains pending");

    deliver_byte(&uart, 0xB2U);
    value = diagnostics();
    expect(value.rx_byte_count == 2U, "RX did not continue after foreground recovery");
    expect(value.rx_buffer_push_count == 2U, "post-recovery push count differs");
    expect(receive_call_count == 4U, "post-recovery completion did not re-arm");
    expect(value.rx_armed, "post-recovery completion left RX unarmed");
    expect_pop(0xB1U);
    expect_pop(0xB2U);
}

static void test_active_rx_state_avoids_duplicate_rearm(void)
{
    UART_HandleTypeDef uart = {0};
    depth_transport_stm32_diagnostics_t value;

    uart.Instance = USART6;
    reset_mock();
    depth_transport_stm32_init(&uart);
    expect(uart.RxState == HAL_UART_STATE_BUSY_RX,
           "initial arm should leave HAL RX active");

    uart.ErrorCode = HAL_UART_ERROR_FE | HAL_UART_ERROR_NE;
    depth_transport_stm32_on_error(&uart);
    value = diagnostics();
    expect(value.uart_error_count == 1U,
           "active-state UART error was not recorded");
    expect(!value.rx_armed,
           "error callback should mark active-state recovery pending");
    expect(value.rx_needs_rearm,
           "active-state UART error should request reconciliation");

    depth_transport_stm32_poll();
    value = diagnostics();
    expect(receive_call_count == 1U,
           "active HAL RX should not receive a duplicate re-arm");
    expect(value.rx_armed,
           "active HAL RX should reconcile to armed state");
    expect(!value.rx_needs_rearm,
           "active HAL RX reconciliation should clear pending state");
    expect(value.rx_rearm_deferred_count == 0U,
           "active HAL RX should not be counted as a deferred HAL_BUSY");
    expect(value.hard_rearm_failure_count == 0U,
           "active HAL RX should not be counted as a hard failure");

    deliver_byte(&uart, 0xD1U);
    value = diagnostics();
    expect(value.rx_byte_count == 1U,
           "RX did not continue after active-state reconciliation");
    expect(receive_call_count == 2U,
           "completed active RX did not re-arm once");
    expect(value.rx_armed,
           "completed active RX should remain armed");
    expect(!value.rx_needs_rearm,
           "completed active RX should not remain pending");
    expect_pop(0xD1U);
}

static void test_error_completion_is_hard_failure_then_recovers(void)
{
    static const HAL_StatusTypeDef statuses[] = {HAL_OK, HAL_ERROR, HAL_OK};
    UART_HandleTypeDef uart = {0};
    depth_transport_stm32_diagnostics_t value;

    uart.Instance = USART6;
    reset_mock();
    set_status_sequence(statuses, sizeof statuses / sizeof statuses[0]);
    depth_transport_stm32_init(&uart);
    deliver_byte(&uart, 0xC1U);

    value = diagnostics();
    expect(value.hard_rearm_failure_count == 1U, "HAL_ERROR was not a hard failure");
    expect(value.rx_rearm_deferred_count == 0U, "HAL_ERROR was marked deferred");
    expect(!value.rx_armed, "HAL_ERROR should leave RX unarmed");
    expect(value.rx_needs_rearm, "HAL_ERROR should leave RX pending");

    depth_transport_stm32_poll();
    value = diagnostics();
    expect(value.hard_rearm_failure_count == 1U, "recovery changed hard-failure count");
    expect(value.rx_armed, "HAL_ERROR recovery did not arm");
    expect(!value.rx_needs_rearm, "HAL_ERROR recovery remains pending");
    expect_pop(0xC1U);
}

static void test_uart_error_records_subtypes_and_foreground_rearms(void)
{
    UART_HandleTypeDef uart = {0};
    depth_transport_stm32_diagnostics_t value;

    uart.Instance = USART6;
    uart.ErrorCode = HAL_UART_ERROR_ORE |
                     HAL_UART_ERROR_FE |
                     HAL_UART_ERROR_NE |
                     HAL_UART_ERROR_PE |
                     HAL_UART_ERROR_DMA;
    reset_mock();
    depth_transport_stm32_init(&uart);
    depth_transport_stm32_on_error(&uart);

    value = diagnostics();
    expect(value.uart_error_count == 1U, "UART aggregate error count differs");
    expect(value.uart_overrun_error_count == 1U, "UART overrun count differs");
    expect(value.uart_framing_error_count == 1U, "UART framing count differs");
    expect(value.uart_noise_error_count == 1U, "UART noise count differs");
    expect(value.uart_parity_error_count == 1U, "UART parity count differs");
    expect(value.uart_dma_error_count == 1U, "UART DMA count differs");
    expect(!value.rx_armed, "UART error should clear armed state");
    expect(value.rx_needs_rearm, "UART error should request re-arm");
    expect(receive_call_count == 1U, "UART error should defer re-arm to poll");

    depth_transport_stm32_poll();
    value = diagnostics();
    expect(value.rx_armed, "UART error foreground recovery did not arm");
    expect(!value.rx_needs_rearm, "UART error foreground recovery remains pending");
}

static void test_ring_overflow_does_not_stop_continued_rx(void)
{
    UART_HandleTypeDef uart = {0};
    depth_transport_stm32_diagnostics_t value;

    uart.Instance = USART6;
    reset_mock();
    depth_transport_stm32_init(&uart);

    for (uint16_t index = 0U; index < 512U; ++index)
    {
        deliver_byte(&uart, (uint8_t)index);
    }

    value = diagnostics();
    expect(value.rx_byte_count == 512U, "overflow RX byte count differs");
    expect(value.rx_buffer_push_count == 511U, "overflow push count differs");
    expect(value.rx_buffer_overflow_count == 1U, "overflow count differs");
    expect(receive_call_count == 513U, "RX did not continue after overflow");
    expect(value.rx_armed, "RX should remain armed after overflow");
    expect(!value.rx_needs_rearm, "overflow should not leave RX pending");

    for (uint16_t index = 0U; index < 511U; ++index)
    {
        expect_pop((uint8_t)index);
    }
    expect(
        !depth_transport_stm32_pop(&(uint8_t){0U}),
        "USART6 ring should be empty after effective capacity pops");
}

static void test_non_usart6_callbacks_are_ignored(void)
{
    UART_HandleTypeDef uart6 = {0};
    UART_HandleTypeDef uart3 = {0};
    depth_transport_stm32_diagnostics_t before;
    depth_transport_stm32_diagnostics_t after;

    uart6.Instance = USART6;
    uart3.Instance = USART3;
    uart3.ErrorCode = HAL_UART_ERROR_ORE;
    reset_mock();
    depth_transport_stm32_init(&uart6);
    before = diagnostics();
    depth_transport_stm32_on_rx_complete(&uart3);
    depth_transport_stm32_on_error(&uart3);
    after = diagnostics();

    expect(receive_call_count == 1U, "non-USART6 callback changed HAL calls");
    expect(after.rx_byte_count == before.rx_byte_count, "non-USART6 changed RX count");
    expect(after.uart_error_count == before.uart_error_count, "non-USART6 changed errors");
    expect(after.rx_buffer_push_count == before.rx_buffer_push_count, "non-USART6 changed ring");
}

int main(void)
{
    test_initial_and_completion_rearm();
    test_initial_busy_and_error_recover_in_foreground();
    test_busy_completion_defers_to_foreground();
    test_active_rx_state_avoids_duplicate_rearm();
    test_error_completion_is_hard_failure_then_recovers();
    test_uart_error_records_subtypes_and_foreground_rearms();
    test_ring_overflow_does_not_stop_continued_rx();
    test_non_usart6_callbacks_are_ignored();

    if (failures == 0)
    {
        (void)puts("All depth STM32 transport tests passed");
    }

    return failures == 0 ? 0 : 1;
}
