#include "jy901s_transport_stm32.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

static int failures = 0;
static uint8_t *receive_destination = NULL;
static HAL_StatusTypeDef receive_status = HAL_OK;
static bool busy_sets_rx_state = false;
static bool inject_completion_during_receive = false;
static uint8_t injected_completion_byte = 0U;
static bool inject_error_during_receive = false;
static uint32_t injected_error_code = HAL_UART_ERROR_NONE;
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

    if ((receive_status == HAL_OK) ||
        ((receive_status == HAL_BUSY) && busy_sets_rx_state))
    {
        huart->RxState = HAL_UART_STATE_BUSY_RX;
    }

    if (inject_completion_during_receive)
    {
        if (receive_destination != NULL)
        {
            *receive_destination = injected_completion_byte;
        }
        huart->RxState = HAL_UART_STATE_READY;
        inject_completion_during_receive = false;
        jy901s_transport_stm32_on_rx_complete(huart);
    }

    if (inject_error_during_receive)
    {
        huart->ErrorCode = injected_error_code;
        inject_error_during_receive = false;
        jy901s_transport_stm32_on_error(huart);
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
    busy_sets_rx_state = false;
    inject_completion_during_receive = false;
    injected_completion_byte = 0U;
    inject_error_during_receive = false;
    injected_error_code = HAL_UART_ERROR_NONE;
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
    huart->RxState = HAL_UART_STATE_READY;
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

static void test_rearm_deferred_recovery_and_subsequent_rx(void)
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

    deliver_byte(&uart, 0xA1U);
    value = diagnostics();
    expect(value.rx_byte_count == 1U, "RX byte counter did not increment");
    expect(value.rx_buffer_push_count == 1U, "RX push counter did not increment");
    expect(receive_call_count == 1U,
           "RX callback should defer HAL re-arm to foreground");
    expect(value.rx_rearm_failure_count == 0U,
           "normal callback must not report a re-arm failure");
    expect(value.rx_rearm_deferred_count == 0U,
           "normal callback must not report a deferred re-arm");
    expect(!value.rx_armed, "completed RX should clear armed state");
    expect(value.rx_needs_rearm, "completed RX should set pending state");

    receive_status = HAL_BUSY;
    jy901s_transport_stm32_poll();
    value = diagnostics();
    expect(receive_call_count == 2U,
           "foreground should attempt one deferred re-arm");
    expect(value.rx_rearm_failure_count == 0U,
           "HAL_BUSY must not count as a hard re-arm failure");
    expect(value.rx_rearm_deferred_count == 1U,
           "HAL_BUSY deferred re-arm counter differs");
    expect(!value.rx_armed, "deferred re-arm must remain unarmed");
    expect(value.rx_needs_rearm, "deferred re-arm must remain pending");

    receive_status = HAL_OK;
    {
        uint32_t calls_before = receive_call_count;
        jy901s_transport_stm32_poll();
        expect(receive_call_count == calls_before + 1U,
               "foreground recovery did not attempt one re-arm");
    }
    value = diagnostics();
    expect(value.rx_rearm_failure_count == 0U,
           "successful recovery changed hard-failure count");
    expect(value.rx_rearm_deferred_count == 1U,
           "successful recovery changed deferred count");
    expect(value.rx_armed, "foreground recovery did not arm RX");
    expect(!value.rx_needs_rearm,
           "foreground recovery did not clear pending state");

    deliver_byte(&uart, 0xA2U);
    value = diagnostics();
    expect(value.rx_byte_count == 2U,
           "RX callback should still account for a second delivered byte");
    expect(value.rx_rearm_failure_count == 0U,
           "post-recovery callback should not fail re-arm");

    jy901s_transport_stm32_poll();
    deliver_byte(&uart, 0xA3U);
    value = diagnostics();
    expect(value.rx_byte_count == 3U,
           "RX did not remain functional after recovery");
    expect(value.rx_buffer_push_count == 3U,
           "post-recovery RX push was not counted");
    expect(value.rx_rearm_failure_count == 0U,
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
                     HAL_UART_ERROR_NE |
                     HAL_UART_ERROR_PE;
    reset_mock();
    jy901s_transport_stm32_init(&uart);
    uart.RxState = HAL_UART_STATE_READY;
    jy901s_transport_stm32_on_error(&uart);

    value = diagnostics();
    expect(value.uart_error_count == 1U, "UART error count differs");
    expect(value.uart_overrun_error_count == 1U,
           "UART overrun error count differs");
    expect(value.uart_framing_error_count == 1U,
           "UART framing error count differs");
    expect(value.uart_noise_error_count == 1U,
           "UART noise error count differs");
    expect(value.uart_parity_error_count == 1U,
           "UART parity error count differs");
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

static void test_recoverable_uart_error_keeps_active_rx(void)
{
    UART_HandleTypeDef uart = {0};
    jy901s_transport_stm32_diagnostics_t value;

    uart.Instance = USART3;
    reset_mock();
    jy901s_transport_stm32_init(&uart);
    uart.ErrorCode = HAL_UART_ERROR_FE | HAL_UART_ERROR_NE;
    jy901s_transport_stm32_on_error(&uart);

    value = diagnostics();
    expect(value.uart_error_count == 1U,
           "recoverable UART error was not counted");
    expect(value.rx_armed,
           "recoverable UART error should preserve active RX");
    expect(!value.rx_needs_rearm,
           "recoverable UART error should not request re-arm");
    expect(receive_call_count == 1U,
           "recoverable UART error should not re-arm an active RX");

    deliver_byte(&uart, 0xB1U);
    jy901s_transport_stm32_poll();
    value = diagnostics();
    expect(value.rx_byte_count == 1U,
           "RX did not continue after recoverable UART error");
    expect(value.rx_armed,
           "RX did not re-arm after recoverable UART error completed");
    expect(!value.rx_needs_rearm,
           "completed recoverable-error RX remains pending");
    expect(value.rx_rearm_failure_count == 0U,
           "recoverable UART error caused hard re-arm failure");
}

static void test_genuine_rearm_error_remains_distinct(void)
{
    UART_HandleTypeDef uart = {0};
    jy901s_transport_stm32_diagnostics_t value;

    uart.Instance = USART3;
    reset_mock();
    jy901s_transport_stm32_init(&uart);
    deliver_byte(&uart, 0xC1U);

    receive_status = HAL_ERROR;
    jy901s_transport_stm32_poll();
    value = diagnostics();
    expect(value.rx_rearm_failure_count == 1U,
           "HAL_ERROR hard re-arm failure was not counted");
    expect(value.rx_rearm_deferred_count == 0U,
           "HAL_ERROR was misclassified as deferred");
    expect(!value.rx_armed, "HAL_ERROR should leave RX unarmed");
    expect(value.rx_needs_rearm, "HAL_ERROR should leave RX pending");

    receive_status = HAL_OK;
    jy901s_transport_stm32_poll();
    value = diagnostics();
    expect(value.rx_armed, "HAL_ERROR recovery did not re-arm RX");
    expect(!value.rx_needs_rearm,
           "HAL_ERROR recovery did not clear pending state");
}

static void test_busy_with_active_rx_is_deferred(void)
{
    UART_HandleTypeDef uart = {0};
    jy901s_transport_stm32_diagnostics_t value;

    uart.Instance = USART3;
    reset_mock();
    jy901s_transport_stm32_init(&uart);
    deliver_byte(&uart, 0xD1U);

    receive_status = HAL_BUSY;
    busy_sets_rx_state = true;
    jy901s_transport_stm32_poll();
    value = diagnostics();
    expect(receive_call_count == 2U,
           "active-state HAL_BUSY re-arm attempt is missing");
    expect(value.rx_rearm_deferred_count == 1U,
           "active-state HAL_BUSY was not recorded as deferred");
    expect(value.rx_rearm_failure_count == 0U,
           "active-state HAL_BUSY became a hard failure");
    expect(value.rx_armed,
           "active-state HAL_BUSY should preserve the active RX");
    expect(!value.rx_needs_rearm,
           "active-state HAL_BUSY should clear the pending state");

    receive_status = HAL_OK;
    busy_sets_rx_state = false;
    deliver_byte(&uart, 0xD2U);
    jy901s_transport_stm32_poll();
    value = diagnostics();
    expect(value.rx_byte_count == 2U,
           "RX did not continue after active-state HAL_BUSY");
    expect(value.rx_armed,
           "RX was not armed after active-state HAL_BUSY recovery");
    expect_pop(0xD1U);
    expect_pop(0xD2U);
}

static void test_rearm_completion_race_remains_pending(void)
{
    UART_HandleTypeDef uart = {0};
    jy901s_transport_stm32_diagnostics_t value;

    uart.Instance = USART3;
    reset_mock();
    jy901s_transport_stm32_init(&uart);
    deliver_byte(&uart, 0xE1U);

    injected_completion_byte = 0xE2U;
    inject_completion_during_receive = true;
    jy901s_transport_stm32_poll();
    value = diagnostics();
    expect(receive_call_count == 2U,
           "re-arm completion race did not invoke HAL once");
    expect(value.rx_byte_count == 2U,
           "re-arm completion race dropped the received byte");
    expect(!value.rx_armed,
           "re-arm completion race falsely reported armed RX");
    expect(value.rx_needs_rearm,
           "re-arm completion race lost the pending state");
    expect(value.rx_rearm_failure_count == 0U,
           "re-arm completion race became a hard failure");

    jy901s_transport_stm32_poll();
    deliver_byte(&uart, 0xE3U);
    jy901s_transport_stm32_poll();
    value = diagnostics();
    expect(value.rx_byte_count == 3U,
           "RX did not recover after the re-arm completion race");
    expect(value.rx_armed,
           "RX did not remain armed after the re-arm completion race");
    expect(!value.rx_needs_rearm,
           "recovered RX remains pending after the re-arm completion race");
    expect_pop(0xE1U);
    expect_pop(0xE2U);
    expect_pop(0xE3U);
}

static void test_rearm_error_race_remains_recoverable(void)
{
    UART_HandleTypeDef uart = {0};
    jy901s_transport_stm32_diagnostics_t value;

    uart.Instance = USART3;
    reset_mock();
    jy901s_transport_stm32_init(&uart);
    deliver_byte(&uart, 0xF1U);

    injected_error_code = HAL_UART_ERROR_FE | HAL_UART_ERROR_NE;
    inject_error_during_receive = true;
    jy901s_transport_stm32_poll();
    value = diagnostics();
    expect(receive_call_count == 2U,
           "re-arm error race did not invoke HAL once");
    expect(value.uart_error_count == 1U,
           "re-arm error race hid the UART error");
    expect(value.rx_rearm_failure_count == 0U,
           "re-arm error race became a hard failure");
    expect(value.rx_needs_rearm,
           "re-arm error race should defer state reconciliation");

    jy901s_transport_stm32_poll();
    value = diagnostics();
    expect(receive_call_count == 2U,
           "active RX error reconciliation attempted an extra HAL receive");
    expect(value.rx_armed,
           "active RX error reconciliation did not restore armed state");
    expect(!value.rx_needs_rearm,
           "active RX error reconciliation remains pending");

    deliver_byte(&uart, 0xF2U);
    jy901s_transport_stm32_poll();
    value = diagnostics();
    expect(value.rx_byte_count == 2U,
           "RX did not continue after the re-arm error race");
    expect(value.rx_armed,
           "RX did not remain armed after the re-arm error race");
    expect_pop(0xF1U);
    expect_pop(0xF2U);
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
        jy901s_transport_stm32_poll();
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
    test_rearm_deferred_recovery_and_subsequent_rx();
    test_uart_error_enters_recovery_path();
    test_recoverable_uart_error_keeps_active_rx();
    test_genuine_rearm_error_remains_distinct();
    test_busy_with_active_rx_is_deferred();
    test_rearm_completion_race_remains_pending();
    test_rearm_error_race_remains_recoverable();
    test_ring_overflow_does_not_stop_rearm();
    test_other_uart_is_ignored();

    if (failures == 0)
    {
        (void)puts("All JY901S STM32 transport tests passed");
    }

    return failures == 0 ? 0 : 1;
}
