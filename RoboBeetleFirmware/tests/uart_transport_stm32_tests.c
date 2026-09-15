#include "uart_transport_stm32.h"
#include "uart_tx_queue.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures;
static UART_HandleTypeDef uart1;
static UART_HandleTypeDef uart3;
static UART_HandleTypeDef uart6;

static uint8_t *receive_destination;
static HAL_StatusTypeDef receive_status;
static HAL_StatusTypeDef transmit_status;
static HAL_StatusTypeDef abort_status;
static uint32_t receive_call_count;
static uint32_t transmit_call_count;
static uint32_t abort_call_count;
static uint32_t blocking_transmit_call_count;
static uint8_t last_transmit_bytes[RBP2_MAX_WIRE_SIZE];
static uint16_t last_transmit_length;
static bool abort_inject_tx_complete;
static uint32_t abort_inline_callback_count;

HAL_StatusTypeDef HAL_UART_Receive_IT(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size)
{
    receive_destination = data;
    ++receive_call_count;

    if ((huart == NULL) ||
        (huart->Instance != USART1) ||
        (size != 1U))
    {
        return HAL_ERROR;
    }

    if (receive_status == HAL_OK)
    {
        huart->RxState = HAL_UART_STATE_BUSY_RX;
    }
    return receive_status;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(
    UART_HandleTypeDef *huart,
    const uint8_t *data,
    uint16_t size)
{
    ++transmit_call_count;
    last_transmit_length = size;
    if ((data != NULL) && (size <= sizeof last_transmit_bytes))
    {
        (void)memcpy(last_transmit_bytes, data, size);
    }

    if ((huart == NULL) || (huart->Instance != USART1))
    {
        return HAL_ERROR;
    }

    if (transmit_status == HAL_OK)
    {
        huart->gState = HAL_UART_STATE_BUSY_TX;
    }
    return transmit_status;
}

HAL_StatusTypeDef HAL_UART_Transmit(
    UART_HandleTypeDef *huart,
    const uint8_t *data,
    uint16_t size,
    uint32_t timeout)
{
    (void)huart;
    (void)data;
    (void)size;
    (void)timeout;
    ++blocking_transmit_call_count;
    return HAL_ERROR;
}

HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(
    UART_HandleTypeDef *huart)
{
    ++abort_call_count;
    if (huart != NULL)
    {
        huart->gState = HAL_UART_STATE_READY;
    }

    if (abort_inject_tx_complete)
    {
        abort_inject_tx_complete = false;
        uart_transport_stm32_on_tx_complete(huart);
    }

    while (abort_inline_callback_count > 0U)
    {
        --abort_inline_callback_count;
        uart_transport_stm32_on_abort_transmit_complete(huart);
    }
    return abort_status;
}

HAL_UART_StateTypeDef HAL_UART_GetState(
    const UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return HAL_UART_STATE_RESET;
    }
    return (HAL_UART_StateTypeDef)(huart->gState | huart->RxState);
}

uint32_t HAL_UART_GetError(
    const UART_HandleTypeDef *huart)
{
    return huart == NULL ? HAL_UART_ERROR_NONE : huart->ErrorCode;
}

static void expect(bool condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

static void reset_mock(void)
{
    (void)memset(&uart1, 0, sizeof uart1);
    (void)memset(&uart3, 0, sizeof uart3);
    (void)memset(&uart6, 0, sizeof uart6);
    uart1.Instance = USART1;
    uart3.Instance = USART3;
    uart6.Instance = USART6;
    receive_destination = NULL;
    receive_status = HAL_OK;
    transmit_status = HAL_OK;
    abort_status = HAL_OK;
    receive_call_count = 0U;
    transmit_call_count = 0U;
    abort_call_count = 0U;
    blocking_transmit_call_count = 0U;
    last_transmit_length = 0U;
    (void)memset(last_transmit_bytes, 0, sizeof last_transmit_bytes);
    abort_inject_tx_complete = false;
    abort_inline_callback_count = 0U;
}

static void init_transport(void)
{
    reset_mock();
    uart_transport_stm32_init(&uart1);
}

static uart_transport_stm32_diagnostics_t diagnostics(void)
{
    uart_transport_stm32_diagnostics_t value;

    uart_transport_stm32_get_diagnostics(&value);
    return value;
}

static void deliver_byte(uint8_t byte)
{
    expect(receive_destination != NULL, "USART1 RX staging pointer is missing");
    if (receive_destination != NULL)
    {
        *receive_destination = byte;
    }
    uart1.RxState = HAL_UART_STATE_READY;
    uart_transport_stm32_on_rx_complete(&uart1);
}

static void complete_tx(void)
{
    uart1.gState = HAL_UART_STATE_READY;
    uart_transport_stm32_on_tx_complete(&uart1);
}

static void test_init_and_idle_it_start(void)
{
    const uint8_t frame[] = {0xA1U, 0xA2U};
    uart_transport_stm32_diagnostics_t value;

    init_transport();
    value = diagnostics();
    expect(receive_call_count == 1U, "init should arm one-byte RX once");
    expect(value.rx_armed, "init should report RX armed");
    expect(transmit_call_count == 0U, "init must not start TX");
    expect(uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_IDLE,
           "init should leave transport idle");
    expect(
        uart_transport_stm32_enqueue(
            frame,
            (uint16_t)sizeof frame,
            UART_TX_MESSAGE_ACK) == UART_TX_ENQUEUED,
        "idle ACK should be accepted");
    expect(transmit_call_count == 1U, "idle enqueue should start IT TX once");
    expect(blocking_transmit_call_count == 0U,
           "transport must not call blocking HAL TX");
    expect(last_transmit_length == sizeof frame,
           "IT TX length should match owned frame");
}

static void test_active_telemetry_keeps_ack_waiting_and_completion_is_safe(void)
{
    const uint8_t telemetry[] = {0x10U, 0x11U, 0x12U};
    const uint8_t ack[] = {0xA0U, 0xA1U};
    uart_transport_stm32_diagnostics_t value;

    init_transport();
    expect(
        uart_transport_stm32_enqueue(
            telemetry,
            (uint16_t)sizeof telemetry,
            UART_TX_MESSAGE_IMU) == UART_TX_ENQUEUED,
        "active telemetry should be accepted");
    expect(
        uart_transport_stm32_enqueue(
            ack,
            (uint16_t)sizeof ack,
            UART_TX_MESSAGE_ACK) == UART_TX_ENQUEUED,
        "ACK should be accepted behind active telemetry");
    expect(transmit_call_count == 1U,
           "waiting ACK must not start before telemetry completion");
    expect(last_transmit_bytes[0] == telemetry[0],
           "active telemetry bytes must remain unchanged");

    complete_tx();
    expect(transmit_call_count == 2U,
           "completion should start the waiting ACK once");
    expect(last_transmit_bytes[0] == ack[0],
           "completion should select the waiting ACK");

    /* HAL has accepted the new ACK, so a duplicate old callback must not
     * release it. */
    uart_transport_stm32_on_tx_complete(&uart1);
    value = diagnostics();
    expect(last_transmit_bytes[0] == ack[0],
           "duplicate completion must not corrupt the next active frame");
    expect(value.completed_count[UART_TX_MESSAGE_IMU] == 1U,
           "telemetry completion should be counted once");
    complete_tx();
}

static void test_other_uart_callbacks_are_ignored(void)
{
    const uint8_t frame[] = {0x55U};
    uart_transport_stm32_diagnostics_t before;
    uart_transport_stm32_diagnostics_t after;

    init_transport();
    (void)uart_transport_stm32_enqueue(
        frame,
        (uint16_t)sizeof frame,
        UART_TX_MESSAGE_ACK);
    before = diagnostics();
    uart_transport_stm32_on_tx_complete(&uart3);
    uart_transport_stm32_on_abort_transmit_complete(&uart6);
    uart3.ErrorCode = HAL_UART_ERROR_ORE;
    uart_transport_stm32_on_error(&uart3);
    uart_transport_stm32_on_rx_complete(&uart6);
    after = diagnostics();
    expect(transmit_call_count == 1U,
           "other UART callbacks must not start USART1 TX");
    expect(after.completed_count[UART_TX_MESSAGE_ACK] ==
               before.completed_count[UART_TX_MESSAGE_ACK],
           "other UART completion must not release USART1 TX");
    expect(after.uart_error_count == before.uart_error_count,
           "other UART error must not change USART1 diagnostics");
}

static void test_rx_completion_rearms_while_tx_is_active(void)
{
    const uint8_t frame[] = {0x66U};
    uint8_t byte = 0U;

    init_transport();
    (void)uart_transport_stm32_enqueue(
        frame,
        (uint16_t)sizeof frame,
        UART_TX_MESSAGE_ACK);
    deliver_byte(0xC1U);
    expect(receive_call_count == 2U,
           "RX completion should rearm one-byte RX");
    expect(transmit_call_count == 1U,
           "RX rearm must not disturb active TX");
    expect(uart_transport_stm32_pop(&byte),
           "completed RX byte should reach host ring");
    expect(byte == 0xC1U, "host RX ring should preserve the received byte");
}

static void test_busy_retries_once_per_process_call(void)
{
    const uint8_t frame[] = {0x71U};
    uart_transport_stm32_diagnostics_t value;

    init_transport();
    transmit_status = HAL_BUSY;
    expect(
        uart_transport_stm32_enqueue(
            frame,
            (uint16_t)sizeof frame,
            UART_TX_MESSAGE_ACK) == UART_TX_ENQUEUED,
        "HAL_BUSY start should preserve accepted ownership");
    expect(transmit_call_count == 1U,
           "initial enqueue should make one HAL start attempt");
    expect(uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_START_DEFERRED,
           "HAL_BUSY should enter START_DEFERRED");
    uart_transport_stm32_process();
    value = diagnostics();
    expect(transmit_call_count == 2U,
           "one process call should make one deferred retry");
    expect(value.start_busy_count == 2U,
           "deferred HAL_BUSY attempts should be counted");
}

static void test_hal_error_does_not_need_fresh_enqueue_to_progress_pending(void)
{
    const uint8_t first[] = {0x81U};
    const uint8_t second[] = {0x82U};
    const uint8_t third[] = {0x83U};

    init_transport();
    (void)uart_transport_stm32_enqueue(
        first,
        (uint16_t)sizeof first,
        UART_TX_MESSAGE_ACK);
    (void)uart_transport_stm32_enqueue(
        second,
        (uint16_t)sizeof second,
        UART_TX_MESSAGE_ACK);
    (void)uart_transport_stm32_enqueue(
        third,
        (uint16_t)sizeof third,
        UART_TX_MESSAGE_ACK);

    transmit_status = HAL_ERROR;
    complete_tx();
    expect(transmit_call_count == 2U,
           "completion should attempt the next frame once");
    expect(uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_IDLE,
           "failed next start should leave pending queue wakeable");

    transmit_status = HAL_OK;
    uart_transport_stm32_process();
    expect(transmit_call_count == 3U,
           "process should advance later pending frame after HAL_ERROR");
    expect(last_transmit_bytes[0] == third[0],
           "later pending frame should progress without fresh enqueue");
}

static void test_uart_error_keeps_active_tx(void)
{
    const uint8_t frame[] = {0x91U};
    uart_transport_stm32_diagnostics_t value;
    uint32_t receives_before;

    init_transport();
    receives_before = receive_call_count;
    (void)uart_transport_stm32_enqueue(
        frame,
        (uint16_t)sizeof frame,
        UART_TX_MESSAGE_ACK);
    uart1.ErrorCode = HAL_UART_ERROR_FE | HAL_UART_ERROR_NE;
    uart1.RxState = HAL_UART_STATE_BUSY_RX;
    uart_transport_stm32_on_error(&uart1);
    value = diagnostics();
    expect(value.uart_error_count == 1U,
           "USART1 UART error should be recorded");
    expect(receive_call_count == receives_before,
           "recoverable PE/FE/NE must not rearm from the error callback");
    complete_tx();
    expect(value.rx_armed || diagnostics().rx_armed,
           "recoverable RX should remain armed while TX completes");
}

static void test_ore_rearms_in_foreground_without_disturbing_tx(void)
{
    const uint8_t frame[] = {0xA6U};
    uart_transport_stm32_diagnostics_t value;
    uint32_t transmit_calls_before;

    init_transport();
    transmit_calls_before = transmit_call_count;
    (void)uart_transport_stm32_enqueue(
        frame,
        (uint16_t)sizeof frame,
        UART_TX_MESSAGE_ACK);
    uart1.RxState = HAL_UART_STATE_READY;
    uart1.ErrorCode = HAL_UART_ERROR_ORE;
    uart_transport_stm32_on_error(&uart1);
    value = diagnostics();
    expect(value.rx_needs_rearm, "ORE should request RX rearm");
    expect(!value.rx_armed, "ORE should clear RX armed state");

    uart_transport_stm32_process();
    value = diagnostics();
    expect(value.rx_armed && !value.rx_needs_rearm,
           "foreground process should rearm ORE-ended RX");
    expect(transmit_call_count == transmit_calls_before + 1U,
           "RX recovery must not start or replace TX");
    deliver_byte(0xB6U);
    complete_tx();
}

static void test_pe_fe_ne_keep_active_rx_armed(void)
{
    const uint32_t errors[] = {
        HAL_UART_ERROR_PE,
        HAL_UART_ERROR_FE,
        HAL_UART_ERROR_NE
    };
    uint32_t index;

    init_transport();
    for (index = 0U; index < sizeof errors / sizeof errors[0]; ++index)
    {
        uart1.ErrorCode = errors[index];
        uart1.RxState = HAL_UART_STATE_BUSY_RX;
        uart_transport_stm32_on_error(&uart1);
        expect(
            diagnostics().rx_armed,
            "PE/FE/NE while HAL RX is active should remain armed");
        expect(
            !diagnostics().rx_needs_rearm,
            "PE/FE/NE while HAL RX is active should not request rearm");
    }
    expect(receive_call_count == 1U,
           "recoverable PE/FE/NE should not call HAL receive again");
}

static void test_inline_double_abort_cleanup_and_later_duplicate(void)
{
    const uint8_t active[] = {0xC1U};
    const uint8_t pending[] = {0xC2U};
    const uint8_t fresh[] = {0xC3U};
    uart_transport_stm32_diagnostics_t value;

    init_transport();
    (void)uart_transport_stm32_enqueue(
        active,
        (uint16_t)sizeof active,
        UART_TX_MESSAGE_ACK);
    (void)uart_transport_stm32_enqueue(
        pending,
        (uint16_t)sizeof pending,
        UART_TX_MESSAGE_ACK);
    abort_inline_callback_count = 2U;
    expect(
        uart_transport_stm32_reinitialize() == HAL_OK,
        "inline abort completion should return HAL_OK");
    value = diagnostics();
    expect(uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_IDLE,
           "inline abort completion should return IDLE");
    expect(value.dropped_count[UART_TX_MESSAGE_ACK] == 2U,
           "inline double abort should drop active/pending exactly once");

    transmit_status = HAL_OK;
    (void)uart_transport_stm32_enqueue(
        fresh,
        (uint16_t)sizeof fresh,
        UART_TX_MESSAGE_ACK);
    uart_transport_stm32_on_abort_transmit_complete(&uart1);
    expect(uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_ACTIVE,
           "later duplicate abort callback must not release fresh TX");
    expect(last_transmit_bytes[0] == fresh[0],
           "later duplicate abort callback must not corrupt fresh TX");
}

static void test_tx_completion_race_does_not_promote_pending(void)
{
    const uint8_t active[] = {0xD1U};
    const uint8_t pending[] = {0xD2U};
    uart_transport_stm32_diagnostics_t value;

    init_transport();
    (void)uart_transport_stm32_enqueue(
        active,
        (uint16_t)sizeof active,
        UART_TX_MESSAGE_ACK);
    (void)uart_transport_stm32_enqueue(
        pending,
        (uint16_t)sizeof pending,
        UART_TX_MESSAGE_ACK);
    abort_inject_tx_complete = true;
    abort_inline_callback_count = 1U;
    expect(
        uart_transport_stm32_reinitialize() == HAL_OK,
        "completion race should finish through matching abort callback");
    value = diagnostics();
    expect(transmit_call_count == 1U,
           "REINITIALIZING completion must not start pending TX");
    expect(value.completed_count[UART_TX_MESSAGE_ACK] == 1U,
           "race-completed active frame should complete once");
    expect(value.dropped_count[UART_TX_MESSAGE_ACK] == 1U,
           "race should drop only the remaining pending frame");
    expect(uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_IDLE,
           "race cleanup should finish in IDLE");
}

static void test_abort_failure_retains_hal_owned_active(void)
{
    const uint8_t active[] = {0xE1U};
    const uint8_t pending[] = {0xE2U};
    const uint8_t fresh[] = {0xE3U};

    init_transport();
    (void)uart_transport_stm32_enqueue(
        active,
        (uint16_t)sizeof active,
        UART_TX_MESSAGE_ACK);
    (void)uart_transport_stm32_enqueue(
        pending,
        (uint16_t)sizeof pending,
        UART_TX_MESSAGE_ACK);
    abort_status = HAL_ERROR;
    expect(
        uart_transport_stm32_reinitialize() == HAL_ERROR,
        "abort failure should be surfaced");
    expect(
        uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_RECOVERY_FAILED,
        "abort failure must not return normal IDLE");
    expect(
        uart_transport_stm32_enqueue(
            fresh,
            (uint16_t)sizeof fresh,
            UART_TX_MESSAGE_ACK) == UART_TX_TRANSPORT_ERROR,
        "recovery-failed transport must reject fresh TX");
    expect(transmit_call_count == 1U,
           "recovery failure must not reuse the HAL-owned active slot");

    uart_transport_stm32_on_abort_transmit_complete(&uart1);
    expect(uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_IDLE,
           "matching later abort callback should prove release");
}

static void test_deferred_abort_failure_is_not_false_recovery(void)
{
    const uint8_t frame[] = {0xF1U};

    init_transport();
    transmit_status = HAL_BUSY;
    (void)uart_transport_stm32_enqueue(
        frame,
        (uint16_t)sizeof frame,
        UART_TX_MESSAGE_ACK);
    abort_status = HAL_ERROR;
    expect(
        uart_transport_stm32_reinitialize() == HAL_ERROR,
        "deferred abort failure should be surfaced");
    expect(
        uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_RECOVERY_FAILED,
        "unresolved deferred HAL state must remain recovery failed");
    uart_transport_stm32_on_abort_transmit_complete(&uart1);
    expect(uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_IDLE,
           "deferred state should recover only after abort completion");
}

static void test_persistent_start_busy_has_finite_recovery(void)
{
    const uint8_t frame[] = {0x01U};
    const uint8_t fresh[] = {0x02U};
    uart_transport_stm32_diagnostics_t value;
    uint32_t attempt;

    init_transport();
    transmit_status = HAL_BUSY;
    (void)uart_transport_stm32_enqueue(
        frame,
        (uint16_t)sizeof frame,
        UART_TX_MESSAGE_ACK);
    for (attempt = 1U;
         attempt < UART_TX_MAX_CONSECUTIVE_START_BUSY;
         ++attempt)
    {
        uart_transport_stm32_process();
    }

    value = diagnostics();
    expect(value.busy_recovery_count == 1U,
           "persistent start BUSY should escalate exactly once");
    expect(value.dropped_count[UART_TX_MESSAGE_ACK] == 1U,
           "finite BUSY recovery should drop stale deferred frame once");
    expect(uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_IDLE,
           "successful abort recovery should return IDLE");

    transmit_status = HAL_OK;
    expect(
        uart_transport_stm32_enqueue(
            fresh,
            (uint16_t)sizeof fresh,
            UART_TX_MESSAGE_ACK) == UART_TX_ENQUEUED,
        "fresh frame should transmit after finite BUSY recovery");
    expect(last_transmit_bytes[0] == fresh[0],
           "finite BUSY recovery must not replay stale frame");
}

static void test_abort_busy_retries_are_bounded_and_fail_closed(void)
{
    const uint8_t active[] = {0x11U};
    const uint8_t fresh[] = {0x12U};

    init_transport();
    (void)uart_transport_stm32_enqueue(
        active,
        (uint16_t)sizeof active,
        UART_TX_MESSAGE_ACK);
    abort_status = HAL_BUSY;
    expect(
        uart_transport_stm32_reinitialize() == HAL_BUSY,
        "HAL_BUSY abort should remain pending for bounded retry");
    expect(
        uart_transport_stm32_get_state() ==
            UART_TRANSPORT_STATE_REINITIALIZING,
        "first HAL_BUSY abort should remain reinitializing");

    uart_transport_stm32_process();
    expect(abort_call_count == 2U,
           "abort BUSY should receive only one foreground retry");
    expect(
        uart_transport_stm32_get_state() ==
            UART_TRANSPORT_STATE_RECOVERY_FAILED,
        "abort BUSY exhaustion must fail closed");
    expect(
        uart_transport_stm32_enqueue(
            fresh,
            (uint16_t)sizeof fresh,
            UART_TX_MESSAGE_ACK) == UART_TX_TRANSPORT_ERROR,
        "unresolved abort ownership must reject fresh TX");

    uart_transport_stm32_on_abort_transmit_complete(&uart1);
    expect(uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_IDLE,
           "matching abort completion should prove release after BUSY");
}

int main(void)
{
    test_init_and_idle_it_start();
    test_active_telemetry_keeps_ack_waiting_and_completion_is_safe();
    test_other_uart_callbacks_are_ignored();
    test_rx_completion_rearms_while_tx_is_active();
    test_busy_retries_once_per_process_call();
    test_hal_error_does_not_need_fresh_enqueue_to_progress_pending();
    test_uart_error_keeps_active_tx();
    test_ore_rearms_in_foreground_without_disturbing_tx();
    test_pe_fe_ne_keep_active_rx_armed();
    test_inline_double_abort_cleanup_and_later_duplicate();
    test_tx_completion_race_does_not_promote_pending();
    test_abort_failure_retains_hal_owned_active();
    test_deferred_abort_failure_is_not_false_recovery();
    test_persistent_start_busy_has_finite_recovery();
    test_abort_busy_retries_are_bounded_and_fail_closed();

    if (failures == 0)
    {
        (void)puts("All USART1 STM32 transport tests passed");
    }

    return failures == 0 ? 0 : 1;
}
