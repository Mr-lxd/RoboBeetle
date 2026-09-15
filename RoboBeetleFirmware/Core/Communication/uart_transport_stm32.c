#include "uart_transport_stm32.h"

#include "motion_timing_diagnostics.h"
#include "ring_buffer.h"

#include <stddef.h>
#include <string.h>

static UART_HandleTypeDef *uart_handle;
static uint8_t rx_byte;
static uint8_t rx_buffer_storage[RING_BUFFER_STORAGE_SIZE];
static ring_buffer_t rx_buffer;
static uart_tx_queue_t tx_queue;
static uart_transport_stm32_state_t transport_state =
    UART_TRANSPORT_STATE_UNINITIALIZED;
static uart_transport_stm32_diagnostics_t transport_diagnostics;
static bool rx_armed;
static bool rx_needs_rearm;
static bool active_hal_owned;
static uint32_t consecutive_start_busy;
static uint32_t abort_busy_retries;
static uint32_t reinitialization_epoch;
static uint32_t active_reinitialization_epoch;
static bool abort_callback_pending;
static bool reinit_completion_handled;
static bool recovery_waiting_for_proof;

#if defined(ROBOBEETLE_UART_TRANSPORT_HOST_TEST)
static void (*host_before_abort_hook)(void);
#endif

static motion_timing_tx_kind_t uart_transport_diagnostic_kind(
    uart_tx_message_kind_t kind)
{
    return (motion_timing_tx_kind_t)kind;
}

typedef enum
{
    UART_TRANSPORT_START_NOT_ATTEMPTED = 0,
    UART_TRANSPORT_START_ACTIVE,
    UART_TRANSPORT_START_DEFERRED,
    UART_TRANSPORT_START_ERROR
} uart_transport_start_result_t;

#if defined(ROBOBEETLE_UART_TRANSPORT_HOST_TEST)
static uint32_t uart_transport_irq_save(void)
{
    return 0U;
}

static void uart_transport_irq_restore(uint32_t primask)
{
    (void)primask;
}
#else
static uint32_t uart_transport_irq_save(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void uart_transport_irq_restore(uint32_t primask)
{
    __set_PRIMASK(primask);
}
#endif

#if defined(ROBOBEETLE_UART_TRANSPORT_HOST_TEST)
static void uart_transport_invoke_before_abort_hook(void)
{
    if (host_before_abort_hook != NULL)
    {
        host_before_abort_hook();
    }
}

void uart_transport_stm32_host_set_before_abort_hook(
    void (*hook)(void))
{
    host_before_abort_hook = hook;
}
#endif

static bool uart_transport_is_usart1(
    const UART_HandleTypeDef *huart)
{
    return (huart != NULL) &&
           (huart == uart_handle) &&
           (huart->Instance == USART1);
}

static bool uart_transport_state_has_busy_tx(
    HAL_UART_StateTypeDef state)
{
    return (state == HAL_UART_STATE_BUSY_TX) ||
           (state == HAL_UART_STATE_BUSY_TX_RX);
}

static bool uart_transport_state_has_busy_rx(
    HAL_UART_StateTypeDef state)
{
    return (state == HAL_UART_STATE_BUSY_RX) ||
           (state == HAL_UART_STATE_BUSY_TX_RX);
}

static bool uart_transport_input_is_valid(
    const uint8_t *data,
    uint16_t length,
    uart_tx_message_kind_t kind)
{
    return (data != NULL) &&
           (length > 0U) &&
           (length <= RBP2_MAX_WIRE_SIZE) &&
           (kind < UART_TX_MESSAGE_KIND_COUNT);
}

static void uart_transport_record_dropped_kind(
    uart_tx_message_kind_t kind)
{
    if (kind < UART_TX_MESSAGE_KIND_COUNT)
    {
        ++transport_diagnostics.dropped_count[kind];
        motion_timing_diagnostics_record_uart_dropped(
            uart_transport_diagnostic_kind(kind));
    }
}

static void uart_transport_record_dropped_pending(void)
{
    uint32_t dropped_count[UART_TX_MESSAGE_KIND_COUNT] = {0U};
    uint32_t index;

    uart_tx_queue_drop_pending(&tx_queue, dropped_count);
    for (index = 0U;
         index < UART_TX_MESSAGE_KIND_COUNT;
         ++index)
    {
        transport_diagnostics.dropped_count[index] +=
            dropped_count[index];
        while (dropped_count[index] > 0U)
        {
            motion_timing_diagnostics_record_uart_dropped(
                uart_transport_diagnostic_kind(
                    (uart_tx_message_kind_t)index));
            --dropped_count[index];
        }
    }
}

static void uart_transport_update_high_water(void)
{
    uint32_t current = uart_tx_queue_pending_count(&tx_queue);
    uart_tx_active_view_t active_view;

    if (uart_tx_queue_get_active_view(&tx_queue, &active_view))
    {
        ++current;
    }

    if (current > transport_diagnostics.high_water_mark)
    {
        transport_diagnostics.high_water_mark = current;
    }
    motion_timing_diagnostics_record_uart_high_water(current);
}

static bool uart_transport_release_active(
    bool completed)
{
    uart_tx_active_view_t view;
    bool released;

    if (!uart_tx_queue_get_active_view(&tx_queue, &view))
    {
        active_hal_owned = false;
        return false;
    }

    if (completed)
    {
        released = uart_tx_queue_complete_active(
            &tx_queue,
            view.token);
    }
    else
    {
        released = uart_tx_queue_fail_active(
            &tx_queue,
            view.token);
    }

    if (released && completed &&
        (view.kind < UART_TX_MESSAGE_KIND_COUNT))
    {
        ++transport_diagnostics.completed_count[view.kind];
        motion_timing_diagnostics_record_uart_completed(
            uart_transport_diagnostic_kind(view.kind));
    }
    if (released && !completed)
    {
        uart_transport_record_dropped_kind(view.kind);
    }
    if (released)
    {
        active_hal_owned = false;
    }
    return released;
}

static uart_transport_start_result_t
uart_transport_start_next_locked(void)
{
    uart_tx_active_view_t view;
    HAL_StatusTypeDef status;

    if ((transport_state != UART_TRANSPORT_STATE_IDLE) ||
        (uart_handle == NULL) ||
        !uart_tx_queue_begin_next(&tx_queue, &view))
    {
        return UART_TRANSPORT_START_NOT_ATTEMPTED;
    }

    transport_state = UART_TRANSPORT_STATE_STARTING;
    status = HAL_UART_Transmit_IT(
        uart_handle,
        view.data,
        view.length);

    if (status == HAL_OK)
    {
        (void)uart_tx_queue_mark_start_active(
            &tx_queue,
            view.token);
        active_hal_owned = true;
        transport_state = UART_TRANSPORT_STATE_ACTIVE;
        consecutive_start_busy = 0U;
        return UART_TRANSPORT_START_ACTIVE;
    }

    if (status == HAL_BUSY)
    {
        (void)uart_tx_queue_mark_start_deferred(
            &tx_queue,
            view.token);
        active_hal_owned = false;
        transport_state = UART_TRANSPORT_STATE_START_DEFERRED;
        ++transport_diagnostics.start_busy_count;
        motion_timing_diagnostics_record_uart_start_busy();
        ++consecutive_start_busy;
        return UART_TRANSPORT_START_DEFERRED;
    }

    (void)uart_tx_queue_fail_active(
        &tx_queue,
        view.token);
    active_hal_owned = false;
    transport_state = UART_TRANSPORT_STATE_IDLE;
    ++transport_diagnostics.start_error_count;
    motion_timing_diagnostics_record_uart_start_error();
    uart_transport_record_dropped_kind(view.kind);
    consecutive_start_busy = 0U;
    return UART_TRANSPORT_START_ERROR;
}

static uart_transport_start_result_t
uart_transport_retry_deferred_locked(void)
{
    uart_tx_active_view_t view;
    HAL_StatusTypeDef status;

    if ((transport_state != UART_TRANSPORT_STATE_START_DEFERRED) ||
        (uart_handle == NULL) ||
        !uart_tx_queue_get_active_view(&tx_queue, &view))
    {
        return UART_TRANSPORT_START_NOT_ATTEMPTED;
    }

    status = HAL_UART_Transmit_IT(
        uart_handle,
        view.data,
        view.length);
    if (status == HAL_OK)
    {
        (void)uart_tx_queue_mark_start_active(
            &tx_queue,
            view.token);
        active_hal_owned = true;
        transport_state = UART_TRANSPORT_STATE_ACTIVE;
        consecutive_start_busy = 0U;
        return UART_TRANSPORT_START_ACTIVE;
    }

    if (status == HAL_BUSY)
    {
        (void)uart_tx_queue_mark_start_deferred(
            &tx_queue,
            view.token);
        ++transport_diagnostics.start_busy_count;
        motion_timing_diagnostics_record_uart_start_busy();
        ++consecutive_start_busy;
        return UART_TRANSPORT_START_DEFERRED;
    }

    (void)uart_tx_queue_fail_active(
        &tx_queue,
        view.token);
    active_hal_owned = false;
    transport_state = UART_TRANSPORT_STATE_IDLE;
    ++transport_diagnostics.start_error_count;
    motion_timing_diagnostics_record_uart_start_error();
    uart_transport_record_dropped_kind(view.kind);
    consecutive_start_busy = 0U;
    return UART_TRANSPORT_START_ERROR;
}

static void uart_transport_enter_recovery_failed_locked(void)
{
    uart_tx_active_view_t active_view;

    if (uart_tx_queue_get_active_view(&tx_queue, &active_view) &&
        !active_hal_owned)
    {
        (void)uart_transport_release_active(false);
    }

    uart_transport_record_dropped_pending();
    transport_state = UART_TRANSPORT_STATE_RECOVERY_FAILED;
    recovery_waiting_for_proof = true;
    reinit_completion_handled = true;
}

static void uart_transport_finish_reinitialize_locked(void)
{
    uart_tx_active_view_t active_view;

    if ((transport_state != UART_TRANSPORT_STATE_REINITIALIZING) ||
        reinit_completion_handled)
    {
        return;
    }

    /* Set the once-only guard before releasing any ownership. */
    reinit_completion_handled = true;
    if (uart_tx_queue_get_active_view(&tx_queue, &active_view))
    {
        (void)uart_transport_release_active(false);
    }
    uart_transport_record_dropped_pending();
    transport_state = UART_TRANSPORT_STATE_IDLE;
    active_hal_owned = false;
    abort_callback_pending = false;
    active_reinitialization_epoch = 0U;
    consecutive_start_busy = 0U;
    abort_busy_retries = 0U;
    recovery_waiting_for_proof = false;
}

static void uart_transport_finish_recovery_proof_locked(void)
{
    if ((transport_state != UART_TRANSPORT_STATE_RECOVERY_FAILED) ||
        !recovery_waiting_for_proof)
    {
        return;
    }

    if (active_hal_owned)
    {
        (void)uart_transport_release_active(false);
    }
    transport_state = UART_TRANSPORT_STATE_IDLE;
    active_hal_owned = false;
    abort_callback_pending = false;
    active_reinitialization_epoch = 0U;
    recovery_waiting_for_proof = false;
    abort_busy_retries = 0U;
    consecutive_start_busy = 0U;
}

static HAL_StatusTypeDef uart_transport_handle_abort_result(
    HAL_StatusTypeDef status,
    bool initial_call)
{
    uint32_t primask;
    HAL_StatusTypeDef result = status;

    primask = uart_transport_irq_save();
    if (transport_state != UART_TRANSPORT_STATE_REINITIALIZING)
    {
        uart_transport_irq_restore(primask);
        return result;
    }

    if (reinit_completion_handled)
    {
        uart_transport_irq_restore(primask);
        return result;
    }

    if (status == HAL_OK)
    {
        uart_tx_active_view_t active_view;

        if (uart_tx_queue_get_active_view(&tx_queue, &active_view) &&
            active_hal_owned)
        {
            /* A physical ACTIVE frame still needs the abort callback as
             * proof that HAL no longer owns its buffer. */
            uart_transport_enter_recovery_failed_locked();
        }
        else
        {
            uart_transport_finish_reinitialize_locked();
        }
    }
    else if (status == HAL_BUSY)
    {
        if (initial_call)
        {
            abort_busy_retries = 1U;
        }
        else
        {
            ++abort_busy_retries;
        }

        if (abort_busy_retries >=
            UART_TX_MAX_REINITIALIZE_ABORT_BUSY_RETRIES)
        {
            uart_transport_enter_recovery_failed_locked();
        }
    }
    else
    {
        uart_transport_enter_recovery_failed_locked();
    }

    uart_transport_irq_restore(primask);
    return result;
}

static bool uart_transport_abort_callback_is_current_locked(
    const UART_HandleTypeDef *huart)
{
    /* The HAL callback has no generation argument.  Bind it to the one
     * outstanding abort epoch and require the HAL TX state to have crossed
     * the ready edge.  A late callback from an older epoch therefore cannot
     * complete a newer abort while its active frame is still HAL-owned. */
    return abort_callback_pending &&
           (active_reinitialization_epoch == reinitialization_epoch) &&
           !uart_transport_state_has_busy_tx(HAL_UART_GetState(huart));
}

static HAL_StatusTypeDef uart_transport_rearm_rx_locked(void)
{
    HAL_StatusTypeDef status;

    if (!rx_needs_rearm || (uart_handle == NULL))
    {
        return HAL_OK;
    }

    if (uart_transport_state_has_busy_rx(HAL_UART_GetState(uart_handle)))
    {
        rx_armed = true;
        rx_needs_rearm = false;
        return HAL_OK;
    }

    ++transport_diagnostics.rx_rearm_attempt_count;
    motion_timing_diagnostics_record_uart_rearm(
        MOTION_TIMING_UART_REARM_ATTEMPT);
    status = HAL_UART_Receive_IT(
        uart_handle,
        &rx_byte,
        1U);
    if (status == HAL_OK)
    {
        rx_armed = true;
        rx_needs_rearm = false;
    }
    else if (status == HAL_BUSY)
    {
        ++transport_diagnostics.rx_rearm_busy_count;
        motion_timing_diagnostics_record_uart_rearm(
            MOTION_TIMING_UART_REARM_BUSY);
    }
    else
    {
        ++transport_diagnostics.rx_rearm_error_count;
        motion_timing_diagnostics_record_uart_rearm(
            MOTION_TIMING_UART_REARM_ERROR);
    }
    return status;
}

void uart_transport_stm32_init(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef status;

    uart_handle = huart;
    (void)memset(&transport_diagnostics, 0, sizeof transport_diagnostics);
    uart_tx_queue_init(&tx_queue);
    ring_buffer_init(
        &rx_buffer,
        rx_buffer_storage,
        (uint16_t)sizeof rx_buffer_storage);
    transport_state = (huart == NULL) ?
        UART_TRANSPORT_STATE_UNINITIALIZED :
        UART_TRANSPORT_STATE_IDLE;
    rx_armed = false;
    rx_needs_rearm = false;
    active_hal_owned = false;
    consecutive_start_busy = 0U;
    abort_busy_retries = 0U;
    reinit_completion_handled = false;
    recovery_waiting_for_proof = false;
    abort_callback_pending = false;
    active_reinitialization_epoch = 0U;
#if defined(ROBOBEETLE_UART_TRANSPORT_HOST_TEST)
    host_before_abort_hook = NULL;
#endif

    if (huart == NULL)
    {
        return;
    }

    status = HAL_UART_Receive_IT(
        uart_handle,
        &rx_byte,
        1U);
    if (status == HAL_OK)
    {
        rx_armed = true;
    }
    else
    {
        rx_needs_rearm = true;
    }
    transport_diagnostics.rx_armed = rx_armed;
    transport_diagnostics.rx_needs_rearm = rx_needs_rearm;
}

uart_tx_enqueue_result_t uart_transport_stm32_enqueue(
    const uint8_t *data,
    uint16_t length,
    uart_tx_message_kind_t kind)
{
    uart_tx_enqueue_result_t result;
    uart_transport_start_result_t start_result =
        UART_TRANSPORT_START_NOT_ATTEMPTED;
    uint32_t primask;

    if (!uart_transport_input_is_valid(data, length, kind))
    {
        return UART_TX_INVALID;
    }

    primask = uart_transport_irq_save();
    if ((transport_state == UART_TRANSPORT_STATE_UNINITIALIZED) ||
        (transport_state == UART_TRANSPORT_STATE_REINITIALIZING) ||
        (transport_state == UART_TRANSPORT_STATE_RECOVERY_FAILED))
    {
        ++transport_diagnostics.rejected_count[kind];
        motion_timing_diagnostics_record_uart_enqueue(
            uart_transport_diagnostic_kind(kind),
            MOTION_TIMING_UART_EVENT_REJECTED);
        uart_transport_irq_restore(primask);
        return UART_TX_TRANSPORT_ERROR;
    }

    result = uart_tx_queue_offer(
        &tx_queue,
        data,
        length,
        kind);
    if (result == UART_TX_ENQUEUED)
    {
        ++transport_diagnostics.enqueued_count[kind];
        motion_timing_diagnostics_record_uart_enqueue(
            uart_transport_diagnostic_kind(kind),
            MOTION_TIMING_UART_EVENT_ENQUEUED);
    }
    else if (result == UART_TX_COALESCED)
    {
        ++transport_diagnostics.coalesced_count[kind];
        motion_timing_diagnostics_record_uart_enqueue(
            uart_transport_diagnostic_kind(kind),
            MOTION_TIMING_UART_EVENT_COALESCED);
    }
    else if (result == UART_TX_CONTROL_FULL)
    {
        ++transport_diagnostics.control_queue_full_count;
        ++transport_diagnostics.rejected_count[kind];
        uart_transport_record_dropped_kind(kind);
        motion_timing_diagnostics_record_uart_enqueue(
            uart_transport_diagnostic_kind(kind),
            MOTION_TIMING_UART_EVENT_REJECTED);
        motion_timing_diagnostics_record_uart_queue_full(true);
    }
    else if (result == UART_TX_TELEMETRY_FULL)
    {
        ++transport_diagnostics.telemetry_queue_full_count;
        ++transport_diagnostics.rejected_count[kind];
        uart_transport_record_dropped_kind(kind);
        motion_timing_diagnostics_record_uart_enqueue(
            uart_transport_diagnostic_kind(kind),
            MOTION_TIMING_UART_EVENT_REJECTED);
        motion_timing_diagnostics_record_uart_queue_full(false);
    }

    if ((result == UART_TX_ENQUEUED) ||
        (result == UART_TX_COALESCED))
    {
        if (transport_state == UART_TRANSPORT_STATE_IDLE)
        {
            start_result = uart_transport_start_next_locked();
        }
        uart_transport_update_high_water();
    }
    uart_transport_irq_restore(primask);

    if (start_result == UART_TRANSPORT_START_ERROR)
    {
        return UART_TX_TRANSPORT_ERROR;
    }
    return result;
}

void uart_transport_stm32_on_rx_complete(UART_HandleTypeDef *huart)
{
    uint32_t primask;

    if (!uart_transport_is_usart1(huart))
    {
        return;
    }

    primask = uart_transport_irq_save();
    rx_armed = false;
    rx_needs_rearm = true;
    (void)ring_buffer_push(&rx_buffer, rx_byte);
    (void)uart_transport_rearm_rx_locked();
    transport_diagnostics.rx_armed = rx_armed;
    transport_diagnostics.rx_needs_rearm = rx_needs_rearm;
    uart_transport_irq_restore(primask);
}

void uart_transport_stm32_on_tx_complete(UART_HandleTypeDef *huart)
{
    uart_tx_active_view_t view;
    uart_transport_start_result_t start_result;
    uint32_t primask;

    if (!uart_transport_is_usart1(huart))
    {
        return;
    }

    primask = uart_transport_irq_save();
    if (transport_state == UART_TRANSPORT_STATE_REINITIALIZING)
    {
        if (active_hal_owned &&
            uart_tx_queue_get_active_view(&tx_queue, &view) &&
            !uart_transport_state_has_busy_tx(HAL_UART_GetState(huart)))
        {
            (void)uart_transport_release_active(true);
        }
        else
        {
            ++transport_diagnostics.unexpected_callback_count;
            motion_timing_diagnostics_record_uart_unexpected_callback();
        }
        uart_transport_irq_restore(primask);
        return;
    }

    if (transport_state == UART_TRANSPORT_STATE_RECOVERY_FAILED)
    {
        if (active_hal_owned &&
            uart_tx_queue_get_active_view(&tx_queue, &view) &&
            !uart_transport_state_has_busy_tx(HAL_UART_GetState(huart)))
        {
            (void)uart_transport_release_active(true);
            uart_transport_finish_recovery_proof_locked();
        }
        else
        {
            ++transport_diagnostics.unexpected_callback_count;
            motion_timing_diagnostics_record_uart_unexpected_callback();
        }
        uart_transport_irq_restore(primask);
        return;
    }

    if ((transport_state != UART_TRANSPORT_STATE_ACTIVE) ||
        !active_hal_owned ||
        !uart_tx_queue_get_active_view(&tx_queue, &view) ||
        uart_transport_state_has_busy_tx(HAL_UART_GetState(huart)))
    {
        ++transport_diagnostics.unexpected_callback_count;
        motion_timing_diagnostics_record_uart_unexpected_callback();
        uart_transport_irq_restore(primask);
        return;
    }

    (void)uart_transport_release_active(true);
    transport_state = UART_TRANSPORT_STATE_IDLE;
    start_result = uart_transport_start_next_locked();
    (void)start_result;
    uart_transport_update_high_water();
    uart_transport_irq_restore(primask);
}

void uart_transport_stm32_on_abort_transmit_complete(
    UART_HandleTypeDef *huart)
{
    uint32_t primask;

    if (!uart_transport_is_usart1(huart))
    {
        return;
    }

    primask = uart_transport_irq_save();
    if (transport_state == UART_TRANSPORT_STATE_REINITIALIZING)
    {
        if (reinit_completion_handled ||
            !uart_transport_abort_callback_is_current_locked(huart))
        {
            ++transport_diagnostics.unexpected_callback_count;
            motion_timing_diagnostics_record_uart_unexpected_callback();
        }
        else
        {
            uart_transport_finish_reinitialize_locked();
        }
    }
    else if ((transport_state == UART_TRANSPORT_STATE_RECOVERY_FAILED) &&
             recovery_waiting_for_proof &&
             uart_transport_abort_callback_is_current_locked(huart))
    {
        uart_transport_finish_recovery_proof_locked();
    }
    else
    {
        ++transport_diagnostics.unexpected_callback_count;
        motion_timing_diagnostics_record_uart_unexpected_callback();
    }
    uart_transport_irq_restore(primask);
}

void uart_transport_stm32_on_error(UART_HandleTypeDef *huart)
{
    uint32_t error;
    uint32_t primask;
    bool rx_still_busy;

    if (!uart_transport_is_usart1(huart))
    {
        return;
    }

    error = HAL_UART_GetError(huart);
    rx_still_busy =
        uart_transport_state_has_busy_rx(HAL_UART_GetState(huart));
    primask = uart_transport_irq_save();
    ++transport_diagnostics.uart_error_count;
    {
        const bool rx_rearm_required =
            ((error & HAL_UART_ERROR_ORE) != 0U) || !rx_still_busy;

        motion_timing_diagnostics_record_uart_error(rx_rearm_required);
    }
    if (((error & HAL_UART_ERROR_ORE) != 0U) || !rx_still_busy)
    {
        ++transport_diagnostics.rx_error_count;
        rx_armed = false;
        rx_needs_rearm = true;
    }
    transport_diagnostics.rx_armed = rx_armed;
    transport_diagnostics.rx_needs_rearm = rx_needs_rearm;
    uart_transport_irq_restore(primask);
}

void uart_transport_stm32_process(void)
{
    bool retry_abort = false;
    bool escalate_reinitialize = false;
    uint32_t primask;
    HAL_StatusTypeDef abort_status;

    primask = uart_transport_irq_save();
    if ((transport_state == UART_TRANSPORT_STATE_REINITIALIZING) &&
        !reinit_completion_handled &&
        (abort_busy_retries > 0U) &&
        (abort_busy_retries <
         UART_TX_MAX_REINITIALIZE_ABORT_BUSY_RETRIES))
    {
        retry_abort = true;
    }
    else if (transport_state == UART_TRANSPORT_STATE_START_DEFERRED)
    {
        (void)uart_transport_retry_deferred_locked();
        if ((transport_state == UART_TRANSPORT_STATE_START_DEFERRED) &&
            (consecutive_start_busy >=
             UART_TX_MAX_CONSECUTIVE_START_BUSY))
        {
            ++transport_diagnostics.busy_recovery_count;
            motion_timing_diagnostics_record_uart_busy_recovery();
            escalate_reinitialize = true;
        }
    }
    else if (transport_state == UART_TRANSPORT_STATE_IDLE)
    {
        (void)uart_transport_start_next_locked();
        uart_transport_update_high_water();
    }
    uart_transport_irq_restore(primask);

    if (retry_abort)
    {
#if defined(ROBOBEETLE_UART_TRANSPORT_HOST_TEST)
        uart_transport_invoke_before_abort_hook();
#endif
        abort_status = HAL_UART_AbortTransmit_IT(uart_handle);
        (void)uart_transport_handle_abort_result(
            abort_status,
            false);
    }
    else if (escalate_reinitialize)
    {
        (void)uart_transport_stm32_reinitialize();
    }

    primask = uart_transport_irq_save();
    (void)uart_transport_rearm_rx_locked();
    transport_diagnostics.rx_armed = rx_armed;
    transport_diagnostics.rx_needs_rearm = rx_needs_rearm;
    uart_transport_irq_restore(primask);
}

HAL_StatusTypeDef uart_transport_stm32_reinitialize(void)
{
    bool needs_abort;
    uint32_t primask;
    HAL_StatusTypeDef status;

    primask = uart_transport_irq_save();
    if ((uart_handle == NULL) ||
        (transport_state == UART_TRANSPORT_STATE_UNINITIALIZED) ||
        (transport_state == UART_TRANSPORT_STATE_RECOVERY_FAILED))
    {
        uart_transport_irq_restore(primask);
        return HAL_ERROR;
    }

    if (transport_state == UART_TRANSPORT_STATE_REINITIALIZING)
    {
        uart_transport_irq_restore(primask);
        return HAL_BUSY;
    }

    transport_state = UART_TRANSPORT_STATE_REINITIALIZING;
    reinit_completion_handled = false;
    recovery_waiting_for_proof = false;
    abort_busy_retries = 0U;
    ++reinitialization_epoch;
    if (reinitialization_epoch == 0U)
    {
        ++reinitialization_epoch;
    }
    active_reinitialization_epoch = reinitialization_epoch;
    abort_callback_pending = true;
    ++transport_diagnostics.reinitialization_count;
    motion_timing_diagnostics_record_uart_reinitialization();
    {
        uart_tx_active_view_t active_view;

        needs_abort = uart_tx_queue_get_active_view(
            &tx_queue,
            &active_view);
    }
    if (!needs_abort)
    {
        uart_transport_finish_reinitialize_locked();
        uart_transport_irq_restore(primask);
        return HAL_OK;
    }
    uart_transport_irq_restore(primask);

#if defined(ROBOBEETLE_UART_TRANSPORT_HOST_TEST)
    uart_transport_invoke_before_abort_hook();
#endif
    status = HAL_UART_AbortTransmit_IT(uart_handle);
    return uart_transport_handle_abort_result(status, true);
}

bool uart_transport_stm32_pop(uint8_t *byte)
{
    return ring_buffer_pop(&rx_buffer, byte);
}

uart_transport_stm32_state_t uart_transport_stm32_get_state(void)
{
    return transport_state;
}

void uart_transport_stm32_get_diagnostics(
    uart_transport_stm32_diagnostics_t *diagnostics)
{
    uint32_t primask;

    if (diagnostics == NULL)
    {
        return;
    }

    primask = uart_transport_irq_save();
    *diagnostics = transport_diagnostics;
    diagnostics->rx_armed = rx_armed;
    diagnostics->rx_needs_rearm = rx_needs_rearm;
    uart_transport_irq_restore(primask);
}
