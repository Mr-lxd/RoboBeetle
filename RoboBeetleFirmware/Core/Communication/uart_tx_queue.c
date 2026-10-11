#include "uart_tx_queue.h"

#include <stddef.h>
#include <string.h>

static bool uart_tx_kind_is_control(
    uart_tx_message_kind_t kind)
{
    return kind == UART_TX_MESSAGE_ACK;
}

static bool uart_tx_kind_is_valid(
    uart_tx_message_kind_t kind)
{
    return kind < UART_TX_MESSAGE_KIND_COUNT;
}

static void uart_tx_clear_frame(
    uart_tx_frame_t *frame)
{
    if (frame == NULL)
    {
        return;
    }

    frame->length = 0U;
    frame->kind = UART_TX_MESSAGE_ACK;
    frame->token = 0U;
    frame->occupied = false;
}

static void uart_tx_copy_frame(
    uart_tx_frame_t *destination,
    const uint8_t *data,
    uint16_t length,
    uart_tx_message_kind_t kind)
{
    (void)memcpy(destination->bytes, data, length);
    destination->length = length;
    destination->kind = kind;
    destination->occupied = true;
}

static uart_tx_active_token_t uart_tx_next_token(
    uart_tx_queue_t *queue)
{
    ++queue->next_token;
    if (queue->next_token == 0U)
    {
        ++queue->next_token;
    }
    return queue->next_token;
}

static void uart_tx_set_view(
    const uart_tx_queue_t *queue,
    uart_tx_active_view_t *view)
{
    view->data = queue->active.bytes;
    view->length = queue->active.length;
    view->kind = queue->active.kind;
    view->token = queue->active.token;
}

void uart_tx_queue_init(uart_tx_queue_t *queue)
{
    if (queue == NULL)
    {
        return;
    }

    (void)memset(queue, 0, sizeof *queue);
    uart_tx_clear_frame(&queue->active);
}

uart_tx_enqueue_result_t uart_tx_queue_offer(
    uart_tx_queue_t *queue,
    const uint8_t *data,
    uint16_t length,
    uart_tx_message_kind_t kind)
{
    uint32_t index;

    if ((queue == NULL) ||
        (data == NULL) ||
        (length == 0U) ||
        (length > RBP2_MAX_WIRE_SIZE) ||
        !uart_tx_kind_is_valid(kind))
    {
        return UART_TX_INVALID;
    }

    if (uart_tx_kind_is_control(kind))
    {
        uint8_t tail;

        if (queue->control_count >= UART_TX_CONTROL_QUEUE_CAPACITY)
        {
            ++queue->stats.control_queue_full_count;
            ++queue->stats.dropped_count[kind];
            return UART_TX_CONTROL_FULL;
        }

        tail = (uint8_t)(
            (queue->control_head + queue->control_count) %
            UART_TX_CONTROL_QUEUE_CAPACITY);
        uart_tx_copy_frame(
            &queue->control[tail],
            data,
            length,
            kind);
        ++queue->control_count;
        return UART_TX_ENQUEUED;
    }

    if(kind==UART_TX_MESSAGE_CPG) {
        const bool replacing=queue->cpg.occupied;
        uart_tx_copy_frame(&queue->cpg,data,length,kind);
        return replacing?UART_TX_COALESCED:UART_TX_ENQUEUED;
    }
    if (kind == UART_TX_MESSAGE_MOTION)
    {
        if (queue->motion.occupied)
        {
            return UART_TX_MOTION_FULL;
        }
        uart_tx_copy_frame(&queue->motion, data, length, kind);
        return UART_TX_ENQUEUED;
    }

    for (index = 0U;
         index < UART_TX_TELEMETRY_QUEUE_CAPACITY;
         ++index)
    {
        if (queue->telemetry[index].occupied &&
            (queue->telemetry[index].kind == kind))
        {
            uart_tx_copy_frame(
                &queue->telemetry[index],
                data,
                length,
                kind);
            return UART_TX_COALESCED;
        }
    }

    for (index = 0U;
         index < UART_TX_TELEMETRY_QUEUE_CAPACITY;
         ++index)
    {
        if (!queue->telemetry[index].occupied)
        {
            uart_tx_copy_frame(
                &queue->telemetry[index],
                data,
                length,
                kind);
            ++queue->telemetry_count;
            return UART_TX_ENQUEUED;
        }
    }

    ++queue->stats.telemetry_queue_full_count;
    ++queue->stats.dropped_count[kind];
    return UART_TX_TELEMETRY_FULL;
}

bool uart_tx_queue_begin_next(
    uart_tx_queue_t *queue,
    uart_tx_active_view_t *view)
{
    uint32_t offset;
    uint32_t index;

    if ((queue == NULL) || (view == NULL) ||
        (queue->active_state != UART_TX_ACTIVE_NONE))
    {
        return false;
    }

    if (queue->control_count > 0U)
    {
        uart_tx_frame_t *source =
            &queue->control[queue->control_head];

        queue->active = *source;
        uart_tx_clear_frame(source);
        queue->control_head = (uint8_t)(
            (queue->control_head + 1U) %
            UART_TX_CONTROL_QUEUE_CAPACITY);
        --queue->control_count;
    }
    else if(queue->cpg.occupied)
    {
        queue->active=queue->cpg;
        uart_tx_clear_frame(&queue->cpg);
    }
    else
    {
        index = UART_TX_TELEMETRY_QUEUE_CAPACITY;
        for (offset = 0U;
             offset < UART_TX_TELEMETRY_QUEUE_CAPACITY;
             ++offset)
        {
            const uint32_t candidate =
                (queue->telemetry_cursor + offset) %
                UART_TX_TELEMETRY_QUEUE_CAPACITY;

            if (queue->telemetry[candidate].occupied)
            {
                index = candidate;
                break;
            }
        }

        if (index >= UART_TX_TELEMETRY_QUEUE_CAPACITY)
        {
            if (!queue->motion.occupied)
            {
                return false;
            }
            queue->active = queue->motion;
            uart_tx_clear_frame(&queue->motion);
        }
        else
        {
            queue->active = queue->telemetry[index];
            uart_tx_clear_frame(&queue->telemetry[index]);
            --queue->telemetry_count;
            queue->telemetry_cursor = (uint8_t)(
                (index + 1U) % UART_TX_TELEMETRY_QUEUE_CAPACITY);
        }
    }

    queue->active.token = 0U;
    queue->active_state = UART_TX_ACTIVE_STARTING;
    queue->active.token = uart_tx_next_token(queue);
    uart_tx_set_view(queue, view);
    return true;
}

bool uart_tx_queue_get_active_view(
    const uart_tx_queue_t *queue,
    uart_tx_active_view_t *view)
{
    if ((queue == NULL) || (view == NULL) ||
        (queue->active_state == UART_TX_ACTIVE_NONE) ||
        !queue->active.occupied)
    {
        return false;
    }

    uart_tx_set_view(queue, view);
    return true;
}

bool uart_tx_queue_mark_start_active(
    uart_tx_queue_t *queue,
    uart_tx_active_token_t token)
{
    if ((queue == NULL) ||
        (queue->active_state == UART_TX_ACTIVE_NONE) ||
        (queue->active.token != token) ||
        ((queue->active_state != UART_TX_ACTIVE_STARTING) &&
         (queue->active_state != UART_TX_ACTIVE_START_DEFERRED)))
    {
        return false;
    }

    queue->active_state = UART_TX_ACTIVE;
    return true;
}

bool uart_tx_queue_mark_start_deferred(
    uart_tx_queue_t *queue,
    uart_tx_active_token_t token)
{
    if ((queue == NULL) ||
        (queue->active_state == UART_TX_ACTIVE_NONE) ||
        (queue->active.token != token) ||
        ((queue->active_state != UART_TX_ACTIVE_STARTING) &&
         (queue->active_state != UART_TX_ACTIVE_START_DEFERRED)))
    {
        return false;
    }

    queue->active_state = UART_TX_ACTIVE_START_DEFERRED;
    return true;
}

static bool uart_tx_release_active(
    uart_tx_queue_t *queue,
    uart_tx_active_token_t token)
{
    if ((queue == NULL) ||
        (queue->active_state == UART_TX_ACTIVE_NONE) ||
        (queue->active.token != token))
    {
        return false;
    }

    uart_tx_clear_frame(&queue->active);
    queue->active.token = 0U;
    queue->active_state = UART_TX_ACTIVE_NONE;
    return true;
}

bool uart_tx_queue_complete_active(
    uart_tx_queue_t *queue,
    uart_tx_active_token_t token)
{
    return uart_tx_release_active(queue, token);
}

bool uart_tx_queue_fail_active(
    uart_tx_queue_t *queue,
    uart_tx_active_token_t token)
{
    return uart_tx_release_active(queue, token);
}

void uart_tx_queue_drop_pending(
    uart_tx_queue_t *queue,
    uint32_t dropped_count[UART_TX_MESSAGE_KIND_COUNT])
{
    uint32_t index;

    if (queue == NULL)
    {
        return;
    }

    for (index = 0U;
         index < UART_TX_CONTROL_QUEUE_CAPACITY;
         ++index)
    {
        uart_tx_frame_t *frame = &queue->control[index];

        if (frame->occupied)
        {
            if ((dropped_count != NULL) &&
                uart_tx_kind_is_valid(frame->kind))
            {
                ++dropped_count[frame->kind];
            }
            uart_tx_clear_frame(frame);
        }
    }

    for (index = 0U;
         index < UART_TX_TELEMETRY_QUEUE_CAPACITY;
         ++index)
    {
        uart_tx_frame_t *frame = &queue->telemetry[index];

        if (frame->occupied)
        {
            if ((dropped_count != NULL) &&
                uart_tx_kind_is_valid(frame->kind))
            {
                ++dropped_count[frame->kind];
            }
            uart_tx_clear_frame(frame);
        }
    }

    if(queue->cpg.occupied) {
        if(dropped_count!=NULL) ++dropped_count[UART_TX_MESSAGE_CPG];
        uart_tx_clear_frame(&queue->cpg);
    }
    queue->control_head = 0U;
    if (queue->motion.occupied)
    {
        if (dropped_count != NULL)
        {
            ++dropped_count[UART_TX_MESSAGE_MOTION];
        }
        uart_tx_clear_frame(&queue->motion);
    }
    queue->control_count = 0U;
    queue->telemetry_count = 0U;
}

uint32_t uart_tx_queue_pending_count(
    const uart_tx_queue_t *queue)
{
    if (queue == NULL)
    {
        return 0U;
    }

    return (uint32_t)queue->control_count +
           (uint32_t)queue->telemetry_count +
           (queue->motion.occupied ? 1U : 0U) + (queue->cpg.occupied ? 1U : 0U);
}

void uart_tx_queue_get_stats(
    const uart_tx_queue_t *queue,
    uart_tx_queue_stats_t *stats)
{
    if ((queue == NULL) || (stats == NULL))
    {
        return;
    }

    *stats = queue->stats;
}
