#include "ring_buffer.h"

void ring_buffer_init(ring_buffer_t *buffer)
{
    buffer->head = 0U;
    buffer->tail = 0U;
}

bool ring_buffer_push(ring_buffer_t *buffer, uint8_t byte)
{
    uint16_t next =
        (uint16_t)((buffer->head + 1U) % RING_BUFFER_STORAGE_SIZE);

    if (next == buffer->tail)
    {
        return false;
    }

    buffer->data[buffer->head] = byte;
    buffer->head = next;
    return true;
}

bool ring_buffer_pop(ring_buffer_t *buffer, uint8_t *byte)
{
    if (buffer->head == buffer->tail)
    {
        return false;
    }

    *byte = buffer->data[buffer->tail];
    buffer->tail =
        (uint16_t)((buffer->tail + 1U) % RING_BUFFER_STORAGE_SIZE);
    return true;
}
