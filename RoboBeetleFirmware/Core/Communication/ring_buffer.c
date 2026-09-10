#include "ring_buffer.h"

static bool ring_buffer_is_configured(const ring_buffer_t *buffer)
{
    return (buffer != NULL) &&
           (buffer->data != NULL) &&
           (buffer->storage_size >= 2U);
}

void ring_buffer_init(
    ring_buffer_t *buffer,
    uint8_t *storage,
    uint16_t storage_size)
{
    if (buffer == NULL)
    {
        return;
    }

    buffer->head = 0U;
    buffer->tail = 0U;

    if ((storage == NULL) || (storage_size < 2U))
    {
        buffer->data = NULL;
        buffer->storage_size = 0U;
        return;
    }

    buffer->data = storage;
    buffer->storage_size = storage_size;
}

bool ring_buffer_push(ring_buffer_t *buffer, uint8_t byte)
{
    if (!ring_buffer_is_configured(buffer))
    {
        return false;
    }

    uint16_t next = (uint16_t)(buffer->head + 1U);
    if (next >= buffer->storage_size)
    {
        next = 0U;
    }

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
    if (!ring_buffer_is_configured(buffer) || (byte == NULL))
    {
        return false;
    }

    if (buffer->head == buffer->tail)
    {
        return false;
    }

    *byte = buffer->data[buffer->tail];
    ++buffer->tail;
    if (buffer->tail >= buffer->storage_size)
    {
        buffer->tail = 0U;
    }
    return true;
}
