#ifndef ROBOBEETLE_RING_BUFFER_H
#define ROBOBEETLE_RING_BUFFER_H

#include <stdbool.h>
#include <stdint.h>

#define RING_BUFFER_STORAGE_SIZE 128U

typedef struct
{
    uint8_t data[RING_BUFFER_STORAGE_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} ring_buffer_t;

void ring_buffer_init(ring_buffer_t *buffer);
bool ring_buffer_push(ring_buffer_t *buffer, uint8_t byte);
bool ring_buffer_pop(ring_buffer_t *buffer, uint8_t *byte);

#endif /* ROBOBEETLE_RING_BUFFER_H */
