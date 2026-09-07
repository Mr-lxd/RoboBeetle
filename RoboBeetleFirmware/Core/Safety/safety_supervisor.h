#ifndef SAFETY_SUPERVISOR_H
#define SAFETY_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

#define SAFETY_SUPERVISOR_HEARTBEAT_TIMEOUT_MS 500U

typedef struct
{
    uint8_t host_alive;
    uint8_t timeout_reported;
    uint32_t last_heartbeat_rx_ms;
} safety_supervisor_t;

void safety_supervisor_init(
    safety_supervisor_t *supervisor);

void safety_supervisor_on_heartbeat(
    safety_supervisor_t *supervisor,
    uint32_t now_ms);

bool safety_supervisor_is_host_alive(
    const safety_supervisor_t *supervisor);

bool safety_supervisor_process(
    safety_supervisor_t *supervisor,
    uint32_t now_ms);

#endif /* SAFETY_SUPERVISOR_H */
