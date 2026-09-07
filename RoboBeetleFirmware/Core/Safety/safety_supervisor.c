#include "safety_supervisor.h"

void safety_supervisor_init(
    safety_supervisor_t *supervisor)
{
    supervisor->host_alive = 0U;
    supervisor->timeout_reported = 0U;
    supervisor->last_heartbeat_rx_ms = 0U;
}

void safety_supervisor_on_heartbeat(
    safety_supervisor_t *supervisor,
    uint32_t now_ms)
{
    supervisor->host_alive = 1U;
    supervisor->timeout_reported = 0U;
    supervisor->last_heartbeat_rx_ms = now_ms;
}

bool safety_supervisor_is_host_alive(
    const safety_supervisor_t *supervisor)
{
    return supervisor->host_alive != 0U;
}

bool safety_supervisor_process(
    safety_supervisor_t *supervisor,
    uint32_t now_ms)
{
    if (supervisor->host_alive == 0U)
    {
        return false;
    }

    if ((now_ms - supervisor->last_heartbeat_rx_ms) >
        SAFETY_SUPERVISOR_HEARTBEAT_TIMEOUT_MS)
    {
        supervisor->host_alive = 0U;

        if (supervisor->timeout_reported == 0U)
        {
            supervisor->timeout_reported = 1U;
            return true;
        }
    }

    return false;
}
