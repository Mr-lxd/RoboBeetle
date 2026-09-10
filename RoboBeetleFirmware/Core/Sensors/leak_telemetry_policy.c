#include "leak_telemetry_policy.h"

#include <stddef.h>

void leak_telemetry_policy_init(
    leak_telemetry_policy_t *policy)
{
    policy->last_published_state = LEAK_SENSOR_STATE_UNKNOWN;
    policy->last_published_ms = 0U;
    policy->has_published = 0U;
}

bool leak_telemetry_policy_should_publish(
    const leak_telemetry_policy_t *policy,
    leak_sensor_state_t current_state,
    uint32_t now_ms,
    uint32_t refresh_interval_ms)
{
    if ((policy == NULL) || !leak_sensor_state_is_valid(current_state))
    {
        return false;
    }

    if (policy->has_published == 0U ||
        current_state != policy->last_published_state)
    {
        return true;
    }

    return (uint32_t)(now_ms - policy->last_published_ms) >=
           refresh_interval_ms;
}

void leak_telemetry_policy_mark_published(
    leak_telemetry_policy_t *policy,
    leak_sensor_state_t state,
    uint32_t now_ms)
{
    if ((policy == NULL) || !leak_sensor_state_is_valid(state))
    {
        return;
    }

    policy->last_published_state = state;
    policy->last_published_ms = now_ms;
    policy->has_published = 1U;
}
