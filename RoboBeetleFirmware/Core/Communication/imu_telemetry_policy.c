#include "imu_telemetry_policy.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void imu_telemetry_policy_init(
    imu_telemetry_policy_t *policy)
{
    if (policy != NULL)
    {
        policy->last_published_ms = 0U;
        policy->has_published = false;
    }
}

bool imu_telemetry_policy_should_publish(
    const imu_telemetry_policy_t *policy,
    uint32_t now_ms,
    uint32_t interval_ms)
{
    if (policy == NULL)
    {
        return false;
    }

    if (!policy->has_published)
    {
        return true;
    }

    return (uint32_t)(now_ms - policy->last_published_ms) >=
           interval_ms;
}

void imu_telemetry_policy_mark_published(
    imu_telemetry_policy_t *policy,
    uint32_t now_ms)
{
    if (policy != NULL)
    {
        policy->last_published_ms = now_ms;
        policy->has_published = true;
    }
}
