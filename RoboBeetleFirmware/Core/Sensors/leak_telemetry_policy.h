#ifndef ROBOBEETLE_LEAK_TELEMETRY_POLICY_H
#define ROBOBEETLE_LEAK_TELEMETRY_POLICY_H

#include "leak_sensor.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    leak_sensor_state_t last_published_state;
    uint32_t last_published_ms;
    uint8_t has_published;
} leak_telemetry_policy_t;

void leak_telemetry_policy_init(
    leak_telemetry_policy_t *policy);

bool leak_telemetry_policy_should_publish(
    const leak_telemetry_policy_t *policy,
    leak_sensor_state_t current_state,
    uint32_t now_ms,
    uint32_t refresh_interval_ms);

void leak_telemetry_policy_mark_published(
    leak_telemetry_policy_t *policy,
    leak_sensor_state_t state,
    uint32_t now_ms);

#endif /* ROBOBEETLE_LEAK_TELEMETRY_POLICY_H */
