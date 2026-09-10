#ifndef ROBOBEETLE_IMU_TELEMETRY_POLICY_H
#define ROBOBEETLE_IMU_TELEMETRY_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#define JY901S_IMU_TELEMETRY_INTERVAL_MS 1000U

typedef struct
{
    uint32_t last_published_ms;
    bool has_published;
} imu_telemetry_policy_t;

void imu_telemetry_policy_init(
    imu_telemetry_policy_t *policy);

bool imu_telemetry_policy_should_publish(
    const imu_telemetry_policy_t *policy,
    uint32_t now_ms,
    uint32_t interval_ms);

void imu_telemetry_policy_mark_published(
    imu_telemetry_policy_t *policy,
    uint32_t now_ms);

#endif /* ROBOBEETLE_IMU_TELEMETRY_POLICY_H */
