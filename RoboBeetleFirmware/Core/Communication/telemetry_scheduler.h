#ifndef ROBOBEETLE_TELEMETRY_SCHEDULER_H
#define ROBOBEETLE_TELEMETRY_SCHEDULER_H

#include <stdbool.h>

typedef enum
{
    TELEMETRY_SLOT_NONE = 0,
    TELEMETRY_SLOT_LEAK_STATUS,
    TELEMETRY_SLOT_IMU_SNAPSHOT,
    TELEMETRY_SLOT_DEPTH_SNAPSHOT
} telemetry_slot_t;

typedef struct
{
    telemetry_slot_t last_successful_slot;
} telemetry_scheduler_t;

void telemetry_scheduler_init(
    telemetry_scheduler_t *scheduler);

telemetry_slot_t telemetry_scheduler_select(
    const telemetry_scheduler_t *scheduler,
    bool leak_due,
    bool imu_due,
    bool depth_due);

void telemetry_scheduler_mark_success(
    telemetry_scheduler_t *scheduler,
    telemetry_slot_t slot);

#endif /* ROBOBEETLE_TELEMETRY_SCHEDULER_H */
