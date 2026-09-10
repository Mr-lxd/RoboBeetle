#ifndef ROBOBEETLE_TELEMETRY_SCHEDULER_H
#define ROBOBEETLE_TELEMETRY_SCHEDULER_H

#include <stdbool.h>

typedef enum
{
    TELEMETRY_SLOT_NONE = 0,
    TELEMETRY_SLOT_LEAK_STATUS,
    TELEMETRY_SLOT_IMU_SNAPSHOT
} telemetry_slot_t;

telemetry_slot_t telemetry_scheduler_select(
    bool leak_due,
    bool imu_due);

#endif /* ROBOBEETLE_TELEMETRY_SCHEDULER_H */
