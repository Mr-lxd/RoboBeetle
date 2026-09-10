#include "telemetry_scheduler.h"

telemetry_slot_t telemetry_scheduler_select(
    bool leak_due,
    bool imu_due)
{
    if (leak_due)
    {
        return TELEMETRY_SLOT_LEAK_STATUS;
    }

    if (imu_due)
    {
        return TELEMETRY_SLOT_IMU_SNAPSHOT;
    }

    return TELEMETRY_SLOT_NONE;
}
