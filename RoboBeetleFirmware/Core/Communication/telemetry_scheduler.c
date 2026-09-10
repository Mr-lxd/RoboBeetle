#include "telemetry_scheduler.h"

#include <stddef.h>

void telemetry_scheduler_init(
    telemetry_scheduler_t *scheduler)
{
    if (scheduler != NULL)
    {
        scheduler->last_successful_slot = TELEMETRY_SLOT_NONE;
    }
}

telemetry_slot_t telemetry_scheduler_select(
    const telemetry_scheduler_t *scheduler,
    bool leak_due,
    bool imu_due)
{
    /* Leak wins the first shared opportunity; a still-due IMU gets the next
     * shared opportunity after a successful LeakStatus publication. */
    if (leak_due && imu_due &&
        (scheduler != NULL) &&
        (scheduler->last_successful_slot == TELEMETRY_SLOT_LEAK_STATUS))
    {
        return TELEMETRY_SLOT_IMU_SNAPSHOT;
    }

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

void telemetry_scheduler_mark_success(
    telemetry_scheduler_t *scheduler,
    telemetry_slot_t slot)
{
    if ((scheduler != NULL) &&
        ((slot == TELEMETRY_SLOT_LEAK_STATUS) ||
         (slot == TELEMETRY_SLOT_IMU_SNAPSHOT)))
    {
        scheduler->last_successful_slot = slot;
    }
}
