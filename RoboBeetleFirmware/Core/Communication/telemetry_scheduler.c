#include "telemetry_scheduler.h"

#include <stdbool.h>
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
    bool imu_due,
    bool depth_due)
{
    telemetry_slot_t last_slot = TELEMETRY_SLOT_NONE;
    telemetry_slot_t preference[3];

    if (scheduler != NULL)
    {
        last_slot = scheduler->last_successful_slot;
    }

    switch (last_slot)
    {
        case TELEMETRY_SLOT_LEAK_STATUS:
            preference[0] = TELEMETRY_SLOT_IMU_SNAPSHOT;
            preference[1] = TELEMETRY_SLOT_DEPTH_SNAPSHOT;
            preference[2] = TELEMETRY_SLOT_LEAK_STATUS;
            break;

        case TELEMETRY_SLOT_IMU_SNAPSHOT:
            preference[0] = TELEMETRY_SLOT_DEPTH_SNAPSHOT;
            preference[1] = TELEMETRY_SLOT_LEAK_STATUS;
            preference[2] = TELEMETRY_SLOT_IMU_SNAPSHOT;
            break;

        case TELEMETRY_SLOT_DEPTH_SNAPSHOT:
            preference[0] = TELEMETRY_SLOT_LEAK_STATUS;
            preference[1] = TELEMETRY_SLOT_IMU_SNAPSHOT;
            preference[2] = TELEMETRY_SLOT_DEPTH_SNAPSHOT;
            break;

        case TELEMETRY_SLOT_NONE:
        default:
            preference[0] = TELEMETRY_SLOT_LEAK_STATUS;
            preference[1] = TELEMETRY_SLOT_IMU_SNAPSHOT;
            preference[2] = TELEMETRY_SLOT_DEPTH_SNAPSHOT;
            break;
    }

    for (size_t index = 0U; index < 3U; ++index)
    {
        switch (preference[index])
        {
            case TELEMETRY_SLOT_LEAK_STATUS:
                if (leak_due)
                {
                    return preference[index];
                }
                break;

            case TELEMETRY_SLOT_IMU_SNAPSHOT:
                if (imu_due)
                {
                    return preference[index];
                }
                break;

            case TELEMETRY_SLOT_DEPTH_SNAPSHOT:
                if (depth_due)
                {
                    return preference[index];
                }
                break;

            case TELEMETRY_SLOT_NONE:
            default:
                break;
        }
    }

    return TELEMETRY_SLOT_NONE;
}

void telemetry_scheduler_mark_success(
    telemetry_scheduler_t *scheduler,
    telemetry_slot_t slot)
{
    if ((scheduler != NULL) &&
        ((slot == TELEMETRY_SLOT_LEAK_STATUS) ||
         (slot == TELEMETRY_SLOT_IMU_SNAPSHOT) ||
         (slot == TELEMETRY_SLOT_DEPTH_SNAPSHOT)))
    {
        scheduler->last_successful_slot = slot;
    }
}
