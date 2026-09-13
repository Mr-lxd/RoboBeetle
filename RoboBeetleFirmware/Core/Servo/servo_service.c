#include "servo_service.h"

#include "servo_calibration.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static const servo_descriptor_t *servo_service_descriptor(
    uint8_t servo_id)
{
    const servo_descriptor_t *descriptor =
        servo_descriptor_for_id(servo_id);

    if ((descriptor == NULL) || !descriptor->supported)
    {
        return NULL;
    }

    return descriptor;
}

static bool servo_service_is_enabled(
    const servo_service_t *service,
    const servo_descriptor_t *descriptor)
{
    return (service->enabled_mask & descriptor->mask) != 0U;
}

static bool servo_service_motion_owner(
    const servo_service_t *service)
{
    return service->owner == SERVO_SERVICE_OWNER_MOTION;
}

void servo_service_init(
    servo_service_t *service,
    const servo_service_driver_ops_t *driver_ops,
    void *driver_context)
{
    service->driver_ops = driver_ops;
    service->driver_context = driver_context;
    service->enabled_mask = 0U;
    service->owner = SERVO_SERVICE_OWNER_MANUAL;
    service->motion_mask = 0U;
    for (size_t index = 0U; index < SERVO_DESCRIPTOR_COUNT; ++index)
    {
        service->logical_angle_cdeg[index] = 0;
    }
    service->logical_pose_known_mask = 0U;
}

servo_service_result_t servo_service_validate_mask(uint16_t mask)
{
    if (mask == 0U)
    {
        return SERVO_SERVICE_RESULT_INVALID_PAYLOAD;
    }

    if ((mask & (uint16_t)(~servo_descriptor_supported_mask())) != 0U)
    {
        return SERVO_SERVICE_RESULT_UNSUPPORTED_SERVO;
    }

    return SERVO_SERVICE_RESULT_OK;
}

servo_service_result_t servo_service_enable(
    servo_service_t *service,
    uint16_t mask)
{
    if (servo_service_motion_owner(service))
    {
        return SERVO_SERVICE_RESULT_BUSY;
    }

    const servo_service_result_t mask_result =
        servo_service_validate_mask(mask);
    const uint16_t original_mask = service->enabled_mask;
    const uint16_t original_pose_known_mask =
        service->logical_pose_known_mask;
    int16_t original_angles[SERVO_DESCRIPTOR_COUNT];
    uint16_t newly_started_mask = 0U;

    for (size_t index = 0U; index < SERVO_DESCRIPTOR_COUNT; ++index)
    {
        original_angles[index] = service->logical_angle_cdeg[index];
    }

    if (mask_result != SERVO_SERVICE_RESULT_OK)
    {
        return mask_result;
    }

    for (size_t index = 0U; index < servo_descriptor_count(); ++index)
    {
        const servo_descriptor_t *descriptor =
            &servo_descriptor_table()[index];

        if ((mask & descriptor->mask) == 0U)
        {
            continue;
        }

        if ((original_mask & descriptor->mask) != 0U)
        {
            continue;
        }

        service->driver_ops->write_pulse_us(
            service->driver_context,
            descriptor->id,
            descriptor->calibration.neutral_pulse_us);

        if (!service->driver_ops->start(
                service->driver_context,
                descriptor->id))
        {
            for (size_t rollback_index = 0U;
                 rollback_index < servo_descriptor_count();
                 ++rollback_index)
            {
                const servo_descriptor_t *rollback_descriptor =
                    &servo_descriptor_table()[rollback_index];

                if ((newly_started_mask & rollback_descriptor->mask) != 0U)
                {
                    service->driver_ops->stop(
                        service->driver_context,
                        rollback_descriptor->id);
                }
            }

            service->enabled_mask = original_mask;
            service->logical_pose_known_mask = original_pose_known_mask;
            for (size_t restore_index = 0U;
                 restore_index < SERVO_DESCRIPTOR_COUNT;
                 ++restore_index)
            {
                service->logical_angle_cdeg[restore_index] =
                    original_angles[restore_index];
            }
            return SERVO_SERVICE_RESULT_HARDWARE_FAILURE;
        }

        newly_started_mask |= descriptor->mask;
        service->logical_angle_cdeg[descriptor->id] = 0;
        service->logical_pose_known_mask = (uint16_t)(
            service->logical_pose_known_mask | descriptor->mask);
    }

    service->enabled_mask = (uint16_t)(original_mask | mask);
    return SERVO_SERVICE_RESULT_OK;
}

servo_service_result_t servo_service_disable(
    servo_service_t *service,
    uint16_t mask)
{
    const servo_service_result_t mask_result =
        servo_service_validate_mask(mask);

    if (mask_result != SERVO_SERVICE_RESULT_OK)
    {
        return mask_result;
    }

    for (size_t index = 0U; index < servo_descriptor_count(); ++index)
    {
        const servo_descriptor_t *descriptor =
            &servo_descriptor_table()[index];

        if ((mask & descriptor->mask) != 0U)
        {
            service->driver_ops->stop(
                service->driver_context,
                descriptor->id);
        }
    }

    service->enabled_mask &= (uint16_t)(~mask);
    service->logical_pose_known_mask &= (uint16_t)(~mask);
    return SERVO_SERVICE_RESULT_OK;
}

void servo_service_disable_all(servo_service_t *service)
{
    const uint16_t enabled_mask = service->enabled_mask;

    for (size_t index = 0U; index < servo_descriptor_count(); ++index)
    {
        const servo_descriptor_t *descriptor =
            &servo_descriptor_table()[index];

        if ((enabled_mask & descriptor->mask) != 0U)
        {
            service->driver_ops->stop(
                service->driver_context,
                descriptor->id);
        }
    }

    service->enabled_mask = 0U;
    service->owner = SERVO_SERVICE_OWNER_MANUAL;
    service->motion_mask = 0U;
    service->logical_pose_known_mask = 0U;
    for (size_t index = 0U; index < SERVO_DESCRIPTOR_COUNT; ++index)
    {
        service->logical_angle_cdeg[index] = 0;
    }
}

servo_service_result_t servo_service_motion_begin(
    servo_service_t *service,
    uint16_t mask)
{
    const servo_service_result_t mask_result =
        servo_service_validate_mask(mask);

    if (mask_result != SERVO_SERVICE_RESULT_OK)
    {
        return mask_result;
    }

    if (servo_service_motion_owner(service))
    {
        return SERVO_SERVICE_RESULT_BUSY;
    }

    if ((service->enabled_mask & mask) != mask)
    {
        return SERVO_SERVICE_RESULT_SERVO_NOT_ENABLED;
    }

    service->owner = SERVO_SERVICE_OWNER_MOTION;
    service->motion_mask = mask;
    return SERVO_SERVICE_RESULT_OK;
}

void servo_service_motion_end(
    servo_service_t *service)
{
    if (servo_service_motion_owner(service))
    {
        service->owner = SERVO_SERVICE_OWNER_MANUAL;
        service->motion_mask = 0U;
    }
}

void servo_service_motion_set_mask(
    servo_service_t *service,
    uint16_t mask)
{
    if (!servo_service_motion_owner(service) ||
        (servo_service_validate_mask(mask) != SERVO_SERVICE_RESULT_OK))
    {
        return;
    }

    service->motion_mask = mask;
}

void servo_service_motion_abort(
    servo_service_t *service)
{
    servo_service_motion_end(service);
}

bool servo_service_motion_is_active(
    const servo_service_t *service)
{
    return servo_service_motion_owner(service);
}

uint16_t servo_service_motion_mask(
    const servo_service_t *service)
{
    return servo_service_motion_owner(service) ? service->motion_mask : 0U;
}

servo_service_result_t servo_service_set_pwm(
    servo_service_t *service,
    uint8_t servo_id,
    uint16_t pulse_us)
{
    if (servo_service_motion_owner(service))
    {
        return SERVO_SERVICE_RESULT_BUSY;
    }

    const servo_descriptor_t *descriptor =
        servo_service_descriptor(servo_id);

    if (descriptor == NULL)
    {
        return SERVO_SERVICE_RESULT_UNSUPPORTED_SERVO;
    }

    if (!servo_service_is_enabled(service, descriptor))
    {
        return SERVO_SERVICE_RESULT_SERVO_NOT_ENABLED;
    }

    if ((pulse_us < descriptor->command_min_pulse_us) ||
        (pulse_us > descriptor->command_max_pulse_us))
    {
        return SERVO_SERVICE_RESULT_OUT_OF_RANGE;
    }

    service->driver_ops->write_pulse_us(
        service->driver_context,
        descriptor->id,
        pulse_us);

    service->logical_pose_known_mask = (uint16_t)(
        service->logical_pose_known_mask & (uint16_t)(~descriptor->mask));

    return SERVO_SERVICE_RESULT_OK;
}

servo_service_result_t servo_service_set_angle(
    servo_service_t *service,
    uint8_t servo_id,
    int16_t angle_cdeg)
{
    if (servo_service_motion_owner(service))
    {
        return SERVO_SERVICE_RESULT_BUSY;
    }

    const servo_descriptor_t *descriptor =
        servo_service_descriptor(servo_id);

    if ((descriptor == NULL) || !descriptor->angle_supported)
    {
        return SERVO_SERVICE_RESULT_UNSUPPORTED_SERVO;
    }

    if (!servo_service_is_enabled(service, descriptor))
    {
        return SERVO_SERVICE_RESULT_SERVO_NOT_ENABLED;
    }

    if ((angle_cdeg < descriptor->command_min_angle_cdeg) ||
        (angle_cdeg > descriptor->command_max_angle_cdeg))
    {
        return SERVO_SERVICE_RESULT_OUT_OF_RANGE;
    }

    service->driver_ops->write_pulse_us(
        service->driver_context,
        descriptor->id,
        servo_calibration_angle_to_pulse(
            &descriptor->calibration,
            angle_cdeg));

    service->logical_angle_cdeg[descriptor->id] = angle_cdeg;
    service->logical_pose_known_mask = (uint16_t)(
        service->logical_pose_known_mask | descriptor->mask);

    return SERVO_SERVICE_RESULT_OK;
}

servo_service_result_t servo_service_set_angle_from_motion(
    servo_service_t *service,
    uint8_t servo_id,
    int32_t angle_cdeg)
{
    const servo_descriptor_t *descriptor =
        servo_service_descriptor(servo_id);

    if (!servo_service_motion_owner(service))
    {
        return SERVO_SERVICE_RESULT_BUSY;
    }

    if ((descriptor == NULL) || !descriptor->angle_supported)
    {
        return SERVO_SERVICE_RESULT_UNSUPPORTED_SERVO;
    }

    if (!servo_service_is_enabled(service, descriptor))
    {
        return SERVO_SERVICE_RESULT_SERVO_NOT_ENABLED;
    }

    if ((angle_cdeg < descriptor->command_min_angle_cdeg) ||
        (angle_cdeg > descriptor->command_max_angle_cdeg))
    {
        return SERVO_SERVICE_RESULT_OUT_OF_RANGE;
    }

    service->driver_ops->write_pulse_us(
        service->driver_context,
        descriptor->id,
        servo_calibration_angle_to_pulse(
            &descriptor->calibration,
            (int16_t)angle_cdeg));

    service->logical_angle_cdeg[descriptor->id] = (int16_t)angle_cdeg;
    service->logical_pose_known_mask = (uint16_t)(
        service->logical_pose_known_mask | descriptor->mask);

    return SERVO_SERVICE_RESULT_OK;
}

servo_service_result_t servo_service_neutral(
    servo_service_t *service,
    uint16_t mask)
{
    if (servo_service_motion_owner(service))
    {
        return SERVO_SERVICE_RESULT_BUSY;
    }

    const servo_service_result_t mask_result =
        servo_service_validate_mask(mask);

    if (mask_result != SERVO_SERVICE_RESULT_OK)
    {
        return mask_result;
    }

    if ((service->enabled_mask & mask) != mask)
    {
        return SERVO_SERVICE_RESULT_SERVO_NOT_ENABLED;
    }

    for (size_t index = 0U; index < servo_descriptor_count(); ++index)
    {
        const servo_descriptor_t *descriptor =
            &servo_descriptor_table()[index];

        if ((mask & descriptor->mask) != 0U)
        {
            service->driver_ops->write_pulse_us(
                service->driver_context,
                descriptor->id,
                descriptor->calibration.neutral_pulse_us);
            service->logical_angle_cdeg[descriptor->id] = 0;
            service->logical_pose_known_mask = (uint16_t)(
                service->logical_pose_known_mask | descriptor->mask);
        }
    }

    return SERVO_SERVICE_RESULT_OK;
}

uint16_t servo_service_enabled_mask(
    const servo_service_t *service)
{
    return service == NULL ? 0U : service->enabled_mask;
}

bool servo_service_logical_pose_is_known(
    const servo_service_t *service,
    uint16_t mask)
{
    return (service != NULL) && (mask != 0U) &&
           ((service->logical_pose_known_mask & mask) == mask);
}

bool servo_service_logical_angle_cdeg(
    const servo_service_t *service,
    uint8_t servo_id,
    int16_t *angle_cdeg)
{
    const servo_descriptor_t *descriptor;

    if ((service == NULL) || (angle_cdeg == NULL) ||
        (servo_id >= SERVO_DESCRIPTOR_COUNT))
    {
        return false;
    }

    descriptor = servo_service_descriptor(servo_id);
    if ((descriptor == NULL) ||
        !servo_service_logical_pose_is_known(service, descriptor->mask))
    {
        return false;
    }

    *angle_cdeg = service->logical_angle_cdeg[servo_id];
    return true;
}
