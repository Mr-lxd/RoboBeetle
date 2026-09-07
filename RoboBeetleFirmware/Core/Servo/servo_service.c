#include "servo_service.h"

#include "servo_calibration.h"

static bool servo_service_servo1_enabled(
    const servo_service_t *service)
{
    return (service->enabled_mask &
            SERVO_SERVICE_SERVO1_MASK) != 0U;
}

void servo_service_init(
    servo_service_t *service,
    const servo_service_driver_ops_t *driver_ops,
    void *driver_context)
{
    service->driver_ops = driver_ops;
    service->driver_context = driver_context;
    service->enabled_mask = 0U;
}

servo_service_result_t servo_service_validate_mask(uint16_t mask)
{
    if (mask == 0U)
    {
        return SERVO_SERVICE_RESULT_INVALID_PAYLOAD;
    }

    if ((mask & (uint16_t)(~SERVO_SERVICE_SERVO1_MASK)) != 0U)
    {
        return SERVO_SERVICE_RESULT_UNSUPPORTED_SERVO;
    }

    return SERVO_SERVICE_RESULT_OK;
}

servo_service_result_t servo_service_enable(
    servo_service_t *service,
    uint16_t mask)
{
    const servo_calibration_t *calibration =
        servo_calibration_servo1();
    servo_service_result_t mask_result =
        servo_service_validate_mask(mask);

    if (mask_result != SERVO_SERVICE_RESULT_OK)
    {
        return mask_result;
    }

    service->driver_ops->write_pulse_us(
        service->driver_context,
        SERVO_SERVICE_SERVO1_ID,
        calibration->neutral_pulse_us);

    if (!service->driver_ops->start(
            service->driver_context,
            SERVO_SERVICE_SERVO1_ID))
    {
        return SERVO_SERVICE_RESULT_HARDWARE_FAILURE;
    }

    service->enabled_mask |=
        (mask & SERVO_SERVICE_SERVO1_MASK);

    return SERVO_SERVICE_RESULT_OK;
}

servo_service_result_t servo_service_disable(
    servo_service_t *service,
    uint16_t mask)
{
    servo_service_result_t mask_result =
        servo_service_validate_mask(mask);

    if (mask_result != SERVO_SERVICE_RESULT_OK)
    {
        return mask_result;
    }

    if ((mask & SERVO_SERVICE_SERVO1_MASK) != 0U)
    {
        service->driver_ops->stop(
            service->driver_context,
            SERVO_SERVICE_SERVO1_ID);
    }

    service->enabled_mask &=
        (uint16_t)(~mask);

    return SERVO_SERVICE_RESULT_OK;
}

void servo_service_disable_all(servo_service_t *service)
{
    if (servo_service_servo1_enabled(service))
    {
        service->driver_ops->stop(
            service->driver_context,
            SERVO_SERVICE_SERVO1_ID);
    }

    service->enabled_mask = 0U;
}

servo_service_result_t servo_service_set_pwm(
    servo_service_t *service,
    uint8_t servo_id,
    uint16_t pulse_us)
{
    const servo_calibration_t *calibration =
        servo_calibration_servo1();

    if (servo_id != SERVO_SERVICE_SERVO1_ID)
    {
        return SERVO_SERVICE_RESULT_UNSUPPORTED_SERVO;
    }

    if (!servo_service_servo1_enabled(service))
    {
        return SERVO_SERVICE_RESULT_SERVO_NOT_ENABLED;
    }

    if ((pulse_us < calibration->min_pulse_us) ||
        (pulse_us > calibration->max_pulse_us))
    {
        return SERVO_SERVICE_RESULT_OUT_OF_RANGE;
    }

    service->driver_ops->write_pulse_us(
        service->driver_context,
        servo_id,
        pulse_us);

    return SERVO_SERVICE_RESULT_OK;
}

servo_service_result_t servo_service_set_angle(
    servo_service_t *service,
    uint8_t servo_id,
    int16_t angle_cdeg)
{
    const servo_calibration_t *calibration =
        servo_calibration_servo1();

    if (servo_id != SERVO_SERVICE_SERVO1_ID)
    {
        return SERVO_SERVICE_RESULT_UNSUPPORTED_SERVO;
    }

    if (!servo_service_servo1_enabled(service))
    {
        return SERVO_SERVICE_RESULT_SERVO_NOT_ENABLED;
    }

    if ((angle_cdeg < calibration->min_angle_cdeg) ||
        (angle_cdeg > calibration->max_angle_cdeg))
    {
        return SERVO_SERVICE_RESULT_OUT_OF_RANGE;
    }

    service->driver_ops->write_pulse_us(
        service->driver_context,
        servo_id,
        servo_calibration_angle_to_pulse(angle_cdeg));

    return SERVO_SERVICE_RESULT_OK;
}

servo_service_result_t servo_service_neutral(
    servo_service_t *service,
    uint16_t mask)
{
    const servo_calibration_t *calibration =
        servo_calibration_servo1();
    servo_service_result_t mask_result =
        servo_service_validate_mask(mask);

    if (mask_result != SERVO_SERVICE_RESULT_OK)
    {
        return mask_result;
    }

    if ((service->enabled_mask & mask) != mask)
    {
        return SERVO_SERVICE_RESULT_SERVO_NOT_ENABLED;
    }

    service->driver_ops->write_pulse_us(
        service->driver_context,
        SERVO_SERVICE_SERVO1_ID,
        calibration->neutral_pulse_us);

    return SERVO_SERVICE_RESULT_OK;
}

uint16_t servo_service_enabled_mask(
    const servo_service_t *service)
{
    return service->enabled_mask;
}
