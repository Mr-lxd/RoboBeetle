#ifndef SERVO_SERVICE_H
#define SERVO_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "servo_descriptor.h"

typedef enum
{
    SERVO_SERVICE_RESULT_OK = 0,
    SERVO_SERVICE_RESULT_INVALID_PAYLOAD,
    SERVO_SERVICE_RESULT_UNSUPPORTED_SERVO,
    SERVO_SERVICE_RESULT_SERVO_NOT_ENABLED,
    SERVO_SERVICE_RESULT_OUT_OF_RANGE,
    SERVO_SERVICE_RESULT_HARDWARE_FAILURE,
    SERVO_SERVICE_RESULT_BUSY
} servo_service_result_t;

typedef enum
{
    SERVO_SERVICE_OWNER_MANUAL = 0,
    SERVO_SERVICE_OWNER_MOTION
} servo_service_owner_t;

typedef struct
{
    void (*write_pulse_us)(
        void *context,
        uint8_t servo_id,
        uint16_t pulse_us);
    bool (*start)(
        void *context,
        uint8_t servo_id);
    void (*stop)(
        void *context,
        uint8_t servo_id);
} servo_service_driver_ops_t;

typedef struct
{
    const servo_service_driver_ops_t *driver_ops;
    void *driver_context;
    uint16_t enabled_mask;
    servo_service_owner_t owner;
    uint16_t motion_mask;
    int16_t logical_angle_cdeg[SERVO_DESCRIPTOR_COUNT];
    uint16_t logical_pose_known_mask;
} servo_service_t;

void servo_service_init(
    servo_service_t *service,
    const servo_service_driver_ops_t *driver_ops,
    void *driver_context);

servo_service_result_t servo_service_validate_mask(uint16_t mask);

servo_service_result_t servo_service_enable(
    servo_service_t *service,
    uint16_t mask);

servo_service_result_t servo_service_disable(
    servo_service_t *service,
    uint16_t mask);

void servo_service_disable_all(servo_service_t *service);

servo_service_result_t servo_service_motion_begin(
    servo_service_t *service,
    uint16_t mask);

void servo_service_motion_end(
    servo_service_t *service);

void servo_service_motion_set_mask(
    servo_service_t *service,
    uint16_t mask);

void servo_service_motion_abort(
    servo_service_t *service);

bool servo_service_motion_is_active(
    const servo_service_t *service);

uint16_t servo_service_motion_mask(
    const servo_service_t *service);

servo_service_result_t servo_service_set_pwm(
    servo_service_t *service,
    uint8_t servo_id,
    uint16_t pulse_us);

servo_service_result_t servo_service_set_angle(
    servo_service_t *service,
    uint8_t servo_id,
    int16_t angle_cdeg);

servo_service_result_t servo_service_set_angle_from_motion(
    servo_service_t *service,
    uint8_t servo_id,
    int32_t angle_cdeg);

servo_service_result_t servo_service_neutral(
    servo_service_t *service,
    uint16_t mask);

uint16_t servo_service_enabled_mask(
    const servo_service_t *service);

/*
 * Logical pose is known only when the last accepted command for a channel was
 * angle-domain (Enable/SetAngle/Neutral/Motion).  Raw SetPWM intentionally
 * invalidates that channel because its logical angle cannot be inferred from
 * a calibrated pulse without an explicit inverse-mapping contract.
 */
bool servo_service_logical_pose_is_known(
    const servo_service_t *service,
    uint16_t mask);

bool servo_service_logical_angle_cdeg(
    const servo_service_t *service,
    uint8_t servo_id,
    int16_t *angle_cdeg);

#endif /* SERVO_SERVICE_H */
