#ifndef SERVO_SERVICE_H
#define SERVO_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#define SERVO_SERVICE_SERVO1_ID   0U
#define SERVO_SERVICE_SERVO1_MASK 0x0001U

typedef enum
{
    SERVO_SERVICE_RESULT_OK = 0,
    SERVO_SERVICE_RESULT_INVALID_PAYLOAD,
    SERVO_SERVICE_RESULT_UNSUPPORTED_SERVO,
    SERVO_SERVICE_RESULT_SERVO_NOT_ENABLED,
    SERVO_SERVICE_RESULT_OUT_OF_RANGE,
    SERVO_SERVICE_RESULT_HARDWARE_FAILURE
} servo_service_result_t;

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

servo_service_result_t servo_service_set_pwm(
    servo_service_t *service,
    uint8_t servo_id,
    uint16_t pulse_us);

servo_service_result_t servo_service_set_angle(
    servo_service_t *service,
    uint8_t servo_id,
    int16_t angle_cdeg);

servo_service_result_t servo_service_neutral(
    servo_service_t *service,
    uint16_t mask);

uint16_t servo_service_enabled_mask(
    const servo_service_t *service);

#endif /* SERVO_SERVICE_H */
