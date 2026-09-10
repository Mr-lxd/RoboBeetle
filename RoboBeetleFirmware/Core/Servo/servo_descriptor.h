#ifndef SERVO_DESCRIPTOR_H
#define SERVO_DESCRIPTOR_H

#include "servo_calibration.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SERVO_DESCRIPTOR_COUNT 5U
#define SERVO_DESCRIPTOR_SUPPORTED_MASK 0x001FU

typedef enum
{
    SERVO_ID_FRONT_RIGHT = 0U,
    SERVO_ID_FRONT_LEFT = 1U,
    SERVO_ID_FRONT_AXIS = 2U,
    SERVO_ID_REAR_RIGHT = 3U,
    SERVO_ID_REAR_LEFT = 4U,
    SERVO_ID_COUNT = SERVO_DESCRIPTOR_COUNT
} servo_id_t;

/* HAL-independent selectors. The STM32 adapter maps them to HAL handles. */
typedef enum
{
    SERVO_TIMER_TIM3 = 0,
    SERVO_TIMER_TIM4 = 1
} servo_timer_id_t;

typedef enum
{
    SERVO_CHANNEL_1 = 1,
    SERVO_CHANNEL_2 = 2,
    SERVO_CHANNEL_3 = 3
} servo_channel_id_t;

typedef struct
{
    uint8_t id;
    uint16_t mask;
    bool supported;
    bool angle_supported;
    servo_calibration_t calibration;
    uint16_t command_min_pulse_us;
    uint16_t command_max_pulse_us;
    int16_t command_min_angle_cdeg;
    int16_t command_max_angle_cdeg;
    servo_timer_id_t timer;
    servo_channel_id_t channel;
} servo_descriptor_t;

const servo_descriptor_t *servo_descriptor_table(void);
size_t servo_descriptor_count(void);
const servo_descriptor_t *servo_descriptor_for_id(uint8_t id);
uint16_t servo_descriptor_supported_mask(void);

#endif /* SERVO_DESCRIPTOR_H */
