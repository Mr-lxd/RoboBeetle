#include "servo_descriptor.h"

_Static_assert(SERVO_ID_FRONT_RIGHT == 0U, "FrontRight ID must be zero");
_Static_assert(SERVO_ID_FRONT_LEFT == 1U, "FrontLeft ID must be one");
_Static_assert(SERVO_ID_FRONT_AXIS == 2U, "FrontAxis ID must be two");
_Static_assert(SERVO_ID_REAR_RIGHT == 3U, "RearRight ID must be three");
_Static_assert(SERVO_ID_REAR_LEFT == 4U, "RearLeft ID must be four");
_Static_assert(SERVO_DESCRIPTOR_COUNT == 5U, "descriptor count must be five");
_Static_assert(SERVO_DESCRIPTOR_SUPPORTED_MASK == 0x001FU,
               "descriptor mask must include five bits");

static const servo_descriptor_t descriptors[SERVO_DESCRIPTOR_COUNT] = {
    [SERVO_ID_FRONT_RIGHT] = {
        .id = SERVO_ID_FRONT_RIGHT,
        .mask = 0x0001U,
        .supported = true,
        .angle_supported = true,
        .calibration = {
            .min_pulse_us = 1000U,
            .neutral_pulse_us = 1500U,
            .max_pulse_us = 2000U,
            .min_angle_cdeg = -5000,
            .max_angle_cdeg = 5000,
        },
        .command_min_pulse_us = 1050U,
        .command_max_pulse_us = 1950U,
        .command_min_angle_cdeg = -4500,
        .command_max_angle_cdeg = 4500,
        .timer = SERVO_TIMER_TIM3,
        .channel = SERVO_CHANNEL_1,
    },
    [SERVO_ID_FRONT_LEFT] = {
        .id = SERVO_ID_FRONT_LEFT,
        .mask = 0x0002U,
        .supported = true,
        .angle_supported = true,
        .calibration = {
            .min_pulse_us = 1000U,
            .neutral_pulse_us = 1500U,
            .max_pulse_us = 2000U,
            .min_angle_cdeg = -5000,
            .max_angle_cdeg = 5000,
        },
        .command_min_pulse_us = 1050U,
        .command_max_pulse_us = 1950U,
        .command_min_angle_cdeg = -4500,
        .command_max_angle_cdeg = 4500,
        .timer = SERVO_TIMER_TIM3,
        .channel = SERVO_CHANNEL_2,
    },
    [SERVO_ID_FRONT_AXIS] = {
        .id = SERVO_ID_FRONT_AXIS,
        .mask = 0x0004U,
        .supported = true,
        .angle_supported = true,
        .calibration = {
            .min_pulse_us = 1060U,
            .neutral_pulse_us = 1745U,
            .max_pulse_us = 2430U,
            .min_angle_cdeg = -9000,
            .max_angle_cdeg = 9000,
        },
        .command_min_pulse_us = 1060U,
        .command_max_pulse_us = 2430U,
        .command_min_angle_cdeg = -9000,
        .command_max_angle_cdeg = 9000,
        .timer = SERVO_TIMER_TIM3,
        .channel = SERVO_CHANNEL_3,
    },
    [SERVO_ID_REAR_RIGHT] = {
        .id = SERVO_ID_REAR_RIGHT,
        .mask = 0x0008U,
        .supported = true,
        .angle_supported = true,
        .calibration = {
            .min_pulse_us = 520U,
            .neutral_pulse_us = 1520U,
            .max_pulse_us = 2520U,
            .min_angle_cdeg = -9000,
            .max_angle_cdeg = 9000,
        },
        .command_min_pulse_us = 820U,
        .command_max_pulse_us = 2220U,
        .command_min_angle_cdeg = -4500,
        .command_max_angle_cdeg = 4500,
        .timer = SERVO_TIMER_TIM4,
        .channel = SERVO_CHANNEL_1,
    },
    [SERVO_ID_REAR_LEFT] = {
        .id = SERVO_ID_REAR_LEFT,
        .mask = 0x0010U,
        .supported = true,
        .angle_supported = true,
        .calibration = {
            .min_pulse_us = 520U,
            .neutral_pulse_us = 1520U,
            .max_pulse_us = 2520U,
            .min_angle_cdeg = -9000,
            .max_angle_cdeg = 9000,
        },
        .command_min_pulse_us = 820U,
        .command_max_pulse_us = 2220U,
        .command_min_angle_cdeg = -4500,
        .command_max_angle_cdeg = 4500,
        .timer = SERVO_TIMER_TIM4,
        .channel = SERVO_CHANNEL_2,
    },
};

const servo_descriptor_t *servo_descriptor_table(void)
{
    return descriptors;
}

size_t servo_descriptor_count(void)
{
    return SERVO_DESCRIPTOR_COUNT;
}

const servo_descriptor_t *servo_descriptor_for_id(uint8_t id)
{
    if (id >= SERVO_DESCRIPTOR_COUNT)
    {
        return NULL;
    }

    if (descriptors[id].id != id)
    {
        return NULL;
    }

    return &descriptors[id];
}

uint16_t servo_descriptor_supported_mask(void)
{
    uint16_t mask = 0U;

    for (size_t index = 0U; index < SERVO_DESCRIPTOR_COUNT; ++index)
    {
        if (descriptors[index].supported)
        {
            mask |= descriptors[index].mask;
        }
    }

    return mask;
}
