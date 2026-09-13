#ifndef ROBOBEETLE_GAIT_GENERATOR_H
#define ROBOBEETLE_GAIT_GENERATOR_H

#include "joint_targets.h"
#include "motion_types.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    void *context;
    void (*advance)(void *context, uint32_t dt_ms);
    bool (*sample)(
        void *context,
        motion_mode_t mode,
        float amplitude_scale,
        float bias_scale,
        joint_targets_t *targets);
    bool (*is_mode_valid)(void *context, motion_mode_t mode);
    uint32_t (*diagnostic_count)(void *context);
} gait_generator_ops_t;

typedef struct
{
    const gait_generator_ops_t *ops;
    void *context;
} gait_generator_t;

#endif /* ROBOBEETLE_GAIT_GENERATOR_H */
