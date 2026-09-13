#ifndef ROBOBEETLE_JOINT_TARGETS_H
#define ROBOBEETLE_JOINT_TARGETS_H

#include <stdint.h>

typedef struct
{
    int32_t front_right_cdeg;
    int32_t front_left_cdeg;
    int32_t front_axis_cdeg;
    int32_t rear_right_cdeg;
    int32_t rear_left_cdeg;
} joint_targets_t;

#endif /* ROBOBEETLE_JOINT_TARGETS_H */
