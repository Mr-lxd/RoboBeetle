#ifndef ROBOBEETLE_MOTION_TYPES_H
#define ROBOBEETLE_MOTION_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTION_STOP = 0,
    MOTION_FORWARD,
    MOTION_BACKWARD,
    MOTION_TURN_LEFT,
    MOTION_TURN_RIGHT,
    MOTION_ASCEND,
    MOTION_DESCEND,
    MOTION_COUNT
} motion_mode_t;

typedef enum
{
    MOTION_STATE_STOPPED = 0,
    MOTION_STATE_RUNNING,
    MOTION_STATE_STOPPING,
    MOTION_STATE_FAULTED
} motion_state_t;

typedef enum
{
    MOTION_ACTION_STOP = 0,
    MOTION_ACTION_START = 1
} motion_action_t;

typedef enum
{
    MOTION_GAIT_BACKEND_SIMPLE_GAIT = 0,
    MOTION_GAIT_BACKEND_CPG = 1,
    MOTION_GAIT_BACKEND_COUNT,
    MOTION_GAIT_BACKEND_UNSPECIFIED = 0xff
} motion_gait_backend_t;

_Static_assert(MOTION_STOP == 0, "Motion STOP wire value must remain zero");
_Static_assert(MOTION_FORWARD == 1, "Motion FORWARD wire value must remain one");
_Static_assert(MOTION_BACKWARD == 2, "Motion BACKWARD wire value must remain two");
_Static_assert(MOTION_TURN_LEFT == 3, "Motion TURN_LEFT wire value must remain three");
_Static_assert(MOTION_TURN_RIGHT == 4, "Motion TURN_RIGHT wire value must remain four");
_Static_assert(MOTION_ASCEND == 5, "Motion ASCEND wire value must remain five");
_Static_assert(MOTION_DESCEND == 6, "Motion DESCEND wire value must remain six");
_Static_assert(MOTION_GAIT_BACKEND_SIMPLE_GAIT == 0,
               "SimpleGait backend value must remain zero");
_Static_assert(MOTION_GAIT_BACKEND_CPG == 1,
               "CPG backend value must remain one");

static inline bool motion_mode_is_valid(motion_mode_t mode)
{
    return mode >= MOTION_STOP && mode < MOTION_COUNT;
}

static inline bool motion_gait_backend_is_valid(
    motion_gait_backend_t backend)
{
    return backend >= MOTION_GAIT_BACKEND_SIMPLE_GAIT &&
           backend < MOTION_GAIT_BACKEND_COUNT;
}

#endif /* ROBOBEETLE_MOTION_TYPES_H */
