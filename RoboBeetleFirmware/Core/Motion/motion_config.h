#ifndef ROBOBEETLE_MOTION_CONFIG_H
#define ROBOBEETLE_MOTION_CONFIG_H

#define MOTION_GAIT_TICK_MS 10U
#define MOTION_TRANSITION_DURATION_MS 750U

#define MOTION_PI_F 3.14159265358979323846F

#define MOTION_PROFILE_FREQUENCY_HZ 0.5F
#define MOTION_PROFILE_PADDLE_AMPLITUDE_CDEG 1000
#define MOTION_PROFILE_TURN_REDUCED_SIDE_SCALE 0.5F
#define MOTION_PROFILE_ASCEND_FRONT_AXIS_BIAS_CDEG 1000
#define MOTION_PROFILE_DESCEND_FRONT_AXIS_BIAS_CDEG (-1000)

#define MOTION_REAR_MIN_CDEG (-3000)
#define MOTION_REAR_MAX_CDEG 4500
#define MOTION_FRONT_MIN_CDEG (-4500)
#define MOTION_FRONT_MAX_CDEG 2800

_Static_assert(MOTION_GAIT_TICK_MS == 10U,
               "Bench gait tick must remain 10 ms");
_Static_assert(MOTION_TRANSITION_DURATION_MS > 0U,
               "Motion transition duration must be positive");
_Static_assert(MOTION_REAR_MIN_CDEG == -3000,
               "Rear operational minimum must remain -3000 cdeg");
_Static_assert(MOTION_REAR_MAX_CDEG == 4500,
               "Rear operational maximum must remain +4500 cdeg");
_Static_assert(MOTION_FRONT_MIN_CDEG == -4500,
               "Front operational minimum must remain -4500 cdeg");
_Static_assert(MOTION_FRONT_MAX_CDEG == 2800,
               "Front operational maximum must remain +2800 cdeg");

#endif /* ROBOBEETLE_MOTION_CONFIG_H */
