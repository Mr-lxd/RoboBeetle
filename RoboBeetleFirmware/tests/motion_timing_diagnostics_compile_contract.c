#include "motion_timing_diagnostics.h"

#ifndef EXPECT_DIAGNOSTICS_ACTIVE
#error "EXPECT_DIAGNOSTICS_ACTIVE must be supplied by the compile contract"
#endif

#ifndef EXPECT_REDUCED_TELEMETRY_ACTIVE
#error "EXPECT_REDUCED_TELEMETRY_ACTIVE must be supplied by the compile contract"
#endif

_Static_assert(
    MOTION_TIMING_DIAGNOSTICS_ACTIVE == EXPECT_DIAGNOSTICS_ACTIVE,
    "diagnostics compile contract mismatch");
_Static_assert(
    MOTION_TIMING_REDUCED_TELEMETRY_ACTIVE == EXPECT_REDUCED_TELEMETRY_ACTIVE,
    "reduced telemetry compile contract mismatch");

int main(void)
{
    motion_timing_mark_t mark =
        motion_timing_diagnostics_loop_begin();

    motion_timing_diagnostics_loop_end(mark);
    motion_timing_diagnostics_init(
        MOTION_GAIT_BACKEND_CPG_VALUE);
    motion_timing_diagnostics_motion_tick_begin(10U);
    motion_timing_diagnostics_motion_tick_end(
        MOTION_TIMING_STATUS_OK);
    return 0;
}
