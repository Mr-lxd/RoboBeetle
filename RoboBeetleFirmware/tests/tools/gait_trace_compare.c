#include "gait_trace_compare.h"

#include "cpg_gait_generator.h"
#include "motion_config.h"
#include "simple_gait_generator.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static const char canonical_header[] =
    "time_ms,backend,mode,front_right_cdeg,front_left_cdeg,"
    "rear_right_cdeg,rear_left_cdeg,front_axis_cdeg,"
    "guarded_front_right_cdeg,guarded_front_left_cdeg,"
    "guarded_rear_right_cdeg,guarded_rear_left_cdeg,"
    "guarded_front_axis_cdeg\n";

static const char internal_header[] =
    "time_ms,mode,node0_phase,node1_phase,node2_phase,node3_phase,"
    "node0_amplitude,node1_amplitude,node2_amplitude,node3_amplitude,"
    "node0_target_amplitude,node1_target_amplitude,"
    "node2_target_amplitude,node3_target_amplitude,"
    "theta_dot0,theta_dot1,theta_dot2,theta_dot3,"
    "raw_output0,raw_output1,raw_output2,raw_output3\n";

static bool make_path(
    char *path,
    size_t path_size,
    const char *directory,
    const char *filename)
{
    int written;

    if ((path == NULL) || (directory == NULL) || (filename == NULL) ||
        (directory[0] == '\0'))
    {
        return false;
    }

    written = snprintf(path, path_size, "%s/%s", directory, filename);
    return (written >= 0) && ((size_t)written < path_size);
}

static motion_mode_t mode_at(uint32_t time_ms)
{
    if (time_ms < 5000U)
    {
        return MOTION_FORWARD;
    }
    if (time_ms < 8000U)
    {
        return MOTION_TURN_LEFT;
    }
    if (time_ms < 11000U)
    {
        return MOTION_FORWARD;
    }
    if (time_ms < 14000U)
    {
        return MOTION_TURN_RIGHT;
    }
    if (time_ms < GAIT_TRACE_FINAL_TIME_MS)
    {
        return MOTION_FORWARD;
    }
    return MOTION_STOP;
}

static const char *mode_name(motion_mode_t mode)
{
    switch (mode)
    {
        case MOTION_STOP:
            return "STOP";
        case MOTION_FORWARD:
            return "FORWARD";
        case MOTION_TURN_LEFT:
            return "TURN_LEFT";
        case MOTION_TURN_RIGHT:
            return "TURN_RIGHT";
        case MOTION_ASCEND:
            return "ASCEND";
        case MOTION_DESCEND:
            return "DESCEND";
        case MOTION_BACKWARD:
        case MOTION_COUNT:
        default:
            return "INVALID";
    }
}

static int32_t clamp_cdeg(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static void project_motion_guard(
    const joint_targets_t *raw,
    joint_targets_t *guarded)
{
    *guarded = *raw;
    guarded->front_right_cdeg = clamp_cdeg(
        raw->front_right_cdeg,
        MOTION_FRONT_MIN_CDEG,
        MOTION_FRONT_MAX_CDEG);
    guarded->front_left_cdeg = clamp_cdeg(
        raw->front_left_cdeg,
        MOTION_FRONT_MIN_CDEG,
        MOTION_FRONT_MAX_CDEG);
    guarded->rear_right_cdeg = clamp_cdeg(
        raw->rear_right_cdeg,
        MOTION_REAR_MIN_CDEG,
        MOTION_REAR_MAX_CDEG);
    guarded->rear_left_cdeg = clamp_cdeg(
        raw->rear_left_cdeg,
        MOTION_REAR_MIN_CDEG,
        MOTION_REAR_MAX_CDEG);
}

static bool write_canonical_row(
    FILE *file,
    uint32_t time_ms,
    const char *backend,
    motion_mode_t mode,
    const joint_targets_t *raw,
    const joint_targets_t *guarded)
{
    return fprintf(
        file,
        "%" PRIu32 ",%s,%s,%" PRId32 ",%" PRId32 ",%" PRId32
        ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32
        ",%" PRId32 ",%" PRId32 ",%" PRId32 "\n",
        time_ms,
        backend,
        mode_name(mode),
        raw->front_right_cdeg,
        raw->front_left_cdeg,
        raw->rear_right_cdeg,
        raw->rear_left_cdeg,
        raw->front_axis_cdeg,
        guarded->front_right_cdeg,
        guarded->front_left_cdeg,
        guarded->rear_right_cdeg,
        guarded->rear_left_cdeg,
        guarded->front_axis_cdeg) >= 0;
}

static bool write_internal_row(
    FILE *file,
    uint32_t time_ms,
    motion_mode_t mode,
    const cpg_core_t *core)
{
    int written;

    written = fprintf(
        file,
        "%" PRIu32 ",%s,"
        "%.17g,%.17g,%.17g,%.17g,"
        "%.17g,%.17g,%.17g,%.17g,"
        "%.17g,%.17g,%.17g,%.17g,"
        "%.17g,%.17g,%.17g,%.17g,"
        "%.17g,%.17g,%.17g,%.17g\n",
        time_ms,
        mode_name(mode),
        core->phase[0],
        core->phase[1],
        core->phase[2],
        core->phase[3],
        core->amplitude[0],
        core->amplitude[1],
        core->amplitude[2],
        core->amplitude[3],
        core->params.target_amplitude[0],
        core->params.target_amplitude[1],
        core->params.target_amplitude[2],
        core->params.target_amplitude[3],
        core->theta_dot[0],
        core->theta_dot[1],
        core->theta_dot[2],
        core->theta_dot[3],
        core->raw_output[0],
        core->raw_output[1],
        core->raw_output[2],
        core->raw_output[3]);
    return written >= 0;
}

static bool close_file(FILE **file)
{
    if ((file == NULL) || (*file == NULL))
    {
        return true;
    }

    if (fclose(*file) != 0)
    {
        *file = NULL;
        return false;
    }

    *file = NULL;
    return true;
}

int gait_trace_compare_generate(const char *output_dir)
{
    char simple_path[1024];
    char cpg_path[1024];
    char internal_path[1024];
    FILE *simple_file = NULL;
    FILE *cpg_file = NULL;
    FILE *internal_file = NULL;
    simple_gait_generator_t simple_generator;
    cpg_gait_generator_t cpg_generator;
    uint32_t time_ms;
    bool success = false;

    if (!make_path(
            simple_path,
            sizeof(simple_path),
            output_dir,
            "gait_trace_simple.csv") ||
        !make_path(
            cpg_path,
            sizeof(cpg_path),
            output_dir,
            "gait_trace_cpg.csv") ||
        !make_path(
            internal_path,
            sizeof(internal_path),
            output_dir,
            "gait_trace_cpg_internal.csv"))
    {
        return 1;
    }

    simple_file = fopen(simple_path, "wb");
    cpg_file = fopen(cpg_path, "wb");
    internal_file = fopen(internal_path, "wb");
    if ((simple_file == NULL) || (cpg_file == NULL) ||
        (internal_file == NULL))
    {
        goto cleanup;
    }

    if ((fputs(canonical_header, simple_file) == EOF) ||
        (fputs(canonical_header, cpg_file) == EOF) ||
        (fputs(internal_header, internal_file) == EOF))
    {
        goto cleanup;
    }

    simple_gait_generator_init(&simple_generator);
    cpg_gait_generator_init(&cpg_generator);

    for (time_ms = 0U;; time_ms += GAIT_TRACE_SAMPLE_INTERVAL_MS)
    {
        const motion_mode_t mode = mode_at(time_ms);
        joint_targets_t simple_targets;
        joint_targets_t cpg_targets;
        joint_targets_t simple_guarded;
        joint_targets_t cpg_guarded;

        if (time_ms != 0U)
        {
            simple_gait_generator_advance(
                &simple_generator,
                GAIT_TRACE_SAMPLE_INTERVAL_MS);
            cpg_gait_generator_advance(
                &cpg_generator,
                GAIT_TRACE_SAMPLE_INTERVAL_MS);
        }

        if (!simple_gait_generator_sample(
                &simple_generator,
                mode,
                1.0F,
                1.0F,
                &simple_targets) ||
            !cpg_gait_generator_sample(
                &cpg_generator,
                mode,
                1.0F,
                1.0F,
                &cpg_targets))
        {
            goto cleanup;
        }

        project_motion_guard(&simple_targets, &simple_guarded);
        project_motion_guard(&cpg_targets, &cpg_guarded);
        if (!write_canonical_row(
                simple_file,
                time_ms,
                "simple",
                mode,
                &simple_targets,
                &simple_guarded) ||
            !write_canonical_row(
                cpg_file,
                time_ms,
                "cpg",
                mode,
                &cpg_targets,
                &cpg_guarded) ||
            !write_internal_row(
                internal_file,
                time_ms,
                mode,
                &cpg_generator.core))
        {
            goto cleanup;
        }

        if (time_ms == GAIT_TRACE_FINAL_TIME_MS)
        {
            break;
        }
    }

    success = true;

cleanup:
    if (!close_file(&simple_file) ||
        !close_file(&cpg_file) ||
        !close_file(&internal_file))
    {
        success = false;
    }
    return success ? 0 : 1;
}
