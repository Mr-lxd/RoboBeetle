#include "tools/gait_trace_compare.h"

#include "motion_config.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

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

static const char *expected_mode(unsigned int time_ms)
{
    if (time_ms < 5000U)
    {
        return "FORWARD";
    }
    if (time_ms < 8000U)
    {
        return "TURN_LEFT";
    }
    if (time_ms < 11000U)
    {
        return "FORWARD";
    }
    if (time_ms < 14000U)
    {
        return "TURN_RIGHT";
    }
    if (time_ms < GAIT_TRACE_FINAL_TIME_MS)
    {
        return "FORWARD";
    }
    return "STOP";
}

static bool make_path(
    char *path,
    size_t path_size,
    const char *directory,
    const char *filename)
{
    int written;

    written = snprintf(path, path_size, "%s/%s", directory, filename);
    return (written >= 0) && ((size_t)written < path_size);
}

static bool files_equal(const char *path_a, const char *path_b)
{
    FILE *file_a;
    FILE *file_b;
    unsigned char buffer_a[512];
    unsigned char buffer_b[512];
    size_t count_a;
    size_t count_b;
    bool equal = true;

    file_a = fopen(path_a, "rb");
    file_b = fopen(path_b, "rb");
    if ((file_a == NULL) || (file_b == NULL))
    {
        if (file_a != NULL)
        {
            (void)fclose(file_a);
        }
        if (file_b != NULL)
        {
            (void)fclose(file_b);
        }
        return false;
    }

    do
    {
        count_a = fread(buffer_a, 1U, sizeof(buffer_a), file_a);
        count_b = fread(buffer_b, 1U, sizeof(buffer_b), file_b);
        if ((count_a != count_b) ||
            (memcmp(buffer_a, buffer_b, count_a) != 0))
        {
            equal = false;
            break;
        }
    } while (count_a > 0U);

    if (ferror(file_a) || ferror(file_b))
    {
        equal = false;
    }

    (void)fclose(file_a);
    (void)fclose(file_b);
    return equal;
}

static bool validate_canonical_trace(
    const char *path,
    const char *expected_backend)
{
    FILE *file;
    char line[2048];
    unsigned int row = 0U;
    bool valid = true;

    file = fopen(path, "r");
    if (file == NULL)
    {
        return false;
    }

    if ((fgets(line, sizeof(line), file) == NULL) ||
        (strcmp(line, canonical_header) != 0))
    {
        valid = false;
    }

    while (valid && (fgets(line, sizeof(line), file) != NULL))
    {
        unsigned int time_ms;
        char backend[16];
        char mode[16];
        int front_right;
        int front_left;
        int rear_right;
        int rear_left;
        int front_axis;
        int guarded_front_right;
        int guarded_front_left;
        int guarded_rear_right;
        int guarded_rear_left;
        int guarded_front_axis;
        const int fields = sscanf(
            line,
            "%u,%15[^,],%15[^,],%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
            &time_ms,
            backend,
            mode,
            &front_right,
            &front_left,
            &rear_right,
            &rear_left,
            &front_axis,
            &guarded_front_right,
            &guarded_front_left,
            &guarded_rear_right,
            &guarded_rear_left,
            &guarded_front_axis);

        if ((fields != 13) ||
            (time_ms != row * GAIT_TRACE_SAMPLE_INTERVAL_MS) ||
            (strcmp(backend, expected_backend) != 0) ||
            (strcmp(mode, expected_mode(time_ms)) != 0) ||
            (guarded_front_right < MOTION_FRONT_MIN_CDEG) ||
            (guarded_front_right > MOTION_FRONT_MAX_CDEG) ||
            (guarded_front_left < MOTION_FRONT_MIN_CDEG) ||
            (guarded_front_left > MOTION_FRONT_MAX_CDEG) ||
            (guarded_rear_right < MOTION_REAR_MIN_CDEG) ||
            (guarded_rear_right > MOTION_REAR_MAX_CDEG) ||
            (guarded_rear_left < MOTION_REAR_MIN_CDEG) ||
            (guarded_rear_left > MOTION_REAR_MAX_CDEG))
        {
            valid = false;
            break;
        }

        if ((row == (GAIT_TRACE_ROW_COUNT - 1U)) &&
            ((front_right != 0) || (front_left != 0) ||
             (rear_right != 0) || (rear_left != 0) ||
             (front_axis != 0) || (guarded_front_right != 0) ||
             (guarded_front_left != 0) || (guarded_rear_right != 0) ||
             (guarded_rear_left != 0) || (guarded_front_axis != 0)))
        {
            valid = false;
            break;
        }

        ++row;
    }

    if (ferror(file) || (row != GAIT_TRACE_ROW_COUNT))
    {
        valid = false;
    }

    (void)fclose(file);
    return valid;
}

static bool validate_internal_trace(const char *path)
{
    FILE *file;
    char line[4096];
    unsigned int row = 0U;
    bool valid = true;

    file = fopen(path, "r");
    if (file == NULL)
    {
        return false;
    }

    if ((fgets(line, sizeof(line), file) == NULL) ||
        (strcmp(line, internal_header) != 0))
    {
        valid = false;
    }

    while (valid && (fgets(line, sizeof(line), file) != NULL))
    {
        unsigned int time_ms;
        char mode[16];
        char remainder[4000];
        const int fields = sscanf(
            line,
            "%u,%15[^,],%3999[^\n]",
            &time_ms,
            mode,
            remainder);

        if ((fields != 3) ||
            (time_ms != row * GAIT_TRACE_SAMPLE_INTERVAL_MS) ||
            (strcmp(mode, expected_mode(time_ms)) != 0))
        {
            valid = false;
            break;
        }
        ++row;
    }

    if (ferror(file) || (row != GAIT_TRACE_ROW_COUNT))
    {
        valid = false;
    }

    (void)fclose(file);
    return valid;
}

int main(int argc, char **argv)
{
    char simple_a[1024];
    char cpg_a[1024];
    char internal_a[1024];
    char simple_b[1024];
    char cpg_b[1024];
    char internal_b[1024];

    if (argc != 3)
    {
        (void)fprintf(stderr, "usage: %s <output-a> <output-b>\n", argv[0]);
        return 2;
    }

    if (!make_path(simple_a, sizeof(simple_a), argv[1], "gait_trace_simple.csv") ||
        !make_path(cpg_a, sizeof(cpg_a), argv[1], "gait_trace_cpg.csv") ||
        !make_path(internal_a, sizeof(internal_a), argv[1], "gait_trace_cpg_internal.csv") ||
        !make_path(simple_b, sizeof(simple_b), argv[2], "gait_trace_simple.csv") ||
        !make_path(cpg_b, sizeof(cpg_b), argv[2], "gait_trace_cpg.csv") ||
        !make_path(internal_b, sizeof(internal_b), argv[2], "gait_trace_cpg_internal.csv"))
    {
        (void)fprintf(stderr, "trace path is too long\n");
        return 1;
    }

    if ((gait_trace_compare_generate(argv[1]) != 0) ||
        (gait_trace_compare_generate(argv[2]) != 0) ||
        !files_equal(simple_a, simple_b) ||
        !files_equal(cpg_a, cpg_b) ||
        !files_equal(internal_a, internal_b) ||
        !validate_canonical_trace(simple_a, "simple") ||
        !validate_canonical_trace(cpg_a, "cpg") ||
        !validate_internal_trace(internal_a))
    {
        (void)fprintf(stderr, "deterministic gait trace regression failed\n");
        return 1;
    }

    (void)puts("deterministic gait trace regression passed");
    return 0;
}
