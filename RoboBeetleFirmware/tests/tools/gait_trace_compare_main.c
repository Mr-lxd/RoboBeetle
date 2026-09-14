#include "gait_trace_compare.h"

#include <stdio.h>

int main(int argc, char **argv)
{
    const char *output_dir;

    if (argc > 2)
    {
        (void)fprintf(stderr, "usage: %s [output-directory]\n", argv[0]);
        return 2;
    }

    output_dir = argc == 2 ? argv[1] : ".";
    if (gait_trace_compare_generate(output_dir) != 0)
    {
        (void)fprintf(stderr, "failed to generate gait trace in %s\n", output_dir);
        return 1;
    }

    (void)printf("output_directory=%s\n", output_dir);
    (void)puts("gait_trace_simple.csv");
    (void)puts("gait_trace_cpg.csv");
    (void)puts("gait_trace_cpg_internal.csv");
    return 0;
}
