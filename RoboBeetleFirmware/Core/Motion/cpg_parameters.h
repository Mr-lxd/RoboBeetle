#ifndef ROBOBEETLE_CPG_PARAMETERS_H
#define ROBOBEETLE_CPG_PARAMETERS_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define CPG_PARAMETERS_WIRE_SIZE 58U
#define CPG_PARAMETERS_SNAPSHOT_SIZE 63U
typedef struct {
    double front_amp, rear_amp, period, beta, F, L, w;
    uint8_t mask;
} cpg_parameters_t;
typedef enum { CPG_PARAMETERS_OK, CPG_PARAMETERS_INVALID_PAYLOAD,
               CPG_PARAMETERS_OUT_OF_RANGE } cpg_parameters_result_t;
void cpg_parameters_default(cpg_parameters_t *p);
cpg_parameters_result_t cpg_parameters_validate(const cpg_parameters_t *p);
bool cpg_parameters_equal(const cpg_parameters_t *a, const cpg_parameters_t *b);
size_t cpg_parameters_encode(const cpg_parameters_t *p, uint8_t *wire, size_t capacity);
cpg_parameters_result_t cpg_parameters_decode(const uint8_t *wire, size_t length, cpg_parameters_t *p);
#endif
