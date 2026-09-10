#ifndef ROBOBEETLE_JY901S_TELEMETRY_H
#define ROBOBEETLE_JY901S_TELEMETRY_H

#include "jy901s_parser.h"

#include <stddef.h>
#include <stdint.h>

#define JY901S_IMU_TELEMETRY_SCHEMA_VERSION 1U
#define JY901S_IMU_TELEMETRY_PAYLOAD_LENGTH 56U

#define JY901S_IMU_VALID_ACC   0x01U
#define JY901S_IMU_VALID_GYRO  0x02U
#define JY901S_IMU_VALID_ANGLE 0x04U

typedef struct
{
    uint32_t rx_byte_count;
    uint32_t rx_buffer_overflow_count;
    uint32_t rx_rearm_failure_count;
    uint32_t uart_error_count;
} jy901s_imu_telemetry_diagnostics_t;

size_t jy901s_imu_telemetry_encode(
    const jy901s_imu_state_t *state,
    const jy901s_parser_stats_t *parser_stats,
    const jy901s_imu_telemetry_diagnostics_t *transport_diagnostics,
    uint8_t *payload,
    size_t payload_capacity);

#endif /* ROBOBEETLE_JY901S_TELEMETRY_H */
