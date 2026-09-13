#ifndef ROBOBEETLE_MOTION_MANAGER_H
#define ROBOBEETLE_MOTION_MANAGER_H

#include "gait_generator.h"
#include "joint_targets.h"
#include "motion_types.h"

#include "safety_supervisor.h"
#include "servo_service.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTION_MANAGER_RESULT_OK = 0,
    MOTION_MANAGER_RESULT_INVALID_MODE,
    MOTION_MANAGER_RESULT_HOST_NOT_ALIVE,
    MOTION_MANAGER_RESULT_SERVO_NOT_ENABLED,
    MOTION_MANAGER_RESULT_BUSY,
    MOTION_MANAGER_RESULT_HARDWARE_FAILURE
} motion_manager_result_t;

typedef enum
{
    MOTION_MANAGER_TRANSITION_NONE = 0,
    MOTION_MANAGER_TRANSITION_START,
    MOTION_MANAGER_TRANSITION_MODE
} motion_manager_transition_t;

typedef struct
{
    servo_service_t *servo_service;
    safety_supervisor_t *safety_supervisor;
    gait_generator_t generator;
    motion_state_t state;
    motion_mode_t active_mode;
    motion_mode_t transition_mode;
    motion_mode_t transition_from_mode;
    motion_manager_transition_t transition;
    uint16_t write_mask;
    uint32_t transition_elapsed_ms;
    uint32_t stop_elapsed_ms;
    uint32_t last_tick_ms;
    uint8_t scheduler_started;
    joint_targets_t last_targets;
    joint_targets_t stop_start_targets;
    uint32_t operational_clamp_count;
} motion_manager_t;

void motion_manager_init(
    motion_manager_t *manager,
    servo_service_t *servo_service,
    safety_supervisor_t *safety_supervisor,
    gait_generator_t generator);

motion_manager_result_t motion_manager_start(
    motion_manager_t *manager,
    motion_mode_t mode);

motion_manager_result_t motion_manager_request_stop(
    motion_manager_t *manager);

motion_manager_result_t motion_manager_process(
    motion_manager_t *manager,
    uint32_t now_ms);

void motion_manager_stop_immediate(
    motion_manager_t *manager);

motion_state_t motion_manager_state(
    const motion_manager_t *manager);

motion_mode_t motion_manager_mode(
    const motion_manager_t *manager);

const joint_targets_t *motion_manager_last_targets(
    const motion_manager_t *manager);

uint32_t motion_manager_stop_elapsed_ms(
    const motion_manager_t *manager);

uint32_t motion_manager_operational_clamp_count(
    const motion_manager_t *manager);

bool motion_manager_is_active(
    const motion_manager_t *manager);

#endif /* ROBOBEETLE_MOTION_MANAGER_H */
