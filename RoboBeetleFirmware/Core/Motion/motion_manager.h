#ifndef ROBOBEETLE_MOTION_MANAGER_H
#define ROBOBEETLE_MOTION_MANAGER_H

#include "gait_generator.h"
#include "cpg_gait_generator.h"
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
    MOTION_MANAGER_RESULT_HARDWARE_FAILURE,
    MOTION_MANAGER_RESULT_INVALID_BACKEND,
    MOTION_MANAGER_RESULT_INVALID_STATE,
    MOTION_MANAGER_RESULT_OUT_OF_RANGE,
    MOTION_MANAGER_RESULT_INVALID_PAYLOAD,
    MOTION_MANAGER_RESULT_INVALID_COORDINATION
} motion_manager_result_t;

typedef enum
{
    MOTION_MANAGER_TRANSITION_NONE = 0,
    MOTION_MANAGER_TRANSITION_START,
    MOTION_MANAGER_TRANSITION_MODE
} motion_manager_transition_t;

typedef struct
{
    uint16_t max_scale;
    uint16_t turn_gain;
    uint16_t pitch_limit_cdeg;
    uint16_t slew_per_second;
} proportional_config_t;

typedef struct
{
    motion_control_mode_t control_mode;
    proportional_config_t proportional_config;
    bool proportional_session_valid;
    bool proportional_sequence_valid;
    uint8_t proportional_session_id;
    uint16_t proportional_sequence;
    uint32_t proportional_last_input_ms;
    float proportional_throttle, proportional_turn, proportional_pitch;
    float slewed_throttle, slewed_turn, slewed_pitch;
    float effective_throttle, effective_turn, effective_pitch;
    float stop_throttle, stop_turn, stop_pitch;
    cpg_gait_generator_t *cpg_generator;
    uint16_t cpg_parameter_version;
    uint8_t stop_reason;
    servo_service_t *servo_service;
    safety_supervisor_t *safety_supervisor;
    gait_generator_t generator;
    gait_generator_t registered_generators[MOTION_GAIT_BACKEND_COUNT];
    motion_gait_backend_t gait_backend;
    motion_front_rear_coordination_t front_rear_coordination;
    bool backend_selector_available;
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
    bool phase_tick_valid;
    joint_targets_t last_targets;
    joint_targets_t start_from_targets;
    joint_targets_t stop_start_targets;
    uint32_t operational_clamp_count;
} motion_manager_t;

motion_manager_result_t motion_manager_start_proportional(motion_manager_t *manager,
                                                          const proportional_config_t *config,
                                                          uint8_t session_id, uint32_t now_ms);

bool motion_manager_update_proportional(motion_manager_t *manager, uint8_t session_id,
                                        uint16_t sequence, uint16_t throttle, int16_t turn,
                                        int16_t pitch, uint32_t now_ms);

void motion_manager_init(
    motion_manager_t *manager,
    servo_service_t *servo_service,
    safety_supervisor_t *safety_supervisor,
    gait_generator_t generator);

void motion_manager_init_with_backends(
    motion_manager_t *manager,
    servo_service_t *servo_service,
    safety_supervisor_t *safety_supervisor,
    gait_generator_t simple_gait,
    gait_generator_t cpg,
    gait_generator_t experimental_flex,
    motion_gait_backend_t initial_backend);

motion_manager_result_t motion_manager_set_cpg_parameters(motion_manager_t *manager,
                                                          const cpg_parameters_t *parameters);

motion_manager_result_t motion_manager_set_gait_backend(
    motion_manager_t *manager,
    motion_gait_backend_t backend);

motion_gait_backend_t motion_manager_gait_backend(
    const motion_manager_t *manager);

motion_manager_result_t motion_manager_set_front_rear_coordination(
    motion_manager_t *manager,
    motion_front_rear_coordination_t coordination);

motion_front_rear_coordination_t motion_manager_front_rear_coordination(
    const motion_manager_t *manager);

motion_manager_result_t motion_manager_start(
    motion_manager_t *manager,
    motion_mode_t mode);

motion_manager_result_t motion_manager_request_stop(
    motion_manager_t *manager);

/* Prefer this entry point when the caller has the STOP acceptance timestamp. */
motion_manager_result_t motion_manager_request_stop_at(
    motion_manager_t *manager,
    uint32_t now_ms);

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
