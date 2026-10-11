#include "motion_manager.h"
#include "motion_timing_diagnostics.h"

#include "motion_config.h"

#include "servo_descriptor.h"

#include <stddef.h>

static const joint_targets_t zero_targets = {0};

static bool motion_manager_generator_is_usable(
    const gait_generator_t *generator)
{
    return (generator != NULL) &&
           (generator->ops != NULL) &&
           (generator->ops->advance != NULL) &&
           (generator->ops->reset != NULL) &&
           (generator->ops->sample != NULL) &&
           (generator->ops->is_mode_valid != NULL);
}

static bool motion_manager_is_running_state(
    const motion_manager_t *manager)
{
    return (manager->state == MOTION_STATE_RUNNING) ||
           (manager->state == MOTION_STATE_STOPPING);
}

static uint16_t motion_manager_required_mask(
    motion_mode_t mode)
{
    uint16_t mask = 0U;

    for (uint8_t id = 0U; id < SERVO_DESCRIPTOR_COUNT; ++id)
    {
        const servo_descriptor_t *descriptor =
            servo_descriptor_for_id(id);

        if ((descriptor == NULL) ||
            ((mode != MOTION_ASCEND && mode != MOTION_DESCEND) &&
             (id == SERVO_ID_FRONT_AXIS)))
        {
            continue;
        }

        mask |= descriptor->mask;
    }

    return mask;
}

static bool motion_manager_read_logical_pose(
    const motion_manager_t *manager,
    uint16_t mask,
    joint_targets_t *targets)
{
    static const uint8_t ids[SERVO_DESCRIPTOR_COUNT] = {
        SERVO_ID_FRONT_RIGHT,
        SERVO_ID_FRONT_LEFT,
        SERVO_ID_FRONT_AXIS,
        SERVO_ID_REAR_RIGHT,
        SERVO_ID_REAR_LEFT,
    };
    int16_t angle_cdeg;

    if ((manager == NULL) || (manager->servo_service == NULL) ||
        (targets == NULL) ||
        !servo_service_logical_pose_is_known(
            manager->servo_service,
            mask))
    {
        return false;
    }

    *targets = zero_targets;
    for (size_t index = 0U; index < SERVO_DESCRIPTOR_COUNT; ++index)
    {
        const uint16_t servo_mask = (uint16_t)(1U << ids[index]);

        if ((mask & servo_mask) == 0U)
        {
            continue;
        }

        if (!servo_service_logical_angle_cdeg(
                manager->servo_service,
                ids[index],
                &angle_cdeg))
        {
            return false;
        }

        switch (ids[index])
        {
            case SERVO_ID_FRONT_RIGHT:
                targets->front_right_cdeg = angle_cdeg;
                break;
            case SERVO_ID_FRONT_LEFT:
                targets->front_left_cdeg = angle_cdeg;
                break;
            case SERVO_ID_FRONT_AXIS:
                targets->front_axis_cdeg = angle_cdeg;
                break;
            case SERVO_ID_REAR_RIGHT:
                targets->rear_right_cdeg = angle_cdeg;
                break;
            case SERVO_ID_REAR_LEFT:
                targets->rear_left_cdeg = angle_cdeg;
                break;
            default:
                return false;
        }
    }

    return true;
}

static bool motion_manager_pose_within_operational_envelope(
    const joint_targets_t *targets,
    uint16_t mask)
{
    if (targets == NULL)
    {
        return false;
    }

    if (((mask & (uint16_t)(1U << SERVO_ID_FRONT_RIGHT)) != 0U) &&
        ((targets->front_right_cdeg < MOTION_FRONT_MIN_CDEG) ||
         (targets->front_right_cdeg > MOTION_FRONT_MAX_CDEG)))
    {
        return false;
    }

    if (((mask & (uint16_t)(1U << SERVO_ID_FRONT_LEFT)) != 0U) &&
        ((targets->front_left_cdeg < MOTION_FRONT_MIN_CDEG) ||
         (targets->front_left_cdeg > MOTION_FRONT_MAX_CDEG)))
    {
        return false;
    }

    if (((mask & (uint16_t)(1U << SERVO_ID_REAR_RIGHT)) != 0U) &&
        ((targets->rear_right_cdeg < MOTION_REAR_MIN_CDEG) ||
         (targets->rear_right_cdeg > MOTION_REAR_MAX_CDEG)))
    {
        return false;
    }

    if (((mask & (uint16_t)(1U << SERVO_ID_REAR_LEFT)) != 0U) &&
        ((targets->rear_left_cdeg < MOTION_REAR_MIN_CDEG) ||
         (targets->rear_left_cdeg > MOTION_REAR_MAX_CDEG)))
    {
        return false;
    }

    return true;
}

static motion_manager_result_t motion_manager_map_servo_result(
    servo_service_result_t result)
{
    switch (result)
    {
        case SERVO_SERVICE_RESULT_OK:
            return MOTION_MANAGER_RESULT_OK;
        case SERVO_SERVICE_RESULT_SERVO_NOT_ENABLED:
            return MOTION_MANAGER_RESULT_SERVO_NOT_ENABLED;
        case SERVO_SERVICE_RESULT_BUSY:
            return MOTION_MANAGER_RESULT_BUSY;
        case SERVO_SERVICE_RESULT_HARDWARE_FAILURE:
        case SERVO_SERVICE_RESULT_INVALID_PAYLOAD:
        case SERVO_SERVICE_RESULT_UNSUPPORTED_SERVO:
        case SERVO_SERVICE_RESULT_OUT_OF_RANGE:
        default:
            return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
    }
}

static int32_t interpolate_cdeg(
    int32_t start,
    int32_t end,
    uint32_t elapsed_ms)
{
    int64_t delta;

    if (elapsed_ms >= MOTION_TRANSITION_DURATION_MS)
    {
        return end;
    }

    delta = (int64_t)end - (int64_t)start;
    return start + (int32_t)(
        (delta * (int64_t)elapsed_ms) /
        (int64_t)MOTION_TRANSITION_DURATION_MS);
}

static joint_targets_t interpolate_targets(
    const joint_targets_t *start,
    const joint_targets_t *end,
    uint32_t elapsed_ms)
{
    joint_targets_t result;

    result.front_right_cdeg = interpolate_cdeg(
        start->front_right_cdeg,
        end->front_right_cdeg,
        elapsed_ms);
    result.front_left_cdeg = interpolate_cdeg(
        start->front_left_cdeg,
        end->front_left_cdeg,
        elapsed_ms);
    result.front_axis_cdeg = interpolate_cdeg(
        start->front_axis_cdeg,
        end->front_axis_cdeg,
        elapsed_ms);
    result.rear_right_cdeg = interpolate_cdeg(
        start->rear_right_cdeg,
        end->rear_right_cdeg,
        elapsed_ms);
    result.rear_left_cdeg = interpolate_cdeg(
        start->rear_left_cdeg,
        end->rear_left_cdeg,
        elapsed_ms);
    return result;
}

static joint_targets_t blend_targets(
    const joint_targets_t *from,
    const joint_targets_t *to,
    uint32_t elapsed_ms)
{
    return interpolate_targets(from, to, elapsed_ms);
}

static uint32_t saturating_transition_elapsed(
    uint32_t elapsed_ms,
    uint32_t delta_ms)
{
    if (elapsed_ms >= MOTION_TRANSITION_DURATION_MS ||
        delta_ms >= (MOTION_TRANSITION_DURATION_MS - elapsed_ms))
    {
        return MOTION_TRANSITION_DURATION_MS;
    }

    return elapsed_ms + delta_ms;
}

static void motion_manager_sanitize_targets(
    joint_targets_t *targets,
    uint32_t *clamp_count)
{
    if ((targets == NULL) || (clamp_count == NULL))
    {
        return;
    }

    if (targets->front_right_cdeg < MOTION_FRONT_MIN_CDEG)
    {
        targets->front_right_cdeg = MOTION_FRONT_MIN_CDEG;
        ++(*clamp_count);
    }
    else if (targets->front_right_cdeg > MOTION_FRONT_MAX_CDEG)
    {
        targets->front_right_cdeg = MOTION_FRONT_MAX_CDEG;
        ++(*clamp_count);
    }

    if (targets->front_left_cdeg < MOTION_FRONT_MIN_CDEG)
    {
        targets->front_left_cdeg = MOTION_FRONT_MIN_CDEG;
        ++(*clamp_count);
    }
    else if (targets->front_left_cdeg > MOTION_FRONT_MAX_CDEG)
    {
        targets->front_left_cdeg = MOTION_FRONT_MAX_CDEG;
        ++(*clamp_count);
    }

    if (targets->rear_right_cdeg < MOTION_REAR_MIN_CDEG)
    {
        targets->rear_right_cdeg = MOTION_REAR_MIN_CDEG;
        ++(*clamp_count);
    }
    else if (targets->rear_right_cdeg > MOTION_REAR_MAX_CDEG)
    {
        targets->rear_right_cdeg = MOTION_REAR_MAX_CDEG;
        ++(*clamp_count);
    }

    if (targets->rear_left_cdeg < MOTION_REAR_MIN_CDEG)
    {
        targets->rear_left_cdeg = MOTION_REAR_MIN_CDEG;
        ++(*clamp_count);
    }
    else if (targets->rear_left_cdeg > MOTION_REAR_MAX_CDEG)
    {
        targets->rear_left_cdeg = MOTION_REAR_MAX_CDEG;
        ++(*clamp_count);
    }
}

static motion_manager_result_t motion_manager_apply_targets(
    motion_manager_t *manager,
    const joint_targets_t *targets)
{
    static const uint8_t ids[SERVO_DESCRIPTOR_COUNT] = {
        SERVO_ID_FRONT_RIGHT,
        SERVO_ID_FRONT_LEFT,
        SERVO_ID_FRONT_AXIS,
        SERVO_ID_REAR_RIGHT,
        SERVO_ID_REAR_LEFT,
    };
    const int32_t values[SERVO_DESCRIPTOR_COUNT] = {
        targets->front_right_cdeg,
        targets->front_left_cdeg,
        targets->front_axis_cdeg,
        targets->rear_right_cdeg,
        targets->rear_left_cdeg,
    };
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    const motion_timing_mark_t apply_start =
        motion_timing_diagnostics_mark();
#endif

    for (size_t index = 0U; index < SERVO_DESCRIPTOR_COUNT; ++index)
    {
        const uint16_t mask = (uint16_t)(1U << ids[index]);

        if ((manager->write_mask & mask) == 0U)
        {
            continue;
        }

        if (servo_service_set_angle_from_motion(
                manager->servo_service,
                ids[index],
                values[index]) != SERVO_SERVICE_RESULT_OK)
        {
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
            motion_timing_diagnostics_record_apply(apply_start);
#endif
            return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
        }
    }

#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    motion_timing_diagnostics_record_apply(apply_start);
#endif
    return MOTION_MANAGER_RESULT_OK;
}

static motion_manager_result_t motion_manager_sample(
    motion_manager_t *manager,
    motion_mode_t mode,
    float amplitude_scale,
    float bias_scale,
    joint_targets_t *targets)
{
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    const motion_timing_mark_t sample_start =
        motion_timing_diagnostics_mark();
#endif
    const bool sample_ok =
        (manager->generator.ops != NULL) &&
        (manager->generator.ops->sample != NULL) &&
        manager->generator.ops->sample(
            manager->generator.context,
            mode,
            amplitude_scale,
            bias_scale,
            targets);
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    motion_timing_diagnostics_record_sample(sample_start);
#endif

    if (!sample_ok)
    {
        return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
    }

    if (manager->front_rear_coordination ==
        MOTION_FRONT_REAR_OPPOSITE_DIRECTION)
    {
        targets->rear_right_cdeg = -targets->rear_right_cdeg;
        targets->rear_left_cdeg = -targets->rear_left_cdeg;
    }

    if (mode == MOTION_TURN_LEFT)
    {
        targets->front_left_cdeg = 0;
        targets->rear_left_cdeg = 0;
    }
    else if (mode == MOTION_TURN_RIGHT)
    {
        targets->front_right_cdeg = 0;
        targets->rear_right_cdeg = 0;
    }

    return MOTION_MANAGER_RESULT_OK;
}

static float proportional_slew(float current, float target, float step)
{
    if (target > current + step)
    {
        return current + step;
    }
    if (target < current - step)
    {
        return current - step;
    }
    return target;
}

static motion_manager_result_t proportional_sample(motion_manager_t *manager, uint32_t elapsed_ms,
                                                   joint_targets_t *targets)
{
    const uint32_t slew_elapsed_ms =
        elapsed_ms > MOTION_GAIT_TICK_MS ? MOTION_GAIT_TICK_MS : elapsed_ms;
    const float step =
        (float)manager->proportional_config.slew_per_second * (float)slew_elapsed_ms / 1000000.0F;
    manager->slewed_throttle =
        proportional_slew(manager->slewed_throttle, manager->proportional_throttle, step);
    manager->slewed_turn =
        proportional_slew(manager->slewed_turn, manager->proportional_turn, step);
    manager->slewed_pitch =
        proportional_slew(manager->slewed_pitch, manager->proportional_pitch, step);
    manager->effective_throttle = manager->slewed_throttle;
    manager->effective_turn = manager->slewed_turn;
    manager->effective_pitch = manager->slewed_pitch;
    motion_manager_result_t result =
        motion_manager_sample(manager, MOTION_FORWARD, 1.0F, 0.0F, targets);
    if (result != MOTION_MANAGER_RESULT_OK)
    {
        return result;
    }
    float turn = manager->effective_turn * manager->proportional_config.turn_gain / 1000.0F;
    if (turn > 1.0F)
    {
        turn = 1.0F;
    }
    if (turn < -1.0F)
    {
        turn = -1.0F;
    }
    const float scale =
        manager->effective_throttle * manager->proportional_config.max_scale / 1000.0F;
    const float left = scale * (1.0F + (turn < 0.0F ? turn : 0.0F));
    const float right = scale * (1.0F - (turn > 0.0F ? turn : 0.0F));
    targets->front_left_cdeg = (int32_t)(targets->front_left_cdeg * left);
    targets->rear_left_cdeg = (int32_t)(targets->rear_left_cdeg * left);
    targets->front_right_cdeg = (int32_t)(targets->front_right_cdeg * right);
    targets->rear_right_cdeg = (int32_t)(targets->rear_right_cdeg * right);
    targets->front_axis_cdeg =
        (int32_t)(-manager->proportional_config.pitch_limit_cdeg * manager->effective_pitch);
    return result;
}

static motion_manager_result_t motion_manager_tick(
    motion_manager_t *manager,
    uint32_t elapsed_ms)
{
    joint_targets_t targets;
    motion_manager_result_t result;

    if (manager->state == MOTION_STATE_STOPPING)
    {
        manager->stop_elapsed_ms = saturating_transition_elapsed(
            manager->stop_elapsed_ms,
            elapsed_ms);

        targets = interpolate_targets(
            &manager->stop_start_targets,
            &zero_targets,
            manager->stop_elapsed_ms);
        motion_manager_sanitize_targets(
            &targets,
            &manager->operational_clamp_count);
        result = motion_manager_apply_targets(manager, &targets);
        if (result != MOTION_MANAGER_RESULT_OK)
        {
            motion_manager_stop_immediate(manager);
            return result;
        }
        manager->last_targets = targets;
        if (manager->control_mode == MOTION_CONTROL_PROPORTIONAL)
        {
            const float envelope =
                1.0F - (float)manager->stop_elapsed_ms / MOTION_TRANSITION_DURATION_MS;
            manager->effective_throttle = manager->stop_throttle * envelope;
            manager->effective_turn = manager->stop_elapsed_ms < MOTION_TRANSITION_DURATION_MS
                                          ? manager->stop_turn
                                          : 0.0F;
            manager->effective_pitch = manager->stop_pitch * envelope;
        }

        if (manager->stop_elapsed_ms >= MOTION_TRANSITION_DURATION_MS)
        {
            servo_service_motion_end(manager->servo_service);
            manager->state = MOTION_STATE_STOPPED;
            manager->control_mode = MOTION_CONTROL_DISCRETE;
            manager->active_mode = MOTION_STOP;
            manager->transition_mode = MOTION_STOP;
            manager->transition_from_mode = MOTION_STOP;
            manager->transition = MOTION_MANAGER_TRANSITION_NONE;
            manager->write_mask = 0U;
        }
        return MOTION_MANAGER_RESULT_OK;
    }

    if ((manager->generator.ops == NULL) ||
        (manager->generator.ops->advance == NULL))
    {
        motion_manager_stop_immediate(manager);
        return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
    }

#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    const motion_timing_mark_t advance_start =
        motion_timing_diagnostics_mark();
#endif
    manager->generator.ops->advance(
        manager->generator.context,
        elapsed_ms);
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    motion_timing_diagnostics_record_generator_advance(advance_start);
#endif

    if (manager->control_mode == MOTION_CONTROL_PROPORTIONAL)
    {
        result = proportional_sample(manager, elapsed_ms, &targets);
        if (manager->transition == MOTION_MANAGER_TRANSITION_START)
        {
            manager->transition_elapsed_ms =
                saturating_transition_elapsed(manager->transition_elapsed_ms, elapsed_ms);
            targets = interpolate_targets(&manager->start_from_targets, &targets,
                                          manager->transition_elapsed_ms);
            const float envelope =
                (float)manager->transition_elapsed_ms / MOTION_TRANSITION_DURATION_MS;
            manager->effective_throttle *= envelope;
            manager->effective_pitch *= envelope;
        }
    } else if (manager->transition == MOTION_MANAGER_TRANSITION_START)
    {
        joint_targets_t gait_targets;

        manager->transition_elapsed_ms = saturating_transition_elapsed(
            manager->transition_elapsed_ms,
            elapsed_ms);
        result = motion_manager_sample(
            manager,
            manager->active_mode,
            1.0F,
            1.0F,
            &gait_targets);
        if (result == MOTION_MANAGER_RESULT_OK)
        {
            targets = interpolate_targets(
                &manager->start_from_targets,
                &gait_targets,
                manager->transition_elapsed_ms);
        }
    }
    else if (manager->transition == MOTION_MANAGER_TRANSITION_MODE)
    {
        joint_targets_t from_targets;
        joint_targets_t to_targets;

        manager->transition_elapsed_ms = saturating_transition_elapsed(
            manager->transition_elapsed_ms,
            elapsed_ms);
        result = motion_manager_sample(
            manager,
            manager->transition_from_mode,
            1.0F,
            1.0F,
            &from_targets);
        if (result != MOTION_MANAGER_RESULT_OK)
        {
            motion_manager_stop_immediate(manager);
            return result;
        }
        result = motion_manager_sample(
            manager,
            manager->transition_mode,
            1.0F,
            1.0F,
            &to_targets);
        if (result != MOTION_MANAGER_RESULT_OK)
        {
            motion_manager_stop_immediate(manager);
            return result;
        }
        targets = blend_targets(
            &from_targets,
            &to_targets,
            manager->transition_elapsed_ms);
    }
    else
    {
        result = motion_manager_sample(
            manager,
            manager->active_mode,
            1.0F,
            1.0F,
            &targets);
    }

    if (result != MOTION_MANAGER_RESULT_OK)
    {
        motion_manager_stop_immediate(manager);
        return result;
    }

    // Every generator, including future alternate implementations, passes
    // through the common operational envelope before reaching calibration and
    // PWM conversion.  This keeps physical limits independent of gait math.
    motion_manager_sanitize_targets(
        &targets,
        &manager->operational_clamp_count);

    result = motion_manager_apply_targets(manager, &targets);
    if (result != MOTION_MANAGER_RESULT_OK)
    {
        motion_manager_stop_immediate(manager);
        return result;
    }
    manager->last_targets = targets;

    if ((manager->transition == MOTION_MANAGER_TRANSITION_START) &&
        (manager->transition_elapsed_ms >= MOTION_TRANSITION_DURATION_MS))
    {
        manager->transition = MOTION_MANAGER_TRANSITION_NONE;
    }
    else if ((manager->transition == MOTION_MANAGER_TRANSITION_MODE) &&
             (manager->transition_elapsed_ms >=
              MOTION_TRANSITION_DURATION_MS))
    {
        manager->active_mode = manager->transition_mode;
        manager->write_mask = motion_manager_required_mask(
            manager->active_mode);
        servo_service_motion_set_mask(
            manager->servo_service,
            manager->write_mask);
        manager->transition = MOTION_MANAGER_TRANSITION_NONE;
    }

    return MOTION_MANAGER_RESULT_OK;
}

void motion_manager_init(
    motion_manager_t *manager,
    servo_service_t *servo_service,
    safety_supervisor_t *safety_supervisor,
    gait_generator_t generator)
{
    if (manager == NULL)
    {
        return;
    }

    *manager = (motion_manager_t){0};
    manager->servo_service = servo_service;
    manager->safety_supervisor = safety_supervisor;
    manager->generator = generator;
    manager->gait_backend = MOTION_GAIT_BACKEND_UNSPECIFIED;
    manager->backend_selector_available = false;
    manager->state = MOTION_STATE_STOPPED;
    manager->active_mode = MOTION_STOP;
    manager->transition_mode = MOTION_STOP;
    manager->transition_from_mode = MOTION_STOP;
}

void motion_manager_init_with_backends(
    motion_manager_t *manager,
    servo_service_t *servo_service,
    safety_supervisor_t *safety_supervisor,
    gait_generator_t simple_gait,
    gait_generator_t cpg,
    gait_generator_t experimental_flex,
    motion_gait_backend_t initial_backend)
{
    if (manager == NULL)
    {
        return;
    }

    *manager = (motion_manager_t){0};
    manager->servo_service = servo_service;
    manager->safety_supervisor = safety_supervisor;
    manager->registered_generators[MOTION_GAIT_BACKEND_SIMPLE_GAIT] = simple_gait;
    manager->registered_generators[MOTION_GAIT_BACKEND_CPG] = cpg;
    manager->cpg_generator = (cpg_gait_generator_t *)cpg.context;
    manager->registered_generators[MOTION_GAIT_BACKEND_EXPERIMENTAL_FLEX] =
        experimental_flex;
    manager->gait_backend = MOTION_GAIT_BACKEND_UNSPECIFIED;
    manager->backend_selector_available = false;
    manager->state = MOTION_STATE_STOPPED;
    manager->active_mode = MOTION_STOP;
    manager->transition_mode = MOTION_STOP;
    manager->transition_from_mode = MOTION_STOP;

    if (motion_gait_backend_is_valid(initial_backend) &&
        motion_manager_generator_is_usable(
            &manager->registered_generators[initial_backend]))
    {
        manager->generator = manager->registered_generators[initial_backend];
        manager->gait_backend = initial_backend;
        manager->backend_selector_available = true;
    }
}

motion_manager_result_t motion_manager_set_gait_backend(
    motion_manager_t *manager,
    motion_gait_backend_t backend)
{
    gait_generator_t *selected;

    if (manager == NULL)
    {
        return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
    }

    if (manager->state != MOTION_STATE_STOPPED)
    {
        return MOTION_MANAGER_RESULT_BUSY;
    }

    if (!motion_gait_backend_is_valid(backend))
    {
        return MOTION_MANAGER_RESULT_INVALID_BACKEND;
    }

    if (!manager->backend_selector_available)
    {
        return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
    }

    selected = &manager->registered_generators[backend];
    if (!motion_manager_generator_is_usable(selected))
    {
        return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
    }

    if (manager->gait_backend == backend)
    {
        return MOTION_MANAGER_RESULT_OK;
    }

    selected->ops->reset(selected->context);
    manager->generator = *selected;
    manager->gait_backend = backend;
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    motion_timing_diagnostics_set_runtime_backend((uint32_t)backend);
#endif
    return MOTION_MANAGER_RESULT_OK;
}

motion_gait_backend_t motion_manager_gait_backend(
    const motion_manager_t *manager)
{
    return (manager == NULL) ?
        MOTION_GAIT_BACKEND_UNSPECIFIED :
        manager->gait_backend;
}

motion_manager_result_t motion_manager_set_front_rear_coordination(
    motion_manager_t *manager,
    motion_front_rear_coordination_t coordination)
{
    if (manager == NULL)
    {
        return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
    }

    if (manager->state != MOTION_STATE_STOPPED)
    {
        return MOTION_MANAGER_RESULT_BUSY;
    }

    if (!motion_front_rear_coordination_is_valid(coordination))
    {
        return MOTION_MANAGER_RESULT_INVALID_COORDINATION;
    }

    manager->front_rear_coordination = coordination;
    return MOTION_MANAGER_RESULT_OK;
}

motion_front_rear_coordination_t motion_manager_front_rear_coordination(
    const motion_manager_t *manager)
{
    return (manager == NULL)
        ? MOTION_FRONT_REAR_SAME_DIRECTION
        : manager->front_rear_coordination;
}

motion_manager_result_t motion_manager_start(
    motion_manager_t *manager,
    motion_mode_t mode)
{
    joint_targets_t start_targets;
    uint16_t required_mask;
    motion_manager_result_t result;
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    const bool start_from_stopped =
        (manager != NULL) && (manager->state == MOTION_STATE_STOPPED);
#endif

    if ((manager == NULL) || !motion_mode_is_valid(mode) ||
        (mode == MOTION_STOP))
    {
        return MOTION_MANAGER_RESULT_INVALID_MODE;
    }

    if ((manager->generator.ops != NULL) &&
        (manager->generator.ops->is_mode_valid != NULL) &&
        !manager->generator.ops->is_mode_valid(
            manager->generator.context,
            mode))
    {
        return MOTION_MANAGER_RESULT_INVALID_MODE;
    }

    if (manager->control_mode == MOTION_CONTROL_PROPORTIONAL)
    {
        return MOTION_MANAGER_RESULT_BUSY;
    }
    required_mask = motion_manager_required_mask(mode);

    if (manager->safety_supervisor != NULL &&
        !safety_supervisor_is_host_alive(manager->safety_supervisor))
    {
        return MOTION_MANAGER_RESULT_HOST_NOT_ALIVE;
    }

    if (manager->state == MOTION_STATE_STOPPING)
    {
        return MOTION_MANAGER_RESULT_BUSY;
    }

    if (manager->state == MOTION_STATE_RUNNING)
    {
        if (manager->transition != MOTION_MANAGER_TRANSITION_NONE)
        {
            return manager->transition_mode == mode
                ? MOTION_MANAGER_RESULT_OK
                : MOTION_MANAGER_RESULT_BUSY;
        }

        if (manager->active_mode == mode)
        {
            return MOTION_MANAGER_RESULT_OK;
        }

        if ((servo_service_enabled_mask(manager->servo_service) &
             required_mask) != required_mask)
        {
            return MOTION_MANAGER_RESULT_SERVO_NOT_ENABLED;
        }

        if (!servo_service_logical_pose_is_known(
                manager->servo_service,
                required_mask))
        {
            return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
        }

        manager->transition_from_mode = manager->active_mode;
        manager->transition_mode = mode;
        manager->transition_elapsed_ms = 0U;
        manager->write_mask = (uint16_t)(
            manager->write_mask | required_mask);
        servo_service_motion_set_mask(
            manager->servo_service,
            manager->write_mask);
        manager->transition = MOTION_MANAGER_TRANSITION_MODE;
        return MOTION_MANAGER_RESULT_OK;
    }

    if ((servo_service_enabled_mask(manager->servo_service) &
         required_mask) != required_mask)
    {
        return MOTION_MANAGER_RESULT_SERVO_NOT_ENABLED;
    }

    if (!motion_manager_read_logical_pose(
            manager,
            required_mask,
            &start_targets))
    {
        return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
    }

    if (!motion_manager_pose_within_operational_envelope(
            &start_targets,
            required_mask))
    {
        return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
    }

    result = motion_manager_map_servo_result(
        servo_service_motion_begin(
            manager->servo_service,
            required_mask));
    if (result != MOTION_MANAGER_RESULT_OK)
    {
        return result;
    }

    manager->state = MOTION_STATE_RUNNING;
    manager->active_mode = mode;
    manager->transition_mode = mode;
    manager->transition_from_mode = mode;
    manager->transition = MOTION_MANAGER_TRANSITION_START;
    manager->write_mask = required_mask;
    manager->transition_elapsed_ms = 0U;
    manager->stop_elapsed_ms = 0U;
    manager->last_targets = start_targets;
    manager->start_from_targets = start_targets;
    manager->stop_start_targets = start_targets;
    manager->scheduler_started = 0U;
    manager->phase_tick_valid = false;
    manager->stop_reason = MOTION_STOP_REASON_NONE;
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    if (start_from_stopped)
    {
        motion_timing_diagnostics_begin_run(
            (uint32_t)manager->gait_backend);
    }
#endif
    return MOTION_MANAGER_RESULT_OK;
}

motion_manager_result_t motion_manager_request_stop(
    motion_manager_t *manager)
{
    const uint32_t acceptance_ms =
        ((manager != NULL) && (manager->scheduler_started != 0U))
            ? manager->last_tick_ms
            : 0U;

    return motion_manager_request_stop_at(manager, acceptance_ms);
}

motion_manager_result_t motion_manager_request_stop_at(
    motion_manager_t *manager,
    uint32_t now_ms)
{
    if (manager == NULL)
    {
        return MOTION_MANAGER_RESULT_INVALID_MODE;
    }

    manager->proportional_session_valid = false;
    if ((manager->state == MOTION_STATE_STOPPED) ||
        (manager->state == MOTION_STATE_FAULTED))
    {
        return MOTION_MANAGER_RESULT_OK;
    }

    if (manager->state == MOTION_STATE_STOPPING)
    {
        return MOTION_MANAGER_RESULT_OK;
    }

    manager->stop_throttle = manager->effective_throttle;
    manager->stop_turn = manager->effective_turn;
    manager->stop_pitch = manager->effective_pitch;
    manager->stop_reason = MOTION_STOP_REASON_OPERATOR;
    manager->stop_start_targets = manager->last_targets;
    manager->stop_elapsed_ms = 0U;
    // The graceful-stop clock starts when the request is accepted, not at
    // the previous foreground tick.  The next process() call still uses the
    // normal minimum cadence, but its elapsed delta is measured from here.
    manager->last_tick_ms = now_ms;
    manager->scheduler_started = 1U;
    manager->state = MOTION_STATE_STOPPING;
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    motion_timing_diagnostics_freeze(
        MOTION_TIMING_TERMINATION_NORMAL_STOP);
#endif
    return MOTION_MANAGER_RESULT_OK;
}

motion_manager_result_t motion_manager_process(
    motion_manager_t *manager,
    uint32_t now_ms)
{
    if (manager == NULL)
    {
        return MOTION_MANAGER_RESULT_INVALID_MODE;
    }

    if (!motion_manager_is_running_state(manager))
    {
        return MOTION_MANAGER_RESULT_OK;
    }

    if ((manager->safety_supervisor != NULL) &&
        !safety_supervisor_is_host_alive(manager->safety_supervisor))
    {
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
        motion_timing_diagnostics_freeze(
            MOTION_TIMING_TERMINATION_SAFETY_STOP);
#endif
        motion_manager_stop_immediate(manager);
        manager->stop_reason = MOTION_STOP_REASON_LINK_LOST;
        return MOTION_MANAGER_RESULT_HOST_NOT_ALIVE;
    }

    if (manager->proportional_session_valid &&
        (uint32_t)(now_ms - manager->proportional_last_input_ms) > 600U)
    {
        (void)motion_manager_request_stop_at(manager, now_ms);
        manager->stop_reason = MOTION_STOP_REASON_INPUT_TIMEOUT;
    }

    if (manager->scheduler_started == 0U)
    {
        manager->last_tick_ms = now_ms;
        manager->scheduler_started = 1U;
        return MOTION_MANAGER_RESULT_OK;
    }

    const uint32_t elapsed_ms =
        (uint32_t)(now_ms - manager->last_tick_ms);

    if (elapsed_ms < MOTION_GAIT_TICK_MS)
    {
        return MOTION_MANAGER_RESULT_OK;
    }

    manager->last_tick_ms = now_ms;
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    motion_timing_diagnostics_motion_tick_begin(elapsed_ms);
#endif
    const motion_manager_result_t result =
        motion_manager_tick(manager, elapsed_ms);
    if ((result == MOTION_MANAGER_RESULT_OK) &&
        (manager->state == MOTION_STATE_RUNNING))
    {
        manager->phase_tick_valid = true;
    }
#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    motion_timing_diagnostics_motion_tick_end((uint32_t)result);
#endif
    return result;
}

void motion_manager_stop_immediate(
    motion_manager_t *manager)
{
    const bool was_active =
        (manager != NULL) &&
        (motion_manager_is_running_state(manager) ||
         ((manager->servo_service != NULL) &&
          servo_service_motion_is_active(manager->servo_service)));
    const bool was_faulted =
        (manager != NULL) && (manager->state == MOTION_STATE_FAULTED);

    if (manager == NULL)
    {
        return;
    }

#if MOTION_TIMING_DIAGNOSTICS_ACTIVE
    if (was_active)
    {
        motion_timing_diagnostics_freeze(
            MOTION_TIMING_TERMINATION_FAULT);
    }
#endif

    if (manager->servo_service != NULL)
    {
        servo_service_motion_abort(manager->servo_service);
    }
    manager->proportional_session_valid = false;
    manager->control_mode = MOTION_CONTROL_DISCRETE;
    manager->effective_throttle = 0.0F;
    manager->effective_turn = 0.0F;
    manager->effective_pitch = 0.0F;
    manager->last_targets = zero_targets;
    manager->start_from_targets = zero_targets;
    manager->stop_start_targets = zero_targets;
    manager->transition_elapsed_ms = 0U;
    manager->stop_elapsed_ms = 0U;
    manager->scheduler_started = 0U;
    manager->phase_tick_valid = false;
    manager->write_mask = 0U;
    manager->active_mode = MOTION_STOP;
    manager->transition_mode = MOTION_STOP;
    manager->transition_from_mode = MOTION_STOP;
    manager->transition = MOTION_MANAGER_TRANSITION_NONE;
    manager->state = (was_active || was_faulted) ? MOTION_STATE_FAULTED :
                                                    MOTION_STATE_STOPPED;
}

motion_state_t motion_manager_state(
    const motion_manager_t *manager)
{
    return manager == NULL ? MOTION_STATE_FAULTED : manager->state;
}

motion_mode_t motion_manager_mode(
    const motion_manager_t *manager)
{
    return manager == NULL ? MOTION_STOP : manager->active_mode;
}

const joint_targets_t *motion_manager_last_targets(
    const motion_manager_t *manager)
{
    return manager == NULL ? &zero_targets : &manager->last_targets;
}

uint32_t motion_manager_stop_elapsed_ms(
    const motion_manager_t *manager)
{
    return manager == NULL ? 0U : manager->stop_elapsed_ms;
}

uint32_t motion_manager_operational_clamp_count(
    const motion_manager_t *manager)
{
    return manager == NULL ? 0U : manager->operational_clamp_count;
}

bool motion_manager_is_active(
    const motion_manager_t *manager)
{
    return (manager != NULL) && motion_manager_is_running_state(manager);
}

motion_manager_result_t motion_manager_set_cpg_parameters(motion_manager_t *m,
                                                          const cpg_parameters_t *p)
{
    if (!m || !m->cpg_generator)
        return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
    if (m->safety_supervisor && !safety_supervisor_is_host_alive(m->safety_supervisor))
        return MOTION_MANAGER_RESULT_HOST_NOT_ALIVE;
    if (m->state == MOTION_STATE_FAULTED)
        return MOTION_MANAGER_RESULT_INVALID_STATE;
    if (m->state != MOTION_STATE_STOPPED)
        return MOTION_MANAGER_RESULT_BUSY;
    cpg_parameters_result_t result = cpg_parameters_validate(p);
    if (result == CPG_PARAMETERS_INVALID_PAYLOAD)
        return MOTION_MANAGER_RESULT_INVALID_PAYLOAD;
    if (result != CPG_PARAMETERS_OK)
        return MOTION_MANAGER_RESULT_OUT_OF_RANGE;
    cpg_parameters_t current;
    cpg_gait_generator_get_parameters(m->cpg_generator, &current);
    if (!cpg_parameters_equal(&current, p))
    {
        /* Validation is complete. Main-loop-only update cannot interleave with sampling. */
        cpg_gait_generator_apply_parameters(m->cpg_generator, p);
        ++m->cpg_parameter_version;
    }
    return MOTION_MANAGER_RESULT_OK;
}

motion_manager_result_t motion_manager_start_proportional(motion_manager_t *manager,
                                                          const proportional_config_t *config,
                                                          uint8_t session_id, uint32_t now_ms)
{
    joint_targets_t pose;
    if (manager == NULL || config == NULL)
    {
        return MOTION_MANAGER_RESULT_INVALID_PAYLOAD;
    }
    if (config->max_scale < 100U || config->max_scale > 1000U || config->turn_gain > 2000U ||
        config->pitch_limit_cdeg > 2000U || config->slew_per_second < 100U ||
        config->slew_per_second > 10000U)
    {
        return MOTION_MANAGER_RESULT_OUT_OF_RANGE;
    }
    if (manager->state != MOTION_STATE_STOPPED)
    {
        return MOTION_MANAGER_RESULT_BUSY;
    }
    if (manager->safety_supervisor != NULL &&
        !safety_supervisor_is_host_alive(manager->safety_supervisor))
    {
        return MOTION_MANAGER_RESULT_HOST_NOT_ALIVE;
    }
    if ((servo_service_enabled_mask(manager->servo_service) & 0x1FU) != 0x1FU)
    {
        return MOTION_MANAGER_RESULT_SERVO_NOT_ENABLED;
    }
    if (!motion_manager_read_logical_pose(manager, 0x1FU, &pose) ||
        !motion_manager_pose_within_operational_envelope(&pose, 0x1FU))
    {
        return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
    }
    if (!motion_manager_generator_is_usable(&manager->generator))
    {
        return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
    }
    motion_manager_result_t result = motion_manager_start(manager, MOTION_FORWARD);
    if (result != MOTION_MANAGER_RESULT_OK)
    {
        return result;
    }
    manager->generator.ops->reset(manager->generator.context);
    manager->control_mode = MOTION_CONTROL_PROPORTIONAL;
    manager->proportional_config = *config;
    manager->proportional_session_valid = true;
    manager->proportional_sequence_valid = false;
    manager->proportional_session_id = session_id;
    manager->proportional_last_input_ms = now_ms;
    manager->proportional_throttle = manager->proportional_turn = manager->proportional_pitch =
        0.0F;
    manager->slewed_throttle = manager->slewed_turn = manager->slewed_pitch = 0.0F;
    manager->effective_throttle = manager->effective_turn = manager->effective_pitch = 0.0F;
    manager->start_from_targets = manager->last_targets = pose;
    manager->write_mask = 0x1FU;
    servo_service_motion_set_mask(manager->servo_service, manager->write_mask);
    return MOTION_MANAGER_RESULT_OK;
}

bool motion_manager_update_proportional(motion_manager_t *manager, uint8_t session_id,
                                        uint16_t sequence, uint16_t throttle, int16_t turn,
                                        int16_t pitch, uint32_t now_ms)
{
    if (manager == NULL || !manager->proportional_session_valid ||
        manager->state != MOTION_STATE_RUNNING || manager->proportional_session_id != session_id ||
        throttle > 1000U || turn < -1000 || turn > 1000 || pitch < -1000 || pitch > 1000)
    {
        return false;
    }
    if ((uint32_t)(now_ms - manager->proportional_last_input_ms) > 600U)
    {
        (void)motion_manager_request_stop_at(manager, now_ms);
        manager->stop_reason = MOTION_STOP_REASON_INPUT_TIMEOUT;
        return false;
    }
    const uint16_t delta = (uint16_t)(sequence - manager->proportional_sequence);
    if (manager->proportional_sequence_valid && (delta == 0U || delta >= 0x8000U))
    {
        return false;
    }
    manager->proportional_sequence_valid = true;
    manager->proportional_sequence = sequence;
    manager->proportional_last_input_ms = now_ms;
    manager->proportional_throttle = throttle / 1000.0F;
    manager->proportional_turn = turn / 1000.0F;
    manager->proportional_pitch = pitch / 1000.0F;
    return true;
}
