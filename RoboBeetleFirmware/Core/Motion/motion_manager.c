#include "motion_manager.h"

#include "motion_config.h"

#include "servo_descriptor.h"

#include <stddef.h>

static const joint_targets_t zero_targets = {0};

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
            return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
        }
    }

    return MOTION_MANAGER_RESULT_OK;
}

static uint32_t motion_manager_generator_diagnostic_count(
    const motion_manager_t *manager)
{
    if ((manager->generator.ops == NULL) ||
        (manager->generator.ops->diagnostic_count == NULL))
    {
        return 0U;
    }

    return manager->generator.ops->diagnostic_count(
        manager->generator.context);
}

static motion_manager_result_t motion_manager_sample(
    motion_manager_t *manager,
    motion_mode_t mode,
    float amplitude_scale,
    float bias_scale,
    joint_targets_t *targets)
{
    if ((manager->generator.ops == NULL) ||
        (manager->generator.ops->sample == NULL) ||
        !manager->generator.ops->sample(
            manager->generator.context,
            mode,
            amplitude_scale,
            bias_scale,
            targets))
    {
        return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
    }

    manager->operational_clamp_count =
        motion_manager_generator_diagnostic_count(manager);
    return MOTION_MANAGER_RESULT_OK;
}

static motion_manager_result_t motion_manager_tick(
    motion_manager_t *manager)
{
    joint_targets_t targets;
    motion_manager_result_t result;

    if (manager->state == MOTION_STATE_STOPPING)
    {
        if (manager->stop_elapsed_ms < MOTION_TRANSITION_DURATION_MS)
        {
            manager->stop_elapsed_ms += MOTION_GAIT_TICK_MS;
            if (manager->stop_elapsed_ms > MOTION_TRANSITION_DURATION_MS)
            {
                manager->stop_elapsed_ms = MOTION_TRANSITION_DURATION_MS;
            }
        }

        targets = interpolate_targets(
            &manager->stop_start_targets,
            &zero_targets,
            manager->stop_elapsed_ms);
        result = motion_manager_apply_targets(manager, &targets);
        if (result != MOTION_MANAGER_RESULT_OK)
        {
            motion_manager_stop_immediate(manager);
            return result;
        }
        manager->last_targets = targets;

        if (manager->stop_elapsed_ms >= MOTION_TRANSITION_DURATION_MS)
        {
            servo_service_motion_end(manager->servo_service);
            manager->state = MOTION_STATE_STOPPED;
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

    manager->generator.ops->advance(
        manager->generator.context,
        MOTION_GAIT_TICK_MS);

    if (manager->transition == MOTION_MANAGER_TRANSITION_START)
    {
        if (manager->transition_elapsed_ms < MOTION_TRANSITION_DURATION_MS)
        {
            manager->transition_elapsed_ms += MOTION_GAIT_TICK_MS;
            if (manager->transition_elapsed_ms >
                MOTION_TRANSITION_DURATION_MS)
            {
                manager->transition_elapsed_ms =
                    MOTION_TRANSITION_DURATION_MS;
            }
        }
        const float scale =
            (float)manager->transition_elapsed_ms /
            (float)MOTION_TRANSITION_DURATION_MS;
        result = motion_manager_sample(
            manager,
            manager->active_mode,
            scale,
            scale,
            &targets);
    }
    else if (manager->transition == MOTION_MANAGER_TRANSITION_MODE)
    {
        joint_targets_t from_targets;
        joint_targets_t to_targets;

        if (manager->transition_elapsed_ms < MOTION_TRANSITION_DURATION_MS)
        {
            manager->transition_elapsed_ms += MOTION_GAIT_TICK_MS;
            if (manager->transition_elapsed_ms >
                MOTION_TRANSITION_DURATION_MS)
            {
                manager->transition_elapsed_ms =
                    MOTION_TRANSITION_DURATION_MS;
            }
        }
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
    manager->state = MOTION_STATE_STOPPED;
    manager->active_mode = MOTION_STOP;
    manager->transition_mode = MOTION_STOP;
    manager->transition_from_mode = MOTION_STOP;
}

motion_manager_result_t motion_manager_start(
    motion_manager_t *manager,
    motion_mode_t mode)
{
    uint16_t required_mask;
    motion_manager_result_t result;

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
        if (manager->active_mode == mode)
        {
            return MOTION_MANAGER_RESULT_OK;
        }

        if ((servo_service_enabled_mask(manager->servo_service) &
             required_mask) != required_mask)
        {
            return MOTION_MANAGER_RESULT_SERVO_NOT_ENABLED;
        }

        manager->transition_from_mode = manager->active_mode;
        manager->transition_mode = mode;
        manager->transition_elapsed_ms = 0U;
        manager->write_mask = (uint16_t)(
            manager->write_mask | required_mask);
        manager->transition = MOTION_MANAGER_TRANSITION_MODE;
        return MOTION_MANAGER_RESULT_OK;
    }

    if ((servo_service_enabled_mask(manager->servo_service) &
         required_mask) != required_mask)
    {
        return MOTION_MANAGER_RESULT_SERVO_NOT_ENABLED;
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
    manager->last_targets = zero_targets;
    manager->stop_start_targets = zero_targets;
    manager->scheduler_started = 0U;
    return MOTION_MANAGER_RESULT_OK;
}

motion_manager_result_t motion_manager_request_stop(
    motion_manager_t *manager)
{
    if (manager == NULL)
    {
        return MOTION_MANAGER_RESULT_INVALID_MODE;
    }

    if ((manager->state == MOTION_STATE_STOPPED) ||
        (manager->state == MOTION_STATE_FAULTED))
    {
        return MOTION_MANAGER_RESULT_OK;
    }

    if (manager->state == MOTION_STATE_STOPPING)
    {
        return MOTION_MANAGER_RESULT_OK;
    }

    manager->stop_start_targets = manager->last_targets;
    manager->stop_elapsed_ms = 0U;
    manager->state = MOTION_STATE_STOPPING;
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
        motion_manager_stop_immediate(manager);
        return MOTION_MANAGER_RESULT_HOST_NOT_ALIVE;
    }

    if (manager->scheduler_started == 0U)
    {
        manager->last_tick_ms = now_ms;
        manager->scheduler_started = 1U;
        return MOTION_MANAGER_RESULT_OK;
    }

    if ((uint32_t)(now_ms - manager->last_tick_ms) < MOTION_GAIT_TICK_MS)
    {
        return MOTION_MANAGER_RESULT_OK;
    }

    manager->last_tick_ms = now_ms;
    return motion_manager_tick(manager);
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

    if (manager->servo_service != NULL)
    {
        servo_service_motion_abort(manager->servo_service);
    }
    manager->last_targets = zero_targets;
    manager->stop_start_targets = zero_targets;
    manager->transition_elapsed_ms = 0U;
    manager->stop_elapsed_ms = 0U;
    manager->scheduler_started = 0U;
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
