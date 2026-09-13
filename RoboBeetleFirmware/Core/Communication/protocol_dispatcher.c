#include "protocol_dispatcher.h"

static uint16_t read_le16(
    const uint8_t *data)
{
    return (uint16_t)data[0]
         | ((uint16_t)data[1] << 8U);
}

static uint32_t read_le32(
    const uint8_t *data)
{
    return (uint32_t)data[0]
         | ((uint32_t)data[1] << 8U)
         | ((uint32_t)data[2] << 16U)
         | ((uint32_t)data[3] << 24U);
}

static rbp2_result_t map_servo_service_result(
    servo_service_result_t result)
{
    switch (result)
    {
        case SERVO_SERVICE_RESULT_OK:
            return RBP2_RESULT_OK;

        case SERVO_SERVICE_RESULT_INVALID_PAYLOAD:
            return RBP2_RESULT_INVALID_PAYLOAD;

        case SERVO_SERVICE_RESULT_UNSUPPORTED_SERVO:
            return RBP2_RESULT_UNSUPPORTED_SERVO;

        case SERVO_SERVICE_RESULT_SERVO_NOT_ENABLED:
            return RBP2_RESULT_SERVO_NOT_ENABLED;

        case SERVO_SERVICE_RESULT_OUT_OF_RANGE:
            return RBP2_RESULT_OUT_OF_RANGE;

        case SERVO_SERVICE_RESULT_HARDWARE_FAILURE:
            return RBP2_RESULT_HARDWARE_FAILURE;

        case SERVO_SERVICE_RESULT_BUSY:
            return RBP2_RESULT_BUSY;

        default:
            return RBP2_RESULT_HARDWARE_FAILURE;
    }
}

static rbp2_result_t map_motion_manager_result(
    motion_manager_result_t result)
{
    switch (result)
    {
        case MOTION_MANAGER_RESULT_OK:
            return RBP2_RESULT_OK;

        case MOTION_MANAGER_RESULT_INVALID_MODE:
            return RBP2_RESULT_INVALID_PAYLOAD;

        case MOTION_MANAGER_RESULT_HOST_NOT_ALIVE:
            return RBP2_RESULT_HOST_NOT_ALIVE;

        case MOTION_MANAGER_RESULT_SERVO_NOT_ENABLED:
            return RBP2_RESULT_SERVO_NOT_ENABLED;

        case MOTION_MANAGER_RESULT_BUSY:
            return RBP2_RESULT_BUSY;

        case MOTION_MANAGER_RESULT_HARDWARE_FAILURE:
        default:
            return RBP2_RESULT_HARDWARE_FAILURE;
    }
}

static protocol_dispatcher_outcome_t make_outcome(
    rbp2_result_t result)
{
    protocol_dispatcher_outcome_t outcome = {0};

    outcome.result = result;
    return outcome;
}

static protocol_dispatcher_outcome_t complete_command(
    protocol_dispatcher_t *dispatcher,
    const rbp2_frame_t *frame,
    rbp2_result_t result)
{
    if (result == RBP2_RESULT_OK)
    {
        dispatcher->last_request_valid = true;
        dispatcher->last_request_sequence = frame->sequence;
        dispatcher->last_request_type = frame->type;
        dispatcher->last_request_result = result;
    }

    return make_outcome(result);
}

void protocol_dispatcher_init(
    protocol_dispatcher_t *dispatcher,
    servo_service_t *servo_service,
    safety_supervisor_t *safety_supervisor,
    motion_manager_t *motion_manager)
{
    dispatcher->servo_service = servo_service;
    dispatcher->safety_supervisor = safety_supervisor;
    dispatcher->motion_manager = motion_manager;
    dispatcher->last_request_valid = false;
    dispatcher->last_request_sequence = 0U;
    dispatcher->last_request_type = 0U;
    dispatcher->last_request_result = RBP2_RESULT_OK;
}

void protocol_dispatcher_invalidate_action_cache(
    protocol_dispatcher_t *dispatcher)
{
    dispatcher->last_request_valid = false;
}

protocol_dispatcher_outcome_t protocol_dispatcher_handle(
    protocol_dispatcher_t *dispatcher,
    const rbp2_frame_t *frame,
    uint32_t now_ms)
{
    if ((frame->type != RBP2_MSG_HEARTBEAT) &&
        dispatcher->last_request_valid &&
        (frame->sequence == dispatcher->last_request_sequence) &&
        (frame->type == dispatcher->last_request_type))
    {
        return make_outcome(dispatcher->last_request_result);
    }

    switch (frame->type)
    {
        case RBP2_MSG_HEARTBEAT:
        {
            protocol_dispatcher_outcome_t outcome;

            if (frame->payload_length != 4U)
            {
                outcome = make_outcome(
                    RBP2_RESULT_INVALID_PAYLOAD);
                outcome.count_bad_frame = true;
                return outcome;
            }

            outcome = make_outcome(RBP2_RESULT_OK);
            outcome.heartbeat_accepted = true;
            outcome.heartbeat_uptime_ms =
                read_le32(frame->payload);

            safety_supervisor_on_heartbeat(
                dispatcher->safety_supervisor,
                now_ms);

            return outcome;
        }

        case RBP2_MSG_SERVO_ENABLE:
        {
            if (frame->payload_length != 2U)
            {
                return complete_command(
                    dispatcher,
                    frame,
                    RBP2_RESULT_INVALID_PAYLOAD);
            }

            if (!safety_supervisor_is_host_alive(
                    dispatcher->safety_supervisor))
            {
                return complete_command(
                    dispatcher,
                    frame,
                    RBP2_RESULT_HOST_NOT_ALIVE);
            }

            uint16_t mask =
                read_le16(frame->payload);

            return complete_command(
                dispatcher,
                frame,
                map_servo_service_result(
                    servo_service_enable(
                        dispatcher->servo_service,
                        mask)));
        }

        case RBP2_MSG_SERVO_DISABLE:
        {
            if (frame->payload_length != 2U)
            {
                return complete_command(
                    dispatcher,
                    frame,
                    RBP2_RESULT_INVALID_PAYLOAD);
            }

            uint16_t mask =
                read_le16(frame->payload);

            const servo_service_result_t mask_result =
                servo_service_validate_mask(mask);

            if (mask_result != SERVO_SERVICE_RESULT_OK)
            {
                return complete_command(
                    dispatcher,
                    frame,
                    map_servo_service_result(mask_result));
            }

            if ((dispatcher->motion_manager != NULL) &&
                ((servo_service_motion_mask(
                      dispatcher->servo_service) & mask) != 0U))
            {
                motion_manager_stop_immediate(
                    dispatcher->motion_manager);
            }

            return complete_command(
                dispatcher,
                frame,
                map_servo_service_result(
                    servo_service_disable(
                        dispatcher->servo_service,
                        mask)));
        }

        case RBP2_MSG_SET_SERVO_PWM:
        {
            if (!safety_supervisor_is_host_alive(
                    dispatcher->safety_supervisor))
            {
                return complete_command(
                    dispatcher,
                    frame,
                    RBP2_RESULT_HOST_NOT_ALIVE);
            }

            if (frame->payload_length != 4U)
            {
                return complete_command(
                    dispatcher,
                    frame,
                    RBP2_RESULT_INVALID_PAYLOAD);
            }

            uint8_t count =
                frame->payload[0];
            uint8_t servo_id =
                frame->payload[1];
            uint16_t pulse_us =
                read_le16(&frame->payload[2]);

            if (count != 1U)
            {
                return complete_command(
                    dispatcher,
                    frame,
                    RBP2_RESULT_INVALID_PAYLOAD);
            }

            return complete_command(
                dispatcher,
                frame,
                map_servo_service_result(
                    servo_service_set_pwm(
                        dispatcher->servo_service,
                        servo_id,
                        pulse_us)));
        }

        case RBP2_MSG_NEUTRAL:
        {
            if (frame->payload_length != 2U)
            {
                return complete_command(
                    dispatcher,
                    frame,
                    RBP2_RESULT_INVALID_PAYLOAD);
            }

            if (!safety_supervisor_is_host_alive(
                    dispatcher->safety_supervisor))
            {
                return complete_command(
                    dispatcher,
                    frame,
                    RBP2_RESULT_HOST_NOT_ALIVE);
            }

            uint16_t mask =
                read_le16(frame->payload);

            return complete_command(
                dispatcher,
                frame,
                map_servo_service_result(
                    servo_service_neutral(
                        dispatcher->servo_service,
                        mask)));
        }

        case RBP2_MSG_SET_SERVO_ANGLE:
        {
            if (!safety_supervisor_is_host_alive(
                    dispatcher->safety_supervisor))
            {
                return complete_command(
                    dispatcher,
                    frame,
                    RBP2_RESULT_HOST_NOT_ALIVE);
            }

            if (frame->payload_length != 4U)
            {
                return complete_command(
                    dispatcher,
                    frame,
                    RBP2_RESULT_INVALID_PAYLOAD);
            }

            uint8_t count =
                frame->payload[0];
            uint8_t servo_id =
                frame->payload[1];
            int16_t angle_cdeg =
                (int16_t)read_le16(&frame->payload[2]);

            if (count != 1U)
            {
                return complete_command(
                    dispatcher,
                    frame,
                    RBP2_RESULT_INVALID_PAYLOAD);
            }

            return complete_command(
                dispatcher,
                frame,
                map_servo_service_result(
                    servo_service_set_angle(
                        dispatcher->servo_service,
                        servo_id,
                        angle_cdeg)));
        }

        case RBP2_MSG_SET_MOTION_MODE:
        {
            motion_mode_t mode;
            motion_action_t action;
            motion_manager_result_t manager_result;

            if (frame->payload_length != 3U)
            {
                return complete_command(
                    dispatcher,
                    frame,
                    RBP2_RESULT_INVALID_PAYLOAD);
            }

            if (frame->payload[0] != 1U)
            {
                return complete_command(
                    dispatcher,
                    frame,
                    RBP2_RESULT_INVALID_PAYLOAD);
            }

            mode = (motion_mode_t)frame->payload[1];
            action = (motion_action_t)frame->payload[2];
            if (!motion_mode_is_valid(mode) ||
                ((action != MOTION_ACTION_STOP) &&
                 (action != MOTION_ACTION_START)) ||
                ((action == MOTION_ACTION_START) &&
                 (mode == MOTION_STOP)) ||
                ((action == MOTION_ACTION_STOP) &&
                 (mode != MOTION_STOP)))
            {
                return complete_command(
                    dispatcher,
                    frame,
                    RBP2_RESULT_INVALID_PAYLOAD);
            }

            if ((action == MOTION_ACTION_START) &&
                !safety_supervisor_is_host_alive(
                    dispatcher->safety_supervisor))
            {
                return complete_command(
                    dispatcher,
                    frame,
                    RBP2_RESULT_HOST_NOT_ALIVE);
            }

            if (dispatcher->motion_manager == NULL)
            {
                return complete_command(
                    dispatcher,
                    frame,
                    RBP2_RESULT_HARDWARE_FAILURE);
            }

            manager_result = action == MOTION_ACTION_START
                ? motion_manager_start(
                    dispatcher->motion_manager,
                    mode)
                : motion_manager_request_stop_at(
                    dispatcher->motion_manager,
                    now_ms);

            return complete_command(
                dispatcher,
                frame,
                map_motion_manager_result(manager_result));
        }

        default:
        {
            return complete_command(
                dispatcher,
                frame,
                RBP2_RESULT_INVALID_PAYLOAD);
        }
    }
}
