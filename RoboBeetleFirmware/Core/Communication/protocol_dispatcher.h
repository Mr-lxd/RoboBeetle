#ifndef PROTOCOL_DISPATCHER_H
#define PROTOCOL_DISPATCHER_H

#include <stdbool.h>
#include <stdint.h>

#include "rb_protocol_v2.h"
#include "motion_manager.h"
#include "safety_supervisor.h"
#include "servo_service.h"

typedef struct
{
    servo_service_t *servo_service;
    safety_supervisor_t *safety_supervisor;
    motion_manager_t *motion_manager;

    bool last_request_valid;
    uint16_t last_request_sequence;
    uint8_t last_request_type;
    rbp2_result_t last_request_result;
} protocol_dispatcher_t;

typedef struct
{
    rbp2_result_t result;
    bool cpg_snapshot;
    bool heartbeat_accepted;
    uint32_t heartbeat_uptime_ms;
    bool count_bad_frame;
} protocol_dispatcher_outcome_t;

void protocol_dispatcher_init(
    protocol_dispatcher_t *dispatcher,
    servo_service_t *servo_service,
    safety_supervisor_t *safety_supervisor,
    motion_manager_t *motion_manager);

protocol_dispatcher_outcome_t protocol_dispatcher_handle(
    protocol_dispatcher_t *dispatcher,
    const rbp2_frame_t *frame,
    uint32_t now_ms);

size_t protocol_dispatcher_cpg_snapshot(const protocol_dispatcher_t *dispatcher, uint16_t request_sequence, uint8_t *payload, size_t capacity);

void protocol_dispatcher_invalidate_action_cache(
    protocol_dispatcher_t *dispatcher);

#endif /* PROTOCOL_DISPATCHER_H */
