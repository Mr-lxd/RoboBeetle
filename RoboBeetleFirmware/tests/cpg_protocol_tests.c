#include "cpg_gait_generator.h"
#include "motion_config.h"
#include "motion_manager.h"
#include "safety_supervisor.h"
#include "servo_descriptor.h"
#include "servo_service.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "protocol_dispatcher.h"

typedef struct
{
    bool start_result;
    uint32_t write_calls;
    uint32_t start_calls;
    uint32_t stop_calls;
} fake_driver_t;

static void fake_write_pulse(void *context, uint8_t servo_id, uint16_t pulse_us)
{
    fake_driver_t *driver = (fake_driver_t *)context;

    (void)servo_id;
    (void)pulse_us;
    ++driver->write_calls;
}

static bool fake_start(void *context, uint8_t servo_id)
{
    fake_driver_t *driver = (fake_driver_t *)context;

    (void)servo_id;
    ++driver->start_calls;
    return driver->start_result;
}

static void fake_stop(void *context, uint8_t servo_id)
{
    fake_driver_t *driver = (fake_driver_t *)context;

    (void)servo_id;
    ++driver->stop_calls;
}

static const servo_service_driver_ops_t fake_driver_ops = {
    .write_pulse_us = fake_write_pulse,
    .start = fake_start,
    .stop = fake_stop,
    .is_stop_pending = NULL,
};

typedef struct
{
    fake_driver_t driver;
    servo_service_t servo_service;
    safety_supervisor_t safety_supervisor;
    cpg_gait_generator_t generator;
    motion_manager_t manager;
} fixture_t;

static void fixture_init(fixture_t *fixture)
{
    (void)memset(fixture, 0, sizeof(*fixture));
    fixture->driver.start_result = true;
    servo_service_init(&fixture->servo_service, &fake_driver_ops, &fixture->driver);
    assert(servo_service_enable(&fixture->servo_service, SERVO_DESCRIPTOR_SUPPORTED_MASK) ==
           SERVO_SERVICE_RESULT_OK);
    safety_supervisor_init(&fixture->safety_supervisor);
    safety_supervisor_on_heartbeat(&fixture->safety_supervisor, 0U);
    cpg_gait_generator_init(&fixture->generator);
    motion_manager_init(&fixture->manager, &fixture->servo_service, &fixture->safety_supervisor,
                        cpg_gait_generator_interface(&fixture->generator));
}

int main(void)
{
    fixture_t x;
    fixture_init(&x);
    x.manager.cpg_generator = &x.generator;
    protocol_dispatcher_t d;
    protocol_dispatcher_init(&d, &x.servo_service, &x.safety_supervisor, &x.manager);
    cpg_parameters_t params;
    cpg_parameters_default(&params);
    params.front_amp = 12;
    rbp2_frame_t frame = {0};
    frame.type = RBP2_MSG_SET_CPG_PARAMETERS;
    frame.sequence = 123;
    frame.payload_length = 58;
    cpg_parameters_encode(&params, frame.payload, 58);
    protocol_dispatcher_outcome_t out = protocol_dispatcher_handle(&d, &frame, 0);
    assert(out.result == RBP2_RESULT_OK && out.cpg_snapshot);
    assert(x.manager.cpg_parameter_version == 1 && x.generator.profile.front_amplitude_deg == 12);
    out = protocol_dispatcher_handle(&d, &frame, 1);
    assert(out.cpg_snapshot && x.manager.cpg_parameter_version == 1);
    uint8_t snapshot[63];
    assert(protocol_dispatcher_cpg_snapshot(&d, 123, snapshot, 63) == 63);
    assert(snapshot[0] == 123 && snapshot[1] == 0 && snapshot[2] == 1 && snapshot[3] == 0 &&
           snapshot[4] == 1);
    cpg_parameters_t back;
    assert(cpg_parameters_decode(snapshot + 5, 58, &back) == CPG_PARAMETERS_OK);
    assert(cpg_parameters_equal(&params, &back));
    frame.sequence = 124;
    frame.type = RBP2_MSG_QUERY_CPG_PARAMETERS;
    frame.payload_length = 0;
    out = protocol_dispatcher_handle(&d, &frame, 2);
    assert(out.result == RBP2_RESULT_OK && out.cpg_snapshot);
    frame.sequence = 125;
    frame.payload_length = 1;
    out = protocol_dispatcher_handle(&d, &frame, 3);
    assert(out.result == RBP2_RESULT_INVALID_PAYLOAD && !out.cpg_snapshot);
    frame.type = RBP2_MSG_SET_CPG_PARAMETERS;
    frame.sequence = 126;
    frame.payload_length = 58;
    x.manager.state = MOTION_STATE_FAULTED;
    out = protocol_dispatcher_handle(&d, &frame, 4);
    assert(out.result == RBP2_RESULT_INVALID_STATE && !out.cpg_snapshot);
    assert(x.manager.cpg_parameter_version == 1);
    return 0;
}
