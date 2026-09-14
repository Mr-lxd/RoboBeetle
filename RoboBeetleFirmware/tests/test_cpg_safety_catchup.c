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

typedef struct
{
    bool start_result;
    uint32_t write_calls;
    uint32_t start_calls;
    uint32_t stop_calls;
} fake_driver_t;

static void fake_write_pulse(
    void *context,
    uint8_t servo_id,
    uint16_t pulse_us)
{
    fake_driver_t *driver = (fake_driver_t *)context;

    (void)servo_id;
    (void)pulse_us;
    ++driver->write_calls;
}

static bool fake_start(
    void *context,
    uint8_t servo_id)
{
    fake_driver_t *driver = (fake_driver_t *)context;

    (void)servo_id;
    ++driver->start_calls;
    return driver->start_result;
}

static void fake_stop(
    void *context,
    uint8_t servo_id)
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
    servo_service_init(
        &fixture->servo_service,
        &fake_driver_ops,
        &fixture->driver);
    assert(servo_service_enable(
               &fixture->servo_service,
               SERVO_DESCRIPTOR_SUPPORTED_MASK) ==
           SERVO_SERVICE_RESULT_OK);
    safety_supervisor_init(&fixture->safety_supervisor);
    safety_supervisor_on_heartbeat(
        &fixture->safety_supervisor,
        0U);
    cpg_gait_generator_init(&fixture->generator);
    motion_manager_init(
        &fixture->manager,
        &fixture->servo_service,
        &fixture->safety_supervisor,
        cpg_gait_generator_interface(&fixture->generator));
}

static void start_running_motion(fixture_t *fixture)
{
    assert(motion_manager_start(
               &fixture->manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK);
    assert(motion_manager_process(&fixture->manager, 0U) ==
           MOTION_MANAGER_RESULT_OK);
    assert(motion_manager_state(&fixture->manager) ==
           MOTION_STATE_RUNNING);
}

static void test_safety_abort_precedes_stale_catch_up(void)
{
    fixture_t fixture;
    const uint32_t timeout_ms =
        SAFETY_SUPERVISOR_HEARTBEAT_TIMEOUT_MS;
    uint32_t writes_before_gap;
    uint32_t steps_before_gap;

    fixture_init(&fixture);
    start_running_motion(&fixture);
    assert(safety_supervisor_process(
               &fixture.safety_supervisor,
               10U) == false);
    assert(motion_manager_process(&fixture.manager, 10U) ==
           MOTION_MANAGER_RESULT_OK);
    writes_before_gap = fixture.driver.write_calls;
    steps_before_gap = cpg_core_executed_step_count(
        &fixture.generator.core);

    assert(safety_supervisor_process(
               &fixture.safety_supervisor,
               timeout_ms + 200U));
    assert(!safety_supervisor_is_host_alive(
        &fixture.safety_supervisor));
    assert(motion_manager_process(
               &fixture.manager,
               timeout_ms + 200U) ==
           MOTION_MANAGER_RESULT_HOST_NOT_ALIVE);
    assert(cpg_core_executed_step_count(
               &fixture.generator.core) == steps_before_gap);
    assert(fixture.driver.write_calls == writes_before_gap);
    assert(motion_manager_state(&fixture.manager) ==
           MOTION_STATE_FAULTED);

    safety_supervisor_on_heartbeat(
        &fixture.safety_supervisor,
        timeout_ms + 300U);
    assert(motion_manager_process(
               &fixture.manager,
               timeout_ms + 300U) ==
           MOTION_MANAGER_RESULT_OK);
    assert(motion_manager_state(&fixture.manager) ==
           MOTION_STATE_FAULTED);
    assert(cpg_core_executed_step_count(
               &fixture.generator.core) == steps_before_gap);
    assert(fixture.driver.write_calls == writes_before_gap);
}

static void test_live_catch_up_has_expected_substeps(void)
{
    fixture_t fixture;
    uint32_t steps_before;

    fixture_init(&fixture);
    start_running_motion(&fixture);
    steps_before = cpg_core_executed_step_count(
        &fixture.generator.core);

    assert(safety_supervisor_process(
               &fixture.safety_supervisor,
               20U) == false);
    assert(motion_manager_process(&fixture.manager, 20U) ==
           MOTION_MANAGER_RESULT_OK);
    assert(cpg_core_executed_step_count(
               &fixture.generator.core) == steps_before + 2U);

    assert(safety_supervisor_process(
               &fixture.safety_supervisor,
               90U) == false);
    assert(motion_manager_process(&fixture.manager, 90U) ==
           MOTION_MANAGER_RESULT_OK);
    assert(cpg_core_executed_step_count(
               &fixture.generator.core) == steps_before + 9U);

    assert(safety_supervisor_process(
               &fixture.safety_supervisor,
               190U) == false);
    assert(motion_manager_process(&fixture.manager, 190U) ==
           MOTION_MANAGER_RESULT_OK);
    assert(cpg_core_executed_step_count(
               &fixture.generator.core) == steps_before + 19U);
}

static void test_cpg_output_clamp_remains_in_motion_manager(void)
{
    fixture_t fixture;
    int16_t front_right_angle;
    int16_t front_left_angle;
    int16_t rear_right_angle;
    int16_t rear_left_angle;

    fixture_init(&fixture);
    start_running_motion(&fixture);
    safety_supervisor_on_heartbeat(
        &fixture.safety_supervisor,
        760U);
    assert(motion_manager_process(&fixture.manager, 760U) ==
           MOTION_MANAGER_RESULT_OK);
    fixture.generator.core.output_memory[0] = 100.0;
    fixture.generator.core.output_memory[1] = 100.0;
    fixture.generator.core.output_memory[2] = 100.0;
    fixture.generator.core.output_memory[3] = 100.0;
    assert(motion_manager_process(&fixture.manager, 770U) ==
           MOTION_MANAGER_RESULT_OK);
    assert(servo_service_logical_angle_cdeg(
               &fixture.servo_service,
               SERVO_ID_FRONT_RIGHT,
               &front_right_angle));
    assert(servo_service_logical_angle_cdeg(
               &fixture.servo_service,
               SERVO_ID_FRONT_LEFT,
               &front_left_angle));
    assert(servo_service_logical_angle_cdeg(
               &fixture.servo_service,
               SERVO_ID_REAR_RIGHT,
               &rear_right_angle));
    assert(servo_service_logical_angle_cdeg(
               &fixture.servo_service,
               SERVO_ID_REAR_LEFT,
               &rear_left_angle));
    assert(front_right_angle == MOTION_FRONT_MAX_CDEG);
    assert(front_left_angle == MOTION_FRONT_MAX_CDEG);
    assert(rear_right_angle == MOTION_REAR_MAX_CDEG);
    assert(rear_left_angle == MOTION_REAR_MAX_CDEG);
    assert(motion_manager_operational_clamp_count(
               &fixture.manager) >= 4U);
}

int main(void)
{
    test_safety_abort_precedes_stale_catch_up();
    test_live_catch_up_has_expected_substeps();
    test_cpg_output_clamp_remains_in_motion_manager();
    return 0;
}
