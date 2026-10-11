#include "cpg_gait_generator.h"
#include "protocol_dispatcher.h"
#include "simple_gait_generator.h"
#include "experimental_flex_gait_generator.h"
#include <math.h>
#include "motion_config.h"
#include "motion_manager.h"
#include "safety_supervisor.h"
#include "servo_descriptor.h"
#include "servo_service.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

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

static void test_mapping(void)
{
    for (unsigned backend = 0; backend < 3; ++backend)
    {
        fixture_t f;
        fixture_init(&f);
        simple_gait_generator_t simple;
        experimental_flex_gait_generator_t flex;
        simple_gait_generator_init(&simple);
        experimental_flex_gait_generator_init(&flex);
        motion_manager_init_with_backends(
            &f.manager, &f.servo_service, &f.safety_supervisor,
            simple_gait_generator_interface(&simple), cpg_gait_generator_interface(&f.generator),
            experimental_flex_gait_generator_interface(&flex), (motion_gait_backend_t)backend);
        proportional_config_t q = {1000, 1000, 1000, 10000};
        assert(motion_manager_start_proportional(&f.manager, &q, 255, 0) ==
               MOTION_MANAGER_RESULT_OK);
        assert(f.manager.write_mask == 0x1F &&
               f.manager.control_mode == MOTION_CONTROL_PROPORTIONAL);
        assert(motion_manager_start(&f.manager, MOTION_FORWARD) == MOTION_MANAGER_RESULT_BUSY);
        motion_manager_process(&f.manager, 0);
        for (uint32_t ms = 10; ms <= 800; ms += 10)
        {
            assert(motion_manager_update_proportional(&f.manager, 255, (uint16_t)ms, 500, 500, -500,
                                                      ms));
            motion_manager_process(&f.manager, ms);
            if (ms == 10)
            {
                assert(fabsf(f.manager.effective_throttle - 0.1F / 75.0F) < 0.00001F);
                assert(fabsf(f.manager.effective_turn - 0.1F) < 0.00001F);
            }
        }
        joint_targets_t base;
        assert(f.manager.generator.ops->sample(f.manager.generator.context, MOTION_FORWARD, 1, 0,
                                               &base));
        assert(f.manager.last_targets.front_left_cdeg == (int32_t)(base.front_left_cdeg * 0.5F));
        assert(f.manager.last_targets.front_right_cdeg == (int32_t)(base.front_right_cdeg * 0.25F));
        assert(f.manager.last_targets.rear_left_cdeg == (int32_t)(base.rear_left_cdeg * 0.5F));
        assert(f.manager.last_targets.rear_right_cdeg == (int32_t)(base.rear_right_cdeg * 0.25F));
        assert(f.manager.last_targets.front_axis_cdeg == 500);
        motion_manager_request_stop_at(&f.manager, 800);
        assert(!f.manager.proportional_session_valid);
        const joint_targets_t frozen = f.manager.last_targets;
        motion_manager_process(&f.manager, 1175);
        assert(f.manager.last_targets.front_left_cdeg ==
               frozen.front_left_cdeg + (0 - frozen.front_left_cdeg) / 2);
        assert(fabsf(f.manager.effective_throttle - 0.25F) < 0.00001F);
        assert(fabsf(f.manager.effective_turn - 0.5F) < 0.00001F);
        motion_manager_process(&f.manager, 1550);
        assert(f.manager.state == MOTION_STATE_STOPPED &&
               f.manager.control_mode == MOTION_CONTROL_DISCRETE);
        assert(servo_service_enabled_mask(&f.servo_service) == 0x1F);
    }
    fixture_t delayed;
    fixture_init(&delayed);
    const proportional_config_t q = {1000, 1000, 1000, 2000};
    assert(motion_manager_start_proportional(&delayed.manager, &q, 1, 0) ==
           MOTION_MANAGER_RESULT_OK);
    motion_manager_process(&delayed.manager, 0);
    assert(motion_manager_update_proportional(&delayed.manager, 1, 1, 1000, 1000, 1000, 100));
    motion_manager_process(&delayed.manager, 100);
    assert(fabsf(delayed.manager.slewed_throttle - 0.02F) < 0.00001F);
    assert(fabsf(delayed.manager.slewed_turn - 0.02F) < 0.00001F);
    assert(fabsf(delayed.manager.slewed_pitch - 0.02F) < 0.00001F);
}

static void test_bounds_sessions_timeout(void)
{
    fixture_t f;
    fixture_init(&f);
    proportional_config_t q = {1000, 1000, 1000, 2000};
    uint16_t *fields[] = {&q.max_scale, &q.turn_gain, &q.pitch_limit_cdeg, &q.slew_per_second};
    const uint16_t invalid[] = {99, 2001, 2001, 99};
    for (unsigned i = 0; i < 4; ++i)
    {
        const uint16_t original = *fields[i];
        *fields[i] = invalid[i];
        assert(motion_manager_start_proportional(&f.manager, &q, 0, 0) ==
               MOTION_MANAGER_RESULT_OUT_OF_RANGE);
        *fields[i] = original;
    }
    q.max_scale = 1001;
    assert(motion_manager_start_proportional(&f.manager, &q, 0, 0) ==
           MOTION_MANAGER_RESULT_OUT_OF_RANGE);
    q.max_scale = 1000;
    q.slew_per_second = 10001;
    assert(motion_manager_start_proportional(&f.manager, &q, 0, 0) ==
           MOTION_MANAGER_RESULT_OUT_OF_RANGE);
    q.slew_per_second = 2000;
    assert(motion_manager_start_proportional(&f.manager, &q, 0, 0) == MOTION_MANAGER_RESULT_OK);
    assert(!motion_manager_update_proportional(&f.manager, 1, 0, 0, 0, 0, 1));
    assert(!motion_manager_update_proportional(&f.manager, 0, 0, 1001, 0, 0, 1));
    assert(!motion_manager_update_proportional(&f.manager, 0, 0, 0, 1001, 0, 1));
    assert(!motion_manager_update_proportional(&f.manager, 0, 0, 0, 0, -1001, 1));
    assert(motion_manager_update_proportional(&f.manager, 0, 65535, 1000, -1000, 1000, 10));
    assert(motion_manager_update_proportional(&f.manager, 0, 0, 0, 0, 0, 20));
    assert(!motion_manager_update_proportional(&f.manager, 0, 0, 0, 0, 0, 30));
    assert(!motion_manager_update_proportional(&f.manager, 0, 65535, 0, 0, 0, 30));
    assert(!motion_manager_update_proportional(&f.manager, 0, 32768, 0, 0, 0, 30));
    assert(f.manager.proportional_last_input_ms == 20);
    motion_manager_process(&f.manager, 20);
    motion_manager_process(&f.manager, 620);
    assert(f.manager.state == MOTION_STATE_RUNNING);
    motion_manager_process(&f.manager, 630);
    assert(f.manager.state == MOTION_STATE_STOPPING && f.manager.stop_reason == 3);
    assert(!motion_manager_update_proportional(&f.manager, 0, 1, 0, 0, 0, 640));
    motion_manager_process(&f.manager, 1380);
    assert(f.manager.state == MOTION_STATE_STOPPED);
    assert(motion_manager_start_proportional(&f.manager, &q, 1, 1400) == MOTION_MANAGER_RESULT_OK);
    assert(!motion_manager_update_proportional(&f.manager, 1, 0, 0, 0, 0, 2001));
    assert(f.manager.state == MOTION_STATE_STOPPING);
}

int main(void)
{
    fixture_t f;
    fixture_init(&f);
    protocol_dispatcher_t d;
    protocol_dispatcher_init(&d, &f.servo_service, &f.safety_supervisor, &f.manager);
    rbp2_frame_t frame = {0};
    frame.type = 0x1B;
    frame.payload_length = 9;
    const uint8_t payload[9] = {0xE8, 3, 0xE8, 3, 0xE8, 3, 0xD0, 7, 42};
    memcpy(frame.payload, payload, sizeof(payload));
    assert(protocol_dispatcher_handle(&d, &frame, 0).result == RBP2_RESULT_OK);
    assert(f.manager.proportional_session_id == 42);
    frame.type = RBP2_MSG_PROPORTIONAL_INPUT;
    const uint8_t input[9] = {42, 255, 255, 0xE8, 3, 0x18, 0xFC, 0xE8, 3};
    memcpy(frame.payload, input, sizeof(input));
    assert(protocol_dispatcher_handle(&d, &frame, 10).result == RBP2_RESULT_OK);
    frame.payload[1] = frame.payload[2] = 0;
    assert(protocol_dispatcher_handle(&d, &frame, 20).result == RBP2_RESULT_OK);
    assert(protocol_dispatcher_handle(&d, &frame, 30).result != RBP2_RESULT_OK);
    assert(f.manager.proportional_last_input_ms == 20);
    frame.type = RBP2_MSG_SERVO_DISABLE;
    frame.payload_length = 2;
    frame.payload[0] = 1;
    frame.payload[1] = 0;
    assert(protocol_dispatcher_handle(&d, &frame, 40).result == RBP2_RESULT_OK);
    assert(!f.manager.proportional_session_valid);
    assert(!motion_manager_update_proportional(&f.manager, 42, 1, 0, 0, 0, 50));
    fixture_init(&f);
    proportional_config_t q = {1000, 1000, 1000, 2000};
    assert(motion_manager_start_proportional(&f.manager, &q, 0, 0) == MOTION_MANAGER_RESULT_OK);
    assert(!safety_supervisor_process(&f.safety_supervisor, 500));
    assert(safety_supervisor_process(&f.safety_supervisor, 501));
    assert(motion_manager_process(&f.manager, 501) == MOTION_MANAGER_RESULT_HOST_NOT_ALIVE);
    assert(!f.manager.proportional_session_valid && f.manager.stop_reason == 5);
    assert(f.manager.state == MOTION_STATE_FAULTED);
    assert(!servo_service_motion_is_active(&f.servo_service));
    test_mapping();
    test_bounds_sessions_timeout();
    puts("proportional firmware tests passed");
    return 0;
}
