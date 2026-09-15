#include "motion_config.h"
#include "motion_manager.h"
#include "cpg_gait_generator.h"
#include "simple_gait_generator.h"

#include "safety_supervisor.h"
#include "servo_descriptor.h"
#include "servo_service.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void expect(bool condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

typedef struct
{
    bool start_result;
    unsigned int write_calls;
    unsigned int start_calls;
    unsigned int stop_calls;
    uint8_t last_servo_id;
    uint16_t last_pulse_us;
    uint16_t current_pulse[SERVO_DESCRIPTOR_COUNT];
} fake_driver_t;

static void fake_write_pulse(
    void *context,
    uint8_t servo_id,
    uint16_t pulse_us)
{
    fake_driver_t *driver = (fake_driver_t *)context;

    ++driver->write_calls;
    driver->last_servo_id = servo_id;
    driver->last_pulse_us = pulse_us;
    driver->current_pulse[servo_id] = pulse_us;
}

static bool fake_start(
    void *context,
    uint8_t servo_id)
{
    fake_driver_t *driver = (fake_driver_t *)context;

    ++driver->start_calls;
    driver->last_servo_id = servo_id;
    return driver->start_result;
}

static void fake_stop(
    void *context,
    uint8_t servo_id)
{
    fake_driver_t *driver = (fake_driver_t *)context;

    ++driver->stop_calls;
    driver->last_servo_id = servo_id;
}

static const servo_service_driver_ops_t fake_ops = {
    .write_pulse_us = fake_write_pulse,
    .start = fake_start,
    .stop = fake_stop,
};

typedef struct
{
    uint32_t advanced_ms;
    joint_targets_t targets;
} fake_gait_generator_t;

static void fake_gait_advance(
    void *context,
    uint32_t dt_ms)
{
    fake_gait_generator_t *generator =
        (fake_gait_generator_t *)context;

    generator->advanced_ms += dt_ms;
}

static bool fake_gait_sample(
    void *context,
    motion_mode_t mode,
    float amplitude_scale,
    float bias_scale,
    joint_targets_t *targets)
{
    fake_gait_generator_t *generator =
        (fake_gait_generator_t *)context;

    (void)amplitude_scale;
    (void)bias_scale;
    if ((targets == NULL) || !motion_mode_is_valid(mode))
    {
        return false;
    }

    if (mode == MOTION_STOP)
    {
        *targets = (joint_targets_t){0};
    }
    else
    {
        *targets = generator->targets;
    }
    return true;
}

static bool fake_gait_is_mode_valid(
    void *context,
    motion_mode_t mode)
{
    (void)context;
    return motion_mode_is_valid(mode);
}

static const gait_generator_ops_t fake_gait_ops = {
    .advance = fake_gait_advance,
    .sample = fake_gait_sample,
    .is_mode_valid = fake_gait_is_mode_valid,
    .diagnostic_count = NULL,
};

static gait_generator_t fake_gait_interface(
    fake_gait_generator_t *generator)
{
    const gait_generator_t interface = {
        .ops = &fake_gait_ops,
        .context = generator,
    };
    return interface;
}

typedef struct
{
    fake_driver_t driver;
    servo_service_t servo_service;
    safety_supervisor_t safety_supervisor;
    simple_gait_generator_t generator;
    motion_manager_t manager;
} fixture_t;

typedef struct
{
    fake_driver_t driver;
    servo_service_t servo_service;
    safety_supervisor_t safety_supervisor;
    simple_gait_generator_t simple_generator;
    cpg_gait_generator_t cpg_generator;
    motion_manager_t manager;
} backend_fixture_t;

static void fixture_init(
    fixture_t *fixture,
    uint16_t enabled_mask)
{
    (void)memset(fixture, 0, sizeof(*fixture));
    fixture->driver.start_result = true;

    servo_service_init(
        &fixture->servo_service,
        &fake_ops,
        &fixture->driver);
    expect(servo_service_enable(
               &fixture->servo_service,
               enabled_mask) == SERVO_SERVICE_RESULT_OK,
           "fixture servos should enable successfully");
    safety_supervisor_init(&fixture->safety_supervisor);
    safety_supervisor_on_heartbeat(&fixture->safety_supervisor, 0U);
    simple_gait_generator_init(&fixture->generator);
    motion_manager_init(
        &fixture->manager,
        &fixture->servo_service,
        &fixture->safety_supervisor,
        simple_gait_generator_interface(&fixture->generator));
}

static void fixture_init_with_generator(
    fixture_t *fixture,
    uint16_t enabled_mask,
    gait_generator_t generator)
{
    (void)memset(fixture, 0, sizeof(*fixture));
    fixture->driver.start_result = true;

    servo_service_init(
        &fixture->servo_service,
        &fake_ops,
        &fixture->driver);
    expect(servo_service_enable(
               &fixture->servo_service,
               enabled_mask) == SERVO_SERVICE_RESULT_OK,
           "custom-generator fixture servos should enable successfully");
    safety_supervisor_init(&fixture->safety_supervisor);
    safety_supervisor_on_heartbeat(&fixture->safety_supervisor, 0U);
    motion_manager_init(
        &fixture->manager,
        &fixture->servo_service,
        &fixture->safety_supervisor,
        generator);
}

static void backend_fixture_init(
    backend_fixture_t *fixture,
    motion_gait_backend_t initial_backend)
{
    (void)memset(fixture, 0, sizeof(*fixture));
    fixture->driver.start_result = true;

    servo_service_init(
        &fixture->servo_service,
        &fake_ops,
        &fixture->driver);
    expect(servo_service_enable(
               &fixture->servo_service,
               0x001BU) == SERVO_SERVICE_RESULT_OK,
           "dual-backend fixture servos should enable successfully");
    safety_supervisor_init(&fixture->safety_supervisor);
    safety_supervisor_on_heartbeat(&fixture->safety_supervisor, 0U);
    simple_gait_generator_init(&fixture->simple_generator);
    cpg_gait_generator_init(&fixture->cpg_generator);
    motion_manager_init_with_backends(
        &fixture->manager,
        &fixture->servo_service,
        &fixture->safety_supervisor,
        simple_gait_generator_interface(&fixture->simple_generator),
        cpg_gait_generator_interface(&fixture->cpg_generator),
        initial_backend);
}

static void keep_host_alive(
    fixture_t *fixture,
    uint32_t now_ms)
{
    if ((now_ms % 100U) == 0U)
    {
        safety_supervisor_on_heartbeat(
            &fixture->safety_supervisor,
            now_ms);
    }
}

static void complete_start_transition(fixture_t *fixture)
{
    uint32_t now_ms;

    for (now_ms = 100U; now_ms <= 700U; now_ms += 100U)
    {
        keep_host_alive(fixture, now_ms);
        (void)motion_manager_process(&fixture->manager, now_ms);
    }
    (void)motion_manager_process(&fixture->manager, 760U);
}

static int32_t abs_cdeg(int32_t value)
{
    return value < 0 ? -value : value;
}

static void expect_targets_zero(const joint_targets_t *targets);

static void test_stopped_running_and_start_gate(void)
{
    fixture_t fixture;

    fixture_init(&fixture, 0x001BU);
    expect(motion_manager_state(&fixture.manager) ==
               MOTION_STATE_STOPPED,
           "MotionManager should initialize STOPPED");
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "FORWARD should start with four enabled paddles");
    expect(motion_manager_state(&fixture.manager) ==
               MOTION_STATE_RUNNING,
           "successful Start should enter RUNNING");
    expect(servo_service_motion_is_active(&fixture.servo_service),
           "RUNNING should own the enabled paddle servos");

    fixture_init(&fixture, 0x000FU);
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_ASCEND) == MOTION_MANAGER_RESULT_SERVO_NOT_ENABLED,
           "ASCEND should require the enabled FrontAxis");
    expect(motion_manager_state(&fixture.manager) ==
               MOTION_STATE_STOPPED,
           "failed start should remain STOPPED");
}

static void test_running_mode_change_preserves_phase(void)
{
    fixture_t fixture;
    float phase_before_change;

    fixture_init(&fixture, 0x001BU);
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "mode-change setup should start FORWARD");
    (void)motion_manager_process(&fixture.manager, 0U);
    (void)motion_manager_process(&fixture.manager, 10U);
    complete_start_transition(&fixture);
    phase_before_change = simple_gait_generator_phase(&fixture.generator);

    expect(motion_manager_start(
               &fixture.manager,
               MOTION_TURN_LEFT) == MOTION_MANAGER_RESULT_OK,
           "running Motion should accept a mode change");
    expect(motion_manager_mode(&fixture.manager) == MOTION_FORWARD,
           "cross-fade should retain the active mode until its deadline");
    keep_host_alive(&fixture, 800U);
    (void)motion_manager_process(&fixture.manager, 800U);
    expect(simple_gait_generator_phase(&fixture.generator) >
               phase_before_change,
           "mode change should not reset the generator phase");
}

static void test_start_transition_rejects_different_mode_reentry(void)
{
    fixture_t fixture;

    fixture_init(&fixture, 0x001BU);
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "START transition reentry setup should start FORWARD");
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "repeated START for the transition target should be idempotent");
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_TURN_LEFT) == MOTION_MANAGER_RESULT_BUSY,
           "different START during the initial ramp should be BUSY");
    expect(fixture.manager.transition ==
               MOTION_MANAGER_TRANSITION_START &&
               fixture.manager.transition_mode == MOTION_FORWARD &&
               fixture.manager.transition_elapsed_ms == 0U,
           "different START must not overwrite the initial transition");
}

static void test_mode_transition_rejects_reentrant_mode_changes(void)
{
    fixture_t fixture;
    uint32_t elapsed_before_reentry;

    fixture_init(&fixture, 0x001BU);
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "mode transition reentry setup should start FORWARD");
    (void)motion_manager_process(&fixture.manager, 0U);
    (void)motion_manager_process(&fixture.manager, 10U);
    complete_start_transition(&fixture);
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_TURN_LEFT) == MOTION_MANAGER_RESULT_OK,
           "mode transition reentry setup should accept TURN_LEFT");
    keep_host_alive(&fixture, 800U);
    (void)motion_manager_process(&fixture.manager, 800U);
    elapsed_before_reentry = fixture.manager.transition_elapsed_ms;
    expect(elapsed_before_reentry > 0U,
           "mode transition reentry test should advance the cross-fade");

    expect(motion_manager_start(
               &fixture.manager,
               MOTION_TURN_LEFT) == MOTION_MANAGER_RESULT_OK,
           "repeated START for the cross-fade target should be idempotent");
    expect(fixture.manager.transition_elapsed_ms == elapsed_before_reentry,
           "idempotent START must not reset cross-fade elapsed time");
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_BUSY,
           "START for the stale active mode must be BUSY during cross-fade");
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_TURN_RIGHT) == MOTION_MANAGER_RESULT_BUSY,
           "START for a third mode must be BUSY during cross-fade");
    expect(fixture.manager.transition == MOTION_MANAGER_TRANSITION_MODE &&
               fixture.manager.transition_from_mode == MOTION_FORWARD &&
               fixture.manager.transition_mode == MOTION_TURN_LEFT,
           "reentrant START must not overwrite the active cross-fade");
}

static void test_graceful_stop_contract_and_monotonic_targets(void)
{
    fixture_t fixture;
    joint_targets_t previous;
    uint32_t now_ms;

    fixture_init(&fixture, 0x001FU);
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_ASCEND) == MOTION_MANAGER_RESULT_OK,
           "STOP setup should start ASCEND with all logical targets enabled");
    (void)motion_manager_process(&fixture.manager, 0U);
    for (now_ms = 10U; now_ms <= 100U; now_ms += 10U)
    {
        keep_host_alive(&fixture, now_ms);
        (void)motion_manager_process(&fixture.manager, now_ms);
    }

    previous = *motion_manager_last_targets(&fixture.manager);
    expect(motion_manager_request_stop(&fixture.manager) ==
               MOTION_MANAGER_RESULT_OK,
           "ordinary STOP should be accepted immediately");
    expect(motion_manager_state(&fixture.manager) ==
               MOTION_STATE_STOPPING,
           "accepted STOP should enter STOPPING immediately");
    expect(servo_service_motion_is_active(&fixture.servo_service),
           "STOPPING should retain Motion ownership");
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_BUSY,
           "Start during STOPPING should be BUSY");
    expect(servo_service_set_angle(
               &fixture.servo_service,
               SERVO_ID_FRONT_RIGHT,
               0) == SERVO_SERVICE_RESULT_BUSY,
           "manual SetAngle during STOPPING should be BUSY");
    expect(servo_service_set_pwm(
               &fixture.servo_service,
               SERVO_ID_FRONT_RIGHT,
               1450U) == SERVO_SERVICE_RESULT_BUSY,
           "manual SetPWM during STOPPING should be BUSY");

    for (now_ms = 110U;
         now_ms <= 100U + MOTION_TRANSITION_DURATION_MS;
         now_ms += MOTION_GAIT_TICK_MS)
    {
        const joint_targets_t *current;

        keep_host_alive(&fixture, now_ms);
        (void)motion_manager_process(&fixture.manager, now_ms);
        current = motion_manager_last_targets(&fixture.manager);
        expect(abs_cdeg(current->front_right_cdeg) <=
                   abs_cdeg(previous.front_right_cdeg),
               "FrontRight STOP target should converge monotonically");
        expect(abs_cdeg(current->front_left_cdeg) <=
                   abs_cdeg(previous.front_left_cdeg),
               "FrontLeft STOP target should converge monotonically");
        expect(abs_cdeg(current->front_axis_cdeg) <=
                   abs_cdeg(previous.front_axis_cdeg),
               "FrontAxis STOP target should converge monotonically");
        expect(abs_cdeg(current->rear_right_cdeg) <=
                   abs_cdeg(previous.rear_right_cdeg),
               "RearRight STOP target should converge monotonically");
        expect(abs_cdeg(current->rear_left_cdeg) <=
                   abs_cdeg(previous.rear_left_cdeg),
               "RearLeft STOP target should converge monotonically");
        previous = *current;
    }

    expect(motion_manager_state(&fixture.manager) ==
               MOTION_STATE_STOPPED,
           "750 ms graceful STOP should enter STOPPED");
    expect(motion_manager_stop_elapsed_ms(&fixture.manager) ==
               MOTION_TRANSITION_DURATION_MS,
           "graceful STOP should consume the centralized 750 ms duration");
    expect(!servo_service_motion_is_active(&fixture.servo_service),
           "final neutral should release Motion ownership");
    expect_targets_zero(motion_manager_last_targets(&fixture.manager));
}

static void expect_targets_zero(const joint_targets_t *targets)
{
    expect(targets->front_right_cdeg == 0 &&
               targets->front_left_cdeg == 0 &&
               targets->front_axis_cdeg == 0 &&
               targets->rear_right_cdeg == 0 &&
               targets->rear_left_cdeg == 0,
           "completed STOP should retain all-zero logical targets");
}

static void test_stop_is_idempotent_when_stopped(void)
{
    fixture_t fixture;

    fixture_init(&fixture, 0x001BU);
    expect(motion_manager_request_stop(&fixture.manager) ==
               MOTION_MANAGER_RESULT_OK,
           "STOP while STOPPED should be safe and idempotent");
    expect(motion_manager_state(&fixture.manager) ==
               MOTION_STATE_STOPPED,
           "STOP while STOPPED should not change state");
}

static void test_watchdog_interrupts_graceful_stop_without_auto_resume(void)
{
    fixture_t fixture;
    unsigned int writes_before_abort;

    fixture_init(&fixture, 0x001BU);
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "watchdog setup should start FORWARD");
    (void)motion_manager_process(&fixture.manager, 0U);
    (void)motion_manager_process(&fixture.manager, 10U);
    expect(motion_manager_request_stop(&fixture.manager) ==
               MOTION_MANAGER_RESULT_OK,
           "watchdog setup should enter graceful STOPPING");
    writes_before_abort = fixture.driver.write_calls;

    expect(safety_supervisor_process(
               &fixture.safety_supervisor,
               SAFETY_SUPERVISOR_HEARTBEAT_TIMEOUT_MS + 1U),
           "watchdog should report host liveness loss");
    expect(motion_manager_process(
               &fixture.manager,
               SAFETY_SUPERVISOR_HEARTBEAT_TIMEOUT_MS + 1U) ==
               MOTION_MANAGER_RESULT_HOST_NOT_ALIVE,
           "MotionManager should abort immediately when liveness is lost");
    expect(motion_manager_state(&fixture.manager) ==
               MOTION_STATE_FAULTED,
           "watchdog should fault the Motion state immediately");
    motion_manager_stop_immediate(&fixture.manager);
    expect(motion_manager_state(&fixture.manager) ==
               MOTION_STATE_FAULTED,
           "repeated immediate abort should preserve the faulted state");
    expect(!servo_service_motion_is_active(&fixture.servo_service),
           "watchdog should release Motion ownership immediately");
    servo_service_disable_all(&fixture.servo_service);
    expect(servo_service_enabled_mask(&fixture.servo_service) == 0U,
           "Safety takeover should disable actuators after Motion abort");
    (void)motion_manager_process(&fixture.manager, 600U);
    expect(fixture.driver.write_calls == writes_before_abort,
           "watchdog interruption should not continue the graceful ramp");

    safety_supervisor_on_heartbeat(&fixture.safety_supervisor, 1000U);
    (void)motion_manager_process(&fixture.manager, 1000U);
    expect(motion_manager_state(&fixture.manager) ==
               MOTION_STATE_FAULTED,
           "reconnect/liveness recovery must not auto-resume Motion");
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_SERVO_NOT_ENABLED,
           "explicit restart after fail-safe requires Servo re-enable");
}

static void test_disable_all_preempts_stop_immediately(void)
{
    fixture_t fixture;
    unsigned int writes_before_abort;

    fixture_init(&fixture, 0x001BU);
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "Disable All setup should start FORWARD");
    (void)motion_manager_process(&fixture.manager, 0U);
    (void)motion_manager_process(&fixture.manager, 10U);
    (void)motion_manager_request_stop(&fixture.manager);
    writes_before_abort = fixture.driver.write_calls;

    motion_manager_stop_immediate(&fixture.manager);
    servo_service_disable_all(&fixture.servo_service);
    expect(motion_manager_state(&fixture.manager) ==
               MOTION_STATE_FAULTED,
           "Disable All should immediately terminate graceful STOPPING");
    expect(servo_service_enabled_mask(&fixture.servo_service) == 0U,
           "Disable All should clear enabled channels");
    (void)motion_manager_process(&fixture.manager, 760U);
    expect(fixture.driver.write_calls == writes_before_abort,
           "Disable All should prevent later graceful-ramp writes");
}

static void test_start_crossfades_from_recorded_manual_pose(void)
{
    fixture_t fixture;
    fake_gait_generator_t generator = {
        .advanced_ms = 0U,
        .targets = {
            .front_right_cdeg = 1000,
            .front_left_cdeg = -1000,
            .front_axis_cdeg = 0,
            .rear_right_cdeg = 0,
            .rear_left_cdeg = 0,
        },
    };

    fixture_init_with_generator(
        &fixture,
        0x001BU,
        fake_gait_interface(&generator));
    expect(servo_service_set_angle(
               &fixture.servo_service,
               SERVO_ID_FRONT_RIGHT,
               2500) == SERVO_SERVICE_RESULT_OK,
           "manual FrontRight pose should be established before Motion START");
    expect(servo_service_set_angle(
               &fixture.servo_service,
               SERVO_ID_FRONT_LEFT,
               -2500) == SERVO_SERVICE_RESULT_OK,
           "manual mirrored FrontLeft pose should be established before Motion START");

    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "Motion START should accept a known non-neutral pose");
    expect(motion_manager_last_targets(&fixture.manager)->front_right_cdeg == 2500 &&
               motion_manager_last_targets(&fixture.manager)->front_left_cdeg == -2500,
           "Motion START should retain the recorded logical pose before its first tick");

    (void)motion_manager_process(&fixture.manager, 0U);
    (void)motion_manager_process(&fixture.manager, 10U);
    expect(motion_manager_last_targets(&fixture.manager)->front_right_cdeg == 2480,
           "FrontRight should cross-fade from its manual logical pose");
    expect(motion_manager_last_targets(&fixture.manager)->front_left_cdeg == -2480,
           "mirrored FrontLeft should cross-fade from its manual logical pose");
}

static void test_motion_start_rejects_front_pose_outside_operational_envelope(void)
{
    fixture_t fixture;

    fixture_init(&fixture, 0x001BU);
    expect(servo_service_set_angle(
               &fixture.servo_service,
               SERVO_ID_FRONT_RIGHT,
               3000) == SERVO_SERVICE_RESULT_OK,
           "manual FrontRight setup should accept the full calibration range");
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_HARDWARE_FAILURE,
           "Motion START should reject a front pose above +2800 cdeg");
    expect(motion_manager_state(&fixture.manager) == MOTION_STATE_STOPPED &&
               !servo_service_motion_is_active(&fixture.servo_service),
           "out-of-envelope front pose rejection should not acquire Motion ownership");
}

static void test_motion_start_rejects_unknown_raw_pwm_pose(void)
{
    fixture_t fixture;

    fixture_init(&fixture, 0x001BU);
    expect(servo_service_set_pwm(
               &fixture.servo_service,
               SERVO_ID_FRONT_RIGHT,
               1600U) == SERVO_SERVICE_RESULT_OK,
           "raw PWM should be accepted before Motion START");
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_HARDWARE_FAILURE,
           "Motion START should reject an unknown raw-PWM logical pose");
    expect(motion_manager_state(&fixture.manager) == MOTION_STATE_STOPPED &&
               !servo_service_motion_is_active(&fixture.servo_service),
           "unknown pose rejection should not acquire Motion ownership");

    expect(servo_service_neutral(
               &fixture.servo_service,
               (uint16_t)(1U << SERVO_ID_FRONT_RIGHT)) ==
               SERVO_SERVICE_RESULT_OK,
           "Neutral should make the raw-PWM pose known again");
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "Motion START should recover after Neutral establishes the pose");
}

static void test_motion_start_rejects_rear_pose_outside_operational_envelope(void)
{
    fixture_t fixture;

    fixture_init(&fixture, 0x001BU);
    expect(servo_service_set_angle(
               &fixture.servo_service,
               SERVO_ID_REAR_RIGHT,
               -4500) == SERVO_SERVICE_RESULT_OK,
           "manual rear pose should accept the wider Servo angle range");
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_HARDWARE_FAILURE,
           "Motion START should reject a rear pose below its operational minimum");
    expect(motion_manager_state(&fixture.manager) == MOTION_STATE_STOPPED &&
               !servo_service_motion_is_active(&fixture.servo_service),
           "out-of-envelope rear pose rejection should not acquire Motion ownership");

    expect(servo_service_neutral(
               &fixture.servo_service,
               (uint16_t)(1U << SERVO_ID_REAR_RIGHT)) ==
               SERVO_SERVICE_RESULT_OK,
           "Neutral should restore a valid rear pose");
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "Motion START should recover after the rear pose returns in-envelope");
}

static void test_motion_process_uses_actual_elapsed_wall_time(void)
{
    fixture_t fixture;
    fake_gait_generator_t generator = {
        .advanced_ms = 0U,
        .targets = {0},
    };

    fixture_init_with_generator(
        &fixture,
        0x001BU,
        fake_gait_interface(&generator));
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "elapsed-time setup should start Motion");
    (void)motion_manager_process(&fixture.manager, 0U);
    (void)motion_manager_process(&fixture.manager, 10U);
    (void)motion_manager_process(&fixture.manager, 30U);
    (void)motion_manager_process(&fixture.manager, 100U);
    (void)motion_manager_process(&fixture.manager, 200U);
    expect(generator.advanced_ms == 200U,
           "MotionManager should advance by 10/20/70/100 ms wall-time gaps");
}

static void test_motion_process_elapsed_time_is_wrap_safe(void)
{
    fixture_t fixture;
    fake_gait_generator_t generator = {
        .advanced_ms = 0U,
        .targets = {0},
    };
    const uint32_t before_wrap = UINT32_MAX - 10U;

    fixture_init_with_generator(
        &fixture,
        0x001BU,
        fake_gait_interface(&generator));
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "wrap-time setup should start Motion");
    safety_supervisor_on_heartbeat(&fixture.safety_supervisor, before_wrap);
    (void)motion_manager_process(&fixture.manager, before_wrap);
    (void)motion_manager_process(&fixture.manager, 20U);
    expect(generator.advanced_ms == 31U,
           "MotionManager should use uint32 wrap-safe elapsed time");
}

static void test_graceful_stop_uses_actual_750_ms_duration(void)
{
    fixture_t fixture;
    fake_gait_generator_t generator = {
        .advanced_ms = 0U,
        .targets = {
            .front_right_cdeg = 1000,
            .front_left_cdeg = -1000,
            .front_axis_cdeg = 0,
            .rear_right_cdeg = 0,
            .rear_left_cdeg = 0,
        },
    };
    const uint32_t stop_times[] = {170U, 270U, 370U, 470U, 570U, 670U, 850U};

    fixture_init_with_generator(
        &fixture,
        0x001BU,
        fake_gait_interface(&generator));
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "delayed STOP setup should start Motion");
    (void)motion_manager_process(&fixture.manager, 0U);
    (void)motion_manager_process(&fixture.manager, 100U);
    expect(motion_manager_request_stop(&fixture.manager) ==
               MOTION_MANAGER_RESULT_OK,
           "delayed STOP should be accepted");

    for (size_t index = 0U;
         index < sizeof(stop_times) / sizeof(stop_times[0]);
         ++index)
    {
        safety_supervisor_on_heartbeat(
            &fixture.safety_supervisor,
            stop_times[index]);
        (void)motion_manager_process(&fixture.manager, stop_times[index]);
    }

    expect(motion_manager_stop_elapsed_ms(&fixture.manager) ==
               MOTION_TRANSITION_DURATION_MS,
           "graceful STOP should consume 750 ms of actual wall time");
    expect(motion_manager_state(&fixture.manager) == MOTION_STATE_STOPPED,
           "graceful STOP should complete after delayed foreground calls");
    expect_targets_zero(motion_manager_last_targets(&fixture.manager));
}

static void test_stop_elapsed_time_starts_at_acceptance(void)
{
    fixture_t fixture;

    fixture_init(&fixture, 0x001BU);
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "STOP acceptance timing setup should start Motion");
    (void)motion_manager_process(&fixture.manager, 0U);
    (void)motion_manager_process(&fixture.manager, 100U);

    expect(motion_manager_request_stop_at(&fixture.manager, 250U) ==
               MOTION_MANAGER_RESULT_OK,
           "STOP acceptance timing should accept a timestamped STOP");
    keep_host_alive(&fixture, 250U);
    (void)motion_manager_process(&fixture.manager, 250U);
    expect(motion_manager_stop_elapsed_ms(&fixture.manager) == 0U,
           "STOP should not consume pre-acceptance wall time");

    keep_host_alive(&fixture, 500U);
    (void)motion_manager_process(&fixture.manager, 500U);
    expect(motion_manager_stop_elapsed_ms(&fixture.manager) == 250U,
           "STOP ramp should measure from acceptance to the next delayed pass");

    keep_host_alive(&fixture, 1000U);
    (void)motion_manager_process(&fixture.manager, 1000U);
    expect(motion_manager_stop_elapsed_ms(&fixture.manager) ==
               MOTION_TRANSITION_DURATION_MS,
           "STOP ramp should complete 750 ms after acceptance");
    expect(motion_manager_state(&fixture.manager) == MOTION_STATE_STOPPED,
           "timestamped STOP should complete at its acceptance-based deadline");

    fixture_init(&fixture, 0x001BU);
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "pre-first-tick STOP timing setup should start Motion");
    expect(motion_manager_request_stop_at(&fixture.manager, 500U) ==
               MOTION_MANAGER_RESULT_OK,
           "STOP should be timestampable before the first Motion tick");
    keep_host_alive(&fixture, 500U);
    (void)motion_manager_process(&fixture.manager, 500U);
    expect(motion_manager_stop_elapsed_ms(&fixture.manager) == 0U,
           "pre-first-tick STOP should start its clock at acceptance");
    keep_host_alive(&fixture, 1250U);
    (void)motion_manager_process(&fixture.manager, 1250U);
    expect(motion_manager_stop_elapsed_ms(&fixture.manager) ==
               MOTION_TRANSITION_DURATION_MS,
           "pre-first-tick STOP should complete after 750 ms wall time");
}

static void test_common_motion_guard_clamps_alternate_generator_output(void)
{
    fixture_t fixture;
    fake_gait_generator_t generator = {
        .advanced_ms = 0U,
        .targets = {
            .front_right_cdeg = -5000,
            .front_left_cdeg = 4000,
            .front_axis_cdeg = 0,
            .rear_right_cdeg = -4000,
            .rear_left_cdeg = 5000,
        },
    };

    fixture_init_with_generator(
        &fixture,
        0x001BU,
        fake_gait_interface(&generator));
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "alternate-generator envelope setup should start Motion");
    (void)motion_manager_process(&fixture.manager, 0U);
    safety_supervisor_on_heartbeat(&fixture.safety_supervisor, 750U);
    (void)motion_manager_process(&fixture.manager, 750U);
    expect(motion_manager_last_targets(&fixture.manager)->front_right_cdeg ==
               MOTION_FRONT_MIN_CDEG,
            "common Motion output guard should clamp FrontRight to -4500");
    expect(motion_manager_last_targets(&fixture.manager)->front_left_cdeg ==
               MOTION_FRONT_MAX_CDEG,
            "common Motion output guard should clamp FrontLeft to +2800");
    expect(motion_manager_last_targets(&fixture.manager)->rear_right_cdeg ==
               MOTION_REAR_MIN_CDEG,
            "common Motion output guard should clamp RearRight to -3000");
    expect(motion_manager_last_targets(&fixture.manager)->rear_left_cdeg ==
               MOTION_REAR_MAX_CDEG,
            "common Motion output guard should clamp RearLeft to +4500");
    expect(motion_manager_operational_clamp_count(&fixture.manager) == 4U,
            "common Motion output guard should count each front and rear clamp");
}

static void test_simple_gait_production_pipeline_preserves_physical_directions(void)
{
    fixture_t fixture;
    const joint_targets_t *targets;

    fixture_init(&fixture, 0x001BU);
    fixture.generator.phase_rad = -MOTION_PI_F / 4.0F;
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "SimpleGait pipeline setup should start FORWARD");

    (void)motion_manager_process(&fixture.manager, 0U);
    keep_host_alive(&fixture, 750U);
    expect(motion_manager_process(&fixture.manager, 750U) ==
               MOTION_MANAGER_RESULT_OK,
           "SimpleGait quarter-cycle tick should be accepted");

    targets = motion_manager_last_targets(&fixture.manager);
    expect(targets->front_right_cdeg == 1000 &&
               targets->front_left_cdeg == 1000 &&
               targets->rear_right_cdeg == -1000 &&
               targets->rear_left_cdeg == -1000,
           "SimpleGait quarter-cycle must produce the logical anti-phase target pattern");
    expect(fixture.driver.current_pulse[SERVO_ID_FRONT_RIGHT] > 1580U,
           "FrontRight logical +10 degrees must map above neutral for backward stroke");
    expect(fixture.driver.current_pulse[SERVO_ID_FRONT_LEFT] < 1450U,
           "FrontLeft logical +10 degrees must map below neutral for backward stroke");
    expect(fixture.driver.current_pulse[SERVO_ID_REAR_RIGHT] < 1570U,
           "RearRight logical -10 degrees must map below neutral for recovery stroke");
    expect(fixture.driver.current_pulse[SERVO_ID_REAR_LEFT] > 1450U,
           "RearLeft logical -10 degrees must map above neutral for recovery stroke");

    safety_supervisor_on_heartbeat(&fixture.safety_supervisor, 1750U);
    expect(motion_manager_process(&fixture.manager, 1750U) ==
               MOTION_MANAGER_RESULT_OK,
           "SimpleGait half-cycle tick should be accepted");
    targets = motion_manager_last_targets(&fixture.manager);
    expect(targets->front_right_cdeg == -1000 &&
               targets->front_left_cdeg == -1000 &&
               targets->rear_right_cdeg == 1000 &&
               targets->rear_left_cdeg == 1000,
           "SimpleGait half-cycle must reverse every logical target sign");
    expect(fixture.driver.current_pulse[SERVO_ID_FRONT_RIGHT] < 1580U,
           "FrontRight half-cycle must reverse below neutral");
    expect(fixture.driver.current_pulse[SERVO_ID_FRONT_LEFT] > 1450U,
           "FrontLeft half-cycle must reverse above neutral");
    expect(fixture.driver.current_pulse[SERVO_ID_REAR_RIGHT] > 1570U,
           "RearRight half-cycle must reverse above neutral");
    expect(fixture.driver.current_pulse[SERVO_ID_REAR_LEFT] < 1450U,
           "RearLeft half-cycle must reverse below neutral");
}

static void test_stop_transition_reapplies_operational_sanitizer(void)
{
    fixture_t fixture;

    fixture_init(&fixture, 0x001BU);
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "STOP sanitizer setup should start FORWARD");
    (void)motion_manager_process(&fixture.manager, 0U);
    (void)motion_manager_process(&fixture.manager, 10U);

    /* Model a retained transition target from an alternate Motion path. */
    fixture.manager.last_targets.rear_right_cdeg = -4500;
    fixture.manager.last_targets.front_left_cdeg = 5000;
    expect(motion_manager_request_stop_at(&fixture.manager, 100U) ==
               MOTION_MANAGER_RESULT_OK,
           "STOP sanitizer setup should enter STOPPING");
    keep_host_alive(&fixture, 100U);
    (void)motion_manager_process(&fixture.manager, 110U);

    expect(motion_manager_state(&fixture.manager) == MOTION_STATE_STOPPING,
           "STOP sanitizer test should still be inside the provisional ramp");
    expect(motion_manager_last_targets(&fixture.manager)->rear_right_cdeg ==
               MOTION_REAR_MIN_CDEG,
           "STOPPING must clamp retained rear targets to the operational minimum");
    expect(motion_manager_last_targets(&fixture.manager)->front_left_cdeg ==
               MOTION_FRONT_MAX_CDEG,
           "STOPPING must clamp retained front targets to the operational maximum");
    expect(motion_manager_operational_clamp_count(&fixture.manager) == 2U,
               "STOPPING sanitizer should count front and rear operational clamps");
}

static void test_gait_backend_switch_is_stopped_only_and_has_no_output_side_effect(void)
{
    backend_fixture_t fixture;
    cpg_gait_generator_t fresh_cpg;
    joint_targets_t targets;
    unsigned int writes_before;
    cpg_gait_generator_t cpg_before_same_backend;

    backend_fixture_init(&fixture, MOTION_GAIT_BACKEND_CPG);
    writes_before = fixture.driver.write_calls;
    cpg_gait_generator_advance(&fixture.cpg_generator, 37U);
    expect(cpg_gait_generator_sample(
               &fixture.cpg_generator,
               MOTION_TURN_LEFT,
               1.0F,
               1.0F,
               &targets),
           "selector reset setup TURN sample should succeed");

    expect(motion_manager_gait_backend(&fixture.manager) ==
               MOTION_GAIT_BACKEND_CPG,
           "explicit production-style fixture should start in CPG");
    expect(motion_manager_set_gait_backend(
               &fixture.manager,
               MOTION_GAIT_BACKEND_SIMPLE_GAIT) ==
               MOTION_MANAGER_RESULT_OK,
           "STOPPED CPG to SimpleGait should succeed");
    expect(motion_manager_state(&fixture.manager) == MOTION_STATE_STOPPED,
           "successful selector must remain STOPPED");
    expect(!servo_service_motion_is_active(&fixture.servo_service),
           "successful selector must not start Motion");
    expect(fixture.driver.write_calls == writes_before,
           "successful selector must not write Servo output");
    expect(motion_manager_gait_backend(&fixture.manager) ==
               MOTION_GAIT_BACKEND_SIMPLE_GAIT,
           "successful selector must update the active backend");

    expect(motion_manager_set_gait_backend(
               &fixture.manager,
               MOTION_GAIT_BACKEND_SIMPLE_GAIT) ==
               MOTION_MANAGER_RESULT_OK,
           "STOPPED same-backend selection should be a deterministic no-op");
    expect(fixture.driver.write_calls == writes_before,
           "same-backend no-op must not write Servo output");

    expect(motion_manager_set_gait_backend(
               &fixture.manager,
               MOTION_GAIT_BACKEND_CPG) ==
               MOTION_MANAGER_RESULT_OK,
           "STOPPED SimpleGait to CPG should succeed");
    cpg_gait_generator_init(&fresh_cpg);
    expect(memcmp(&fixture.cpg_generator.core, &fresh_cpg.core,
                  sizeof(fresh_cpg.core)) == 0,
           "switching back to CPG must reset the runtime core");
    cpg_before_same_backend = fixture.cpg_generator;
    expect(motion_manager_set_gait_backend(
               &fixture.manager,
               MOTION_GAIT_BACKEND_CPG) ==
               MOTION_MANAGER_RESULT_OK,
           "reselecting CPG while STOPPED should be a no-op");
    expect(memcmp(&fixture.cpg_generator.core,
                  &cpg_before_same_backend.core,
                  sizeof(fixture.cpg_generator.core)) == 0,
           "same-backend CPG no-op must not reset or advance state");
    expect(fixture.driver.write_calls == writes_before,
           "all successful selector operations must preserve Servo output");
}

static void test_gait_backend_selector_busy_precedes_value_validation(void)
{
    backend_fixture_t fixture;
    const motion_state_t running_state = MOTION_STATE_RUNNING;
    const motion_gait_backend_t current_backend =
        MOTION_GAIT_BACKEND_CPG;
    unsigned int writes_before;
    joint_targets_t targets_before;
    cpg_core_t core_before;

    backend_fixture_init(&fixture, current_backend);
    expect(motion_manager_start(
               &fixture.manager,
               MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK,
           "selector BUSY setup should start Motion");
    (void)motion_manager_process(&fixture.manager, 0U);
    safety_supervisor_on_heartbeat(&fixture.safety_supervisor, 10U);
    (void)motion_manager_process(&fixture.manager, 10U);

    writes_before = fixture.driver.write_calls;
    targets_before = *motion_manager_last_targets(&fixture.manager);
    core_before = fixture.cpg_generator.core;
    expect(motion_manager_set_gait_backend(
               &fixture.manager,
               current_backend) == MOTION_MANAGER_RESULT_BUSY,
           "RUNNING same-backend selection must return BUSY");
    expect(motion_manager_set_gait_backend(
               &fixture.manager,
               (motion_gait_backend_t)0xffU) ==
               MOTION_MANAGER_RESULT_BUSY,
           "RUNNING invalid backend must return BUSY before validation");
    expect(motion_manager_state(&fixture.manager) == running_state &&
               motion_manager_gait_backend(&fixture.manager) == current_backend &&
               memcmp(motion_manager_last_targets(&fixture.manager),
                      &targets_before,
                      sizeof(targets_before)) == 0 &&
               memcmp(&fixture.cpg_generator.core,
                      &core_before,
                      sizeof(core_before)) == 0 &&
               fixture.driver.write_calls == writes_before,
           "RUNNING selector rejection must preserve Motion, generator, and output state");

    expect(motion_manager_request_stop_at(&fixture.manager, 10U) ==
               MOTION_MANAGER_RESULT_OK,
           "selector STOPPING setup should accept STOP");
    expect(motion_manager_state(&fixture.manager) == MOTION_STATE_STOPPING,
           "selector STOPPING setup should enter STOPPING");
    expect(motion_manager_set_gait_backend(
               &fixture.manager,
               MOTION_GAIT_BACKEND_SIMPLE_GAIT) ==
               MOTION_MANAGER_RESULT_BUSY,
           "STOPPING selector must return BUSY");
    expect(fixture.driver.write_calls == writes_before,
           "STOPPING selector rejection must not write Servo output");

    safety_supervisor_on_heartbeat(&fixture.safety_supervisor, 760U);
    (void)motion_manager_process(&fixture.manager, 760U);
    expect(motion_manager_state(&fixture.manager) == MOTION_STATE_STOPPED,
           "selector should become available after STOP completes");
    expect(motion_manager_set_gait_backend(
               &fixture.manager,
               MOTION_GAIT_BACKEND_SIMPLE_GAIT) ==
               MOTION_MANAGER_RESULT_OK,
           "selector should be allowed after STOPPED is restored");
}

static void test_legacy_and_missing_backend_registration_never_guess_or_mutate(void)
{
    fixture_t legacy;
    fake_gait_generator_t fake = {0};
    gait_generator_t active_before;
    joint_targets_t targets_before;
    unsigned int writes_before;

    fixture_init(&legacy, 0x001BU);
    active_before = legacy.manager.generator;
    targets_before = *motion_manager_last_targets(&legacy.manager);
    writes_before = legacy.driver.write_calls;
    expect(motion_manager_gait_backend(&legacy.manager) ==
               MOTION_GAIT_BACKEND_UNSPECIFIED,
           "legacy single-generator initialization must not infer backend identity");
    expect(motion_manager_set_gait_backend(
               &legacy.manager,
               MOTION_GAIT_BACKEND_CPG) ==
               MOTION_MANAGER_RESULT_HARDWARE_FAILURE,
           "legacy selector must reject an unregistered backend");
    expect(memcmp(&legacy.manager.generator, &active_before,
                  sizeof(active_before)) == 0 &&
               memcmp(motion_manager_last_targets(&legacy.manager),
                      &targets_before,
                      sizeof(targets_before)) == 0 &&
               legacy.driver.write_calls == writes_before,
           "legacy unavailable selector must not mutate active state or output");

    motion_manager_init_with_backends(
        &legacy.manager,
        &legacy.servo_service,
        &legacy.safety_supervisor,
        fake_gait_interface(&fake),
        fake_gait_interface(&fake),
        MOTION_GAIT_BACKEND_SIMPLE_GAIT);
    active_before = legacy.manager.generator;
    targets_before = *motion_manager_last_targets(&legacy.manager);
    writes_before = legacy.driver.write_calls;
    expect(motion_manager_set_gait_backend(
               &legacy.manager,
               MOTION_GAIT_BACKEND_CPG) ==
               MOTION_MANAGER_RESULT_HARDWARE_FAILURE,
           "missing registered reset infrastructure must return HardwareFailure");
    expect(memcmp(&legacy.manager.generator, &active_before,
                  sizeof(active_before)) == 0 &&
               memcmp(motion_manager_last_targets(&legacy.manager),
                      &targets_before,
                      sizeof(targets_before)) == 0 &&
               legacy.driver.write_calls == writes_before,
           "missing reset infrastructure must not mutate manager or output");
}

int main(void)
{
    test_stopped_running_and_start_gate();
    test_running_mode_change_preserves_phase();
    test_start_transition_rejects_different_mode_reentry();
    test_mode_transition_rejects_reentrant_mode_changes();
    test_graceful_stop_contract_and_monotonic_targets();
    test_stop_is_idempotent_when_stopped();
    test_watchdog_interrupts_graceful_stop_without_auto_resume();
    test_disable_all_preempts_stop_immediately();
    test_start_crossfades_from_recorded_manual_pose();
    test_motion_start_rejects_unknown_raw_pwm_pose();
    test_motion_start_rejects_front_pose_outside_operational_envelope();
    test_motion_start_rejects_rear_pose_outside_operational_envelope();
    test_motion_process_uses_actual_elapsed_wall_time();
    test_motion_process_elapsed_time_is_wrap_safe();
    test_graceful_stop_uses_actual_750_ms_duration();
    test_stop_elapsed_time_starts_at_acceptance();
    test_common_motion_guard_clamps_alternate_generator_output();
    test_simple_gait_production_pipeline_preserves_physical_directions();
    test_stop_transition_reapplies_operational_sanitizer();
    test_gait_backend_switch_is_stopped_only_and_has_no_output_side_effect();
    test_gait_backend_selector_busy_precedes_value_validation();
    test_legacy_and_missing_backend_registration_never_guess_or_mutate();

    if (failures == 0)
    {
        (void)puts("All firmware MotionManager tests passed");
    }

    return failures == 0 ? 0 : 1;
}
