#include "motion_config.h"
#include "motion_manager.h"
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
    phase_before_change = simple_gait_generator_phase(&fixture.generator);

    expect(motion_manager_start(
               &fixture.manager,
               MOTION_TURN_LEFT) == MOTION_MANAGER_RESULT_OK,
           "running Motion should accept a mode change");
    expect(motion_manager_mode(&fixture.manager) == MOTION_FORWARD,
           "cross-fade should retain the active mode until its deadline");
    (void)motion_manager_process(&fixture.manager, 20U);
    expect(simple_gait_generator_phase(&fixture.generator) >
               phase_before_change,
           "mode change should not reset the generator phase");
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
               3000) == SERVO_SERVICE_RESULT_OK,
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
    expect(motion_manager_last_targets(&fixture.manager)->front_right_cdeg == 3000 &&
               motion_manager_last_targets(&fixture.manager)->front_left_cdeg == -2500,
           "Motion START should retain the recorded logical pose before its first tick");

    (void)motion_manager_process(&fixture.manager, 0U);
    (void)motion_manager_process(&fixture.manager, 10U);
    expect(motion_manager_last_targets(&fixture.manager)->front_right_cdeg == 2974,
           "FrontRight should cross-fade from its manual logical pose");
    expect(motion_manager_last_targets(&fixture.manager)->front_left_cdeg == -2480,
           "mirrored FrontLeft should cross-fade from its manual logical pose");
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
            .front_right_cdeg = 0,
            .front_left_cdeg = 0,
            .front_axis_cdeg = 0,
            .rear_right_cdeg = -4000,
            .rear_left_cdeg = 0,
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
    expect(motion_manager_last_targets(&fixture.manager)->rear_right_cdeg ==
               MOTION_REAR_MIN_CDEG,
           "common Motion output guard should clamp alternate rear output to -3000");
    expect(motion_manager_operational_clamp_count(&fixture.manager) == 1U,
           "common Motion output guard should own its clamp diagnostic");
}

int main(void)
{
    test_stopped_running_and_start_gate();
    test_running_mode_change_preserves_phase();
    test_graceful_stop_contract_and_monotonic_targets();
    test_stop_is_idempotent_when_stopped();
    test_watchdog_interrupts_graceful_stop_without_auto_resume();
    test_disable_all_preempts_stop_immediately();
    test_start_crossfades_from_recorded_manual_pose();
    test_motion_start_rejects_unknown_raw_pwm_pose();
    test_motion_process_uses_actual_elapsed_wall_time();
    test_motion_process_elapsed_time_is_wrap_safe();
    test_graceful_stop_uses_actual_750_ms_duration();
    test_stop_elapsed_time_starts_at_acceptance();
    test_common_motion_guard_clamps_alternate_generator_output();

    if (failures == 0)
    {
        (void)puts("All firmware MotionManager tests passed");
    }

    return failures == 0 ? 0 : 1;
}
