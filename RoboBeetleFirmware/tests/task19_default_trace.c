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

static void put32(FILE *f, uint32_t v)
{
    for (unsigned i = 0; i < 4; ++i)
        fputc((v >> (8 * i)) & 255, f);
}
static void record(FILE *f, const motion_manager_t *m)
{
    put32(f, (uint32_t)m->last_targets.front_left_cdeg);
    put32(f, (uint32_t)m->last_targets.front_right_cdeg);
    put32(f, (uint32_t)m->last_targets.rear_left_cdeg);
    put32(f, (uint32_t)m->last_targets.rear_right_cdeg);
    put32(f, (uint32_t)m->last_targets.front_axis_cdeg);
    fputc(m->state, f);
    fputc(m->active_mode, f);
    fputc(m->transition_mode, f);
    fputc(m->write_mask & 255, f);
    fputc(m->write_mask >> 8, f);
}
static void run_ticks(fixture_t *x, FILE *f, uint32_t *now, uint32_t duration)
{
    for (uint32_t t = 0; t < duration; t += 10)
    {
        safety_supervisor_on_heartbeat(&x->safety_supervisor, *now);
        assert(motion_manager_process(&x->manager, *now) == MOTION_MANAGER_RESULT_OK);
        record(f, &x->manager);
        *now += 10;
    }
}
int main(int argc, char **argv)
{
    assert(argc == 3);
    fixture_t x;
    fixture_init(&x);
    x.manager.front_rear_coordination = (motion_front_rear_coordination_t)(argv[2][0] - '0');
#ifdef TASK19_APPLY_DEFAULT
    x.manager.cpg_generator = &x.generator;
    cpg_parameters_t defaults;
    cpg_parameters_default(&defaults);
    assert(motion_manager_set_cpg_parameters(&x.manager, &defaults) == MOTION_MANAGER_RESULT_OK);
#endif
    FILE *f = fopen(argv[1], "wb");
    assert(f);
    uint32_t now = 0;
    run_ticks(&x, f, &now, 20);
    assert(motion_manager_start(&x.manager, MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK);
    run_ticks(&x, f, &now, 6750);
    const motion_mode_t modes[] = {MOTION_TURN_LEFT, MOTION_TURN_RIGHT, MOTION_ASCEND,
                                   MOTION_DESCEND, MOTION_FORWARD};
    for (unsigned i = 0; i < 5; i++)
    {
        assert(motion_manager_start(&x.manager, modes[i]) == MOTION_MANAGER_RESULT_OK);
        run_ticks(&x, f, &now, 2000);
    }
    assert(motion_manager_request_stop_at(&x.manager, now) == MOTION_MANAGER_RESULT_OK);
    run_ticks(&x, f, &now, 780);
    assert(motion_manager_start(&x.manager, MOTION_FORWARD) == MOTION_MANAGER_RESULT_OK);
    run_ticks(&x, f, &now, 2750);
    assert(motion_manager_request_stop_at(&x.manager, now) == MOTION_MANAGER_RESULT_OK);
    run_ticks(&x, f, &now, 780);
    assert(fclose(f) == 0);
    return 0;
}
