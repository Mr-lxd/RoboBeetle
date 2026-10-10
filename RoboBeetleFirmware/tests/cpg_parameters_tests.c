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
#include <math.h>
#include "cpg_parameters.h"

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


int main(void) {
    cpg_parameters_t a,b; uint8_t wire[58]; cpg_parameters_default(&a);
    assert(a.front_amp==10 && a.rear_amp==10 && a.period==2.5162 && a.beta==.75 && a.mask==0x3c && a.w==2);
    assert(cpg_parameters_encode(&a,wire,sizeof wire)==58);
    assert(cpg_parameters_decode(wire,58,&b)==CPG_PARAMETERS_OK);
    assert(cpg_parameters_equal(&a,&b));
    assert(cpg_parameters_decode(wire,57,&b)==CPG_PARAMETERS_INVALID_PAYLOAD);
    wire[0]=2; assert(cpg_parameters_decode(wire,58,&b)==CPG_PARAMETERS_INVALID_PAYLOAD); wire[0]=1;
    wire[1]=64; assert(cpg_parameters_decode(wire,58,&b)==CPG_PARAMETERS_INVALID_PAYLOAD);
    const double lo[]={0,0,.5,.1,-180,-180,0}, hi[]={28,30,10,.9,180,180,5};
    for(unsigned field=0;field<7;++field) {
        double *v[]={&b.front_amp,&b.rear_amp,&b.period,&b.beta,&b.F,&b.L,&b.w};
        b=a; *v[field]=lo[field]; assert(cpg_parameters_validate(&b)==CPG_PARAMETERS_OK);
        *v[field]=hi[field]; assert(cpg_parameters_validate(&b)==CPG_PARAMETERS_OK);
        *v[field]=lo[field]-1; assert(cpg_parameters_validate(&b)==CPG_PARAMETERS_OUT_OF_RANGE);
        *v[field]=hi[field]+1; assert(cpg_parameters_validate(&b)==CPG_PARAMETERS_OUT_OF_RANGE);
        *v[field]=NAN; assert(cpg_parameters_validate(&b)==CPG_PARAMETERS_OUT_OF_RANGE);
        *v[field]=INFINITY; assert(cpg_parameters_validate(&b)==CPG_PARAMETERS_OUT_OF_RANGE);
    }
    fixture_t x; fixture_init(&x); x.manager.cpg_generator=&x.generator;
    const joint_targets_t targets=x.manager.last_targets;
    a.F=60; a.L=-45; a.mask=63;
    assert(motion_manager_set_cpg_parameters(&x.manager,&a)==MOTION_MANAGER_RESULT_OK);
    assert(x.manager.cpg_parameter_version==1);
    assert(memcmp(&targets,&x.manager.last_targets,sizeof targets)==0);
    assert(x.generator.profile.front_amplitude_deg==10);
    const double rad=3.14159265358979323846/180;
    const double theta[]={-45,15,60,0};
    const unsigned from[]={0,0,0,1,1,1,2,2,2,3,3,3}, to[]={1,2,3,0,2,3,0,1,3,0,1,2};
    for(unsigned i=0;i<4;++i) { assert(fabs(x.generator.core.phase[i]-theta[i]*rad)<1e-14); assert(x.generator.core.params.target_amplitude[i]==((i==0||i==3)?-10:10)); }
    for(unsigned i=0;i<12;++i) { assert(fabs(x.generator.core.params.desired_phase[i]-(theta[to[i]]-theta[from[i]])*rad)<1e-14); assert(x.generator.core.phase_target[i]==x.generator.core.params.desired_phase[i]); assert(x.generator.core.params.coupling_weight[i]==2); assert(x.generator.core.params.phase_target_gain[i]==20); }
    cpg_gait_generator_reset(&x.generator); assert(fabs(x.generator.core.phase[0]+45*rad)<1e-14);
    assert(motion_manager_set_cpg_parameters(&x.manager,&a)==MOTION_MANAGER_RESULT_OK); assert(x.manager.cpg_parameter_version==1);
    for(unsigned state=1;state<4;++state) {
        x.manager.state=(motion_state_t)state; b=a; b.front_amp=20;
        assert(motion_manager_set_cpg_parameters(&x.manager,&b)==(state==3?MOTION_MANAGER_RESULT_INVALID_STATE:MOTION_MANAGER_RESULT_BUSY));
        assert(x.generator.profile.front_amplitude_deg==10 && x.manager.cpg_parameter_version==1);
    }
    x.manager.state=MOTION_STATE_STOPPED; b=a; b.beta=NAN;
    assert(motion_manager_set_cpg_parameters(&x.manager,&b)==MOTION_MANAGER_RESULT_OUT_OF_RANGE);
    x.manager.cpg_parameter_version=65535; b=a; b.front_amp=20;
    assert(motion_manager_set_cpg_parameters(&x.manager,&b)==MOTION_MANAGER_RESULT_OK); assert(x.manager.cpg_parameter_version==0);
    fixture_init(&x); assert(x.generator.profile.front_amplitude_deg==10 && x.manager.cpg_parameter_version==0);
    return 0;
}
