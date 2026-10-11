#include "motion_state_codec.h"
#include <assert.h>
#include <string.h>
int main(void) {
    const uint8_t golden1[]={1,1,0x34,0x12,0,1,0,0,0x78,0x56,0x34,0x12,9,0,0,0,
        4,3,2,1,0xfe,0xff,3,0,0xfc,0xff,5,0,0xfa,0xff,0,0x80,0x4b,0xff,7,0,8,0};
    const uint8_t golden2[]={2,1,0x34,0x12,0,1,0,0,0x78,0x56,0x34,0x12,9,0,0,0,
        4,3,2,1,0xfe,0xff,3,0,0xfc,0xff,5,0,0xfa,0xff,0,0x80,0x4b,0xff,7,0,8,0,
        0,1,0x67,0x45,0,0,0,0x11,0x11,0x22,0x22,0x33,0x33};
    rb_motion_state_batch_t b={0},decoded; uint8_t wire[64];
    assert(rb_motion_state_decode(golden1,sizeof golden1,&b));
    assert(rb_motion_state_encode(&b,wire,64)==sizeof golden1);
    assert(memcmp(wire,golden1,sizeof golden1)==0);
    b.schema=2; b.samples[0].stop_reason=1; b.samples[0].parameter_version=0x4567;
    b.samples[0].fr_phase_u16=0x1111; b.samples[0].rr_phase_u16=0x2222; b.samples[0].rl_phase_u16=0x3333;
    assert(rb_motion_state_encode(&b,wire,64)==sizeof golden2);
    assert(memcmp(wire,golden2,sizeof golden2)==0);
    assert(rb_motion_state_decode(golden2,sizeof golden2,&decoded));
    assert(decoded.schema==2 && decoded.samples[0].parameter_version==0x4567);
    b.sample_count=0; b.fragment_count=32; b.fragment_index=31;
    assert(rb_motion_state_encode(&b,wire,64)==16);
    assert(rb_motion_state_decode(wire,16,&decoded));
    b.sample_count=2; assert(rb_motion_state_encode(&b,wire,64)==0);
    b.sample_count=1; b.fragment_count=33; assert(rb_motion_state_encode(&b,wire,64)==0);
    return 0;
}
