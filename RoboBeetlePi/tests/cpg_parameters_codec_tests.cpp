#include "robobeetle/protocol/cpg_parameters.hpp"
#include "robobeetle/protocol/motion_state.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>
int main() {
    using namespace robobeetle::protocol;
    const CpgParameters defaults{};
    const auto p = encode_cpg_parameters(defaults);
    assert(p && p->size() == 58 && (*p)[0] == 1 && (*p)[1] == 0x3c);
    const Bytes period{0xc5,0x8f,0x31,0x77,0x2d,0x21,0x04,0x40};
    assert(Bytes(p->begin()+18,p->begin()+26) == period);
    const auto decoded=decode_cpg_parameters(*p);
    assert(decoded && *decoded == defaults);
    MotionStateBatch b{}; b.schema_version=2; b.sample_count=1;
    b.samples[0].cpg_param_version=0x1234;
    b.samples[0].phase_fr_u16=0x5678;
    b.samples[0].phase_rr_u16=0x9abc;
    b.samples[0].phase_rl_u16=0xdef0;
    const auto wire=encode_motion_state_batch(b);
    assert(wire && wire->size()==51 && (*wire)[0]==2);
    const Bytes extension{0,0,0x34,0x12,0,0,0,0x78,0x56,0xbc,0x9a,0xf0,0xde};
    assert(Bytes(wire->begin()+38,wire->end())==extension);
    const auto r=decode_motion_state_batch(*wire);
    assert(r && r->samples[0].cpg_param_version==0x1234 && r->samples[0].phase_rl_u16==0xdef0);
    std::cout << "CPG binary64 and motion schema2 golden vectors passed\n";
}
