#include "robobeetle/protocol/cpg_parameters.hpp"
#include "robobeetle/protocol/motion_state.hpp"
#include "robobeetle/protocol/proportional_control.hpp"
#include "robobeetle/gateway/rbrp_codec.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>
int main()
{
    using namespace robobeetle::protocol;
    const ProportionalStart start{{1000, 1500, 1234, 2000}, 0xab};
    const Bytes start_golden{0xe8, 0x03, 0xdc, 0x05, 0xd2, 0x04, 0xd0, 0x07, 0xab};
    assert(encode_proportional_start(start) == start_golden);
    const auto start_read = decode_proportional_start(start_golden);
    assert(start_read && start_read->session_id == 0xab && start_read->config.turn_gain == 1500);
    const ProportionalSetpoint input{0xab, 0xfffe, 750, -500, 1000};
    const Bytes stream_golden{0xab, 0xfe, 0xff, 0xee, 0x02, 0x0c, 0xfe, 0xe8, 0x03};
    assert(encode_proportional_setpoint(input) == stream_golden);
    const auto stream_read = decode_proportional_setpoint(stream_golden);
    assert(stream_read && stream_read->sequence == 0xfffe && stream_read->turn == -500);
    using namespace robobeetle::gateway;
    Bytes start_command{static_cast<Byte>(RobotCommandKind::StartProportional)};
    start_command.insert(start_command.end(), start_golden.begin(), start_golden.end());
    RbrpFrame start_frame{RbrpMessageKind::CommandRequest, 42, start_command};
    const auto request = decode_remote_message(start_frame);
    assert(request.message && std::get<CommandRequest>(request.message->payload).command);
    RbrpFrame stream_frame{RbrpMessageKind::ProportionalInput, 0, stream_golden};
    assert(decode_remote_message(stream_frame).message);
    stream_frame.request_id = 42;
    assert(!decode_remote_message(stream_frame).message);
    const CpgParameters defaults{};
    const auto p = encode_cpg_parameters(defaults);
    assert(p && p->size() == 58 && (*p)[0] == 1 && (*p)[1] == 0x3c);
    const Bytes period{0xc5, 0x8f, 0x31, 0x77, 0x2d, 0x21, 0x04, 0x40};
    assert(Bytes(p->begin() + 18, p->begin() + 26) == period);
    const auto decoded = decode_cpg_parameters(*p);
    assert(decoded && *decoded == defaults);
    MotionStateBatch b{};
    b.schema_version = 2;
    b.sample_count = 1;
    b.samples[0].cpg_param_version = 0x1234;
    b.samples[0].phase_fr_u16 = 0x5678;
    b.samples[0].phase_rr_u16 = 0x9abc;
    b.samples[0].phase_rl_u16 = 0xdef0;
    const auto wire = encode_motion_state_batch(b);
    assert(wire && wire->size() == 51 && (*wire)[0] == 2);
    const Bytes extension{0, 0, 0x34, 0x12, 0, 0, 0, 0x78, 0x56, 0xbc, 0x9a, 0xf0, 0xde};
    assert(Bytes(wire->begin() + 38, wire->end()) == extension);
    const auto r = decode_motion_state_batch(*wire);
    assert(r && r->samples[0].cpg_param_version == 0x1234 && r->samples[0].phase_rl_u16 == 0xdef0);
    std::cout << "CPG binary64 and motion schema2 golden vectors passed\n";
}
