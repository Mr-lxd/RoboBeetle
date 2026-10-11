#include "robobeetle/gateway/rbrp_codec.hpp"
#include "robobeetle/protocol/codec.hpp"
#include "robobeetle/protocol/message_types.hpp"
#include "robobeetle/protocol/motion_state.hpp"
extern "C" {
#include "motion_state_codec.h"
#include "rb_protocol_v2.h"
}
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>

int main() {
    using namespace robobeetle;
    for (unsigned count = 0; count <= 2; ++count) {
        protocol::MotionStateBatch batch{};
        batch.sample_count = count;
        batch.batch_seq = 0x1234;
        batch.fragment_count = 1;
        batch.mcu_tx_ms = 0x12345678;
        batch.sampler_drop_total = 9;
        rb_motion_state_batch_t c_batch{};
        c_batch.sample_count = count;
        c_batch.batch_seq = batch.batch_seq;
        c_batch.fragment_count = 1;
        c_batch.mcu_tx_ms = batch.mcu_tx_ms;
        c_batch.sampler_drop_total = 9;
        for (unsigned i = 0; i < count; ++i) {
            auto &s = batch.samples[i];
            s.mcu_ms = 0x10203040 + i;
            s.gyro_tenth_dps = {{-32768, 12, 32767}};
            s.roll_centidegrees = -1234;
            s.pitch_centidegrees = 5678;
            s.phase_u16 = 65535;
            s.active_mode = 6;
            s.target_mode = 3;
            s.coordination = 1;
            s.backend = 2;
            s.state = 3;
            s.transition = s.phase_valid = s.gyro_valid = s.angle_valid = true;
            s.angle_age_ms = 65534;
            s.phase_age_ms = 65535;
            auto &c = c_batch.samples[i];
            c.mcu_ms = s.mcu_ms;
            for (unsigned axis = 0; axis < 3; ++axis)
                c.gyro_tenth_dps[axis] = s.gyro_tenth_dps[axis];
            c.roll_centidegrees = s.roll_centidegrees;
            c.pitch_centidegrees = s.pitch_centidegrees;
            c.phase_u16 = s.phase_u16;
            c.active_mode = 6;
            c.target_mode = 3;
            c.coordination = 1;
            c.backend = 2;
            c.state = 3;
            c.transition = c.phase_valid = c.gyro_valid = c.angle_valid = true;
            c.angle_age_ms = 65534;
            c.phase_age_ms = 65535;
        }
        const auto encoded = protocol::encode_motion_state_batch(batch);
        assert(encoded && encoded->size() == 16 + 22 * count);
        assert((*encoded)[6] == 0 && (*encoded)[7] == 0 && (*encoded)[8] == 0x78);
        uint8_t c_payload[60]{};
        assert(rb_motion_state_encode(&c_batch, c_payload, sizeof c_payload) == encoded->size());
        assert(protocol::Bytes(c_payload, c_payload + encoded->size()) == *encoded);
        rb_motion_state_batch_t c_decoded{};
        assert(rb_motion_state_decode(encoded->data(), encoded->size(), &c_decoded));
        const auto decoded = protocol::decode_motion_state_batch(*encoded);
        assert(decoded && decoded->sample_count == count && decoded->mcu_tx_ms == batch.mcu_tx_ms);
        if (count) {
            assert((*encoded)[32] == 0x5e && (*encoded)[33] == 0xfe);
            const auto &s = decoded->samples[0];
            assert(s.gyro_tenth_dps[0] == -32768 && s.roll_centidegrees == -1234);
            assert(s.active_mode == 6 && s.target_mode == 3 && s.coordination == 1);
            assert(s.backend == 2 && s.state == 3 && s.phase_valid && s.transition &&
                   s.gyro_valid && s.angle_valid);
            assert(c_decoded.samples[0].gyro_tenth_dps[0] == -32768 &&
                   c_decoded.samples[0].phase_valid);
        }
        protocol::Frame frame{};
        frame.message_type = static_cast<protocol::Byte>(protocol::MessageType::MotionStateBatch);
        frame.payload = *encoded;
        auto wire = protocol::Codec::encodeWire(frame);
        assert(wire.size() == 28 + 22 * count);
        uint8_t c_wire[76]{};
        assert(rbp2_encode_wire(RBP2_MSG_MOTION_STATE_BATCH, 0, c_payload, encoded->size(), c_wire,
                                sizeof c_wire) == wire.size());
        assert(protocol::Bytes(c_wire, c_wire + wire.size()) == wire);
        wire.pop_back();
        assert(protocol::Codec::decodeWire(wire).ok());
        gateway::GatewayMotionStateTelemetry telemetry{4, 0x0102030405060708ULL, 7, *encoded};
        const auto rbrp_payload = gateway::encode_motion_state_telemetry(telemetry);
        assert(rbrp_payload && rbrp_payload->size() == 32 + 22 * count);
        assert((*rbrp_payload)[0] == 4 && (*rbrp_payload)[4] == 8 && (*rbrp_payload)[11] == 1 &&
               (*rbrp_payload)[12] == 7);
        assert(protocol::Bytes(rbrp_payload->begin() + 16, rbrp_payload->end()) == *encoded);
        const auto rbrp_decoded = gateway::decode_motion_state_telemetry(*rbrp_payload);
        assert(rbrp_decoded && rbrp_decoded->pi_rx_ms == telemetry.pi_rx_ms &&
               rbrp_decoded->batch_payload == *encoded);
        const auto framed = gateway::encode_gateway_message({0, telemetry});
        assert(framed.status == gateway::RbrpEncodeStatus::Ok);
        gateway::RbrpDecoder decoder;
        std::vector<gateway::RbrpFrame> frames;
        assert(decoder.feed(framed.wire.data(), framed.wire.size(), frames) ==
               gateway::RbrpFeedStatus::Ok);
        assert(frames.size() == 1 &&
               frames[0].kind == gateway::RbrpMessageKind::MotionStateTelemetry);
    }
    const protocol::Bytes golden2{2,    1,    0x34, 0x12, 0,    1,    0,    0,    0x78, 0x56, 0x34,
                                  0x12, 9,    0,    0,    0,    4,    3,    2,    1,    0xfe, 0xff,
                                  3,    0,    0xfc, 0xff, 5,    0,    0xfa, 0xff, 0,    0x80, 0x4b,
                                  0xff, 7,    0,    8,    0,    0,    1,    0x67, 0x45, 0,    0,
                                  0,    0x11, 0x11, 0x22, 0x22, 0x33, 0x33};
    const auto v2 = protocol::decode_motion_state_batch(golden2);
    assert(v2 && v2->schema_version == 2 && v2->samples[0].cpg_param_version == 0x4567);
    assert(*protocol::encode_motion_state_batch(*v2) == golden2);
    rb_motion_state_batch_t c_v2{};
    assert(rb_motion_state_decode(golden2.data(), golden2.size(), &c_v2));
    std::uint8_t c_payload[60]{};
    assert(rb_motion_state_encode(&c_v2, c_payload, sizeof c_payload) == golden2.size());
    assert(protocol::Bytes(c_payload, c_payload + golden2.size()) == golden2);
    const auto wrapped = gateway::encode_motion_state_telemetry({1, 123456, 0, golden2});
    assert(wrapped && wrapped->size() == 67);
    assert(gateway::decode_motion_state_telemetry(*wrapped)->batch_payload == golden2);
    std::cout << "motion state codec: v1/v2 C/Pi goldens, bitfields and RBRP prefix "
                 "passed\n";
}
