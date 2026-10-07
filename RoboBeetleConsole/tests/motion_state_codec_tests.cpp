#include "protocol/MotionStateCodec.h"
#include "protocol/PacketCodec.h"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>

int main() {
    for (unsigned count = 0; count <= 2; ++count) {
        rb::MotionStateBatch batch{};
        batch.sample_count = count;
        batch.mcu_tx_ms = 12345;
        batch.samples[0].gyro_tenth_dps[2] = -256;
        batch.samples[0].phase_valid = true;
        const auto payload = rb::encodeMotionStateBatch(batch);
        assert(payload && payload->size() == 16 + 22 * count);
        const auto decoded = rb::decodeMotionStateBatch(*payload);
        assert(decoded && decoded->sample_count == count && decoded->mcu_tx_ms == 12345);
        if (count)
            assert(decoded->samples[0].gyro_tenth_dps[2] == -256 &&
                   decoded->samples[0].phase_valid);
        const auto wire =
            rb::PacketCodec::encodeWire({rb::MessageType::MotionStateBatch, 7, *payload});
        assert(wire.size() == 28 + 22 * count);
        assert(rb::PacketCodec::decodeWire(wire.first(wire.size() - 1)).ok());
        const auto telemetry = robobeetle::gateway::encode_motion_state_telemetry(
            {1, 123456789, 0, rb::motionStateBytes(*payload)});
        assert(telemetry);
        const auto qtelemetry =
            QByteArray(reinterpret_cast<const char *>(telemetry->data()), telemetry->size());
        assert(rb::decodeMotionStateTelemetry(qtelemetry)->pi_rx_ms == 123456789);
    }
    std::cout << "Qt motion state codec: 0/1/2 sample wrappers and P2 framing passed\n";
}
