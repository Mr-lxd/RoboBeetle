#include "robobeetle/gateway/motion_telemetry_fifo.hpp"
#include "robobeetle/protocol/motion_state.hpp"
#include <cstdlib>
#include <iostream>
using namespace robobeetle::gateway;
GatewayMotionStateTelemetry fragment(unsigned index) {
    robobeetle::protocol::MotionStateBatch batch;
    batch.batch_seq = static_cast<std::uint16_t>(index / 16);
    batch.fragment_index = index % 16;
    batch.fragment_count = 16;
    return {3, 1000 + index, 0, *robobeetle::protocol::encode_motion_state_batch(batch)};
}
int main() {
    MotionTelemetryFifo queue;
    for (unsigned i = 0; i < 64; ++i) {
        if (!queue.push(fragment(i))) return EXIT_FAILURE;
    }
    if (queue.push(fragment(64)) || queue.push(fragment(65)) || queue.drop_total() != 2)
        return EXIT_FAILURE;
    for (unsigned i = 0; i < 64; ++i) {
        const auto value = queue.pop();
        if (!value || value->pi_rx_ms != 1000 + i || value->link_epoch != 3 ||
            value->gateway_drop_total != 2 || value->batch_payload != fragment(i).batch_payload)
            return EXIT_FAILURE;
    }
    if (queue.pop() || !queue.empty()) return EXIT_FAILURE;
    std::cout << "PASS: 64 distinct fragments in FIFO order; newest overflow dropped; dequeue sees both drops\n";
    return EXIT_SUCCESS;
}
