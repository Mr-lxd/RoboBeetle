#pragma once

#include <cstddef>
#include <cstdint>

namespace robobeetle::link_core {

struct LinkCoreConfig {
    std::uint64_t heartbeat_interval_ms{100U};
    std::uint64_t ack_timeout_ms{200U};
    std::uint64_t liveness_timeout_ms{450U};
    std::uint16_t initial_sequence{0U};
    std::size_t ordinary_queue_capacity{8U};
    std::size_t heartbeat_history_capacity{8U};
    std::size_t correlation_history_capacity{32U};
};

} // namespace robobeetle::link_core
