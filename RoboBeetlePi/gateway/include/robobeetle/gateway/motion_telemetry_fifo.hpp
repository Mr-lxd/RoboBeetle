#pragma once
#include "robobeetle/gateway/gateway_types.hpp"
#include <array>
#include <optional>
#include <utility>

namespace robobeetle::gateway {
// One instance per TCP client. Accepted fragments are never replaced.
// The TCP adapter owns synchronization and the partially transmitted frame.
class MotionTelemetryFifo {
public:
    static constexpr std::size_t capacity = 64;
    bool push(GatewayMotionStateTelemetry value) {
        if (size_ == capacity) { ++drop_total_; return false; }
        entries_[(head_ + size_) % capacity] = std::move(value);
        ++size_;
        return true;
    }
    std::optional<GatewayMotionStateTelemetry> pop() {
        if (empty()) return std::nullopt;
        auto value = std::move(entries_[head_]);
        entries_[head_].reset();
        head_ = (head_ + 1) % capacity;
        --size_;
        value->gateway_drop_total = drop_total_;
        return value;
    }
    bool empty() const noexcept { return size_ == 0; }
    std::uint32_t drop_total() const noexcept { return drop_total_; }
private:
    std::array<std::optional<GatewayMotionStateTelemetry>, capacity> entries_{};
    std::size_t head_{0}, size_{0};
    std::uint32_t drop_total_{0};
};
}
