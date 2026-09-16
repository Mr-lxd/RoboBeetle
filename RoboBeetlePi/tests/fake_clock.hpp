#pragma once

#include "robobeetle/link_core/link_core.hpp"

namespace rbp2_test {

class FakeClock final {
public:
    explicit FakeClock(robobeetle::link_core::TimeMs now_ms = 0U)
        : now_ms_(now_ms)
    {
    }

    void set(robobeetle::link_core::TimeMs now_ms) { now_ms_ = now_ms; }

    void advance(robobeetle::link_core::TimeMs delta_ms)
    {
        now_ms_ += delta_ms;
    }

    [[nodiscard]] robobeetle::link_core::TimeMs now_ms() const
    {
        return now_ms_;
    }

private:
    robobeetle::link_core::TimeMs now_ms_;
};

} // namespace rbp2_test
