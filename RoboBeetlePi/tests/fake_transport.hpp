#pragma once

#include "robobeetle/transport/transport.hpp"

#include <vector>

namespace rbp2_test {

class FakeTransport final : public robobeetle::transport::Transport {
public:
    bool write(const robobeetle::protocol::Bytes &wire_bytes) override
    {
        if (!write_succeeds_) {
            return false;
        }
        writes_.push_back(wire_bytes);
        return true;
    }

    void set_write_succeeds(bool succeeds) { write_succeeds_ = succeeds; }

    [[nodiscard]] const std::vector<robobeetle::protocol::Bytes> &writes() const
    {
        return writes_;
    }

    void clear() { writes_.clear(); }

private:
    std::vector<robobeetle::protocol::Bytes> writes_;
    bool write_succeeds_{true};
};

} // namespace rbp2_test
