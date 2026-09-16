#pragma once

#include "robobeetle/protocol/frame.hpp"

#include <cstddef>
#include <deque>

namespace robobeetle::transport {

class FrameTxQueue {
public:
    FrameTxQueue(std::size_t max_frames, std::size_t max_bytes);
    FrameTxQueue(const FrameTxQueue &) = default;
    FrameTxQueue &operator=(const FrameTxQueue &) = default;
    FrameTxQueue(FrameTxQueue &&other);
    FrameTxQueue &operator=(FrameTxQueue &&other) noexcept;

    bool try_accept(const protocol::Bytes &wire_frame);

    // The pointer is invalidated when the front boundary is consumed or clear()
    // is called.
    const protocol::Bytes *front_frame() const;
    std::size_t front_offset() const;
    std::size_t front_remaining() const;

    std::size_t owned_frames() const;
    // Bytes still pending physical transmission, including a partial front.
    std::size_t owned_bytes() const;

    bool consume_front(std::size_t byte_count);
    // Empty accessors are deterministic: null, zero, and false as applicable.
    void clear();

private:
    std::size_t max_frames_;
    std::size_t max_bytes_;
    std::deque<protocol::Bytes> frames_;
    std::size_t front_offset_{0U};
    std::size_t owned_bytes_{0U};
};

} // namespace robobeetle::transport
