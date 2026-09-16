#include "robobeetle/transport/frame_tx_queue.hpp"

#include <utility>

namespace robobeetle::transport {

FrameTxQueue::FrameTxQueue(std::size_t max_frames, std::size_t max_bytes)
    : max_frames_(max_frames), max_bytes_(max_bytes)
{
}

FrameTxQueue::FrameTxQueue(FrameTxQueue &&other)
    : max_frames_(other.max_frames_),
      max_bytes_(other.max_bytes_),
      frames_(std::move(other.frames_)),
      front_offset_(other.front_offset_),
      owned_bytes_(other.owned_bytes_)
{
    other.clear();
}

FrameTxQueue &FrameTxQueue::operator=(FrameTxQueue &&other) noexcept
{
    if (this != &other) {
        max_frames_ = other.max_frames_;
        max_bytes_ = other.max_bytes_;
        frames_ = std::move(other.frames_);
        front_offset_ = other.front_offset_;
        owned_bytes_ = other.owned_bytes_;
        other.clear();
    }
    return *this;
}

bool FrameTxQueue::try_accept(const protocol::Bytes &wire_frame)
{
    if (frames_.size() >= max_frames_ ||
        wire_frame.size() > max_bytes_ - owned_bytes_) {
        return false;
    }

    frames_.push_back(wire_frame);
    owned_bytes_ += wire_frame.size();
    return true;
}

const protocol::Bytes *FrameTxQueue::front_frame() const
{
    return frames_.empty() ? nullptr : &frames_.front();
}

std::size_t FrameTxQueue::front_offset() const
{
    return frames_.empty() ? 0U : front_offset_;
}

std::size_t FrameTxQueue::front_remaining() const
{
    if (frames_.empty()) {
        return 0U;
    }
    return frames_.front().size() - front_offset_;
}

std::size_t FrameTxQueue::owned_frames() const
{
    return frames_.size();
}

std::size_t FrameTxQueue::owned_bytes() const
{
    return owned_bytes_;
}

bool FrameTxQueue::consume_front(std::size_t byte_count)
{
    if (frames_.empty() || byte_count > front_remaining()) {
        return false;
    }

    front_offset_ += byte_count;
    owned_bytes_ -= byte_count;
    if (front_offset_ == frames_.front().size()) {
        frames_.pop_front();
        front_offset_ = 0U;
    }
    return true;
}

void FrameTxQueue::clear()
{
    frames_.clear();
    front_offset_ = 0U;
    owned_bytes_ = 0U;
}

} // namespace robobeetle::transport
