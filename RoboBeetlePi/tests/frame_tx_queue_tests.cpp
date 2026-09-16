#include "test_support.hpp"

#include "robobeetle/protocol/frame.hpp"
#include "robobeetle/transport/frame_tx_queue.hpp"

#include <utility>

namespace rbp2_test {
namespace {

using robobeetle::protocol::Bytes;
using robobeetle::transport::FrameTxQueue;

Bytes frame(std::uint8_t seed, std::size_t size)
{
    return Bytes(size, seed);
}

void move_assign(FrameTxQueue &destination, FrameTxQueue &source)
{
    destination = std::move(source);
}

void expect_empty(const FrameTxQueue &queue, const char *message)
{
    expect(queue.owned_frames() == 0U, message);
    expect(queue.owned_bytes() == 0U, message);
    expect(queue.front_frame() == nullptr, message);
    expect(queue.front_offset() == 0U, message);
    expect(queue.front_remaining() == 0U, message);
}

} // namespace

void test_frame_tx_queue_contract()
{
    const Bytes a = frame(0xA1U, 3U);
    const Bytes b = frame(0xB2U, 2U);
    const Bytes c = frame(0xC3U, 1U);

    {
        FrameTxQueue queue(3U, 6U);
        expect(queue.try_accept(a), "basic frame must be accepted");
        expect(queue.owned_frames() == 1U && queue.owned_bytes() == a.size(),
               "accepted frame counts must be exact");
        expect(queue.front_frame() != nullptr && *queue.front_frame() == a,
               "front must expose the accepted owned frame");
        expect(queue.front_offset() == 0U && queue.front_remaining() == a.size(),
               "new front must start at offset zero");
    }

    {
        FrameTxQueue queue(3U, 6U);
        expect(queue.try_accept(a) && queue.try_accept(b) && queue.try_accept(c),
               "FIFO frames must be accepted");
        expect(queue.front_frame() != nullptr && *queue.front_frame() == a,
               "FIFO front must remain first frame");
        expect(queue.consume_front(1U), "partial front consume must succeed");
        expect(queue.front_frame() != nullptr && *queue.front_frame() == a &&
                   queue.front_offset() == 1U && queue.front_remaining() == 2U &&
                   queue.owned_frames() == 3U && queue.owned_bytes() == 5U,
               "partial consume must preserve frame ownership and reduce bytes");
        const Bytes invalid_front = *queue.front_frame();
        const auto invalid_offset = queue.front_offset();
        const auto invalid_remaining = queue.front_remaining();
        const auto invalid_frames = queue.owned_frames();
        const auto invalid_bytes = queue.owned_bytes();
        expect(!queue.consume_front(2U + 1U),
               "consume beyond remaining must fail");
        expect(queue.front_frame() != nullptr && *queue.front_frame() == invalid_front &&
                   queue.front_offset() == invalid_offset &&
                   queue.front_remaining() == invalid_remaining &&
                   queue.owned_frames() == invalid_frames &&
                   queue.owned_bytes() == invalid_bytes,
               "invalid consume must leave state unchanged");
        expect(queue.consume_front(2U), "finishing first frame must succeed");
        expect(queue.front_frame() != nullptr && *queue.front_frame() == b &&
                   queue.front_offset() == 0U && queue.front_remaining() == b.size() &&
                   queue.owned_frames() == 2U && queue.owned_bytes() == 3U,
               "frame boundary must advance FIFO and reset offset");
        expect(queue.consume_front(b.size()), "finishing second frame must succeed");
        expect(queue.front_frame() != nullptr && *queue.front_frame() == c &&
                   queue.front_offset() == 0U && queue.front_remaining() == c.size() &&
                   queue.owned_frames() == 1U && queue.owned_bytes() == c.size(),
               "invalid consume must retain the third queued successor");
    }

    {
        FrameTxQueue queue(2U, 6U);
        expect(queue.try_accept(a) && queue.try_accept(b),
               "capacity setup frames must be accepted");
        expect(!queue.try_accept(c), "frame-count-full queue must reject");
        expect(queue.front_frame() != nullptr && *queue.front_frame() == a &&
                   queue.front_offset() == 0U && queue.front_remaining() == a.size() &&
                   queue.owned_frames() == 2U && queue.owned_bytes() == 5U,
               "frame-count rejection must preserve all state");
        expect(queue.consume_front(a.size()), "frame-count rejection drain must succeed");
        expect(queue.front_frame() != nullptr && *queue.front_frame() == b &&
                   queue.front_offset() == 0U && queue.front_remaining() == b.size() &&
                   queue.owned_frames() == 1U && queue.owned_bytes() == b.size(),
               "frame-count rejection must retain the queued successor");
    }

    {
        FrameTxQueue queue(4U, 6U);
        expect(queue.try_accept(a) && queue.try_accept(b),
               "independent byte-budget setup frames must be accepted");
        const Bytes too_large = frame(0xEEU, 2U);
        expect(!queue.try_accept(too_large),
               "insufficient byte budget must reject a complete frame");
        expect(queue.front_frame() != nullptr && *queue.front_frame() == a &&
                   queue.front_offset() == 0U && queue.front_remaining() == a.size() &&
                   queue.owned_frames() == 2U && queue.owned_bytes() == 5U,
               "byte-budget rejection must preserve the exact front state");
        expect(queue.consume_front(a.size()),
               "byte-budget rejection drain of first frame must succeed");
        expect(queue.front_frame() != nullptr && *queue.front_frame() == b &&
                   queue.front_offset() == 0U && queue.front_remaining() == b.size() &&
                   queue.owned_frames() == 1U && queue.owned_bytes() == b.size(),
               "byte-budget rejection must retain the exact queued successor");
    }

    {
        FrameTxQueue queue(4U, 6U);
        expect(queue.try_accept(a) && queue.try_accept(b),
               "byte-budget setup frames must be accepted");
        const Bytes too_large = frame(0xDDU, 3U);
        expect(queue.consume_front(1U), "partial front setup must succeed");
        const Bytes partial_front = *queue.front_frame();
        const auto partial_offset = queue.front_offset();
        const auto partial_remaining = queue.front_remaining();
        const auto partial_frames = queue.owned_frames();
        const auto partial_bytes = queue.owned_bytes();
        expect(!queue.try_accept(too_large),
               "partial-front remaining budget rejection must fail");
        expect(queue.front_frame() != nullptr && *queue.front_frame() == partial_front &&
                   queue.front_offset() == partial_offset &&
                   queue.front_remaining() == partial_remaining &&
                   queue.owned_frames() == partial_frames &&
                   queue.owned_bytes() == partial_bytes,
               "partial-front rejection must preserve front state");
        expect(queue.consume_front(partial_remaining),
               "partial-front rejection drain must succeed");
        expect(queue.front_frame() != nullptr && *queue.front_frame() == b &&
                   queue.front_offset() == 0U && queue.front_remaining() == b.size() &&
                   queue.owned_frames() == 1U && queue.owned_bytes() == b.size(),
               "partial-front rejection must retain the queued successor");
    }

    {
        FrameTxQueue source(3U, 6U);
        expect(source.try_accept(a) && source.try_accept(b),
               "move setup frames must be accepted");
        expect(source.consume_front(1U), "move setup partial consume must succeed");

        FrameTxQueue constructed(std::move(source));
        expect(constructed.front_frame() != nullptr &&
                   *constructed.front_frame() == a &&
                   constructed.front_offset() == 1U &&
                   constructed.front_remaining() == 2U &&
                   constructed.owned_frames() == 2U &&
                   constructed.owned_bytes() == 4U,
               "move construction must retain partial queue state");
        expect(source.front_frame() == nullptr && source.front_offset() == 0U &&
                   source.front_remaining() == 0U && source.owned_frames() == 0U &&
                   source.owned_bytes() == 0U,
               "move construction must empty the source deterministically");
        expect(source.try_accept(c) && source.front_frame() != nullptr &&
                   *source.front_frame() == c && source.front_remaining() == c.size(),
               "moved-from source must remain safely reusable");

        FrameTxQueue assigned(1U, 1U);
        assigned = std::move(source);
        expect(assigned.front_frame() != nullptr && *assigned.front_frame() == c &&
                   assigned.front_offset() == 0U &&
                   assigned.front_remaining() == c.size() &&
                   assigned.owned_frames() == 1U && assigned.owned_bytes() == c.size(),
               "move assignment must retain source queue state");
        move_assign(assigned, assigned);
        expect(assigned.front_frame() != nullptr && *assigned.front_frame() == c &&
                   assigned.front_offset() == 0U &&
                   assigned.front_remaining() == c.size() &&
                   assigned.owned_frames() == 1U && assigned.owned_bytes() == c.size(),
               "self move assignment must preserve queue state");
        expect(source.front_frame() == nullptr && source.front_offset() == 0U &&
                   source.front_remaining() == 0U && source.owned_frames() == 0U &&
                   source.owned_bytes() == 0U,
               "move assignment must empty the source deterministically");
        expect(source.try_accept(a) && source.front_remaining() == a.size(),
               "move-assigned source must remain safely reusable");
    }

    {
        FrameTxQueue queue(3U, 6U);
        expect(queue.try_accept(a) && queue.try_accept(b) && queue.try_accept(c),
               "clear setup frames must be accepted");
        expect(queue.consume_front(2U), "clear setup partial consume must succeed");
        queue.clear();
        expect_empty(queue, "clear must reset all queue state");
        expect(!queue.consume_front(0U), "empty consume must fail deterministically");
        expect(!queue.consume_front(1U), "empty nonzero consume must fail safely");
    }
}

} // namespace rbp2_test
