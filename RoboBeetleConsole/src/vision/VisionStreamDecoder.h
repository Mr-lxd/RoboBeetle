#pragma once

#include "vision/VisionProtocol.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace rb::vision {

struct VisionWireFrame {
    VisionFrameHeader header;
    std::vector<std::uint8_t> jpegPayload;
};

enum class VisionStreamError {
    None,
    HeaderInvalid,
    FrameIdNotIncreasing,
    TruncatedFrame,
};

enum class VisionFeedStatus {
    Ok,
    Fatal,
};

struct VisionFeedResult {
    VisionFeedStatus status{VisionFeedStatus::Ok};
    std::optional<VisionWireFrame> latestFrame;
    std::uint64_t completedFrames{0};
    std::uint64_t replacedFrames{0};
};

class VisionStreamDecoder final {
public:
    VisionStreamDecoder() = default;

    [[nodiscard]] VisionFeedResult
    feed(std::span<const std::uint8_t> data);

    [[nodiscard]] VisionStreamError finish() noexcept;
    void reset() noexcept;

    [[nodiscard]] VisionStreamError fatalError() const noexcept
    {
        return fatalError_;
    }

    [[nodiscard]] VisionProtocolError headerError() const noexcept
    {
        return headerError_;
    }

    [[nodiscard]] std::size_t bufferedPayloadBytes() const noexcept
    {
        return payload_.size();
    }

    [[nodiscard]] std::optional<std::uint64_t> lastFrameId() const noexcept
    {
        return lastFrameId_;
    }

    [[nodiscard]] bool hasPartialFrame() const noexcept
    {
        return headerBytesUsed_ != 0U || currentHeader_.has_value();
    }

private:
    VisionFeedResult fail(VisionStreamError error,
                          VisionProtocolError headerError =
                              VisionProtocolError::None) noexcept;

    std::array<std::uint8_t, kVisionHeaderSize> headerBytes_{};
    std::size_t headerBytesUsed_{0};
    std::optional<VisionFrameHeader> currentHeader_;
    std::vector<std::uint8_t> payload_;
    std::optional<std::uint64_t> lastFrameId_;
    VisionStreamError fatalError_{VisionStreamError::None};
    VisionProtocolError headerError_{VisionProtocolError::None};
};

} // namespace rb::vision
