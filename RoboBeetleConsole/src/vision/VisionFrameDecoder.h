#pragma once

#include "vision/VisionStreamDecoder.h"

#include <QImage>

#include <cstdint>
#include <optional>
#include <span>

namespace rb::vision {

inline constexpr int kVisionImageAllocationLimitMiB = 64;

enum class VisionFrameError {
    None,
    StreamFatal,
    JpegHeaderInvalid,
    JpegDimensionMismatch,
    JpegDecodeFailed,
};

struct DecodedVisionFrame {
    VisionFrameHeader header;
    QImage image;
};

struct VisionFrameFeedResult {
    bool fatal{false};
    VisionFrameError error{VisionFrameError::None};
    std::optional<DecodedVisionFrame> latestFrame;
    std::uint64_t completedWireFrames{0};
    std::uint64_t replacedWireFrames{0};
};

class VisionFrameDecoder final {
public:
    VisionFrameDecoder();

    [[nodiscard]] VisionFrameFeedResult
    feed(std::span<const std::uint8_t> data);

    [[nodiscard]] VisionStreamError finish() noexcept
    {
        return streamDecoder_.finish();
    }

    void reset() noexcept
    {
        streamDecoder_.reset();
        jpegDecodeErrors_ = 0U;
        dimensionMismatches_ = 0U;
    }

    [[nodiscard]] bool hasPartialFrame() const noexcept
    {
        return streamDecoder_.hasPartialFrame();
    }

    [[nodiscard]] VisionStreamError streamError() const noexcept
    {
        return streamDecoder_.fatalError();
    }

    [[nodiscard]] VisionProtocolError headerError() const noexcept
    {
        return streamDecoder_.headerError();
    }

    [[nodiscard]] std::uint64_t jpegDecodeErrors() const noexcept
    {
        return jpegDecodeErrors_;
    }

    [[nodiscard]] std::uint64_t dimensionMismatches() const noexcept
    {
        return dimensionMismatches_;
    }

private:
    VisionStreamDecoder streamDecoder_;
    std::uint64_t jpegDecodeErrors_{0};
    std::uint64_t dimensionMismatches_{0};
};

} // namespace rb::vision
