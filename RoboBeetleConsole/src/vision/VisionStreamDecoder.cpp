#include "vision/VisionStreamDecoder.h"

#include <algorithm>
#include <iterator>
#include <utility>

namespace rb::vision {

VisionFeedResult VisionStreamDecoder::fail(
    VisionStreamError error, VisionProtocolError headerError) noexcept
{
    fatalError_ = error;
    headerError_ = headerError;
    return {VisionFeedStatus::Fatal, std::nullopt, 0U, 0U};
}

VisionFeedResult VisionStreamDecoder::feed(
    std::span<const std::uint8_t> data)
{
    if (fatalError_ != VisionStreamError::None) {
        return {VisionFeedStatus::Fatal, std::nullopt, 0U, 0U};
    }

    VisionFeedResult result;
    std::size_t offset = 0U;

    while (offset < data.size()) {
        if (!currentHeader_.has_value()) {
            const std::size_t needed = kVisionHeaderSize - headerBytesUsed_;
            const std::size_t count = std::min(needed, data.size() - offset);
            std::copy_n(data.data() + offset, count,
                        headerBytes_.data() + headerBytesUsed_);
            headerBytesUsed_ += count;
            offset += count;

            if (headerBytesUsed_ < kVisionHeaderSize) {
                break;
            }

            const auto decoded = decodeVisionHeader(headerBytes_);
            if (!decoded.header.has_value()) {
                return fail(VisionStreamError::HeaderInvalid, decoded.error);
            }

            if (lastFrameId_.has_value() &&
                decoded.header->frameId <= *lastFrameId_) {
                return fail(VisionStreamError::FrameIdNotIncreasing);
            }

            currentHeader_ = *decoded.header;
            payload_.clear();
            payload_.reserve(currentHeader_->payloadSize);
        }

        const std::size_t needed =
            static_cast<std::size_t>(currentHeader_->payloadSize) - payload_.size();
        const std::size_t count = std::min(needed, data.size() - offset);
        payload_.insert(payload_.end(), data.begin() + static_cast<std::ptrdiff_t>(offset),
                        data.begin() + static_cast<std::ptrdiff_t>(offset + count));
        offset += count;

        if (payload_.size() < currentHeader_->payloadSize) {
            break;
        }

        VisionWireFrame frame{*currentHeader_, std::move(payload_)};
        if (result.latestFrame.has_value()) {
            ++result.replacedFrames;
        }
        result.latestFrame = std::move(frame);
        ++result.completedFrames;
        lastFrameId_ = currentHeader_->frameId;

        currentHeader_.reset();
        headerBytesUsed_ = 0U;
        payload_.clear();
    }

    return result;
}

VisionStreamError VisionStreamDecoder::finish() noexcept
{
    if (fatalError_ != VisionStreamError::None) {
        return fatalError_;
    }
    if (headerBytesUsed_ != 0U || currentHeader_.has_value() || !payload_.empty()) {
        fatalError_ = VisionStreamError::TruncatedFrame;
    }
    return fatalError_;
}

void VisionStreamDecoder::reset() noexcept
{
    headerBytes_.fill(0U);
    headerBytesUsed_ = 0U;
    currentHeader_.reset();
    payload_.clear();
    payload_.shrink_to_fit();
    lastFrameId_.reset();
    fatalError_ = VisionStreamError::None;
    headerError_ = VisionProtocolError::None;
}

} // namespace rb::vision
