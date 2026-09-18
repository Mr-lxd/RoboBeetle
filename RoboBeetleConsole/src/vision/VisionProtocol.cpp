#include "vision/VisionProtocol.h"

#include <algorithm>

namespace rb::vision {
namespace {

std::uint16_t readBe16(const std::uint8_t *data) noexcept
{
    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(data[0]) << 8U) |
        static_cast<std::uint16_t>(data[1]));
}

std::uint32_t readBe32(const std::uint8_t *data) noexcept
{
    return (static_cast<std::uint32_t>(data[0]) << 24U) |
           (static_cast<std::uint32_t>(data[1]) << 16U) |
           (static_cast<std::uint32_t>(data[2]) << 8U) |
           static_cast<std::uint32_t>(data[3]);
}

std::uint64_t readBe64(const std::uint8_t *data) noexcept
{
    std::uint64_t value = 0U;
    for (std::size_t index = 0; index < 8U; ++index) {
        value = (value << 8U) | static_cast<std::uint64_t>(data[index]);
    }
    return value;
}

void writeBe16(std::uint8_t *data, std::uint16_t value) noexcept
{
    data[0] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
    data[1] = static_cast<std::uint8_t>(value & 0xffU);
}

void writeBe32(std::uint8_t *data, std::uint32_t value) noexcept
{
    data[0] = static_cast<std::uint8_t>((value >> 24U) & 0xffU);
    data[1] = static_cast<std::uint8_t>((value >> 16U) & 0xffU);
    data[2] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
    data[3] = static_cast<std::uint8_t>(value & 0xffU);
}

void writeBe64(std::uint8_t *data, std::uint64_t value) noexcept
{
    for (int index = 7; index >= 0; --index) {
        data[index] = static_cast<std::uint8_t>(value & 0xffU);
        value >>= 8U;
    }
}

} // namespace

VisionProtocolError validateVisionHeader(const VisionFrameHeader &header) noexcept
{
    if (header.captureTimestampNs == 0U) {
        return VisionProtocolError::InvalidTimestamp;
    }
    if (header.width == 0U || header.width > kVisionMaxDimension ||
        header.height == 0U || header.height > kVisionMaxDimension) {
        return VisionProtocolError::InvalidDimensions;
    }
    const std::uint64_t decodedPixels =
        static_cast<std::uint64_t>(header.width) * static_cast<std::uint64_t>(header.height);
    if (decodedPixels > kVisionMaxDecodedPixels) {
        return VisionProtocolError::InvalidDimensions;
    }
    if (header.payloadSize == 0U || header.payloadSize > kVisionMaxPayloadSize) {
        return VisionProtocolError::InvalidPayloadSize;
    }
    if (header.codec != PayloadCodec::Jpeg) {
        return VisionProtocolError::UnsupportedCodec;
    }
    if (header.flags != 0U) {
        return VisionProtocolError::NonzeroFlags;
    }
    return VisionProtocolError::None;
}

VisionHeaderEncodeResult encodeVisionHeader(const VisionFrameHeader &header) noexcept
{
    const VisionProtocolError validation = validateVisionHeader(header);
    if (validation != VisionProtocolError::None) {
        return {validation, std::nullopt};
    }
    VisionHeaderWire wire{};
    std::copy(kVisionMagic.begin(), kVisionMagic.end(), wire.begin());
    wire[4] = kVisionVersion;
    wire[5] = static_cast<std::uint8_t>(kVisionHeaderSize);
    wire[6] = static_cast<std::uint8_t>(header.codec);
    wire[7] = header.flags;
    writeBe64(wire.data() + 8U, header.frameId);
    writeBe64(wire.data() + 16U, header.captureTimestampNs);
    writeBe16(wire.data() + 24U, header.width);
    writeBe16(wire.data() + 26U, header.height);
    writeBe32(wire.data() + 28U, header.payloadSize);
    return {VisionProtocolError::None, wire};
}

VisionHeaderDecodeResult
decodeVisionHeader(std::span<const std::uint8_t> data) noexcept
{
    if (data.size() != kVisionHeaderSize) {
        return {VisionProtocolError::WrongHeaderSize, std::nullopt};
    }
    if (!std::equal(kVisionMagic.begin(), kVisionMagic.end(), data.begin())) {
        return {VisionProtocolError::BadMagic, std::nullopt};
    }
    if (data[4] != kVisionVersion) {
        return {VisionProtocolError::UnsupportedVersion, std::nullopt};
    }
    if (data[5] != static_cast<std::uint8_t>(kVisionHeaderSize)) {
        return {VisionProtocolError::UnsupportedHeaderSize, std::nullopt};
    }
    if (data[6] != static_cast<std::uint8_t>(PayloadCodec::Jpeg)) {
        return {VisionProtocolError::UnsupportedCodec, std::nullopt};
    }

    VisionFrameHeader header;
    header.frameId = readBe64(data.data() + 8U);
    header.captureTimestampNs = readBe64(data.data() + 16U);
    header.width = readBe16(data.data() + 24U);
    header.height = readBe16(data.data() + 26U);
    header.payloadSize = readBe32(data.data() + 28U);
    header.codec = PayloadCodec::Jpeg;
    header.flags = data[7];

    const VisionProtocolError validation = validateVisionHeader(header);
    if (validation != VisionProtocolError::None) {
        return {validation, std::nullopt};
    }
    return {VisionProtocolError::None, header};
}

} // namespace rb::vision
