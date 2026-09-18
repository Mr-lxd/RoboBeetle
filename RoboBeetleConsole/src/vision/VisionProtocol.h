#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace rb::vision {

inline constexpr std::array<std::uint8_t, 4> kVisionMagic{{'R', 'B', 'V', 'S'}};
inline constexpr std::uint8_t kVisionVersion = 1U;
inline constexpr std::size_t kVisionHeaderSize = 32U;
inline constexpr std::uint32_t kVisionMaxPayloadSize = 4'194'304U;
inline constexpr std::uint16_t kVisionMaxDimension = 8192U;
inline constexpr std::uint64_t kVisionMaxDecodedPixels = 4096ULL * 2160ULL;

enum class PayloadCodec : std::uint8_t {
    Jpeg = 1U,
};

enum class VisionProtocolError {
    None,
    WrongHeaderSize,
    BadMagic,
    UnsupportedVersion,
    UnsupportedHeaderSize,
    UnsupportedCodec,
    NonzeroFlags,
    InvalidTimestamp,
    InvalidDimensions,
    InvalidPayloadSize,
};

struct VisionFrameHeader {
    std::uint64_t frameId{0};
    std::uint64_t captureTimestampNs{0};
    std::uint16_t width{0};
    std::uint16_t height{0};
    std::uint32_t payloadSize{0};
    PayloadCodec codec{PayloadCodec::Jpeg};
    std::uint8_t flags{0};
};

using VisionHeaderWire = std::array<std::uint8_t, kVisionHeaderSize>;

struct VisionHeaderEncodeResult {
    VisionProtocolError error{VisionProtocolError::None};
    std::optional<VisionHeaderWire> wire;
};

struct VisionHeaderDecodeResult {
    VisionProtocolError error{VisionProtocolError::None};
    std::optional<VisionFrameHeader> header;
};

[[nodiscard]] VisionProtocolError
validateVisionHeader(const VisionFrameHeader &header) noexcept;

[[nodiscard]] VisionHeaderEncodeResult
encodeVisionHeader(const VisionFrameHeader &header) noexcept;

[[nodiscard]] VisionHeaderDecodeResult
decodeVisionHeader(std::span<const std::uint8_t> data) noexcept;

} // namespace rb::vision
