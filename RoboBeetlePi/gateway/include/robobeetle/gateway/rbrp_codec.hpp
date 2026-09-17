#pragma once

#include "robobeetle/gateway/gateway_types.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace robobeetle::gateway {

enum class RbrpFramingError {
    None,
    BadMagic,
    UnsupportedVersion,
    NonzeroFlags,
    PayloadTooLarge,
    UnknownKind,
    InvalidPayloadLength,
    InvalidInput,
};

enum class RbrpFeedStatus {
    Ok,
    Fatal,
};

struct RbrpFrame {
    RbrpMessageKind kind{RbrpMessageKind::Hello};
    RequestId request_id{0};
    Bytes payload;
};

enum class RbrpEncodeStatus {
    Ok,
    PayloadTooLarge,
    UnknownKind,
    InvalidPayloadLength,
};

struct RbrpEncodeResult {
    RbrpEncodeStatus status{RbrpEncodeStatus::Ok};
    Bytes wire;
};

enum class RbrpMessageDecodeStatus {
    Ok,
    WrongDirection,
    InvalidPayload,
};

struct RbrpMessageDecodeResult {
    RbrpMessageDecodeStatus status{RbrpMessageDecodeStatus::InvalidPayload};
    std::optional<RemoteMessage> message;
};

[[nodiscard]] bool is_known_message_kind(RbrpMessageKind kind) noexcept;
[[nodiscard]] std::optional<std::size_t>
expected_payload_size(RbrpMessageKind kind) noexcept;
[[nodiscard]] bool payload_size_is_valid(RbrpMessageKind kind,
                                          std::size_t size) noexcept;

[[nodiscard]] RbrpEncodeResult encode_frame(RbrpMessageKind kind,
                                             RequestId request_id,
                                             const Bytes &payload);

[[nodiscard]] RbrpMessageDecodeResult
decode_remote_message(const RbrpFrame &frame);

[[nodiscard]] RbrpEncodeResult
encode_gateway_message(const GatewayMessage &message);

class RbrpDecoder final {
public:
    static constexpr std::size_t kHeaderSize = 16U;
    static constexpr std::size_t kMaxPayloadSize = 512U;

    RbrpDecoder() = default;

    RbrpFeedStatus feed(const Byte *data, std::size_t size,
                        std::vector<RbrpFrame> &frames);

    [[nodiscard]] RbrpFramingError fatal_error() const noexcept
    {
        return fatal_error_;
    }

    [[nodiscard]] std::size_t buffered_payload_bytes() const noexcept
    {
        return payload_.size();
    }

    void reset() noexcept;

private:
    bool fail(RbrpFramingError error) noexcept;

    std::array<Byte, kHeaderSize> header_{};
    std::size_t header_bytes_{0};
    std::size_t expected_payload_bytes_{0};
    Bytes payload_;
    RbrpFramingError fatal_error_{RbrpFramingError::None};
};

} // namespace robobeetle::gateway
