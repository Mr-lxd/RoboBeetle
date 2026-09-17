#include "robobeetle/gateway/rbrp_codec.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace robobeetle::gateway {
namespace {

constexpr std::array<Byte, 4U> kMagic{{'R', 'B', 'R', 'P'}};
constexpr Byte kVersion = 1U;

std::uint16_t read_le16(const Byte *data) noexcept
{
    return static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(data[0]) |
        (static_cast<std::uint16_t>(data[1]) << 8U));
}

std::uint32_t read_le32(const Byte *data) noexcept
{
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8U) |
           (static_cast<std::uint32_t>(data[2]) << 16U) |
           (static_cast<std::uint32_t>(data[3]) << 24U);
}

void write_le16(Bytes &wire, std::uint16_t value)
{
    wire.push_back(static_cast<Byte>(value & 0xffU));
    wire.push_back(static_cast<Byte>((value >> 8U) & 0xffU));
}

void write_le32(Bytes &wire, std::uint32_t value)
{
    wire.push_back(static_cast<Byte>(value & 0xffU));
    wire.push_back(static_cast<Byte>((value >> 8U) & 0xffU));
    wire.push_back(static_cast<Byte>((value >> 16U) & 0xffU));
    wire.push_back(static_cast<Byte>((value >> 24U) & 0xffU));
}

void put_le16(Bytes &payload, std::size_t offset, std::uint16_t value)
{
    payload[offset] = static_cast<Byte>(value & 0xffU);
    payload[offset + 1U] = static_cast<Byte>((value >> 8U) & 0xffU);
}

void put_le32(Bytes &payload, std::size_t offset, std::uint32_t value)
{
    payload[offset] = static_cast<Byte>(value & 0xffU);
    payload[offset + 1U] = static_cast<Byte>((value >> 8U) & 0xffU);
    payload[offset + 2U] = static_cast<Byte>((value >> 16U) & 0xffU);
    payload[offset + 3U] = static_cast<Byte>((value >> 24U) & 0xffU);
}

bool is_client_kind(RbrpMessageKind kind) noexcept
{
    switch (kind) {
    case RbrpMessageKind::Hello:
    case RbrpMessageKind::AcquireControl:
    case RbrpMessageKind::ControlHeartbeat:
    case RbrpMessageKind::ReleaseControl:
    case RbrpMessageKind::CommandRequest:
        return true;
    default:
        return false;
    }
}

RbrpEncodeResult encode_invalid(RbrpEncodeStatus status)
{
    return {status, {}};
}

template<class T>
void append_i16(Bytes &payload, std::size_t offset, std::int16_t value)
{
    put_le16(payload, offset, static_cast<std::uint16_t>(value));
}

template<class T>
void append_diagnostics(Bytes &payload, std::size_t offset, const T &diagnostics)
{
    put_le32(payload, offset, diagnostics.rx_byte_count);
    put_le32(payload, offset + 4U, diagnostics.header_count);
    put_le32(payload, offset + 8U, diagnostics.valid_frame_count);
    put_le32(payload, offset + 12U, diagnostics.checksum_error_count);
    put_le32(payload, offset + 16U, diagnostics.rx_buffer_overflow_count);
    put_le32(payload, offset + 20U, diagnostics.rx_rearm_failure_count);
    put_le32(payload, offset + 24U, diagnostics.uart_error_count);
}

} // namespace

bool is_known_message_kind(RbrpMessageKind kind) noexcept
{
    switch (kind) {
    case RbrpMessageKind::Hello:
    case RbrpMessageKind::AcquireControl:
    case RbrpMessageKind::ControlHeartbeat:
    case RbrpMessageKind::ReleaseControl:
    case RbrpMessageKind::CommandRequest:
    case RbrpMessageKind::HelloReply:
    case RbrpMessageKind::AcquireReply:
    case RbrpMessageKind::ControlState:
    case RbrpMessageKind::CommandSubmitted:
    case RbrpMessageKind::CommandOutcome:
    case RbrpMessageKind::LeakTelemetry:
    case RbrpMessageKind::ImuTelemetry:
    case RbrpMessageKind::DepthTelemetry:
    case RbrpMessageKind::ServiceError:
        return true;
    default:
        return false;
    }
}

std::optional<std::size_t>
expected_payload_size(RbrpMessageKind kind) noexcept
{
    switch (kind) {
    case RbrpMessageKind::Hello:
        return 2U;
    case RbrpMessageKind::AcquireControl:
    case RbrpMessageKind::ControlHeartbeat:
    case RbrpMessageKind::ReleaseControl:
        return 0U;
    case RbrpMessageKind::CommandRequest:
        return std::nullopt;
    case RbrpMessageKind::HelloReply:
        return 8U;
    case RbrpMessageKind::AcquireReply:
        return 12U;
    case RbrpMessageKind::ControlState:
        return 8U;
    case RbrpMessageKind::CommandSubmitted:
        return 4U;
    case RbrpMessageKind::CommandOutcome:
        return 5U;
    case RbrpMessageKind::LeakTelemetry:
        return 3U;
    case RbrpMessageKind::ImuTelemetry:
        return 58U;
    case RbrpMessageKind::DepthTelemetry:
        return 40U;
    case RbrpMessageKind::ServiceError:
        return 8U;
    default:
        return std::nullopt;
    }
}

bool payload_size_is_valid(RbrpMessageKind kind, std::size_t size) noexcept
{
    if (kind == RbrpMessageKind::CommandRequest) {
        return size >= 1U && size <= 4U;
    }
    const auto expected = expected_payload_size(kind);
    return expected.has_value() && *expected == size;
}

RbrpEncodeResult encode_frame(RbrpMessageKind kind, RequestId request_id,
                               const Bytes &payload)
{
    if (payload.size() > RbrpDecoder::kMaxPayloadSize) {
        return encode_invalid(RbrpEncodeStatus::PayloadTooLarge);
    }
    if (!is_known_message_kind(kind)) {
        return encode_invalid(RbrpEncodeStatus::UnknownKind);
    }
    if (!payload_size_is_valid(kind, payload.size())) {
        return encode_invalid(RbrpEncodeStatus::InvalidPayloadLength);
    }

    Bytes wire;
    wire.reserve(RbrpDecoder::kHeaderSize + payload.size());
    wire.insert(wire.end(), kMagic.begin(), kMagic.end());
    wire.push_back(kVersion);
    wire.push_back(static_cast<Byte>(kind));
    write_le16(wire, 0U);
    write_le32(wire, static_cast<std::uint32_t>(payload.size()));
    write_le32(wire, request_id);
    wire.insert(wire.end(), payload.begin(), payload.end());
    return {RbrpEncodeStatus::Ok, std::move(wire)};
}

RbrpMessageDecodeResult decode_remote_message(const RbrpFrame &frame)
{
    if (!is_client_kind(frame.kind)) {
        return {RbrpMessageDecodeStatus::WrongDirection, std::nullopt};
    }
    if (!payload_size_is_valid(frame.kind, frame.payload.size())) {
        return {RbrpMessageDecodeStatus::InvalidPayload, std::nullopt};
    }

    RemoteMessage message;
    message.kind = frame.kind;
    message.request_id = frame.request_id;
    switch (frame.kind) {
    case RbrpMessageKind::Hello:
        message.payload = HelloRequest{read_le16(frame.payload.data())};
        break;
    case RbrpMessageKind::AcquireControl:
        message.payload = AcquireControlRequest{};
        break;
    case RbrpMessageKind::ControlHeartbeat:
        message.payload = ControlHeartbeatRequest{};
        break;
    case RbrpMessageKind::ReleaseControl:
        message.payload = ReleaseControlRequest{};
        break;
    case RbrpMessageKind::CommandRequest: {
        CommandRequest request;
        request.command_kind = frame.payload[0];
        switch (static_cast<RobotCommandKind>(request.command_kind)) {
        case RobotCommandKind::EnableServos:
            if (frame.payload.size() == 3U) {
                request.command = EnableServos{read_le16(frame.payload.data() + 1U)};
            }
            break;
        case RobotCommandKind::DisableServos:
            if (frame.payload.size() == 3U) {
                request.command = DisableServos{read_le16(frame.payload.data() + 1U)};
            }
            break;
        case RobotCommandKind::SetServoAngle:
            if (frame.payload.size() == 4U) {
                request.command = SetServoAngle{
                    frame.payload[1],
                    static_cast<std::int16_t>(
                        read_le16(frame.payload.data() + 2U))};
            }
            break;
        case RobotCommandKind::NeutralServos:
            if (frame.payload.size() == 3U) {
                request.command = NeutralServos{read_le16(frame.payload.data() + 1U)};
            }
            break;
        case RobotCommandKind::StartMotion:
            if (frame.payload.size() == 2U) {
                request.command = StartMotion{
                    static_cast<MotionMode>(frame.payload[1])};
            }
            break;
        case RobotCommandKind::StopMotion:
            if (frame.payload.size() == 1U) {
                request.command = StopMotion{};
            }
            break;
        case RobotCommandKind::SetGaitBackend:
            if (frame.payload.size() == 2U) {
                request.command = SetGaitBackend{
                    static_cast<GaitBackend>(frame.payload[1])};
            }
            break;
        default:
            break;
        }
        message.payload = std::move(request);
        break;
    }
    default:
        return {RbrpMessageDecodeStatus::WrongDirection, std::nullopt};
    }
    return {RbrpMessageDecodeStatus::Ok, std::move(message)};
}

RbrpEncodeResult encode_gateway_message(const GatewayMessage &message)
{
    return std::visit(
        [&message](const auto &payload) -> RbrpEncodeResult {
            using T = std::decay_t<decltype(payload)>;
            Bytes encoded;
            RbrpMessageKind kind = RbrpMessageKind::HelloReply;

            if constexpr (std::is_same_v<T, HelloReply>) {
                kind = RbrpMessageKind::HelloReply;
                encoded.reserve(8U);
                write_le16(encoded, payload.server_capabilities);
                write_le16(encoded, payload.max_payload);
                write_le16(encoded, payload.control_heartbeat_interval_ms);
                write_le16(encoded, payload.authority_lease_timeout_ms);
            } else if constexpr (std::is_same_v<T, AcquireReply>) {
                kind = RbrpMessageKind::AcquireReply;
                encoded.reserve(12U);
                encoded.push_back(static_cast<Byte>(payload.result));
                encoded.push_back(static_cast<Byte>(payload.authority_state));
                encoded.push_back(static_cast<Byte>(payload.session_state));
                encoded.push_back(static_cast<Byte>(payload.link_state));
                write_le32(encoded, payload.lease_timeout_ms);
                write_le32(encoded, payload.detail);
            } else if constexpr (std::is_same_v<T, ControlStateMessage>) {
                kind = RbrpMessageKind::ControlState;
                encoded.reserve(8U);
                encoded.push_back(static_cast<Byte>(payload.authority_state));
                encoded.push_back(static_cast<Byte>(payload.session_state));
                encoded.push_back(static_cast<Byte>(payload.link_state));
                encoded.push_back(static_cast<Byte>(payload.reason));
                write_le32(encoded, payload.lease_remaining_ms);
            } else if constexpr (std::is_same_v<T, CommandSubmittedMessage>) {
                kind = RbrpMessageKind::CommandSubmitted;
                const bool submitted =
                    payload.status == CommandSubmittedStatus::Submitted;
                if (submitted != payload.sequence.has_value()) {
                    return encode_invalid(RbrpEncodeStatus::InvalidPayloadLength);
                }
                encoded.reserve(4U);
                encoded.push_back(static_cast<Byte>(payload.status));
                encoded.push_back(submitted ? 1U : 0U);
                write_le16(encoded, payload.sequence.value_or(0U));
            } else if constexpr (std::is_same_v<T, GatewayCommandOutcomeMessage>) {
                kind = RbrpMessageKind::CommandOutcome;
                encoded.reserve(5U);
                encoded.push_back(static_cast<Byte>(payload.event.outcome));
                encoded.push_back(static_cast<Byte>(payload.command_kind));
                write_le16(encoded, payload.event.sequence);
                encoded.push_back(payload.event.result);
            } else if constexpr (std::is_same_v<T, GatewayLeakTelemetry>) {
                kind = RbrpMessageKind::LeakTelemetry;
                encoded.reserve(3U);
                write_le16(encoded, payload.sequence);
                encoded.push_back(static_cast<Byte>(payload.state));
            } else if constexpr (std::is_same_v<T, GatewayImuTelemetry>) {
                kind = RbrpMessageKind::ImuTelemetry;
                encoded.assign(58U, 0U);
                put_le16(encoded, 0U, payload.sequence);
                encoded[2] = payload.schema_version;
                encoded[3] = payload.validity_flags;
                for (std::size_t i = 0; i < 3U; ++i) {
                    put_le16(encoded, 4U + 2U * i,
                             static_cast<std::uint16_t>(payload.acc_mg[i]));
                    put_le16(encoded, 10U + 2U * i,
                             static_cast<std::uint16_t>(payload.gyro_tenth_dps[i]));
                    put_le16(encoded, 16U + 2U * i,
                             static_cast<std::uint16_t>(payload.angle_centidegrees[i]));
                }
                append_diagnostics(encoded, 22U, payload.diagnostics);
                put_le32(encoded, 50U, payload.diagnostics.mag_frame_count);
                put_le32(encoded, 54U,
                         payload.diagnostics.unsupported_frame_count);
            } else if constexpr (std::is_same_v<T, GatewayDepthTelemetry>) {
                kind = RbrpMessageKind::DepthTelemetry;
                encoded.assign(40U, 0U);
                put_le16(encoded, 0U, payload.sequence);
                encoded[2] = payload.schema_version;
                encoded[3] = payload.validity_flags;
                put_le32(encoded, 4U, static_cast<std::uint32_t>(payload.depth_mm));
                put_le16(encoded, 8U,
                         static_cast<std::uint16_t>(payload.temperature_centi_c));
                put_le16(encoded, 10U, payload.sample_age_ms);
                put_le32(encoded, 12U, payload.diagnostics.rx_byte_count);
                put_le32(encoded, 16U, payload.diagnostics.valid_line_count);
                put_le32(encoded, 20U, payload.diagnostics.parse_error_count);
                put_le32(encoded, 24U, payload.diagnostics.overlong_line_count);
                put_le32(encoded, 28U, payload.diagnostics.rx_buffer_overflow_count);
                put_le32(encoded, 32U, payload.diagnostics.hard_rearm_failure_count);
                put_le32(encoded, 36U, payload.diagnostics.uart_error_count);
            } else if constexpr (std::is_same_v<T, ServiceErrorMessage>) {
                kind = RbrpMessageKind::ServiceError;
                encoded.reserve(8U);
                write_le16(encoded, static_cast<std::uint16_t>(payload.error_code));
                encoded.push_back(static_cast<Byte>(payload.related_kind));
                encoded.push_back(0U);
                write_le32(encoded, payload.detail);
            }
            return encode_frame(kind, message.request_id, encoded);
        },
        message.payload);
}

bool RbrpDecoder::fail(RbrpFramingError error) noexcept
{
    fatal_error_ = error;
    header_bytes_ = 0U;
    expected_payload_bytes_ = 0U;
    payload_.clear();
    return false;
}

RbrpFeedStatus RbrpDecoder::feed(const Byte *data, std::size_t size,
                                  std::vector<RbrpFrame> &frames)
{
    if (fatal_error_ != RbrpFramingError::None) {
        return RbrpFeedStatus::Fatal;
    }
    if (size != 0U && data == nullptr) {
        fail(RbrpFramingError::InvalidInput);
        return RbrpFeedStatus::Fatal;
    }

    std::size_t offset = 0U;
    while (offset < size) {
        if (header_bytes_ < kHeaderSize) {
            const std::size_t amount =
                std::min(kHeaderSize - header_bytes_, size - offset);
            std::copy_n(data + offset, amount, header_.begin() +
                                              static_cast<std::ptrdiff_t>(header_bytes_));
            header_bytes_ += amount;
            offset += amount;
            if (header_bytes_ < kHeaderSize) {
                continue;
            }

            if (!std::equal(kMagic.begin(), kMagic.end(), header_.begin())) {
                fail(RbrpFramingError::BadMagic);
                return RbrpFeedStatus::Fatal;
            }
            if (header_[4] != kVersion) {
                fail(RbrpFramingError::UnsupportedVersion);
                return RbrpFeedStatus::Fatal;
            }
            if (header_[6] != 0U || header_[7] != 0U) {
                fail(RbrpFramingError::NonzeroFlags);
                return RbrpFeedStatus::Fatal;
            }

            const auto payload_length = read_le32(header_.data() + 8U);
            if (payload_length > kMaxPayloadSize) {
                fail(RbrpFramingError::PayloadTooLarge);
                return RbrpFeedStatus::Fatal;
            }

            const auto kind = static_cast<RbrpMessageKind>(header_[5]);
            if (!is_known_message_kind(kind)) {
                fail(RbrpFramingError::UnknownKind);
                return RbrpFeedStatus::Fatal;
            }
            if (!payload_size_is_valid(kind, payload_length)) {
                fail(RbrpFramingError::InvalidPayloadLength);
                return RbrpFeedStatus::Fatal;
            }

            expected_payload_bytes_ =
                static_cast<std::size_t>(payload_length);
            payload_.clear();
            payload_.reserve(expected_payload_bytes_);
        }

        if (payload_.size() < expected_payload_bytes_) {
            const std::size_t amount = std::min(
                expected_payload_bytes_ - payload_.size(), size - offset);
            payload_.insert(payload_.end(), data + offset,
                            data + offset + amount);
            offset += amount;
        }

        if (payload_.size() == expected_payload_bytes_) {
            RbrpFrame frame;
            frame.kind = static_cast<RbrpMessageKind>(header_[5]);
            frame.request_id = read_le32(header_.data() + 12U);
            frame.payload = payload_;
            frames.push_back(std::move(frame));
            header_bytes_ = 0U;
            expected_payload_bytes_ = 0U;
            payload_.clear();
        }
    }
    return RbrpFeedStatus::Ok;
}

void RbrpDecoder::reset() noexcept
{
    header_.fill(0U);
    header_bytes_ = 0U;
    expected_payload_bytes_ = 0U;
    payload_.clear();
    fatal_error_ = RbrpFramingError::None;
}

} // namespace robobeetle::gateway
