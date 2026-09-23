#include "test_support.hpp"
#include "robobeetle/gateway/rbrp_codec.hpp"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <vector>

namespace rbp2_test { int failures = 0; }

namespace {

using namespace robobeetle::gateway;
using rbp2_test::bytes;
using rbp2_test::concat;
using rbp2_test::expect;
using Bytes = std::vector<std::uint8_t>;

static_assert(static_cast<std::uint16_t>(ServiceErrorCode::None) == 0U);
static_assert(static_cast<std::uint16_t>(ServiceErrorCode::NotHello) == 1U);
static_assert(static_cast<std::uint16_t>(ServiceErrorCode::AlreadyHello) == 2U);
static_assert(static_cast<std::uint16_t>(ServiceErrorCode::InvalidRequestId) ==
              3U);
static_assert(static_cast<std::uint16_t>(ServiceErrorCode::DuplicateRequestId) ==
              4U);
static_assert(static_cast<std::uint16_t>(ServiceErrorCode::NotAuthority) == 5U);
static_assert(static_cast<std::uint16_t>(ServiceErrorCode::AuthorityBusy) == 6U);
static_assert(static_cast<std::uint16_t>(ServiceErrorCode::AcquireOpenFailed) ==
              7U);
static_assert(
    static_cast<std::uint16_t>(ServiceErrorCode::InvalidMessagePayload) == 8U);
static_assert(static_cast<std::uint16_t>(ServiceErrorCode::UnsupportedCommand) ==
              9U);
static_assert(static_cast<std::uint16_t>(ServiceErrorCode::Reserved10) == 10U);
static_assert(static_cast<std::uint16_t>(ServiceErrorCode::LinkUnavailable) ==
              11U);
static_assert(static_cast<std::uint16_t>(ServiceErrorCode::InternalFailure) ==
              12U);
static_assert(static_cast<std::uint16_t>(ServiceErrorCode::Reserved13) == 13U);
static_assert(static_cast<std::uint16_t>(ServiceErrorCode::Reserved14) == 14U);
static_assert(static_cast<std::uint16_t>(ServiceErrorCode::Reserved15) == 15U);

constexpr std::size_t header_size = 16U;

std::uint16_t read_le16(const Bytes &data, std::size_t offset)
{
    return static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(data[offset]) |
        (static_cast<std::uint16_t>(data[offset + 1U]) << 8U));
}

std::uint32_t read_le32(const Bytes &data, std::size_t offset)
{
    return static_cast<std::uint32_t>(data[offset]) |
           (static_cast<std::uint32_t>(data[offset + 1U]) << 8U) |
           (static_cast<std::uint32_t>(data[offset + 2U]) << 16U) |
           (static_cast<std::uint32_t>(data[offset + 3U]) << 24U);
}

Bytes payload_of(const RbrpEncodeResult &encoded)
{
    if (encoded.wire.size() < header_size) {
        return {};
    }
    return Bytes(encoded.wire.begin() + static_cast<std::ptrdiff_t>(header_size),
                 encoded.wire.end());
}

Bytes raw_header(std::uint8_t kind, std::uint32_t payload_length,
                 std::uint32_t request_id = 1U)
{
    Bytes wire(header_size, 0U);
    wire[0] = 'R';
    wire[1] = 'B';
    wire[2] = 'R';
    wire[3] = 'P';
    wire[4] = 1U;
    wire[5] = kind;
    wire[6] = 0U;
    wire[7] = 0U;
    wire[8] = static_cast<std::uint8_t>(payload_length & 0xffU);
    wire[9] = static_cast<std::uint8_t>((payload_length >> 8U) & 0xffU);
    wire[10] = static_cast<std::uint8_t>((payload_length >> 16U) & 0xffU);
    wire[11] = static_cast<std::uint8_t>((payload_length >> 24U) & 0xffU);
    wire[12] = static_cast<std::uint8_t>(request_id & 0xffU);
    wire[13] = static_cast<std::uint8_t>((request_id >> 8U) & 0xffU);
    wire[14] = static_cast<std::uint8_t>((request_id >> 16U) & 0xffU);
    wire[15] = static_cast<std::uint8_t>((request_id >> 24U) & 0xffU);
    return wire;
}

void framing_header_and_little_endian()
{
    const auto encoded = encode_frame(
        RbrpMessageKind::Hello, 0x10203040U, bytes({0x34U, 0x12U}));
    expect(encoded.status == RbrpEncodeStatus::Ok,
           "Hello frame encodes");
    expect(encoded.wire == bytes({
               'R', 'B', 'R', 'P', 1U, 0x01U, 0x00U, 0x00U,
               0x02U, 0x00U, 0x00U, 0x00U,
               0x40U, 0x30U, 0x20U, 0x10U,
               0x34U, 0x12U}),
           "RBRP header is exactly 16 bytes with explicit little endian fields");

    RbrpDecoder decoder;
    std::vector<RbrpFrame> frames;
    expect(decoder.feed(encoded.wire.data(), encoded.wire.size(), frames) ==
               RbrpFeedStatus::Ok,
           "complete frame is accepted");
    expect(frames.size() == 1U && frames[0].kind == RbrpMessageKind::Hello &&
               frames[0].request_id == 0x10203040U &&
               frames[0].payload == bytes({0x34U, 0x12U}),
           "decoded frame preserves kind, request ID, and payload");
}

void fragmentation_and_multiple_frames()
{
    const auto first = encode_frame(
        RbrpMessageKind::Hello, 7U, bytes({0x00U, 0x00U}));
    const auto second = encode_frame(
        RbrpMessageKind::AcquireControl, 8U, {});
    RbrpDecoder decoder;
    std::vector<RbrpFrame> frames;

    for (std::size_t i = 0; i < header_size - 1U; ++i) {
        expect(decoder.feed(&first.wire[i], 1U, frames) == RbrpFeedStatus::Ok,
               "fragmented header remains accepted");
    }
    expect(frames.empty(), "incomplete header produces no frame");
    expect(decoder.feed(&first.wire[header_size - 1U], 1U, frames) ==
               RbrpFeedStatus::Ok,
           "header completion remains accepted");
    expect(frames.empty(), "fragmented payload produces no frame");
    expect(decoder.feed(first.wire.data() + header_size, 1U, frames) ==
               RbrpFeedStatus::Ok,
           "first payload fragment remains accepted");
    expect(frames.empty(), "partial payload remains buffered");
    expect(decoder.feed(first.wire.data() + header_size + 1U, 1U, frames) ==
               RbrpFeedStatus::Ok,
           "final payload fragment is accepted");
    expect(frames.size() == 1U && frames[0].request_id == 7U,
           "fragmented payload produces one complete frame");

    frames.clear();
    const Bytes combined = concat(first.wire, second.wire);
    expect(decoder.feed(combined.data(), combined.size(), frames) ==
               RbrpFeedStatus::Ok,
           "multiple frames in one input buffer are accepted");
    expect(frames.size() == 2U &&
               frames[0].kind == RbrpMessageKind::Hello &&
               frames[1].kind == RbrpMessageKind::AcquireControl &&
               frames[1].payload.empty(),
           "multiple complete frames are emitted in order");
}

void exact_payload_sizes_and_fatal_classification()
{
    const std::array<std::pair<RbrpMessageKind, std::size_t>, 13U> sizes = {{
        {RbrpMessageKind::Hello, 2U},
        {RbrpMessageKind::AcquireControl, 0U},
        {RbrpMessageKind::ControlHeartbeat, 0U},
        {RbrpMessageKind::ReleaseControl, 0U},
        {RbrpMessageKind::HelloReply, 8U},
        {RbrpMessageKind::AcquireReply, 12U},
        {RbrpMessageKind::ControlState, 8U},
        {RbrpMessageKind::CommandSubmitted, 4U},
        {RbrpMessageKind::CommandOutcome, 5U},
        {RbrpMessageKind::LeakTelemetry, 3U},
        {RbrpMessageKind::ImuTelemetry, 58U},
        {RbrpMessageKind::DepthTelemetry, 40U},
        {RbrpMessageKind::ServiceError, 8U},
    }};
    for (const auto &[kind, size] : sizes) {
        expect(expected_payload_size(kind) == size,
               "every RBRP kind has its frozen exact payload size");
        expect(payload_size_is_valid(kind, size),
               "every fixed-size RBRP kind accepts its exact payload size");
        const auto encoded = encode_frame(kind, 0U, Bytes(size, 0U));
        expect(encoded.status == RbrpEncodeStatus::Ok,
               "known kind accepts its exact payload size");
    }
    for (const std::size_t size : {1U, 2U, 3U, 4U}) {
        expect(payload_size_is_valid(RbrpMessageKind::CommandRequest, size),
               "CommandRequest accepts each frozen typed-command length");
        expect(encode_frame(RbrpMessageKind::CommandRequest, 0U,
                            Bytes(size, 0U)).status == RbrpEncodeStatus::Ok,
               "CommandRequest encodes each frozen typed-command length");
    }
    expect(!payload_size_is_valid(RbrpMessageKind::CommandRequest, 0U) &&
               !payload_size_is_valid(RbrpMessageKind::CommandRequest, 5U),
           "CommandRequest rejects impossible lengths");

    const auto invalid_length = raw_header(0x01U, 1U);
    RbrpDecoder invalid_decoder;
    std::vector<RbrpFrame> invalid_frames;
    expect(invalid_decoder.feed(invalid_length.data(), invalid_length.size(),
                                invalid_frames) == RbrpFeedStatus::Fatal,
           "impossible exact payload length is fatal framing");
    expect(invalid_decoder.fatal_error() ==
               RbrpFramingError::InvalidPayloadLength &&
               invalid_frames.empty(),
           "invalid length is classified without emitting a frame");

    const auto too_large = raw_header(0x01U, 513U);
    RbrpDecoder large_decoder;
    std::vector<RbrpFrame> large_frames;
    expect(large_decoder.feed(too_large.data(), too_large.size(), large_frames) ==
               RbrpFeedStatus::Fatal,
           "payload over 512 is fatal");
    expect(large_decoder.fatal_error() == RbrpFramingError::PayloadTooLarge &&
               large_decoder.buffered_payload_bytes() == 0U,
           "oversize payload is rejected before payload allocation");
}

void malformed_headers_are_fatal()
{
    const auto valid = encode_frame(
        RbrpMessageKind::AcquireControl, 1U, {});
    const std::array<std::pair<std::size_t, std::uint8_t>, 5U> mutations = {{
        {0U, static_cast<std::uint8_t>('X')},
        {4U, 2U},
        {6U, 1U},
        {7U, 1U},
        {5U, 0x7fU},
    }};
    const std::array<RbrpFramingError, 5U> errors = {{
        RbrpFramingError::BadMagic,
        RbrpFramingError::UnsupportedVersion,
        RbrpFramingError::NonzeroFlags,
        RbrpFramingError::NonzeroFlags,
        RbrpFramingError::UnknownKind,
    }};
    for (std::size_t i = 0; i < mutations.size(); ++i) {
        Bytes wire = valid.wire;
        wire[mutations[i].first] = mutations[i].second;
        RbrpDecoder decoder;
        std::vector<RbrpFrame> frames;
        expect(decoder.feed(wire.data(), wire.size(), frames) ==
                   RbrpFeedStatus::Fatal,
               "malformed RBRP header closes with fatal classification");
        expect(decoder.fatal_error() == errors[i],
               "malformed RBRP header exposes the specific fatal reason");
        expect(frames.empty(), "fatal framing emits no semantic message");
    }
}

void typed_commands_and_semantic_decode()
{
    const auto angle = encode_frame(
        RbrpMessageKind::CommandRequest, 42U,
        bytes({0x03U, 0x04U, 0xfeU, 0xffU}));
    expect(angle.status == RbrpEncodeStatus::Ok,
           "SetServoAngle command frame encodes");
    RbrpDecoder decoder;
    std::vector<RbrpFrame> frames;
    expect(decoder.feed(angle.wire.data(), angle.wire.size(), frames) ==
               RbrpFeedStatus::Ok && frames.size() == 1U,
           "typed command frame is framed");
    const auto decoded = decode_remote_message(frames[0]);
    expect(decoded.status == RbrpMessageDecodeStatus::Ok &&
               decoded.message.has_value(),
           "typed command frame decodes semantically");
    if (decoded.message) {
        const auto *request =
            std::get_if<CommandRequest>(&decoded.message->payload);
        expect(request != nullptr && request->command.has_value(),
               "SetServoAngle maps to a typed RobotCommand");
        if (request != nullptr && request->command) {
            const auto *set_angle = std::get_if<SetServoAngle>(&*request->command);
            expect(set_angle != nullptr && set_angle->servo_id == 4U &&
                       set_angle->angle_cdeg == static_cast<std::int16_t>(-2),
                   "SetServoAngle preserves signed int16 little endian angle");
        }
    }

    const auto pwm = encode_frame(
        RbrpMessageKind::CommandRequest, 43U,
        bytes({0x08U, 0x00U, 0xdcU, 0x05U}));
    expect(pwm.status == RbrpEncodeStatus::Ok,
           "SetServoPwm command frame encodes");
    RbrpDecoder pwm_decoder;
    std::vector<RbrpFrame> pwm_frames;
    pwm_decoder.feed(pwm.wire.data(), pwm.wire.size(), pwm_frames);
    const auto pwm_message = decode_remote_message(pwm_frames[0]);
    expect(pwm_message.status == RbrpMessageDecodeStatus::Ok
               && pwm_message.message.has_value(),
           "SetServoPwm command frame decodes semantically");
    if (pwm_message.message) {
        const auto *request =
            std::get_if<CommandRequest>(&pwm_message.message->payload);
        expect(request != nullptr && request->command.has_value(),
               "SetServoPwm maps to a typed RobotCommand");
        if (request != nullptr && request->command) {
            const auto *set_pwm = std::get_if<SetServoPwm>(&*request->command);
            expect(set_pwm != nullptr && set_pwm->servo_id == 0U
                       && set_pwm->pulse_us == 1500U,
                   "SetServoPwm preserves servo ID and uint16 little endian pulse");
        }
    }

    const auto backward = encode_frame(
        RbrpMessageKind::CommandRequest, 44U, bytes({0x05U, 0x02U}));
    RbrpDecoder backward_decoder;
    std::vector<RbrpFrame> backward_frames;
    backward_decoder.feed(backward.wire.data(), backward.wire.size(),
                          backward_frames);
    const auto backward_message = decode_remote_message(backward_frames[0]);
    expect(backward_message.message.has_value(),
           "Backward command remains representable on the wire");
    if (backward_message.message) {
        const auto *request =
            std::get_if<CommandRequest>(&backward_message.message->payload);
        expect(request != nullptr && request->command.has_value(),
               "Backward maps to typed StartMotion");
        if (request != nullptr && request->command) {
            const auto *start = std::get_if<StartMotion>(&*request->command);
            expect(start != nullptr &&
                       start->mode == MotionMode::Backward,
                   "Backward is preserved as the typed pending command");
        }
    }

    const auto unsupported = encode_frame(
        RbrpMessageKind::CommandRequest, 44U, bytes({0x7fU}));
    RbrpDecoder unsupported_decoder;
    std::vector<RbrpFrame> unsupported_frames;
    unsupported_decoder.feed(unsupported.wire.data(), unsupported.wire.size(),
                             unsupported_frames);
    const auto unsupported_message =
        decode_remote_message(unsupported_frames[0]);
    expect(unsupported_message.status == RbrpMessageDecodeStatus::Ok &&
               unsupported_message.message.has_value(),
           "unsupported nested command remains a semantic message");
    if (unsupported_message.message) {
        const auto *request =
            std::get_if<CommandRequest>(&unsupported_message.message->payload);
        expect(request != nullptr && !request->command.has_value() &&
                   request->command_kind == 0x7fU,
               "unsupported nested command is not guessed or executed");
    }

    const auto zero_id = encode_frame(
        RbrpMessageKind::Hello, 0U, bytes({0x00U, 0x00U}));
    RbrpDecoder zero_decoder;
    std::vector<RbrpFrame> zero_frames;
    zero_decoder.feed(zero_id.wire.data(), zero_id.wire.size(), zero_frames);
    const auto zero_message = decode_remote_message(zero_frames[0]);
    expect(zero_message.status == RbrpMessageDecodeStatus::Ok &&
               zero_message.message && zero_message.message->request_id == 0U,
           "request ID zero remains a nonfatal semantic case");
}

void wrong_direction_is_rejected_by_remote_decoder()
{
    const auto server_frame = encode_frame(
        RbrpMessageKind::HelloReply, 77U, Bytes(8U, 0U));
    RbrpDecoder decoder;
    std::vector<RbrpFrame> frames;
    expect(decoder.feed(server_frame.wire.data(), server_frame.wire.size(),
                        frames) == RbrpFeedStatus::Ok &&
               frames.size() == 1U,
           "a valid server-direction frame remains well-framed");
    if (frames.size() == 1U) {
        const auto decoded = decode_remote_message(frames.front());
        expect(decoded.status == RbrpMessageDecodeStatus::WrongDirection &&
                   !decoded.message.has_value(),
               "server-direction frame is rejected as WrongDirection without a remote message");
    }
}

void server_message_wire_encodings()
{
    const auto hello = encode_gateway_message(GatewayMessage{
        11U, HelloReply{0U, 512U, 250U, 1000U}});
    expect(hello.status == RbrpEncodeStatus::Ok &&
               hello.wire[5] == static_cast<std::uint8_t>(RbrpMessageKind::HelloReply) &&
               read_le32(hello.wire, 12U) == 11U &&
               payload_of(hello) == bytes({0U, 0U, 0U, 2U, 250U, 0U, 0xe8U, 0x03U}),
           "HelloReply uses its exact server payload encoding");

    const auto acquire = encode_gateway_message(GatewayMessage{
        12U, AcquireReply{AcquireResult::Granted, AuthorityState::Owned,
                           GatewayApplicationSessionState::SafetyQuiet,
                           GatewayApplicationLinkState::Unconfirmed, 1000U, 0U}});
    expect(acquire.status == RbrpEncodeStatus::Ok &&
               payload_of(acquire).size() == 12U &&
               payload_of(acquire)[0] == 0U &&
               payload_of(acquire)[1] == 2U &&
               payload_of(acquire)[2] == 1U &&
               payload_of(acquire)[3] == 0U &&
               read_le32(payload_of(acquire), 4U) == 1000U,
           "AcquireReply preserves all lifecycle fields and little endian lease");

    const auto submitted = encode_gateway_message(GatewayMessage{
        13U, CommandSubmittedMessage{CommandSubmittedStatus::Submitted,
                                     static_cast<std::uint16_t>(813U)}});
    expect(submitted.status == RbrpEncodeStatus::Ok &&
               payload_of(submitted).size() == 4U &&
               payload_of(submitted)[0] == 0U &&
               payload_of(submitted)[1] == 1U &&
               read_le16(payload_of(submitted), 2U) == 813U,
           "CommandSubmitted encodes admission separately from sequence");

    const auto outcome = encode_gateway_message(GatewayMessage{
        13U, GatewayCommandOutcomeMessage{
                 RobotCommandKind::SetServoAngle,
                 GatewayCommandOutcomeEvent{
                     GatewayCommandOutcome::Rejected, 813U, 0x05U}}});
    expect(outcome.status == RbrpEncodeStatus::Ok &&
               payload_of(outcome).size() == 5U &&
               payload_of(outcome)[0] == 1U &&
               payload_of(outcome)[1] ==
                   static_cast<std::uint8_t>(RobotCommandKind::SetServoAngle) &&
               read_le16(payload_of(outcome), 2U) == 813U &&
               payload_of(outcome)[4] == 0x05U,
           "CommandOutcome encodes neutral outcome and command kind");

    const auto telemetry = encode_frame(
        RbrpMessageKind::LeakTelemetry, 0U, Bytes(3U, 0U));
    const auto imu = encode_frame(
        RbrpMessageKind::ImuTelemetry, 0U, Bytes(58U, 0U));
    const auto depth = encode_frame(
        RbrpMessageKind::DepthTelemetry, 0U, Bytes(40U, 0U));
    expect(telemetry.status == RbrpEncodeStatus::Ok &&
               imu.status == RbrpEncodeStatus::Ok &&
               depth.status == RbrpEncodeStatus::Ok &&
               payload_of(telemetry).size() == 3U &&
               payload_of(imu).size() == 58U &&
               payload_of(depth).size() == 40U &&
               read_le32(telemetry.wire, 12U) == 0U &&
               read_le32(imu.wire, 12U) == 0U &&
               read_le32(depth.wire, 12U) == 0U,
           "unsolicited Leak/IMU/Depth frames use request ID zero and exact sizes");

    const auto error = encode_gateway_message(GatewayMessage{
        0U, ServiceErrorMessage{ServiceErrorCode::NotAuthority,
                                RbrpMessageKind::CommandRequest, 99U}});
    expect(error.status == RbrpEncodeStatus::Ok &&
               payload_of(error).size() == 8U &&
               payload_of(error)[0] == 5U && payload_of(error)[1] == 0U &&
               read_le16(payload_of(error), 0U) == 5U &&
               payload_of(error)[2] ==
                   static_cast<std::uint8_t>(RbrpMessageKind::CommandRequest) &&
               payload_of(error)[3] == 0U &&
               read_le32(payload_of(error), 4U) == 99U,
           "ServiceError preserves the NotAuthority registry value and detail");

    const auto invalid_payload = encode_gateway_message(GatewayMessage{
        0U, ServiceErrorMessage{ServiceErrorCode::InvalidMessagePayload,
                                RbrpMessageKind::Hello, 0U}});
    const auto unsupported = encode_gateway_message(GatewayMessage{
        0U, ServiceErrorMessage{ServiceErrorCode::UnsupportedCommand,
                                RbrpMessageKind::CommandRequest, 0U}});
    expect(invalid_payload.status == RbrpEncodeStatus::Ok &&
               payload_of(invalid_payload).size() == 8U &&
               payload_of(invalid_payload)[0] == 8U &&
               payload_of(invalid_payload)[1] == 0U &&
               read_le16(payload_of(invalid_payload), 0U) == 8U &&
               unsupported.status == RbrpEncodeStatus::Ok &&
               payload_of(unsupported).size() == 8U &&
               payload_of(unsupported)[0] == 9U &&
               payload_of(unsupported)[1] == 0U &&
               read_le16(payload_of(unsupported), 0U) == 9U,
           "ServiceError encodes InvalidMessagePayload=8 and UnsupportedCommand=9 in LE");
}

} // namespace

int main()
{
    framing_header_and_little_endian();
    fragmentation_and_multiple_frames();
    exact_payload_sizes_and_fatal_classification();
    malformed_headers_are_fatal();
    typed_commands_and_semantic_decode();
    wrong_direction_is_rejected_by_remote_decoder();
    server_message_wire_encodings();

    if (rbp2_test::failures == 0) {
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}
