#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <variant>
#include <vector>

namespace robobeetle::gateway {

using Byte = std::uint8_t;
using Bytes = std::vector<Byte>;
using ControlSourceId = std::uint64_t;
using RequestId = std::uint32_t;
using GatewayTimeMs = std::uint64_t;

enum class RbrpMessageKind : Byte {
    Hello = 0x01,
    AcquireControl = 0x02,
    ControlHeartbeat = 0x03,
    ReleaseControl = 0x04,
    CommandRequest = 0x05,

    HelloReply = 0x81,
    AcquireReply = 0x82,
    ControlState = 0x83,
    CommandSubmitted = 0x84,
    CommandOutcome = 0x85,
    LeakTelemetry = 0x86,
    ImuTelemetry = 0x87,
    DepthTelemetry = 0x88,
    ServiceError = 0x89,
};

enum class RobotCommandKind : Byte {
    EnableServos = 0x01,
    DisableServos = 0x02,
    SetServoAngle = 0x03,
    NeutralServos = 0x04,
    StartMotion = 0x05,
    StopMotion = 0x06,
    SetGaitBackend = 0x07,
    SetServoPwm = 0x08,
};

enum class MotionMode : Byte {
    Forward = 1,
    Backward = 2,
    TurnLeft = 3,
    TurnRight = 4,
    Ascend = 5,
    Descend = 6,
};

enum class GaitBackend : Byte {
    SimpleGait = 0,
    CPG = 1,
};

struct EnableServos {
    std::uint16_t mask{0};
};

struct DisableServos {
    std::uint16_t mask{0};
};

struct SetServoAngle {
    Byte servo_id{0};
    std::int16_t angle_cdeg{0};
};

struct SetServoPwm {
    Byte servo_id{0};
    std::uint16_t pulse_us{0};
};

struct NeutralServos {
    std::uint16_t mask{0};
};

struct StartMotion {
    MotionMode mode{MotionMode::Forward};
};

struct StopMotion {};

struct SetGaitBackend {
    GaitBackend backend{GaitBackend::SimpleGait};
};

using RobotCommand = std::variant<EnableServos, DisableServos, SetServoAngle,
                                  SetServoPwm, NeutralServos, StartMotion,
                                  StopMotion, SetGaitBackend>;

enum class GatewayApplicationSubmitStatus {
    Submitted,
    InvalidArgument,
    PendingQualification,
    NotActive,
    PayloadTooLarge,
    QueueFull,
    TransportRejected,
};

struct GatewayApplicationSubmitResult {
    GatewayApplicationSubmitStatus status{
        GatewayApplicationSubmitStatus::NotActive};
    std::optional<std::uint16_t> sequence;
};

enum class GatewayApplicationRunStatus {
    Progress,
    Timeout,
    Interrupted,
    NeedsOpen,
    SessionLost,
    PollFatal,
};

enum class GatewayApplicationSessionState {
    ReopenRequired,
    SafetyQuiet,
    Resynchronizing,
    Online,
};

enum class GatewayApplicationLinkState {
    Unconfirmed,
    Active,
    Degraded,
    Lost,
};

enum class GatewayCommandOutcome {
    Accepted,
    Rejected,
    OutcomeUnknown,
    Cancelled,
};

struct GatewayCommandOutcomeEvent {
    GatewayCommandOutcome outcome{GatewayCommandOutcome::OutcomeUnknown};
    std::uint16_t sequence{0};
    Byte result{0};
};

enum class GatewayTelemetryMalformedReason {
    WrongLength,
    WrongSchema,
    ReservedFlags,
    InvalidDomain,
    InvalidValue,
};

struct GatewayTelemetryMalformedDiagnostic {
    RbrpMessageKind kind{RbrpMessageKind::LeakTelemetry};
    std::uint16_t sequence{0};
    GatewayTelemetryMalformedReason reason{
        GatewayTelemetryMalformedReason::WrongLength};
};

enum class LeakState {
    Unknown = 0,
    Dry = 1,
    Wet = 2,
};

struct GatewayLeakTelemetry {
    std::uint16_t sequence{0};
    LeakState state{LeakState::Unknown};
};

struct GatewayImuTelemetryDiagnostics {
    std::uint32_t rx_byte_count{0};
    std::uint32_t header_count{0};
    std::uint32_t valid_frame_count{0};
    std::uint32_t checksum_error_count{0};
    std::uint32_t rx_buffer_overflow_count{0};
    std::uint32_t rx_rearm_failure_count{0};
    std::uint32_t uart_error_count{0};
    std::uint32_t mag_frame_count{0};
    std::uint32_t unsupported_frame_count{0};
};

struct GatewayImuTelemetry {
    std::uint16_t sequence{0};
    Byte schema_version{0};
    Byte validity_flags{0};
    std::array<std::int16_t, 3> acc_mg{{0, 0, 0}};
    std::array<std::int16_t, 3> gyro_tenth_dps{{0, 0, 0}};
    std::array<std::int16_t, 3> angle_centidegrees{{0, 0, 0}};
    GatewayImuTelemetryDiagnostics diagnostics;
};

struct GatewayDepthTelemetryDiagnostics {
    std::uint32_t rx_byte_count{0};
    std::uint32_t valid_line_count{0};
    std::uint32_t parse_error_count{0};
    std::uint32_t overlong_line_count{0};
    std::uint32_t rx_buffer_overflow_count{0};
    std::uint32_t hard_rearm_failure_count{0};
    std::uint32_t uart_error_count{0};
};

struct GatewayDepthTelemetry {
    std::uint16_t sequence{0};
    Byte schema_version{0};
    Byte validity_flags{0};
    std::int32_t depth_mm{0};
    std::int16_t temperature_centi_c{0};
    std::uint16_t sample_age_ms{0};
    GatewayDepthTelemetryDiagnostics diagnostics;
};

using GatewayTelemetryEvent =
    std::variant<GatewayLeakTelemetry, GatewayImuTelemetry,
                 GatewayDepthTelemetry>;

struct GatewayStateLinkEvent {
    GatewayApplicationSessionState session_state{
        GatewayApplicationSessionState::ReopenRequired};
    GatewayApplicationLinkState link_state{
        GatewayApplicationLinkState::Unconfirmed};
};

using GatewayApplicationEvent =
    std::variant<GatewayStateLinkEvent, GatewayCommandOutcomeEvent,
                 GatewayTelemetryEvent, GatewayTelemetryMalformedDiagnostic>;

struct GatewayApplicationRunResult {
    GatewayApplicationRunStatus status{GatewayApplicationRunStatus::Progress};
    std::vector<GatewayApplicationEvent> events;
    int error_number{0};
};

struct GatewayApplicationAbortResult {
    std::vector<GatewayApplicationEvent> events;
};

enum class AuthorityState {
    Unowned,
    Acquiring,
    Owned,
};

enum class AcquireResult {
    Granted,
    RequiresHello,
    AlreadyOwnedByOther,
    AlreadyOwnedBySource,
    OpenFailed,
    InvalidState,
};

enum class CommandSubmittedStatus {
    Submitted,
    InvalidArgument,
    PendingQualification,
    NotActive,
    PayloadTooLarge,
    QueueFull,
    TransportRejected,
};

enum class GatewayStateReason {
    None,
    Acquired,
    Released,
    LeaseExpired,
    SourceDisconnected,
    RemoteProtocolViolation,
    CriticalTxFailure,
    LinkLost,
    OpenFailed,
    Shutdown,
};

enum class ServiceErrorCode {
    None = 0,
    NotHello = 1,
    AlreadyHello = 2,
    InvalidRequestId = 3,
    DuplicateRequestId = 4,
    NotAuthority = 5,
    AuthorityBusy = 6,
    AcquireOpenFailed = 7,
    InvalidMessagePayload = 8,
    UnsupportedCommand = 9,
    Reserved10 = 10,
    LinkUnavailable = 11,
    InternalFailure = 12,
    Reserved13 = 13,
    Reserved14 = 14,
    Reserved15 = 15,
};

struct HelloRequest {
    std::uint16_t client_capabilities{0};
};

struct AcquireControlRequest {};

struct ControlHeartbeatRequest {};

struct ReleaseControlRequest {};

struct CommandRequest {
    Byte command_kind{0};
    std::optional<RobotCommand> command;
};

using RemotePayload =
    std::variant<HelloRequest, AcquireControlRequest,
                 ControlHeartbeatRequest, ReleaseControlRequest,
                 CommandRequest>;

struct RemoteMessage {
    RbrpMessageKind kind{RbrpMessageKind::Hello};
    RequestId request_id{0};
    RemotePayload payload;
};

struct RemoteEnvelope {
    ControlSourceId source{0};
    GatewayTimeMs received_at_ms{0};
    RemoteMessage message;
};

struct HelloReply {
    std::uint16_t server_capabilities{0};
    std::uint16_t max_payload{512};
    std::uint16_t control_heartbeat_interval_ms{250};
    std::uint16_t authority_lease_timeout_ms{1000};
};

struct AcquireReply {
    AcquireResult result{AcquireResult::InvalidState};
    AuthorityState authority_state{AuthorityState::Unowned};
    GatewayApplicationSessionState session_state{
        GatewayApplicationSessionState::ReopenRequired};
    GatewayApplicationLinkState link_state{
        GatewayApplicationLinkState::Unconfirmed};
    std::uint32_t lease_timeout_ms{1000};
    std::uint32_t detail{0};
};

struct ControlStateMessage {
    AuthorityState authority_state{AuthorityState::Unowned};
    GatewayApplicationSessionState session_state{
        GatewayApplicationSessionState::ReopenRequired};
    GatewayApplicationLinkState link_state{
        GatewayApplicationLinkState::Unconfirmed};
    GatewayStateReason reason{GatewayStateReason::None};
    std::uint32_t lease_remaining_ms{0};
};

struct CommandSubmittedMessage {
    CommandSubmittedStatus status{CommandSubmittedStatus::NotActive};
    std::optional<std::uint16_t> sequence;
};

struct GatewayCommandOutcomeMessage {
    RobotCommandKind command_kind{RobotCommandKind::StopMotion};
    GatewayCommandOutcomeEvent event;
};

struct ServiceErrorMessage {
    ServiceErrorCode error_code{ServiceErrorCode::None};
    RbrpMessageKind related_kind{RbrpMessageKind::Hello};
    std::uint32_t detail{0};
};

using GatewayMessagePayload =
    std::variant<HelloReply, AcquireReply, ControlStateMessage,
                 CommandSubmittedMessage, GatewayCommandOutcomeMessage,
                 GatewayLeakTelemetry, GatewayImuTelemetry,
                 GatewayDepthTelemetry, ServiceErrorMessage>;

struct GatewayMessage {
    RequestId request_id{0};
    GatewayMessagePayload payload;
};

struct GatewayOutbound {
    ControlSourceId source{0};
    GatewayMessage message;
};

enum class SourceLostReason {
    Disconnected,
    FatalProtocol,
    InboundQueueExhausted,
    LeaseExpired,
    SessionLost,
    CriticalTxFailure,
    Shutdown,
};

struct SourceLostSignal {
    ControlSourceId source{0};
    SourceLostReason reason{SourceLostReason::Disconnected};
};

enum class CloseSourceReason {
    FatalProtocol,
    InboundQueueExhausted,
    CriticalTxFailure,
    Shutdown,
};

struct CloseSourceSignal {
    ControlSourceId source{0};
    CloseSourceReason reason{CloseSourceReason::FatalProtocol};
};

} // namespace robobeetle::gateway
