#include "robobeetle/gateway/control_gateway_core.hpp"

#include <algorithm>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace robobeetle::gateway {
namespace {

constexpr std::uint16_t kServoMask = 0x001fU;
constexpr GatewayTimeMs kLeaseTimeoutMs = 1000U;
constexpr std::size_t kOutstandingLimit = 32U;
constexpr std::size_t kRecentRequestLimit = 64U;

bool is_active_session(GatewayApplicationSessionState state) noexcept
{
    return state != GatewayApplicationSessionState::ReopenRequired;
}

bool is_valid_motion_mode(MotionMode mode) noexcept
{
    const auto value = static_cast<Byte>(mode);
    return value >= static_cast<Byte>(MotionMode::Forward) &&
           value <= static_cast<Byte>(MotionMode::Descend);
}

bool is_valid_gait_backend(GaitBackend backend) noexcept
{
    return static_cast<Byte>(backend) <=
           static_cast<Byte>(GaitBackend::ExperimentalFlex);
}

bool is_valid_front_rear_coordination(
    FrontRearCoordination coordination) noexcept
{
    return static_cast<Byte>(coordination) <=
           static_cast<Byte>(FrontRearCoordination::OppositeDirection);
}

bool is_authority_dependent(RbrpMessageKind kind) noexcept
{
    return kind == RbrpMessageKind::ControlHeartbeat ||
           kind == RbrpMessageKind::ReleaseControl ||
           kind == RbrpMessageKind::CommandRequest;
}

} // namespace

ControlGatewayCore::ControlGatewayCore(GatewayApplicationPort &application,
                                       GatewayCoreCallbacks callbacks)
    : application_(application), callbacks_(std::move(callbacks))
{
}

void ControlGatewayCore::diagnostic(const char *message)
{
    ++diagnostic_count_;
    if (callbacks_.diagnostic) {
        callbacks_.diagnostic(message);
    }
}

GatewayTimeMs ControlGatewayCore::grant_now(GatewayTimeMs owner_now_ms) const
{
    return callbacks_.now_ms ? callbacks_.now_ms() : owner_now_ms;
}

void ControlGatewayCore::fail_safe_application_invariant(
    const char *message, GatewayTimeMs owner_now_ms)
{
    diagnostic(message);
    if (!source_present_) {
        authority_ = AuthorityState::Unowned;
        authority_granted_at_ms_ = 0U;
        lease_deadline_ms_ = 0U;
        abort_once(owner_now_ms);
        terminalize_outstanding(GatewayCommandOutcome::OutcomeUnknown,
                                owner_now_ms);
        return;
    }

    const auto source = current_source_;
    revoke(source, GatewayStateReason::LinkLost, owner_now_ms, true);
    source_present_ = false;
}

void ControlGatewayCore::source_connected(ControlSourceId source)
{
    if (source == 0U) {
        diagnostic("zero source ID ignored");
        return;
    }
    if (source_present_) {
        if (source != current_source_) {
            diagnostic("second source ignored");
        }
        return;
    }
    if (!accepting_commands_) {
        diagnostic("source rejected after shutdown");
        return;
    }

    if (!correlations_.empty()) {
        terminalize_outstanding(GatewayCommandOutcome::OutcomeUnknown,
                                last_now_ms_);
    }
    current_source_ = source;
    source_present_ = true;
    hello_complete_ = false;
    authority_ = AuthorityState::Unowned;
    authority_granted_at_ms_ = 0U;
    lease_deadline_ms_ = 0U;
    abort_called_ = false;
    recent_request_ids_.clear();
    request_to_sequence_.clear();
    correlations_.clear();
    if (accepting_commands_) {
        return;
    }
    diagnostic("source rejected after shutdown");
}

void ControlGatewayCore::mark_completed(RequestId request_id)
{
    if (request_id == 0U) {
        return;
    }
    if (std::find(recent_request_ids_.begin(), recent_request_ids_.end(),
                  request_id) != recent_request_ids_.end()) {
        return;
    }
    recent_request_ids_.push_back(request_id);
    if (recent_request_ids_.size() > kRecentRequestLimit) {
        recent_request_ids_.erase(recent_request_ids_.begin());
    }
}

bool ControlGatewayCore::request_id_is_duplicate(RequestId request_id) const
{
    if (request_to_sequence_.find(request_id) != request_to_sequence_.end()) {
        return true;
    }
    return std::find(recent_request_ids_.begin(), recent_request_ids_.end(),
                     request_id) != recent_request_ids_.end();
}

void ControlGatewayCore::emit(const GatewayOutbound &output,
                              GatewayTimeMs owner_now_ms)
{
    if (!callbacks_.publish) {
        return;
    }
    if (callbacks_.publish(output) && !handling_output_failure_) {
        return;
    }
    if (handling_output_failure_) {
        return;
    }
    handling_output_failure_ = true;
    network_output_failed(output.source, GatewayStateReason::CriticalTxFailure,
                          owner_now_ms);
    handling_output_failure_ = false;
}

void ControlGatewayCore::emit_error(ControlSourceId source,
                                    RequestId request_id,
                                    RbrpMessageKind related_kind,
                                    ServiceErrorCode code,
                                    std::uint32_t detail,
                                    GatewayTimeMs owner_now_ms)
{
    emit(GatewayOutbound{
             source,
             GatewayMessage{
                 request_id,
                 ServiceErrorMessage{code, related_kind, detail}}},
         owner_now_ms);
}

void ControlGatewayCore::emit_state(ControlSourceId source,
                                    GatewayStateReason reason,
                                    GatewayTimeMs owner_now_ms)
{
    const auto session = application_.session_state();
    const auto link = application_.link_state();
    const std::uint32_t remaining =
        lease_deadline_ms_ > owner_now_ms
            ? static_cast<std::uint32_t>(lease_deadline_ms_ - owner_now_ms)
            : 0U;
    emit_state_snapshot(source, authority_, session, link, reason, remaining,
                        owner_now_ms);
}

void ControlGatewayCore::emit_state_snapshot(
    ControlSourceId source, AuthorityState authority,
    GatewayApplicationSessionState session, GatewayApplicationLinkState link,
    GatewayStateReason reason, std::uint32_t lease_remaining_ms,
    GatewayTimeMs owner_now_ms)
{
    emit(GatewayOutbound{
             source,
             GatewayMessage{
                 0U,
                 ControlStateMessage{authority, session, link, reason,
                                     lease_remaining_ms}}},
         owner_now_ms);
}

void ControlGatewayCore::close_source(ControlSourceId source,
                                       CloseSourceReason reason)
{
    if (!callbacks_.close_source) {
        return;
    }
    if (!callbacks_.close_source(CloseSourceSignal{source, reason})) {
        diagnostic("close-source control channel rejected");
    }
}

bool ControlGatewayCore::valid_command(const RobotCommand &command) const
{
    return std::visit(
        [](const auto &value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, EnableServos> ||
                          std::is_same_v<T, DisableServos> ||
                          std::is_same_v<T, NeutralServos>) {
                return value.mask != 0U && (value.mask & ~kServoMask) == 0U;
            } else if constexpr (std::is_same_v<T, SetServoAngle> ||
                                 std::is_same_v<T, SetServoPwm>) {
                return value.servo_id <= 4U;
            } else if constexpr (std::is_same_v<T, StartMotion>) {
                return is_valid_motion_mode(value.mode);
            } else if constexpr (std::is_same_v<T, SetCpgParameters>) {
                return protocol::valid_cpg_parameters(value.parameters);
            } else if constexpr (std::is_same_v<T, QueryCpgParameters>) {
                return true;
            } else if constexpr (std::is_same_v<T, StopMotion>) {
                return true;
            } else if constexpr (std::is_same_v<T, SetGaitBackend>) {
                return is_valid_gait_backend(value.backend);
            } else if constexpr (
                std::is_same_v<T, SetFrontRearCoordination>) {
                return is_valid_front_rear_coordination(value.coordination);
            } else {
                return false;
            }
        },
        command);
}

bool ControlGatewayCore::is_backward(const RobotCommand &command) const
{
    const auto *start = std::get_if<StartMotion>(&command);
    return start != nullptr && start->mode == MotionMode::Backward;
}

CommandSubmittedStatus ControlGatewayCore::submitted_status(
    GatewayApplicationSubmitStatus status) const
{
    switch (status) {
    case GatewayApplicationSubmitStatus::Submitted:
        return CommandSubmittedStatus::Submitted;
    case GatewayApplicationSubmitStatus::InvalidArgument:
        return CommandSubmittedStatus::InvalidArgument;
    case GatewayApplicationSubmitStatus::PendingQualification:
        return CommandSubmittedStatus::PendingQualification;
    case GatewayApplicationSubmitStatus::NotActive:
        return CommandSubmittedStatus::NotActive;
    case GatewayApplicationSubmitStatus::PayloadTooLarge:
        return CommandSubmittedStatus::PayloadTooLarge;
    case GatewayApplicationSubmitStatus::QueueFull:
        return CommandSubmittedStatus::QueueFull;
    case GatewayApplicationSubmitStatus::TransportRejected:
        return CommandSubmittedStatus::TransportRejected;
    }
    return CommandSubmittedStatus::TransportRejected;
}

void ControlGatewayCore::handle_command(const RemoteEnvelope &envelope,
                                        const CommandRequest &request,
                                        GatewayTimeMs owner_now_ms)
{
    if (!request.command.has_value()) {
        const bool known_command = request.command_kind >=
                                       static_cast<Byte>(
                                           RobotCommandKind::EnableServos) &&
                                   request.command_kind <=
                                       static_cast<Byte>(
                                           RobotCommandKind::QueryCpgParameters);
        if (known_command) {
            emit(GatewayOutbound{
                     envelope.source,
                     GatewayMessage{
                         envelope.message.request_id,
                         CommandSubmittedMessage{
                             CommandSubmittedStatus::InvalidArgument,
                             std::nullopt}}},
                 owner_now_ms);
        } else {
            emit_error(envelope.source, envelope.message.request_id,
                       envelope.message.kind,
                       ServiceErrorCode::UnsupportedCommand,
                       request.command_kind, owner_now_ms);
        }
        mark_completed(envelope.message.request_id);
        return;
    }
    if (!valid_command(*request.command)) {
        emit(GatewayOutbound{
                 envelope.source,
                 GatewayMessage{
                     envelope.message.request_id,
                     CommandSubmittedMessage{
                         CommandSubmittedStatus::InvalidArgument,
                         std::nullopt}}},
             owner_now_ms);
        mark_completed(envelope.message.request_id);
        return;
    }
    if (is_backward(*request.command)) {
        emit(GatewayOutbound{
                 envelope.source,
                 GatewayMessage{
                     envelope.message.request_id,
                     CommandSubmittedMessage{
                         CommandSubmittedStatus::PendingQualification,
                         std::nullopt}}},
             owner_now_ms);
        mark_completed(envelope.message.request_id);
        return;
    }
    if (request_to_sequence_.size() >= kOutstandingLimit) {
        emit(GatewayOutbound{
                 envelope.source,
                 GatewayMessage{
                     envelope.message.request_id,
                     CommandSubmittedMessage{
                         CommandSubmittedStatus::QueueFull, std::nullopt}}},
             owner_now_ms);
        mark_completed(envelope.message.request_id);
        return;
    }

    const auto result = application_.submit(*request.command);
    const auto status = submitted_status(result.status);
    if (result.status == GatewayApplicationSubmitStatus::Submitted &&
        (!result.sequence.has_value() ||
         correlations_.find(*result.sequence) != correlations_.end())) {
        fail_safe_application_invariant(
            result.sequence.has_value()
                ? "application submitted a live sequence twice"
                : "application submitted without a sequence",
            owner_now_ms);
        return;
    }
    if (result.status != GatewayApplicationSubmitStatus::Submitted &&
        result.sequence.has_value()) {
        fail_safe_application_invariant(
            "application returned a sequence for a non-submitted command",
            owner_now_ms);
        return;
    }

    if (status != CommandSubmittedStatus::Submitted) {
        emit(GatewayOutbound{
                 envelope.source,
                 GatewayMessage{
                     envelope.message.request_id,
                     CommandSubmittedMessage{status, std::nullopt}}},
             owner_now_ms);
        mark_completed(envelope.message.request_id);
        return;
    }

    const auto kind = static_cast<RobotCommandKind>(request.command_kind);
    const auto sequence = *result.sequence;
    const auto correlation_inserted = correlations_.emplace(
        sequence, Correlation{envelope.source, envelope.message.request_id, kind,
                              sequence});
    const auto request_inserted =
        request_to_sequence_.emplace(envelope.message.request_id, sequence);
    if (!correlation_inserted.second || !request_inserted.second) {
        correlations_.erase(sequence);
        request_to_sequence_.erase(envelope.message.request_id);
        fail_safe_application_invariant(
            "application correlation registration failed", owner_now_ms);
        return;
    }

    emit(GatewayOutbound{
             envelope.source,
             GatewayMessage{
                 envelope.message.request_id,
                 CommandSubmittedMessage{CommandSubmittedStatus::Submitted,
                                          result.sequence}}},
         owner_now_ms);
}

void ControlGatewayCore::handle_payload(const RemoteEnvelope &envelope,
                                        GatewayTimeMs owner_now_ms)
{
    const auto request_id = envelope.message.request_id;
    const auto kind = envelope.message.kind;

    if (kind != RbrpMessageKind::Hello && !hello_complete_) {
        emit_error(envelope.source, request_id, kind,
                   ServiceErrorCode::NotHello, 0U, owner_now_ms);
        mark_completed(request_id);
        return;
    }

    switch (kind) {
    case RbrpMessageKind::Hello: {
        const auto *request = std::get_if<HelloRequest>(&envelope.message.payload);
        if (request == nullptr) {
            emit_error(envelope.source, request_id, kind,
                       ServiceErrorCode::InvalidMessagePayload, 0U,
                       owner_now_ms);
            mark_completed(request_id);
            return;
        }
        if (hello_complete_) {
            emit_error(envelope.source, request_id, kind,
                       ServiceErrorCode::AlreadyHello, 0U, owner_now_ms);
            mark_completed(request_id);
            return;
        }
        if (request->client_capabilities != 0U) {
            emit_error(envelope.source, request_id, kind,
                       ServiceErrorCode::InvalidMessagePayload,
                       request->client_capabilities, owner_now_ms);
            mark_completed(request_id);
            return;
        }
        hello_complete_ = true;
        emit(GatewayOutbound{
                 envelope.source,
                 GatewayMessage{
                     request_id,
                     HelloReply{0U, 512U, 250U,
                                static_cast<std::uint16_t>(kLeaseTimeoutMs)}}},
             owner_now_ms);
        mark_completed(request_id);
        return;
    }
    case RbrpMessageKind::AcquireControl: {
        if (std::get_if<AcquireControlRequest>(&envelope.message.payload) ==
            nullptr) {
            emit_error(envelope.source, request_id, kind,
                       ServiceErrorCode::InvalidMessagePayload, 0U,
                       owner_now_ms);
            mark_completed(request_id);
            return;
        }
        if (authority_ == AuthorityState::Owned) {
            emit(GatewayOutbound{
                     envelope.source,
                     GatewayMessage{
                         request_id,
                         AcquireReply{AcquireResult::AlreadyOwnedBySource,
                                       authority_, application_.session_state(),
                                       application_.link_state(),
                                       static_cast<std::uint32_t>(
                                           kLeaseTimeoutMs),
                                       0U}}},
                 owner_now_ms);
            mark_completed(request_id);
            return;
        }
        if (application_.session_state() !=
            GatewayApplicationSessionState::ReopenRequired) {
            emit(GatewayOutbound{
                     envelope.source,
                     GatewayMessage{
                         request_id,
                         AcquireReply{AcquireResult::InvalidState,
                                       authority_, application_.session_state(),
                                       application_.link_state(),
                                       static_cast<std::uint32_t>(
                                           kLeaseTimeoutMs),
                                       0U}}},
                 owner_now_ms);
            mark_completed(request_id);
            return;
        }

        authority_ = AuthorityState::Acquiring;
        const int error_number = application_.open();
        if (error_number != 0) {
            authority_ = AuthorityState::Unowned;
            emit(GatewayOutbound{
                     envelope.source,
                     GatewayMessage{
                         request_id,
                         AcquireReply{AcquireResult::OpenFailed, authority_,
                                       application_.session_state(),
                                       application_.link_state(),
                                       static_cast<std::uint32_t>(
                                           kLeaseTimeoutMs),
                                       static_cast<std::uint32_t>(
                                           error_number)}}},
                 owner_now_ms);
            mark_completed(request_id);
            return;
        }

        const auto grant_now_ms = grant_now(owner_now_ms);
        authority_ = AuthorityState::Owned;
        abort_called_ = false;
        authority_granted_at_ms_ = grant_now_ms;
        lease_deadline_ms_ = grant_now_ms + kLeaseTimeoutMs;
        emit(GatewayOutbound{
                 envelope.source,
                 GatewayMessage{
                     request_id,
                     AcquireReply{AcquireResult::Granted, authority_,
                                   application_.session_state(),
                                   application_.link_state(),
                                   static_cast<std::uint32_t>(kLeaseTimeoutMs),
                                   0U}}},
             owner_now_ms);
        emit_state(envelope.source, GatewayStateReason::Acquired,
                   grant_now_ms);
        mark_completed(request_id);
        return;
    }
    case RbrpMessageKind::ControlHeartbeat: {
        if (std::get_if<ControlHeartbeatRequest>(&envelope.message.payload) ==
            nullptr) {
            emit_error(envelope.source, request_id, kind,
                       ServiceErrorCode::InvalidMessagePayload, 0U,
                       owner_now_ms);
            mark_completed(request_id);
            return;
        }
        if (authority_ != AuthorityState::Owned) {
            emit_error(envelope.source, request_id, kind,
                       ServiceErrorCode::NotAuthority, 0U, owner_now_ms);
            mark_completed(request_id);
            return;
        }
        if (envelope.received_at_ms < lease_deadline_ms_) {
            lease_deadline_ms_ = envelope.received_at_ms + kLeaseTimeoutMs;
        } else {
            revoke(envelope.source, GatewayStateReason::LeaseExpired,
                   std::max(owner_now_ms, envelope.received_at_ms), false);
        }
        mark_completed(request_id);
        return;
    }
    case RbrpMessageKind::ReleaseControl: {
        if (std::get_if<ReleaseControlRequest>(&envelope.message.payload) ==
            nullptr) {
            emit_error(envelope.source, request_id, kind,
                       ServiceErrorCode::InvalidMessagePayload, 0U,
                       owner_now_ms);
            mark_completed(request_id);
            return;
        }
        if (authority_ != AuthorityState::Owned) {
            emit_error(envelope.source, request_id, kind,
                       ServiceErrorCode::NotAuthority, 0U, owner_now_ms);
            mark_completed(request_id);
            return;
        }
        mark_completed(request_id);
        revoke(envelope.source, GatewayStateReason::Released,
               owner_now_ms, false);
        return;
    }
    case RbrpMessageKind::CommandRequest: {
        const auto *request =
            std::get_if<CommandRequest>(&envelope.message.payload);
        if (request == nullptr) {
            emit_error(envelope.source, request_id, kind,
                       ServiceErrorCode::InvalidMessagePayload, 0U,
                       owner_now_ms);
            mark_completed(request_id);
            return;
        }
        if (authority_ != AuthorityState::Owned) {
            emit_error(envelope.source, request_id, kind,
                       ServiceErrorCode::NotAuthority, 0U, owner_now_ms);
            mark_completed(request_id);
            return;
        }
        handle_command(envelope, *request, owner_now_ms);
        return;
    }
    default:
        emit_error(envelope.source, request_id, kind,
                   ServiceErrorCode::InvalidMessagePayload, 0U,
                   owner_now_ms);
        mark_completed(request_id);
        return;
    }
}

void ControlGatewayCore::process(const RemoteEnvelope &envelope,
                                 GatewayTimeMs owner_now_ms)
{
    last_now_ms_ = owner_now_ms;
    if (!accepting_commands_ || !source_present_ ||
        envelope.source != current_source_) {
        diagnostic("stale or inactive source envelope ignored");
        return;
    }

    if (envelope_lease_expired(envelope)) {
        revoke(current_source_, GatewayStateReason::LeaseExpired,
               std::max(owner_now_ms, envelope.received_at_ms), false);
        return;
    }

    const auto request_id = envelope.message.request_id;
    if (request_id == 0U) {
        emit_error(envelope.source, request_id, envelope.message.kind,
                   ServiceErrorCode::InvalidRequestId, 0U, owner_now_ms);
        return;
    }
    if (authority_request_before_grant(envelope)) {
        emit_error(envelope.source, request_id, envelope.message.kind,
                   ServiceErrorCode::NotAuthority, 0U, owner_now_ms);
        mark_completed(request_id);
        return;
    }
    if (request_id_is_duplicate(request_id)) {
        emit_error(envelope.source, request_id, envelope.message.kind,
                   ServiceErrorCode::DuplicateRequestId, 0U, owner_now_ms);
        return;
    }

    handle_payload(envelope, owner_now_ms);
}

bool ControlGatewayCore::envelope_lease_expired(
    const RemoteEnvelope &envelope) const noexcept
{
    return authority_ == AuthorityState::Owned &&
           envelope.received_at_ms >= lease_deadline_ms_;
}

bool ControlGatewayCore::authority_request_before_grant(
    const RemoteEnvelope &envelope) const noexcept
{
    return authority_ == AuthorityState::Owned &&
           is_authority_dependent(envelope.message.kind) &&
           envelope.received_at_ms < authority_granted_at_ms_;
}

void ControlGatewayCore::source_lost(const SourceLostSignal &signal,
                                     GatewayTimeMs owner_now_ms)
{
    last_now_ms_ = owner_now_ms;
    if (!source_present_ || signal.source != current_source_) {
        diagnostic("stale source loss ignored");
        return;
    }
    GatewayStateReason reason = GatewayStateReason::SourceDisconnected;
    switch (signal.reason) {
    case SourceLostReason::FatalProtocol:
        reason = GatewayStateReason::RemoteProtocolViolation;
        break;
    case SourceLostReason::LeaseExpired:
        reason = GatewayStateReason::LeaseExpired;
        break;
    case SourceLostReason::SessionLost:
        reason = GatewayStateReason::LinkLost;
        break;
    case SourceLostReason::CriticalTxFailure:
        reason = GatewayStateReason::CriticalTxFailure;
        break;
    case SourceLostReason::Shutdown:
        reason = GatewayStateReason::Shutdown;
        break;
    case SourceLostReason::Disconnected:
    case SourceLostReason::InboundQueueExhausted:
        reason = signal.reason == SourceLostReason::InboundQueueExhausted
                     ? GatewayStateReason::RemoteProtocolViolation
                     : GatewayStateReason::SourceDisconnected;
        break;
    }
    source_present_ = false;
    revoke(signal.source, reason, owner_now_ms, false);
}

void ControlGatewayCore::consume_event(const GatewayApplicationEvent &event,
                                       GatewayTimeMs owner_now_ms)
{
    if (const auto *outcome =
            std::get_if<GatewayCommandOutcomeEvent>(&event)) {
        const auto found = correlations_.find(outcome->sequence);
        if (found == correlations_.end()) {
            diagnostic("unknown application sequence outcome");
            return;
        }
        const Correlation correlation = found->second;
        correlations_.erase(found);
        request_to_sequence_.erase(correlation.request_id);
        emit(GatewayOutbound{
                 correlation.source,
                 GatewayMessage{
                     correlation.request_id,
                     GatewayCommandOutcomeMessage{
                         correlation.command_kind, *outcome}}},
             owner_now_ms);
        mark_completed(correlation.request_id);
        return;
    }
    if (const auto *telemetry =
            std::get_if<GatewayTelemetryEvent>(&event)) {
        if (authority_ != AuthorityState::Owned || !source_present_) {
            return;
        }
        std::visit(
            [this, owner_now_ms](const auto &value) {
                emit(GatewayOutbound{
                         current_source_,
                         GatewayMessage{0U, value}},
                     owner_now_ms);
            },
            *telemetry);
        return;
    }
    if (const auto *malformed =
            std::get_if<GatewayTelemetryMalformedDiagnostic>(&event)) {
        (void)malformed;
        diagnostic("malformed application telemetry");
        return;
    }
    if (const auto *state = std::get_if<GatewayStateLinkEvent>(&event)) {
        if (authority_ != AuthorityState::Owned || !source_present_) {
            return;
        }
        if (state->session_state ==
                GatewayApplicationSessionState::ReopenRequired ||
            (state->session_state ==
                 GatewayApplicationSessionState::Online &&
             state->link_state == GatewayApplicationLinkState::Lost)) {
            revoke(current_source_, GatewayStateReason::LinkLost,
                   owner_now_ms, false);
            return;
        }
        const std::uint32_t remaining =
            lease_deadline_ms_ > owner_now_ms
                ? static_cast<std::uint32_t>(lease_deadline_ms_ - owner_now_ms)
                : 0U;
        emit_state_snapshot(current_source_, authority_, state->session_state,
                            state->link_state, GatewayStateReason::None,
                            remaining, owner_now_ms);
    }
}

void ControlGatewayCore::consume_events(
    const std::vector<GatewayApplicationEvent> &events,
    GatewayTimeMs owner_now_ms)
{
    for (const auto &event : events) {
        consume_event(event, owner_now_ms);
    }
}

void ControlGatewayCore::terminalize_outstanding(
    GatewayCommandOutcome outcome, GatewayTimeMs owner_now_ms)
{
    std::vector<Correlation> pending;
    pending.reserve(correlations_.size());
    for (const auto &entry : correlations_) {
        pending.push_back(entry.second);
    }
    correlations_.clear();
    request_to_sequence_.clear();
    for (const auto &correlation : pending) {
        emit(GatewayOutbound{
                 correlation.source,
                 GatewayMessage{
                     correlation.request_id,
                         GatewayCommandOutcomeMessage{
                             correlation.command_kind,
                         GatewayCommandOutcomeEvent{
                             outcome, correlation.sequence, 0U}}}},
             owner_now_ms);
        mark_completed(correlation.request_id);
    }
}

void ControlGatewayCore::abort_once(GatewayTimeMs owner_now_ms)
{
    if (abort_called_) {
        return;
    }
    abort_called_ = true;
    const auto result = application_.abort();
    consume_application_abort_result(result, owner_now_ms);
}

void ControlGatewayCore::revoke(ControlSourceId source,
                                GatewayStateReason reason,
                                GatewayTimeMs owner_now_ms,
                                bool close)
{
    const bool needs_abort =
        authority_ != AuthorityState::Unowned ||
        is_active_session(application_.session_state());
    authority_ = AuthorityState::Unowned;
    authority_granted_at_ms_ = 0U;
    lease_deadline_ms_ = 0U;
    if (needs_abort) {
        abort_once(owner_now_ms);
    }
    terminalize_outstanding(GatewayCommandOutcome::OutcomeUnknown,
                            owner_now_ms);
    if (source != 0U) {
        emit_state(source, reason, owner_now_ms);
    }
    if (close) {
        close_source(source, reason == GatewayStateReason::Shutdown
                              ? CloseSourceReason::Shutdown
                              : CloseSourceReason::CriticalTxFailure);
    }
}

void ControlGatewayCore::consume_application_run_result(
    const GatewayApplicationRunResult &result, GatewayTimeMs owner_now_ms)
{
    last_now_ms_ = owner_now_ms;
    consume_events(result.events, owner_now_ms);

    const auto session_state = application_.session_state();
    const auto link_state = application_.link_state();

    if (result.status == GatewayApplicationRunStatus::SessionLost ||
        result.status == GatewayApplicationRunStatus::PollFatal) {
        revoke(current_source_, GatewayStateReason::LinkLost, owner_now_ms,
               false);
        return;
    }
    if (authority_ == AuthorityState::Owned &&
        (session_state == GatewayApplicationSessionState::ReopenRequired ||
         (session_state == GatewayApplicationSessionState::Online &&
          link_state == GatewayApplicationLinkState::Lost))) {
        revoke(current_source_, GatewayStateReason::LinkLost, owner_now_ms,
               false);
    }
}

void ControlGatewayCore::consume_application_abort_result(
    const GatewayApplicationAbortResult &result, GatewayTimeMs owner_now_ms)
{
    last_now_ms_ = owner_now_ms;
    consume_events(result.events, owner_now_ms);
    terminalize_outstanding(GatewayCommandOutcome::OutcomeUnknown,
                            owner_now_ms);
}

void ControlGatewayCore::check_time(GatewayTimeMs owner_now_ms)
{
    last_now_ms_ = owner_now_ms;
    if (authority_ == AuthorityState::Owned &&
        owner_now_ms >= lease_deadline_ms_) {
        revoke(current_source_, GatewayStateReason::LeaseExpired,
               owner_now_ms, false);
    }
}

void ControlGatewayCore::network_output_failed(ControlSourceId source,
                                               GatewayStateReason reason,
                                               GatewayTimeMs owner_now_ms)
{
    last_now_ms_ = owner_now_ms;
    if (!source_present_ || source != current_source_) {
        diagnostic("stale network output failure ignored");
        return;
    }
    revoke(source, reason, owner_now_ms, true);
    source_present_ = false;
}

void ControlGatewayCore::shutdown(GatewayTimeMs owner_now_ms)
{
    last_now_ms_ = owner_now_ms;
    accepting_commands_ = false;
    authority_granted_at_ms_ = 0U;
    const auto source = current_source_;
    if (source_present_) {
        revoke(source, GatewayStateReason::Shutdown, owner_now_ms, true);
        source_present_ = false;
    } else if (is_active_session(application_.session_state())) {
        authority_ = AuthorityState::Unowned;
        authority_granted_at_ms_ = 0U;
        abort_once(owner_now_ms);
        terminalize_outstanding(GatewayCommandOutcome::OutcomeUnknown,
                                owner_now_ms);
    }
}

} // namespace robobeetle::gateway
