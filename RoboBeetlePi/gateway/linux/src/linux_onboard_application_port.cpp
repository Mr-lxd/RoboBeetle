#include "robobeetle/gateway/linux_onboard_application_port.hpp"

#include "robobeetle/protocol/message_types.hpp"

#include <chrono>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace robobeetle::gateway {
namespace {

RbrpMessageKind telemetry_kind(std::uint8_t type) noexcept
{
    switch (static_cast<protocol::MessageType>(type)) {
    case protocol::MessageType::LeakStatus:
        return RbrpMessageKind::LeakTelemetry;
    case protocol::MessageType::ImuSnapshot:
        return RbrpMessageKind::ImuTelemetry;
    case protocol::MessageType::DepthSnapshot:
        return RbrpMessageKind::DepthTelemetry;
    case protocol::MessageType::CpgParametersSnapshot:
        return RbrpMessageKind::CpgParametersTelemetry;
    case protocol::MessageType::MotionStateBatch:
        return RbrpMessageKind::MotionStateTelemetry;
    default:
        return RbrpMessageKind::Hello;
    }
}

bool is_telemetry_type(std::uint8_t type) noexcept
{
    switch (static_cast<protocol::MessageType>(type)) {
    case protocol::MessageType::LeakStatus:
    case protocol::MessageType::ImuSnapshot:
    case protocol::MessageType::DepthSnapshot:
    case protocol::MessageType::CpgParametersSnapshot:
    case protocol::MessageType::MotionStateBatch:
        return true;
    default:
        return false;
    }
}

GatewayCommandOutcome map_outcome(link_core::OutcomeKind kind) noexcept
{
    switch (kind) {
    case link_core::OutcomeKind::Accepted:
        return GatewayCommandOutcome::Accepted;
    case link_core::OutcomeKind::Rejected:
        return GatewayCommandOutcome::Rejected;
    case link_core::OutcomeKind::OutcomeUnknown:
        return GatewayCommandOutcome::OutcomeUnknown;
    case link_core::OutcomeKind::Cancelled:
        return GatewayCommandOutcome::Cancelled;
    case link_core::OutcomeKind::None:
        break;
    }
    return GatewayCommandOutcome::OutcomeUnknown;
}

GatewayTelemetryMalformedReason map_malformed_reason(
    application::TelemetryMalformedReason reason) noexcept
{
    switch (reason) {
    case application::TelemetryMalformedReason::WrongLength:
        return GatewayTelemetryMalformedReason::WrongLength;
    case application::TelemetryMalformedReason::WrongSchema:
        return GatewayTelemetryMalformedReason::WrongSchema;
    case application::TelemetryMalformedReason::ReservedFlags:
        return GatewayTelemetryMalformedReason::ReservedFlags;
    case application::TelemetryMalformedReason::InvalidDomain:
        return GatewayTelemetryMalformedReason::InvalidDomain;
    case application::TelemetryMalformedReason::InvalidValue:
        return GatewayTelemetryMalformedReason::InvalidValue;
    case application::TelemetryMalformedReason::None:
        break;
    }
    return GatewayTelemetryMalformedReason::WrongLength;
}

} // namespace

LinuxOnboardApplicationPort::LinuxOnboardApplicationPort(
    std::string device_path, link_core::LinkCoreConfig config,
    std::size_t max_tx_frames, std::size_t max_tx_bytes)
    : device_path_(std::move(device_path)),
      application_(config, max_tx_frames, max_tx_bytes)
{
}

int LinuxOnboardApplicationPort::open()
{
    const int error = application_.open(device_path_.c_str());
    if (error == 0) ++link_epoch_;
    return error;
}

GatewayApplicationSessionState LinuxOnboardApplicationPort::map_session_state(
    session::SessionState state) noexcept
{
    switch (state) {
    case session::SessionState::ReopenRequired:
        return GatewayApplicationSessionState::ReopenRequired;
    case session::SessionState::SafetyQuiet:
        return GatewayApplicationSessionState::SafetyQuiet;
    case session::SessionState::Resynchronizing:
        return GatewayApplicationSessionState::Resynchronizing;
    case session::SessionState::Online:
        return GatewayApplicationSessionState::Online;
    }
    return GatewayApplicationSessionState::ReopenRequired;
}

GatewayApplicationLinkState LinuxOnboardApplicationPort::map_link_state(
    link_core::LinkState state) noexcept
{
    switch (state) {
    case link_core::LinkState::Unconfirmed:
        return GatewayApplicationLinkState::Unconfirmed;
    case link_core::LinkState::Active:
        return GatewayApplicationLinkState::Active;
    case link_core::LinkState::Degraded:
        return GatewayApplicationLinkState::Degraded;
    case link_core::LinkState::Lost:
        return GatewayApplicationLinkState::Lost;
    }
    return GatewayApplicationLinkState::Lost;
}

GatewayApplicationRunStatus LinuxOnboardApplicationPort::map_run_status(
    runtime::RuntimeStatus status) noexcept
{
    switch (status) {
    case runtime::RuntimeStatus::Progress:
        return GatewayApplicationRunStatus::Progress;
    case runtime::RuntimeStatus::Timeout:
        return GatewayApplicationRunStatus::Timeout;
    case runtime::RuntimeStatus::Interrupted:
        return GatewayApplicationRunStatus::Interrupted;
    case runtime::RuntimeStatus::NeedsOpen:
        return GatewayApplicationRunStatus::NeedsOpen;
    case runtime::RuntimeStatus::SessionLost:
        return GatewayApplicationRunStatus::SessionLost;
    case runtime::RuntimeStatus::PollFatal:
        return GatewayApplicationRunStatus::PollFatal;
    }
    return GatewayApplicationRunStatus::PollFatal;
}

GatewayApplicationSubmitResult
LinuxOnboardApplicationPort::map_submit_result(
    const application::CommandSubmitResult &result) const
{
    GatewayApplicationSubmitStatus status =
        GatewayApplicationSubmitStatus::TransportRejected;
    switch (result.status) {
    case application::CommandSubmitStatus::Submitted:
        status = GatewayApplicationSubmitStatus::Submitted;
        break;
    case application::CommandSubmitStatus::InvalidArgument:
        status = GatewayApplicationSubmitStatus::InvalidArgument;
        break;
    case application::CommandSubmitStatus::PendingQualification:
        status = GatewayApplicationSubmitStatus::PendingQualification;
        break;
    case application::CommandSubmitStatus::NotActive:
        status = GatewayApplicationSubmitStatus::NotActive;
        break;
    case application::CommandSubmitStatus::PayloadTooLarge:
        status = GatewayApplicationSubmitStatus::PayloadTooLarge;
        break;
    case application::CommandSubmitStatus::QueueFull:
        status = GatewayApplicationSubmitStatus::QueueFull;
        break;
    case application::CommandSubmitStatus::TransportRejected:
        status = GatewayApplicationSubmitStatus::TransportRejected;
        break;
    }
    return {status, result.sequence};
}

std::vector<GatewayApplicationEvent> LinuxOnboardApplicationPort::map_events(
    const std::vector<application::ApplicationEvent> &events) const
{
    std::vector<GatewayApplicationEvent> mapped;
    mapped.reserve(events.size());
    std::optional<std::pair<RbrpMessageKind, std::uint16_t>> telemetry_frame;

    std::uint64_t motion_rx_ms = 0;
    for (const auto &event : events) {
        if (const auto *raw =
                std::get_if<link_core::LinkEvent>(&event)) {
            if (raw->type == link_core::LinkEventType::FrameReceived &&
                is_telemetry_type(raw->frame.message_type)) {
                telemetry_frame = std::make_pair(
                    telemetry_kind(raw->frame.message_type),
                    raw->frame.sequence);
            } else {
                telemetry_frame.reset();
            }

            if (raw->type == link_core::LinkEventType::FrameReceived &&
                raw->frame.message_type == static_cast<std::uint8_t>(protocol::MessageType::MotionStateBatch)) {
                // Stamp each fragment once after serial event translation.
                // Linux steady_clock uses the same CLOCK_MONOTONIC domain as video.
                motion_rx_ms = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count());
            }
            switch (raw->type) {
            case link_core::LinkEventType::RequestAccepted:
            case link_core::LinkEventType::RequestRejected:
            case link_core::LinkEventType::RequestOutcomeUnknown:
            case link_core::LinkEventType::RequestCancelled:
                mapped.emplace_back(GatewayCommandOutcomeEvent{
                    map_outcome(raw->outcome.kind),
                    raw->outcome.sequence,
                    raw->outcome.result});
                break;
            case link_core::LinkEventType::StateChanged:
                mapped.emplace_back(GatewayStateLinkEvent{
                    session_state(), map_link_state(raw->state)});
                break;
            default:
                break;
            }
            continue;
        }

        if (const auto *leak = std::get_if<application::LeakTelemetry>(&event)) {
            if (telemetry_frame &&
                telemetry_frame->first == RbrpMessageKind::LeakTelemetry) {
                mapped.emplace_back(GatewayTelemetryEvent{
                    GatewayLeakTelemetry{telemetry_frame->second,
                                         static_cast<LeakState>(leak->state)}});
            }
            telemetry_frame.reset();
            continue;
        }
        if (const auto *imu = std::get_if<application::ImuTelemetry>(&event)) {
            if (telemetry_frame &&
                telemetry_frame->first == RbrpMessageKind::ImuTelemetry) {
                GatewayImuTelemetry value;
                value.sequence = telemetry_frame->second;
                value.schema_version = imu->schema_version;
                value.validity_flags = imu->validity_flags;
                value.acc_mg = imu->acc_mg;
                value.gyro_tenth_dps = imu->gyro_tenth_dps;
                value.angle_centidegrees = imu->angle_centidegrees;
                value.diagnostics.rx_byte_count = imu->diagnostics.rx_byte_count;
                value.diagnostics.header_count = imu->diagnostics.header_count;
                value.diagnostics.valid_frame_count =
                    imu->diagnostics.valid_frame_count;
                value.diagnostics.checksum_error_count =
                    imu->diagnostics.checksum_error_count;
                value.diagnostics.rx_buffer_overflow_count =
                    imu->diagnostics.rx_buffer_overflow_count;
                value.diagnostics.rx_rearm_failure_count =
                    imu->diagnostics.rx_rearm_failure_count;
                value.diagnostics.uart_error_count =
                    imu->diagnostics.uart_error_count;
                value.diagnostics.mag_frame_count =
                    imu->diagnostics.mag_frame_count;
                value.diagnostics.unsupported_frame_count =
                    imu->diagnostics.unsupported_frame_count;
                mapped.emplace_back(GatewayTelemetryEvent{value});
            }
            telemetry_frame.reset();
            continue;
        }
        if (const auto *depth =
                std::get_if<application::DepthTelemetry>(&event)) {
            if (telemetry_frame &&
                telemetry_frame->first == RbrpMessageKind::DepthTelemetry) {
                GatewayDepthTelemetry value;
                value.sequence = telemetry_frame->second;
                value.schema_version = depth->schema_version;
                value.validity_flags = depth->validity_flags;
                value.depth_mm = depth->depth_mm;
                value.temperature_centi_c = depth->temperature_centi_c;
                value.sample_age_ms = depth->sample_age_ms;
                value.diagnostics.rx_byte_count =
                    depth->diagnostics.rx_byte_count;
                value.diagnostics.valid_line_count =
                    depth->diagnostics.valid_line_count;
                value.diagnostics.parse_error_count =
                    depth->diagnostics.parse_error_count;
                value.diagnostics.overlong_line_count =
                    depth->diagnostics.overlong_line_count;
                value.diagnostics.rx_buffer_overflow_count =
                    depth->diagnostics.rx_buffer_overflow_count;
                value.diagnostics.hard_rearm_failure_count =
                    depth->diagnostics.hard_rearm_failure_count;
                value.diagnostics.uart_error_count =
                    depth->diagnostics.uart_error_count;
                mapped.emplace_back(GatewayTelemetryEvent{value});
            }
            telemetry_frame.reset();
            continue;
        }
        if (const auto *cpg = std::get_if<application::CpgParametersTelemetry>(&event))
        {
            if (telemetry_frame &&
                telemetry_frame->first == RbrpMessageKind::CpgParametersTelemetry)
            {
                const auto now = static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count());
                mapped.emplace_back(GatewayTelemetryEvent{
                    GatewayCpgParametersTelemetry{link_epoch_, now, cpg->snapshot_payload}});
            }
            telemetry_frame.reset();
            continue;
        }
        if (const auto *motion = std::get_if<application::MotionStateTelemetry>(&event)) {
            if (telemetry_frame && telemetry_frame->first == RbrpMessageKind::MotionStateTelemetry) {
                mapped.emplace_back(GatewayTelemetryEvent{GatewayMotionStateTelemetry{
                    link_epoch_, motion_rx_ms, 0, motion->batch_payload}});
            }
            telemetry_frame.reset();
            continue;
        }
        if (const auto *malformed =
                std::get_if<application::TelemetryMalformed>(&event)) {
            mapped.emplace_back(GatewayTelemetryMalformedDiagnostic{
                telemetry_kind(malformed->message_type),
                malformed->sequence,
                map_malformed_reason(malformed->reason)});
            telemetry_frame.reset();
            continue;
        }
    }
    return mapped;
}

GatewayApplicationRunResult LinuxOnboardApplicationPort::run_once()
{
    const auto result = application_.run_once();
    return {map_run_status(result.status), map_events(result.events),
            result.error_number};
}

GatewayApplicationAbortResult LinuxOnboardApplicationPort::abort()
{
    const auto result = application_.abort();
    return {map_events(result.events)};
}

GatewayApplicationSubmitResult
LinuxOnboardApplicationPort::submit(const RobotCommand &command)
{
    const auto submit = [&application = application_](const auto &value) {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, SetCpgParameters>)
        {
            return application.set_cpg_parameters(value.parameters);
        } else if constexpr (std::is_same_v<T, QueryCpgParameters>)
        {
            return application.query_cpg_parameters();
        } else if constexpr (std::is_same_v<T, EnableServos>)
        {
            return application.enable_servos(value.mask);
        } else if constexpr (std::is_same_v<T, DisableServos>)
        {
            return application.disable_servos(value.mask);
        } else if constexpr (std::is_same_v<T, SetServoAngle>)
        {
            return application.set_servo_angle(
                static_cast<application::ServoId>(value.servo_id),
                value.angle_cdeg);
        } else if constexpr (std::is_same_v<T, SetServoPwm>)
        {
            return application.set_servo_pwm_maintenance(
                static_cast<application::ServoId>(value.servo_id),
                value.pulse_us);
        } else if constexpr (std::is_same_v<T, NeutralServos>)
        {
            return application.neutral_servos(value.mask);
        } else if constexpr (std::is_same_v<T, StartMotion>)
        {
            return application.start_motion(
                static_cast<application::MotionMode>(value.mode));
        } else if constexpr (std::is_same_v<T, StopMotion>)
        {
            return application.stop_motion();
        } else if constexpr (std::is_same_v<T, SetGaitBackend>)
        {
            return application.set_gait_backend(
                static_cast<application::GaitBackend>(value.backend));
        } else if constexpr (std::is_same_v<T, SetFrontRearCoordination>)
        {
            return application.set_front_rear_coordination(
                static_cast<application::FrontRearCoordination>(
                    value.coordination));
        }
    };
    return map_submit_result(std::visit(submit, command));
}

GatewayApplicationSessionState
LinuxOnboardApplicationPort::session_state() const noexcept
{
    return map_session_state(application_.session_state());
}

GatewayApplicationLinkState LinuxOnboardApplicationPort::link_state() const
{
    return map_link_state(application_.link_state());
}

} // namespace robobeetle::gateway
