#include "robobeetle/application/onboard_application.hpp"
#include "robobeetle/protocol/message_types.hpp"
#include "robobeetle/protocol/motion_state.hpp"

#include <utility>

namespace robobeetle::application {
namespace {
constexpr protocol::Byte wire_type(protocol::MessageType type) noexcept
{
    return static_cast<protocol::Byte>(type);
}

template<class T>
void append_telemetry(std::vector<ApplicationEvent> &events,
                      const protocol::Frame &frame,
                      std::optional<T> (*decode)(const protocol::Bytes &,
                                                TelemetryMalformedReason *) noexcept)
{
    TelemetryMalformedReason reason = TelemetryMalformedReason::None;
    const auto decoded = decode(frame.payload, &reason);
    if (decoded) {
        events.emplace_back(*decoded);
    } else {
        events.emplace_back(TelemetryMalformed{frame.message_type, frame.sequence, reason});
    }
}

std::vector<ApplicationEvent> translate(std::vector<link_core::LinkEvent> raw)
{
    std::vector<ApplicationEvent> events;
    events.reserve(raw.size());
    for (const auto &event : raw) {
        // Preserve the complete raw event before appending its decoded value.
        events.emplace_back(event);
        if (event.type != link_core::LinkEventType::FrameReceived) { continue; }
        switch (static_cast<protocol::MessageType>(event.frame.message_type)) {
        case protocol::MessageType::LeakStatus:
            append_telemetry(events, event.frame, decode_leak);
            break;
        case protocol::MessageType::ImuSnapshot:
            append_telemetry(events, event.frame, decode_imu);
            break;
        case protocol::MessageType::DepthSnapshot:
            append_telemetry(events, event.frame, decode_depth);
            break;
        case protocol::MessageType::CpgParametersSnapshot:
            if (protocol::decode_cpg_snapshot(event.frame.payload))
                events.emplace_back(CpgParametersTelemetry{event.frame.payload});
            else
                events.emplace_back(TelemetryMalformed{event.frame.message_type,
                                                       event.frame.sequence,
                                                       TelemetryMalformedReason::InvalidValue});
            break;
        case protocol::MessageType::MotionStateBatch:
            if (protocol::decode_motion_state_batch(event.frame.payload)) {
                events.emplace_back(MotionStateTelemetry{event.frame.payload});
            } else {
                events.emplace_back(TelemetryMalformed{event.frame.message_type,
                    event.frame.sequence, TelemetryMalformedReason::InvalidValue});
            }
            break;
        default:
            break;
        }
    }
    return events;
}
}

OnboardApplication::OnboardApplication(link_core::LinkCoreConfig config,
                                     std::size_t max_tx_frames, std::size_t max_tx_bytes)
    : runtime_(config, max_tx_frames, max_tx_bytes)
{
}

int OnboardApplication::open(const char *device_path)
{
    return runtime_.open(device_path);
}

ApplicationRunResult OnboardApplication::run_once()
{
    auto result = runtime_.run_once();
    return {result.status, translate(std::move(result.events)), result.error_number};
}

ApplicationAbortResult OnboardApplication::abort()
{
    return {translate(runtime_.abort())};
}

CommandSubmitResult OnboardApplication::submit(protocol::Byte type, const CodecResult &encoded)
{
    if (encoded.status != CodecStatus::Ok) {
        return {CommandSubmitStatus::InvalidArgument, std::nullopt};
    }
    const auto admitted = runtime_.submit_request(type, encoded.payload);
    switch (admitted.status) {
    case link_core::SubmitStatus::Accepted:
        return {CommandSubmitStatus::Submitted, admitted.sequence};
    case link_core::SubmitStatus::NotActive:
        return {CommandSubmitStatus::NotActive, std::nullopt};
    case link_core::SubmitStatus::PayloadTooLarge:
        return {CommandSubmitStatus::PayloadTooLarge, std::nullopt};
    case link_core::SubmitStatus::QueueFull:
        return {CommandSubmitStatus::QueueFull, std::nullopt};
    case link_core::SubmitStatus::TransportRejected:
        return {CommandSubmitStatus::TransportRejected, std::nullopt};
    }
    return {CommandSubmitStatus::TransportRejected, std::nullopt};
}

CommandSubmitResult OnboardApplication::enable_servos(std::uint16_t mask)
{
    return submit(wire_type(protocol::MessageType::ServoEnable), encode_servo_mask(mask));
}

CommandSubmitResult OnboardApplication::disable_servos(std::uint16_t mask)
{
    return submit(wire_type(protocol::MessageType::ServoDisable), encode_servo_mask(mask));
}

CommandSubmitResult OnboardApplication::set_servo_angle(ServoId id, std::int16_t angle_cdeg)
{
    return submit(wire_type(protocol::MessageType::SetServoAngle), encode_servo_angle(id, angle_cdeg));
}

CommandSubmitResult OnboardApplication::neutral_servos(std::uint16_t mask)
{
    return submit(wire_type(protocol::MessageType::Neutral), encode_servo_mask(mask));
}

CommandSubmitResult OnboardApplication::start_motion(MotionMode mode)
{
    if (mode == MotionMode::Backward) {
        return {CommandSubmitStatus::PendingQualification, std::nullopt};
    }
    return submit(wire_type(protocol::MessageType::SetMotionMode), encode_motion(mode, MotionAction::Start));
}

CommandSubmitResult OnboardApplication::stop_motion()
{
    return submit(wire_type(protocol::MessageType::SetMotionMode), encode_motion(MotionMode::Stop, MotionAction::Stop));
}

CommandSubmitResult OnboardApplication::set_cpg_parameters(const protocol::CpgParameters &p)
{
    const auto encoded = protocol::encode_cpg_parameters(p);
    if (!encoded)
        return {CommandSubmitStatus::InvalidArgument, std::nullopt};
    return submit(wire_type(protocol::MessageType::SetCpgParameters), {CodecStatus::Ok, *encoded});
}
CommandSubmitResult OnboardApplication::start_proportional(const protocol::ProportionalStart &start)
{
    const auto encoded = protocol::encode_proportional_start(start);
    if (!encoded)
        return {CommandSubmitStatus::InvalidArgument, std::nullopt};
    return submit(wire_type(protocol::MessageType::StartProportional), {CodecStatus::Ok, *encoded});
}

bool OnboardApplication::submit_latest_setpoint(const protocol::ProportionalSetpoint &input)
{
    const auto encoded = protocol::encode_proportional_setpoint(input);
    return encoded && runtime_.submit_latest_setpoint(*encoded);
}

void OnboardApplication::clear_latest_setpoint()
{
    runtime_.clear_latest_setpoint();
}

CommandSubmitResult OnboardApplication::query_cpg_parameters()
{
    return submit(wire_type(protocol::MessageType::QueryCpgParameters), {CodecStatus::Ok, {}});
}

CommandSubmitResult OnboardApplication::set_gait_backend(GaitBackend backend)
{
    return submit(wire_type(protocol::MessageType::SetGaitBackend), encode_gait_backend(backend));
}

CommandSubmitResult OnboardApplication::set_front_rear_coordination(
    FrontRearCoordination coordination)
{
    return submit(
        wire_type(protocol::MessageType::SetFrontRearCoordination),
        encode_front_rear_coordination(coordination));
}

CommandSubmitResult OnboardApplication::set_servo_pwm_maintenance(ServoId id, std::uint16_t pulse_us)
{
    return submit(wire_type(protocol::MessageType::SetServoPwm), encode_servo_pwm(id, pulse_us));
}

session::SessionState OnboardApplication::session_state() const noexcept
{
    return runtime_.state();
}

link_core::LinkState OnboardApplication::link_state() const
{
    return runtime_.link_state();
}

} // namespace robobeetle::application
