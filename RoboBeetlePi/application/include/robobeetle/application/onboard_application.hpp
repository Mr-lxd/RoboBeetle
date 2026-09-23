#pragma once

#include "robobeetle/application/robot_codec.hpp"
#include "robobeetle/runtime/link_runtime.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <variant>
#include <vector>

namespace robobeetle::application {
namespace detail { struct OnboardApplicationTestAccess; }

enum class CommandSubmitStatus {
    Submitted, InvalidArgument, PendingQualification, NotActive,
    PayloadTooLarge, QueueFull, TransportRejected,
};

// Admission only. Execution acknowledgement arrives later as a raw LinkEvent;
// even an accepted ACK does not establish physical robot state.
struct CommandSubmitResult {
    CommandSubmitStatus status{CommandSubmitStatus::NotActive};
    std::optional<std::uint16_t> sequence;
};

struct TelemetryMalformed {
    protocol::Byte message_type{};
    std::uint16_t sequence{};
    TelemetryMalformedReason reason{TelemetryMalformedReason::None};
};

// Every LinkEvent is preserved. A telemetry FrameReceived is immediately
// followed by its typed value or malformed notice, never a synthetic ACK.
using ApplicationEvent = std::variant<link_core::LinkEvent, LeakTelemetry,
    ImuTelemetry, DepthTelemetry, TelemetryMalformed>;

struct ApplicationRunResult {
    runtime::RuntimeStatus status{runtime::RuntimeStatus::Progress};
    std::vector<ApplicationEvent> events;
    int error_number{0};
};

struct ApplicationAbortResult {
    std::vector<ApplicationEvent> events;
};

// Linux-only, single owner/thread. No robot-state cache, command replay, retry,
// or automatic reconnect: the caller owns explicit open/run/command decisions.
class OnboardApplication final {
public:
    explicit OnboardApplication(link_core::LinkCoreConfig config = {},
                                std::size_t max_tx_frames = 64U,
                                std::size_t max_tx_bytes = 65536U);

    int open(const char *device_path);
    ApplicationRunResult run_once();
    ApplicationAbortResult abort();

    CommandSubmitResult enable_servos(std::uint16_t mask);
    CommandSubmitResult disable_servos(std::uint16_t mask);
    CommandSubmitResult set_servo_angle(ServoId id, std::int16_t angle_cdeg);
    CommandSubmitResult neutral_servos(std::uint16_t mask);
    // Backward is PendingQualification; Stop is invalid here (use stop_motion).
    CommandSubmitResult start_motion(MotionMode mode);
    CommandSubmitResult stop_motion();
    CommandSubmitResult set_gait_backend(GaitBackend backend);
    CommandSubmitResult set_front_rear_coordination(
        FrontRearCoordination coordination);
    // Bring-up / maintenance only; normal motion does not use raw PWM.
    CommandSubmitResult set_servo_pwm_maintenance(ServoId id, std::uint16_t pulse_us);

    [[nodiscard]] session::SessionState session_state() const noexcept;
    [[nodiscard]] link_core::LinkState link_state() const;

private:
    friend struct detail::OnboardApplicationTestAccess;
    CommandSubmitResult submit(protocol::Byte type, const CodecResult &encoded);
    runtime::LinkRuntime runtime_;
};

} // namespace robobeetle::application
