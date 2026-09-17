#include "robobeetle/application/onboard_application.hpp"

#include <charconv>
#include <cerrno>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <poll.h>
#include <unistd.h>

namespace {
namespace app = robobeetle::application;
namespace link = robobeetle::link_core;
namespace session = robobeetle::session;
namespace runtime = robobeetle::runtime;

template <typename Integer>
bool parse_number(std::string_view token, Integer &value, bool allow_hex = false)
{
    int base = 10;
    if (allow_hex && token.size() > 2U && token[0] == '0' &&
        (token[1] == 'x' || token[1] == 'X')) {
        token.remove_prefix(2U);
        base = 16;
    }
    if (token.empty()) return false;
    const auto parsed = std::from_chars(token.data(), token.data() + token.size(), value, base);
    return parsed.ec == std::errc{} && parsed.ptr == token.data() + token.size();
}

const char *name(app::CommandSubmitStatus value)
{
    switch (value) {
    case app::CommandSubmitStatus::Submitted: return "Submitted";
    case app::CommandSubmitStatus::InvalidArgument: return "InvalidArgument";
    case app::CommandSubmitStatus::PendingQualification: return "PendingQualification";
    case app::CommandSubmitStatus::NotActive: return "NotActive";
    case app::CommandSubmitStatus::PayloadTooLarge: return "PayloadTooLarge";
    case app::CommandSubmitStatus::QueueFull: return "QueueFull";
    case app::CommandSubmitStatus::TransportRejected: return "TransportRejected";
    }
    return "Unknown";
}

const char *name(session::SessionState value)
{
    switch (value) {
    case session::SessionState::ReopenRequired: return "ReopenRequired";
    case session::SessionState::SafetyQuiet: return "SafetyQuiet";
    case session::SessionState::Resynchronizing: return "Resynchronizing";
    case session::SessionState::Online: return "Online";
    }
    return "Unknown";
}

const char *name(link::LinkState value)
{
    switch (value) {
    case link::LinkState::Unconfirmed: return "Unconfirmed";
    case link::LinkState::Active: return "Active";
    case link::LinkState::Degraded: return "Degraded";
    case link::LinkState::Lost: return "Lost";
    }
    return "Unknown";
}

const char *name(runtime::RuntimeStatus value)
{
    switch (value) {
    case runtime::RuntimeStatus::Progress: return "Progress";
    case runtime::RuntimeStatus::Timeout: return "Timeout";
    case runtime::RuntimeStatus::Interrupted: return "Interrupted";
    case runtime::RuntimeStatus::NeedsOpen: return "NeedsOpen";
    case runtime::RuntimeStatus::SessionLost: return "SessionLost";
    case runtime::RuntimeStatus::PollFatal: return "PollFatal";
    }
    return "Unknown";
}

const char *name(link::LinkEventType value)
{
    switch (value) {
    case link::LinkEventType::HeartbeatDispatched: return "HeartbeatDispatched";
    case link::LinkEventType::OrdinaryDispatched: return "OrdinaryDispatched";
    case link::LinkEventType::RequestAccepted: return "RequestAccepted";
    case link::LinkEventType::RequestRejected: return "RequestRejected";
    case link::LinkEventType::RequestOutcomeUnknown: return "RequestOutcomeUnknown";
    case link::LinkEventType::RequestCancelled: return "RequestCancelled";
    case link::LinkEventType::AckMalformed: return "AckMalformed";
    case link::LinkEventType::AckIgnored: return "AckIgnored";
    case link::LinkEventType::FrameReceived: return "FrameReceived";
    case link::LinkEventType::UnknownMessage: return "UnknownMessage";
    case link::LinkEventType::DecodeError: return "DecodeError";
    case link::LinkEventType::StateChanged: return "StateChanged";
    case link::LinkEventType::TransportWriteFailed: return "TransportWriteFailed";
    }
    return "Unknown";
}

void print_event(const link::LinkEvent &event)
{
    std::cout << "LinkEvent " << name(event.type) << " state=" << name(event.state);
    if (event.outcome.kind != link::OutcomeKind::None) {
        std::cout << " seq=" << event.outcome.sequence
                  << " request_type=" << unsigned(event.outcome.request_type)
                  << " result=" << unsigned(event.outcome.result);
    }
    std::cout << " frame_type=" << unsigned(event.frame.message_type)
              << " frame_seq=" << event.frame.sequence
              << " payload_bytes=" << event.frame.payload.size()
              << " ack_disposition=" << static_cast<int>(event.ack_disposition)
              << " decode_error=" << static_cast<int>(event.decode_error) << '\n';
}

void print_event(const app::LeakTelemetry &event)
{
    const char *state = "Unknown";
    if (event.state == app::LeakState::Dry) state = "Dry";
    if (event.state == app::LeakState::Wet) state = "Wet";
    std::cout << "Leak " << state << '\n';
}

void print_event(const app::ImuTelemetry &event)
{
    std::cout << "IMU flags=" << unsigned(event.validity_flags)
              << " acc_mg=" << event.acc_mg[0] << ',' << event.acc_mg[1] << ',' << event.acc_mg[2]
              << " gyro_0.1dps=" << event.gyro_tenth_dps[0] << ',' << event.gyro_tenth_dps[1]
              << ',' << event.gyro_tenth_dps[2]
              << " angle_cdeg=" << event.angle_centidegrees[0] << ',' << event.angle_centidegrees[1]
              << ',' << event.angle_centidegrees[2]
              << " valid_frames=" << event.diagnostics.valid_frame_count << '\n';
}

void print_event(const app::DepthTelemetry &event)
{
    std::cout << "Depth flags=" << unsigned(event.validity_flags)
              << " depth_mm=" << event.depth_mm
              << " temperature_centi_c=" << event.temperature_centi_c
              << " sample_age_ms=" << event.sample_age_ms
              << " valid_lines=" << event.diagnostics.valid_line_count << '\n';
}

void print_event(const app::TelemetryMalformed &event)
{
    std::cout << "TelemetryMalformed type=" << unsigned(event.message_type)
              << " seq=" << event.sequence << " reason=" << static_cast<int>(event.reason) << '\n';
}

void print_events(const std::vector<app::ApplicationEvent> &events)
{
    for (const auto &event : events) {
        std::visit([](const auto &value) { print_event(value); }, event);
    }
}

void print_submit(const app::CommandSubmitResult &result)
{
    std::cout << name(result.status);
    if (result.sequence) std::cout << " seq=" << *result.sequence;
    if (result.status == app::CommandSubmitStatus::PendingQualification)
        std::cout << ": Pending hardware qualification";
    std::cout << '\n';
}

void help()
{
    std::cout << "Commands (one explicit operator action per line):\n"
              << "  link status\n  reopen\n  telemetry display\n"
              << "  enable <mask> | disable <mask> | neutral <mask>\n"
              << "  angle <servo 0..4> <signed cdeg>\n"
              << "  gait simple|cpg\n"
              << "  motion forward|turn_left|turn_right|ascend|descend|stop|backward\n"
              << "  help | quit | exit\n"
              << "Masks: decimal or 0x hexadecimal, nonzero subset of 0x001f.\n";
}

void execute(app::OnboardApplication &application,
             const std::string &device_path,
             const std::vector<std::string> &tokens)
{
    if (tokens == std::vector<std::string>{"link", "status"}) {
        std::cout << "session=" << name(application.session_state())
                  << " link=" << name(application.link_state()) << '\n';
        return;
    }
    if (tokens == std::vector<std::string>{"reopen"}) {
        if (application.session_state() != session::SessionState::ReopenRequired) {
            std::cout << "Reopen rejected: session=" << name(application.session_state())
                      << " link=" << name(application.link_state()) << '\n';
            return;
        }
        const int error = application.open(device_path.c_str());
        if (error == 0) {
            std::cout << "Reopen accepted errno=0\n";
        } else {
            std::cout << "Reopen failed errno=" << error << '\n';
        }
        std::cout << "session=" << name(application.session_state())
                  << " link=" << name(application.link_state()) << '\n';
        return;
    }
    if (tokens == std::vector<std::string>{"telemetry", "display"}) {
        std::cout << "Incoming telemetry is displayed continuously with its validity flags.\n";
        return;
    }
    if (tokens == std::vector<std::string>{"help"}) { help(); return; }
    app::CommandSubmitResult result{app::CommandSubmitStatus::InvalidArgument, std::nullopt};
    if (tokens.size() == 2U &&
        (tokens[0] == "enable" || tokens[0] == "disable" || tokens[0] == "neutral")) {
        std::uint16_t mask{};
        if (parse_number(tokens[1], mask, true)) {
            if (tokens[0] == "enable") result = application.enable_servos(mask);
            else if (tokens[0] == "disable") result = application.disable_servos(mask);
            else result = application.neutral_servos(mask);
        }
    } else if (tokens.size() == 3U && tokens[0] == "angle") {
        unsigned id{};
        std::int16_t cdeg{};
        if (parse_number(tokens[1], id) && id <= 4U && parse_number(tokens[2], cdeg))
            result = application.set_servo_angle(static_cast<app::ServoId>(id), cdeg);
    } else if (tokens.size() == 2U && tokens[0] == "gait") {
        if (tokens[1] == "simple") result = application.set_gait_backend(app::GaitBackend::SimpleGait);
        else if (tokens[1] == "cpg") result = application.set_gait_backend(app::GaitBackend::CPG);
    } else if (tokens.size() == 2U && tokens[0] == "motion") {
        if (tokens[1] == "stop") result = application.stop_motion();
        else if (tokens[1] == "forward") result = application.start_motion(app::MotionMode::Forward);
        else if (tokens[1] == "turn_left") result = application.start_motion(app::MotionMode::TurnLeft);
        else if (tokens[1] == "turn_right") result = application.start_motion(app::MotionMode::TurnRight);
        else if (tokens[1] == "ascend") result = application.start_motion(app::MotionMode::Ascend);
        else if (tokens[1] == "descend") result = application.start_motion(app::MotionMode::Descend);
        else if (tokens[1] == "backward") result = application.start_motion(app::MotionMode::Backward);
    }
    print_submit(result);
}

enum class InputStatus { Waiting, Line, End, Error };

// Only stdin is monitored here. LinkRuntime alone owns serial readiness/timers.
// A readiness check precedes each byte read, so a partial line cannot block
// runtime service. The per-iteration budget also bounds continuous stdin input.
InputStatus read_input(std::string &line, bool &overlong, bool disconnected)
{
    constexpr std::size_t max_line_size = 256U;
    for (std::size_t count = 0; count < max_line_size; ++count) {
        pollfd input{STDIN_FILENO, POLLIN, 0};
        const int ready = ::poll(&input, 1, disconnected && count == 0U ? 100 : 0);
        if (ready < 0) return errno == EINTR ? InputStatus::Waiting : InputStatus::Error;
        if (ready == 0) return InputStatus::Waiting;
        if (input.revents & (POLLERR | POLLNVAL)) return InputStatus::Error;
        if (!(input.revents & (POLLIN | POLLHUP))) return InputStatus::Waiting;
        char byte{};
        const auto received = ::read(STDIN_FILENO, &byte, 1);
        if (received == 0) return InputStatus::End;
        if (received < 0) {
            return errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK
                ? InputStatus::Waiting : InputStatus::Error;
        }
        if (byte == '\n') return InputStatus::Line;
        if (line.size() < max_line_size) line.push_back(byte);
        else overlong = true;
    }
    return InputStatus::Waiting;
}
} // namespace

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "Usage: robobeetle_pi_smoke <serial-device>\n";
        return 2;
    }
    app::OnboardApplication application;
    const std::string original_device_path = argv[1];
    const int error = application.open(original_device_path.c_str());
    if (error != 0) {
        std::cerr << "Open failed errno=" << error << '\n';
        return 1;
    }
    std::cout << "Hardware Acceptance: PENDING USER VERIFICATION\n"
              << "Submitted is admission only; inspect subsequent ACK/outcome events.\n"
              << "Runtime remains serviced while waiting for complete input lines.\n"
              << "Incoming events are displayed continuously; link status only reads state.\n"
              << "No automatic reopen. Quit aborts the session; it is not a motion-stop ACK.\n";
    help();
    std::string line;
    bool overlong = false;
    std::optional<runtime::RuntimeStatus> previous_status;
    int exit_status = 0;
    std::cout << "> " << std::flush;
    for (;;) {
        const bool disconnected = application.session_state() == session::SessionState::ReopenRequired;
        const auto input_status = read_input(line, overlong, disconnected);
        if (input_status == InputStatus::End) break; // Discard an incomplete final line.
        if (input_status == InputStatus::Error) {
            std::cerr << "stdin read/poll failed; aborting session\n";
            exit_status = 1;
            break;
        }
        if (input_status == InputStatus::Line) {
            if (overlong) {
                std::cout << "InvalidArgument: input line exceeds 256 bytes\n";
            } else {
                std::istringstream input(line);
                std::vector<std::string> tokens;
                for (std::string token; input >> token;) tokens.push_back(token);
                if (tokens == std::vector<std::string>{"quit"} ||
                    tokens == std::vector<std::string>{"exit"}) break;
                if (!tokens.empty()) execute(application, original_device_path, tokens);
            }
            line.clear();
            overlong = false;
            std::cout << "> " << std::flush;
        }
        const auto result = application.run_once();
        print_events(result.events);
        if (!previous_status || result.status != *previous_status || !result.events.empty()) {
            std::cout << "Runtime " << name(result.status) << " errno=" << result.error_number << '\n';
        }
        previous_status = result.status;
        std::cout << std::flush;
    }
    print_events(application.abort().events);
    return exit_status;
}
