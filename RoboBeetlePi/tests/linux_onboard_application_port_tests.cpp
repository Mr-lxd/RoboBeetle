#include "test_support.hpp"
#include "robobeetle/gateway/linux_onboard_application_port.hpp"
#include "robobeetle/protocol/codec.hpp"
#include "robobeetle/protocol/message_types.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fcntl.h>
#include <iterator>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unistd.h>
#include <utility>
#include <vector>

namespace robobeetle::application::detail {
struct OnboardApplicationTestAccess {
    static runtime::LinkRuntime &runtime(OnboardApplication &application)
    {
        return application.runtime_;
    }
};
} // namespace robobeetle::application::detail

namespace robobeetle::runtime::detail {
struct LinkRuntimeTestAccess {
    static void clock(LinkRuntime &runtime, TimeMs (*clock)())
    {
        runtime.clock_fn_ = clock;
    }
};
} // namespace robobeetle::runtime::detail

namespace robobeetle::gateway::detail {
struct LinuxOnboardApplicationPortTestAccess {
    static application::OnboardApplication &
    application(LinuxOnboardApplicationPort &port)
    {
        return port.application_;
    }

    static GatewayApplicationSubmitResult map_submit_result(
        const LinuxOnboardApplicationPort &port,
        const application::CommandSubmitResult &result)
    {
        return port.map_submit_result(result);
    }
};
} // namespace robobeetle::gateway::detail

namespace rbp2_test { int failures = 0; }

namespace {

using namespace robobeetle;
using namespace robobeetle::gateway;
using robobeetle::protocol::Bytes;
using robobeetle::protocol::Codec;
using robobeetle::protocol::Frame;
using robobeetle::protocol::MessageType;
using rbp2_test::expect;

GatewayTimeMs now_ms = 0U;

GatewayTimeMs gateway_clock()
{
    return now_ms;
}

template <typename T, typename Variant>
struct variant_contains;

template <typename T, typename... Alternatives>
struct variant_contains<T, std::variant<Alternatives...>>
    : std::bool_constant<(std::is_same_v<T, Alternatives> || ...)> {};

struct Pty {
    int master{-1};
    std::string path;

    Pty()
    {
        master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (master < 0 || ::grantpt(master) != 0 || ::unlockpt(master) != 0 ||
            ::ptsname(master) == nullptr) {
            if (master >= 0) {
                ::close(master);
            }
            throw std::runtime_error("PTY setup failed");
        }
        path = ::ptsname(master);
    }

    ~Pty()
    {
        if (master >= 0) {
            ::close(master);
        }
    }

    void send(const Bytes &wire) const
    {
        const auto written =
            ::write(master, wire.data(), wire.size());
        if (written != static_cast<ssize_t>(wire.size())) {
            throw std::runtime_error("PTY write failed");
        }
    }

    Bytes receive()
    {
        Bytes result;
        std::array<std::uint8_t, 512U> buffer{};
        for (;;) {
            pollfd ready{master, POLLIN, 0};
            const int polled = ::poll(&ready, 1, 0);
            if (polled <= 0 || (ready.revents & POLLIN) == 0) {
                break;
            }
            const auto received =
                ::read(master, buffer.data(), buffer.size());
            if (received <= 0) {
                break;
            }
            result.insert(result.end(), buffer.begin(),
                          buffer.begin() + received);
        }
        return result;
    }

    Bytes receive_expected(const Bytes &expected)
    {
        Bytes result;
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (result.size() < expected.size() &&
               std::chrono::steady_clock::now() < deadline) {
            pollfd ready{master, POLLIN, 0};
            const int polled = ::poll(&ready, 1, 10);
            if (polled < 0 && errno == EINTR) {
                continue;
            }
            if (polled < 0 ||
                (ready.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                throw std::runtime_error("PTY receive poll failed");
            }
            if (polled == 0) {
                continue;
            }
            const auto available = receive();
            result.insert(result.end(), available.begin(), available.end());
        }
        if (result.size() < expected.size()) {
            throw std::runtime_error("PTY receive timeout");
        }
        return result;
    }
};

Bytes ack(std::uint16_t sequence, std::uint8_t type,
          std::uint8_t result = 0U)
{
    return Codec::encodeWire(Frame{
        type, sequence,
        {static_cast<std::uint8_t>(sequence & 0xffU),
         static_cast<std::uint8_t>(sequence >> 8U), type, result}});
}

void install_test_clock(LinuxOnboardApplicationPort &port)
{
    auto &application =
        detail::LinuxOnboardApplicationPortTestAccess::application(port);
    robobeetle::runtime::detail::LinkRuntimeTestAccess::clock(
        robobeetle::application::detail::OnboardApplicationTestAccess::runtime(
            application),
        gateway_clock);
}

struct Fixture {
    Pty pty;
    LinuxOnboardApplicationPort port;

    Fixture()
        : port(pty.path)
    {
        now_ms = 0U;
        install_test_clock(port);
    }

    application::OnboardApplication &application()
    {
        return detail::LinuxOnboardApplicationPortTestAccess::application(port);
    }

    void activate()
    {
        expect(port.open() == 0, "Linux port explicitly opens the configured PTY");
        expect(port.session_state() ==
                   GatewayApplicationSessionState::SafetyQuiet,
               "Linux port maps open to SafetyQuiet");

        now_ms = 574U;
        pty.send({0x7eU});
        (void)port.run_once();
        expect(pty.receive().empty(),
               "application remains quiet before the safety deadline");

        now_ms = 575U;
        (void)port.run_once();
        expect(pty.receive_expected({0U}) == Bytes{0U},
               "application emits only the standalone resynchronization zero");

        (void)port.run_once();
        const auto heartbeat =
            Codec::encodeWire(Frame{0x01U, 0U, {0U, 0U, 0U, 0U}});
        expect(pty.receive_expected(heartbeat) == heartbeat,
               "Linux port preserves the real heartbeat wire sequence");

        pty.send(ack(0U, 0x01U));
        now_ms = 576U;
        (void)port.run_once();
        expect(port.link_state() == GatewayApplicationLinkState::Active,
               "heartbeat ACK maps the port link to Active");
    }
};

void submit_result_mapping_preserves_lower_layer_invariants()
{
    LinuxOnboardApplicationPort port("/unused");
    const auto submitted_without_sequence =
        detail::LinuxOnboardApplicationPortTestAccess::map_submit_result(
            port,
            application::CommandSubmitResult{
                application::CommandSubmitStatus::Submitted, std::nullopt});
    expect(submitted_without_sequence.status ==
                   GatewayApplicationSubmitStatus::Submitted &&
               !submitted_without_sequence.sequence.has_value(),
           "Linux application-port mapping preserves Submitted without sequence");

    const auto not_active_with_sequence =
        detail::LinuxOnboardApplicationPortTestAccess::map_submit_result(
            port,
            application::CommandSubmitResult{
                application::CommandSubmitStatus::NotActive,
                static_cast<std::uint16_t>(123U)});
    expect(not_active_with_sequence.status ==
                   GatewayApplicationSubmitStatus::NotActive &&
               not_active_with_sequence.sequence.has_value() &&
               *not_active_with_sequence.sequence == 123U,
           "Linux application-port mapping preserves non-Submitted sequence");
}

const GatewayCommandOutcomeEvent *find_outcome(
    const GatewayApplicationRunResult &result)
{
    for (const auto &event : result.events) {
        if (const auto *outcome =
                std::get_if<GatewayCommandOutcomeEvent>(&event)) {
            return outcome;
        }
    }
    return nullptr;
}

const GatewayTelemetryEvent *find_telemetry(
    const GatewayApplicationRunResult &result)
{
    for (const auto &event : result.events) {
        if (const auto *telemetry =
                std::get_if<GatewayTelemetryEvent>(&event)) {
            return telemetry;
        }
    }
    return nullptr;
}

const GatewayTelemetryMalformedDiagnostic *find_malformed(
    const GatewayApplicationRunResult &result)
{
    for (const auto &event : result.events) {
        if (const auto *malformed =
                std::get_if<GatewayTelemetryMalformedDiagnostic>(&event)) {
            return malformed;
        }
    }
    return nullptr;
}

void expect_no_raw_events(const GatewayApplicationRunResult &result)
{
    static_assert(
        !variant_contains<link_core::LinkEvent, GatewayApplicationEvent>::value,
        "neutral application port must not expose raw LinkEvent");
    (void)result;
}

GatewayApplicationRunResult submit_and_ack(
    Fixture &fixture, const RobotCommand &command, std::uint8_t type,
    const Bytes &payload, std::uint8_t ack_result = 0U)
{
    const auto submitted = fixture.port.submit(command);
    expect(submitted.status ==
                   GatewayApplicationSubmitStatus::Submitted &&
               submitted.sequence.has_value(),
           "valid typed command maps to Submitted with a sequence");
    if (!submitted.sequence) {
        return {};
    }

    now_ms = 577U;
    const auto dispatch = fixture.port.run_once();
    expect_no_raw_events(dispatch);
    const auto wire = Codec::encodeWire(
        Frame{type, *submitted.sequence, payload});
    expect(fixture.pty.receive_expected(wire) == wire,
           "typed gateway command maps to exact Protocol V2 wire bytes");

    fixture.pty.send(ack(*submitted.sequence, type, ack_result));
    now_ms = 578U;
    const auto result = fixture.port.run_once();
    expect_no_raw_events(result);
    return result;
}

void typed_commands_and_submit_statuses()
{
    Fixture fixture;
    fixture.activate();

    const auto accepted = submit_and_ack(
        fixture, EnableServos{0x001fU}, 0x10U, {0x1fU, 0x00U});
    const auto *accepted_outcome = find_outcome(accepted);
    expect(accepted_outcome != nullptr &&
               accepted_outcome->outcome ==
                   GatewayCommandOutcome::Accepted &&
               accepted_outcome->sequence != 0U,
           "EnableServos maps accepted ACK to one neutral outcome");

    const auto rejected = submit_and_ack(
        fixture, DisableServos{0x0001U}, 0x11U, {0x01U, 0x00U}, 5U);
    const auto *rejected_outcome = find_outcome(rejected);
    expect(rejected_outcome != nullptr &&
               rejected_outcome->outcome ==
                   GatewayCommandOutcome::Rejected &&
               rejected_outcome->result == 5U,
           "DisableServos maps rejected ACK and preserves result code");

    (void)submit_and_ack(
        fixture, SetServoAngle{4U, -2}, 0x13U,
        {0x01U, 0x04U, 0xfeU, 0xffU});
    (void)submit_and_ack(
        fixture, NeutralServos{0x0002U}, 0x14U, {0x02U, 0x00U});
    (void)submit_and_ack(
        fixture, StartMotion{MotionMode::Forward}, 0x15U,
        {0x01U, 0x01U, 0x01U});
    (void)submit_and_ack(
        fixture, StopMotion{}, 0x15U, {0x01U, 0x00U, 0x00U});
    (void)submit_and_ack(
        fixture, SetGaitBackend{GaitBackend::CPG}, 0x16U, {0x01U});

    const auto invalid = fixture.port.submit(EnableServos{0U});
    expect(invalid.status ==
               GatewayApplicationSubmitStatus::InvalidArgument &&
               !invalid.sequence.has_value(),
           "invalid mask maps to InvalidArgument without a sequence");
    expect(fixture.pty.receive().empty(),
           "invalid typed command emits zero UART command bytes");

    const auto backward =
        fixture.port.submit(StartMotion{MotionMode::Backward});
    expect(backward.status ==
               GatewayApplicationSubmitStatus::PendingQualification &&
               !backward.sequence.has_value(),
           "Backward remains PendingQualification with no sequence");
    expect(fixture.pty.receive().empty(),
           "Backward emits zero UART command bytes");

    const auto aborted = fixture.port.abort();
    expect(fixture.port.session_state() ==
               GatewayApplicationSessionState::ReopenRequired,
           "abort maps the session back to ReopenRequired");
    expect_no_raw_events(
        GatewayApplicationAbortResult{aborted.events}.events.empty()
            ? GatewayApplicationRunResult{}
            : GatewayApplicationRunResult{
                  GatewayApplicationRunStatus::Progress, aborted.events, 0});
    const auto not_active = fixture.port.submit(StopMotion{});
    expect(not_active.status ==
               GatewayApplicationSubmitStatus::NotActive &&
               !not_active.sequence.has_value(),
           "submit outside an online session maps to NotActive");
}

void queue_full_status_has_no_sequence()
{
    Fixture fixture;
    fixture.activate();

    std::vector<GatewayApplicationSubmitResult> results;
    for (std::size_t i = 0U; i < 10U; ++i) {
        results.push_back(fixture.port.submit(StopMotion{}));
    }
    const auto &full = results.back();
    expect(full.status == GatewayApplicationSubmitStatus::QueueFull &&
               !full.sequence.has_value(),
           "underlying bounded command queue maps QueueFull without sequence");
    (void)fixture.port.abort();
}

void telemetry_and_malformed_data_map_once()
{
    Fixture fixture;
    fixture.activate();

    fixture.pty.send(Codec::encodeWire(
        Frame{static_cast<std::uint8_t>(MessageType::LeakStatus), 55U, {2U}}));
    now_ms = 580U;
    const auto leak = fixture.port.run_once();
    expect_no_raw_events(leak);
    const auto *leak_event = find_telemetry(leak);
    expect(leak_event != nullptr &&
               std::get_if<GatewayLeakTelemetry>(leak_event) != nullptr &&
               std::get<GatewayLeakTelemetry>(*leak_event).sequence == 55U &&
               std::get<GatewayLeakTelemetry>(*leak_event).state ==
                   LeakState::Wet,
           "typed Leak telemetry maps with its raw frame sequence exactly once");

    Bytes imu(56U, 0U);
    imu[0] = 1U;
    imu[1] = 0x07U;
    imu[2] = 0x01U;
    fixture.pty.send(Codec::encodeWire(
        Frame{static_cast<std::uint8_t>(MessageType::ImuSnapshot), 56U, imu}));
    now_ms = 581U;
    const auto imu_result = fixture.port.run_once();
    const auto *imu_event = find_telemetry(imu_result);
    expect(imu_event != nullptr &&
               std::get_if<GatewayImuTelemetry>(imu_event) != nullptr &&
               std::get<GatewayImuTelemetry>(*imu_event).sequence == 56U &&
               std::get<GatewayImuTelemetry>(*imu_event).schema_version == 1U &&
               std::get<GatewayImuTelemetry>(*imu_event).validity_flags == 0x07U,
           "typed IMU telemetry maps neutral fields and sequence");

    Bytes depth(38U, 0U);
    depth[0] = 1U;
    depth[1] = 0x03U;
    fixture.pty.send(Codec::encodeWire(
        Frame{static_cast<std::uint8_t>(MessageType::DepthSnapshot), 57U,
              depth}));
    now_ms = 582U;
    const auto depth_result = fixture.port.run_once();
    const auto *depth_event = find_telemetry(depth_result);
    expect(depth_event != nullptr &&
               std::get_if<GatewayDepthTelemetry>(depth_event) != nullptr &&
               std::get<GatewayDepthTelemetry>(*depth_event).sequence == 57U &&
               std::get<GatewayDepthTelemetry>(*depth_event).schema_version == 1U,
           "typed Depth telemetry maps neutral fields and sequence");

    fixture.pty.send(Codec::encodeWire(
        Frame{static_cast<std::uint8_t>(MessageType::LeakStatus), 58U,
              {3U}}));
    now_ms = 583U;
    const auto malformed = fixture.port.run_once();
    expect_no_raw_events(malformed);
    const auto *diagnostic = find_malformed(malformed);
    expect(diagnostic != nullptr &&
               diagnostic->kind == RbrpMessageKind::LeakTelemetry &&
               diagnostic->sequence == 58U,
           "malformed telemetry becomes one local diagnostic");
    expect(find_telemetry(malformed) == nullptr,
           "malformed telemetry never becomes a valid neutral telemetry event");
}

void application_state_and_all_outcome_kinds_map()
{
    Fixture fixture;
    fixture.activate();

    auto queued = fixture.port.submit(StopMotion{});
    expect(queued.status == GatewayApplicationSubmitStatus::Submitted &&
               queued.sequence.has_value(),
           "queued command can be aborted for Cancelled mapping");
    const auto cancelled = fixture.port.abort();
    expect_no_raw_events(
        GatewayApplicationAbortResult{cancelled.events}.events.empty()
            ? GatewayApplicationRunResult{}
            : GatewayApplicationRunResult{
                  GatewayApplicationRunStatus::Progress, cancelled.events, 0});
    const auto *cancelled_outcome =
        find_outcome(GatewayApplicationRunResult{
            GatewayApplicationRunStatus::Progress, cancelled.events, 0});
    expect(cancelled_outcome != nullptr &&
               cancelled_outcome->outcome ==
                   GatewayCommandOutcome::Cancelled,
           "queued command abort maps to Cancelled");

    Fixture unknown_fixture;
    unknown_fixture.activate();
    auto pending = unknown_fixture.port.submit(StopMotion{});
    expect(pending.status == GatewayApplicationSubmitStatus::Submitted &&
               pending.sequence.has_value(),
           "pending command can be aborted for Unknown mapping");
    now_ms = 584U;
    (void)unknown_fixture.port.run_once();
    (void)unknown_fixture.pty.receive();
    const auto unknown = unknown_fixture.port.abort();
    const auto *unknown_outcome =
        find_outcome(GatewayApplicationRunResult{
            GatewayApplicationRunStatus::Progress, unknown.events, 0});
    expect(unknown_outcome != nullptr &&
               unknown_outcome->outcome ==
                   GatewayCommandOutcome::OutcomeUnknown,
           "in-flight command abort maps to OutcomeUnknown");
}

} // namespace

int main()
{
    try {
        submit_result_mapping_preserves_lower_layer_invariants();
        typed_commands_and_submit_statuses();
        queue_full_status_has_no_sequence();
        telemetry_and_malformed_data_map_once();
        application_state_and_all_outcome_kinds_map();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return EXIT_FAILURE;
    }

    if (rbp2_test::failures == 0) {
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}
