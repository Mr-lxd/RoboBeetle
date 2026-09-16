#if !__has_include("robobeetle/application/onboard_application.hpp")
#include <cstdio>
int main()
{
    std::fprintf(stderr, "FAIL: Slice 6 OnboardApplication is not implemented\n");
    return 1;
}
#else
#include "test_support.hpp"
#include "robobeetle/application/onboard_application.hpp"
#include "robobeetle/protocol/codec.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>

// The lifecycle tests substitute only the existing clock seam. The admission
// backpressure test additionally suppresses writable readiness for one poll.
// Serial I/O, decoding, ACK correlation and session lifecycle remain real.
namespace robobeetle::application::detail {
struct OnboardApplicationTestAccess {
    static runtime::LinkRuntime &runtime(OnboardApplication &app) { return app.runtime_; }
};
}
namespace robobeetle::runtime::detail {
struct LinkRuntimeTestAccess {
    static void clock(LinkRuntime &runtime, TimeMs (*clock)()) { runtime.clock_fn_ = clock; }
    static void poller(LinkRuntime &runtime, int (*poller)(pollfd *, nfds_t, int))
    { runtime.poll_fn_ = poller; }
};
}
namespace rbp2_test { int failures = 0; }

namespace {
using namespace robobeetle;
using namespace robobeetle::application;
using rbp2_test::expect;
using protocol::Bytes;
using protocol::Codec;
using link_core::LinkEventType;

runtime::TimeMs now_ms = 0;
runtime::TimeMs clock_now() { return now_ms; }

int readable_only_poll(pollfd *fds, nfds_t count, int timeout)
{
    for (nfds_t i = 0; i < count; ++i) { fds[i].events &= static_cast<short>(~POLLOUT); }
    return ::poll(fds, count, timeout);
}

int poll_error = 0;
int failing_poll(pollfd *, nfds_t, int)
{
    errno = poll_error;
    return -1;
}

struct Pty {
    int master{-1};
    std::string path;
    Pty()
    {
        master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (master < 0) { throw std::runtime_error("posix_openpt failed"); }
        if (::grantpt(master) != 0 || ::unlockpt(master) != 0 || !::ptsname(master)) {
            ::close(master);
            throw std::runtime_error("PTY setup failed");
        }
        path = ::ptsname(master);
    }
    ~Pty() { if (master >= 0) { ::close(master); } }
    void disconnect() { ::close(master); master = -1; }
    void send(const Bytes &bytes) const
    {
        if (::write(master, bytes.data(), bytes.size()) != static_cast<ssize_t>(bytes.size())) {
            throw std::runtime_error("PTY write failed");
        }
    }
    Bytes receive() const
    {
        Bytes bytes;
        std::uint8_t buffer[512]{};
        pollfd ready{master, POLLIN, 0};
        while (::poll(&ready, 1, 0) == 1 && (ready.revents & POLLIN)) {
            const auto n = ::read(master, buffer, sizeof(buffer));
            if (n <= 0) { break; }
            bytes.insert(bytes.end(), buffer, buffer + n);
            ready.revents = 0;
        }
        return bytes;
    }
};

Bytes ack(std::uint16_t sequence, std::uint8_t type, std::uint8_t result = 0)
{
    return Codec::encodeWire({0x02, 0x8000,
        {static_cast<std::uint8_t>(sequence & 0xffU),
         static_cast<std::uint8_t>(sequence >> 8U), type, result}});
}

const link_core::LinkEvent *find_event(const std::vector<ApplicationEvent> &events,
                                     LinkEventType type)
{
    for (const auto &event : events) {
        const auto *raw = std::get_if<link_core::LinkEvent>(&event);
        if (raw && raw->type == type) { return raw; }
    }
    return nullptr;
}

template<class T>
std::size_t count(const std::vector<ApplicationEvent> &events)
{
    return static_cast<std::size_t>(std::count_if(events.begin(), events.end(),
        [](const auto &event) { return std::holds_alternative<T>(event); }));
}

struct Fixture {
    Pty pty;
    OnboardApplication app;
    explicit Fixture(link_core::LinkCoreConfig config = {}, std::size_t max_tx_frames = 64)
        : app(config, max_tx_frames)
    {
        now_ms = 0;
        runtime::detail::LinkRuntimeTestAccess::clock(
            application::detail::OnboardApplicationTestAccess::runtime(app), clock_now);
    }
    void activate(Pty &port, runtime::TimeMs quiet_end, std::uint16_t sequence,
                  std::uint32_t uptime)
    {
        expect(app.open(port.path.c_str()) == 0, "application opens PTY explicitly");
        expect(app.session_state() == session::SessionState::SafetyQuiet,
               "open enters SafetyQuiet");
        now_ms = quiet_end - 1;
        port.send({0x7e}); // readability permits a real poll before the quiet deadline
        app.run_once();
        expect(port.receive().empty() && app.session_state() == session::SessionState::SafetyQuiet,
               "no wire TX before 575-ms deadline");
        now_ms = quiet_end;
        const auto resync = app.run_once();
        expect(port.receive() == Bytes{0}, "standalone raw zero precedes heartbeat");
        expect(find_event(resync.events, LinkEventType::HeartbeatDispatched) != nullptr,
               "raw drain schedules heartbeat through runtime");
        app.run_once();
        expect(port.receive() == Codec::encodeWire({0x01, sequence,
            {static_cast<std::uint8_t>(uptime), static_cast<std::uint8_t>(uptime >> 8U),
             static_cast<std::uint8_t>(uptime >> 16U), static_cast<std::uint8_t>(uptime >> 24U)}}),
               "exact heartbeat frame preserves sequence and uptime");
        port.send(ack(sequence, 0x01));
        app.run_once();
        expect(app.link_state() == link_core::LinkState::Active, "heartbeat ACK activates link");
    }
    void activate() { activate(pty, 575, 0, 0); }
    ApplicationRunResult complete(const CommandSubmitResult &submitted, std::uint8_t type,
                                  const Bytes &payload, std::uint8_t result = 0)
    {
        expect(submitted.status == CommandSubmitStatus::Submitted && submitted.sequence.has_value(),
               "valid command admitted with sequence");
        if (!submitted.sequence) { throw std::runtime_error("missing submitted sequence"); }
        const auto sent = app.run_once();
        expect(!find_event(sent.events, LinkEventType::RequestAccepted),
               "admission and physical TX are not successful ACK");
        expect(pty.receive() == Codec::encodeWire({type, *submitted.sequence, payload}),
               "typed API produces exact Protocol V2 request");
        pty.send(ack(*submitted.sequence, type, result));
        auto completed = app.run_once();
        const auto *outcome = find_event(completed.events,
            result == 0 ? LinkEventType::RequestAccepted : LinkEventType::RequestRejected);
        expect(outcome && outcome->outcome.sequence == *submitted.sequence &&
               outcome->outcome.request_type == type && outcome->outcome.result == result,
               "matching ACK exposes original correlated result");
        return completed;
    }
};

void validation_and_typed_commands()
{
    Fixture f;
    expect(f.app.run_once().status == runtime::RuntimeStatus::NeedsOpen, "closed app needs open");
    expect(f.app.enable_servos(1).status == CommandSubmitStatus::NotActive,
           "valid command before open reports NotActive");
    f.activate();
    for (const auto &result : {
            f.app.enable_servos(0), f.app.disable_servos(0x20), f.app.neutral_servos(0x8000),
            f.app.set_servo_angle(static_cast<ServoId>(5), 0),
            f.app.set_servo_pwm_maintenance(static_cast<ServoId>(255), 1000),
            f.app.start_motion(MotionMode::Stop), f.app.start_motion(static_cast<MotionMode>(7)),
            f.app.set_gait_backend(static_cast<GaitBackend>(2))}) {
        expect(result.status == CommandSubmitStatus::InvalidArgument && !result.sequence,
               "invalid argument is rejected without allocating sequence");
    }
    const auto backward = f.app.start_motion(MotionMode::Backward);
    expect(backward.status == CommandSubmitStatus::PendingQualification && !backward.sequence,
           "Backward is gated pending hardware qualification");
    expect(f.pty.receive().empty(), "local rejection emits no bytes");
    auto first = f.app.enable_servos(0x1f);
    expect(first.sequence == 1, "local rejections do not consume host sequences");
    f.complete(first, 0x10, {0x1f, 0});
    f.complete(f.app.disable_servos(0x10), 0x11, {0x10, 0});
    f.complete(f.app.neutral_servos(3), 0x14, {3, 0});
    f.complete(f.app.set_servo_angle(ServoId::RearLeft, -2), 0x13, {1, 4, 0xfe, 0xff}, 2);
    f.complete(f.app.set_servo_pwm_maintenance(ServoId::FrontAxis, 1500), 0x12, {1, 2, 0xdc, 5});
    for (const auto mode : {MotionMode::Forward, MotionMode::TurnLeft, MotionMode::TurnRight,
                            MotionMode::Ascend, MotionMode::Descend}) {
        f.complete(f.app.start_motion(mode), 0x15, {1, static_cast<std::uint8_t>(mode), 1});
    }
    f.complete(f.app.stop_motion(), 0x15, {1, 0, 0});
    f.complete(f.app.set_gait_backend(GaitBackend::SimpleGait), 0x16, {0});
    f.complete(f.app.set_gait_backend(GaitBackend::CPG), 0x16, {1});
}

void telemetry_interleave_and_malformed()
{
    Fixture f;
    f.activate();
    const auto submitted = f.app.enable_servos(1);
    f.app.run_once();
    f.pty.receive();
    if (!submitted.sequence) { throw std::runtime_error("missing telemetry test sequence"); }
    Bytes imu(56, 0); imu[0] = 1; imu[1] = 7; imu[2] = 0xfe; imu[3] = 0xff;
    Bytes depth(38, 0); depth[0] = 1; depth[1] = 3; depth[2] = 42; depth[8] = 0xff; depth[9] = 0xff;
    const std::vector<protocol::Frame> frames{{0x20, 10, {2}}, {0x21, 11, imu}, {0x22, 12, depth}};
    Bytes incoming = Codec::encodeWire(frames[0]);
    incoming = rbp2_test::concat(incoming, ack(*submitted.sequence, 0x10));
    incoming = rbp2_test::concat(incoming, Codec::encodeWire(frames[1]));
    incoming = rbp2_test::concat(incoming, Codec::encodeWire(frames[2]));
    f.pty.send(incoming);
    const auto result = f.app.run_once();
    expect(find_event(result.events, LinkEventType::RequestAccepted) != nullptr,
           "interleaved telemetry leaves ACK correlation intact");
    expect(count<LeakTelemetry>(result.events) == 1 && count<ImuTelemetry>(result.events) == 1 &&
           count<DepthTelemetry>(result.events) == 1, "one typed event per valid telemetry");
    std::size_t telemetry_index = 0;
    for (std::size_t i = 0; i < result.events.size(); ++i) {
        const auto *raw = std::get_if<link_core::LinkEvent>(&result.events[i]);
        if (!raw || raw->type != LinkEventType::FrameReceived || raw->frame.message_type < 0x20) { continue; }
        expect(telemetry_index < frames.size() && raw->frame == frames[telemetry_index],
               "raw telemetry LinkEvent frame preserved in order");
        expect(i + 1 < result.events.size() && !std::holds_alternative<link_core::LinkEvent>(result.events[i + 1]),
               "raw telemetry event immediately precedes typed event");
        if (i + 1 < result.events.size()) {
            if (const auto *leak = std::get_if<LeakTelemetry>(&result.events[i + 1])) {
                expect(leak->state == LeakState::Wet, "typed Leak value decoded");
            }
            if (const auto *sample = std::get_if<ImuTelemetry>(&result.events[i + 1])) {
                expect(sample->acc_mg[0] == -2, "typed IMU signed field decoded");
            }
            if (const auto *sample = std::get_if<DepthTelemetry>(&result.events[i + 1])) {
                expect(sample->depth_mm == 42 && sample->sample_age_ms == 0xffff, "typed Depth decoded");
            }
        }
        ++telemetry_index;
    }
    expect(telemetry_index == 3, "all raw telemetry events retained");
    const auto pending = f.app.neutral_servos(1);
    f.app.run_once(); f.pty.receive();
    for (const auto type : {0x20, 0x21, 0x22}) {
        f.pty.send(Codec::encodeWire({static_cast<std::uint8_t>(type), 20, {0xff}}));
        const auto malformed = f.app.run_once();
        expect(count<TelemetryMalformed>(malformed.events) == 1 &&
               find_event(malformed.events, LinkEventType::FrameReceived),
               "malformed telemetry yields both raw and malformed events");
        const auto *notice = std::get_if<TelemetryMalformed>(&malformed.events.back());
        expect(notice && notice->message_type == type && notice->sequence == 20 &&
               notice->reason == (type == 0x20 ? TelemetryMalformedReason::InvalidValue : TelemetryMalformedReason::WrongLength),
               "malformed notice identifies frame and exact decode error");
        expect(!find_event(malformed.events, LinkEventType::RequestAccepted) &&
               !find_event(malformed.events, LinkEventType::RequestRejected) &&
               !find_event(malformed.events, LinkEventType::StateChanged) &&
               f.app.link_state() == link_core::LinkState::Active,
               "malformed telemetry neither completes command nor tears down link");
    }
    if (!pending.sequence) { throw std::runtime_error("missing pending sequence"); }
    f.pty.send(ack(*pending.sequence, 0x14));
    expect(find_event(f.app.run_once().events, LinkEventType::RequestAccepted),
           "pending command still completes on its own ACK");
}

void loss_explicit_reopen_no_replay()
{
    Fixture f;
    f.activate();
    const auto pending = f.app.set_servo_angle(ServoId::FrontRight, 1000);
    const auto queued = f.app.neutral_servos(1);
    f.app.run_once();
    expect(pending.sequence == 1 && queued.sequence == 2 &&
           f.pty.receive() == Codec::encodeWire({0x13, 1, {1, 0, 0xe8, 3}}),
           "one request transmitted while later request remains queued");
    now_ms = 580;
    f.pty.disconnect();
    const auto lost = f.app.run_once();
    const auto *unknown = find_event(lost.events, LinkEventType::RequestOutcomeUnknown);
    const auto *cancelled = find_event(lost.events, LinkEventType::RequestCancelled);
    expect(lost.status == runtime::RuntimeStatus::SessionLost && lost.error_number == EIO,
           "runtime status and errno survive facade");
    expect(unknown && unknown->outcome.sequence == 1 && cancelled && cancelled->outcome.sequence == 2,
           "loss preserves pending Unknown and queued Cancelled");
    expect(f.app.session_state() == session::SessionState::ReopenRequired &&
           f.app.link_state() == link_core::LinkState::Lost, "lost state is directly observed");
    expect(f.app.run_once().status == runtime::RuntimeStatus::NeedsOpen && f.app.abort().events.empty(),
           "loss never auto-reopens or repeats outcomes");
    expect(f.app.enable_servos(1).status == CommandSubmitStatus::NotActive, "lost admission stays gated");
    Pty reopened;
    f.activate(reopened, 1155, 3, 580);
    expect(reopened.receive().empty(), "reopen has no replayed actuator frames");
    now_ms = 1255;
    f.app.run_once();
    expect(reopened.receive() == Codec::encodeWire({0x01, 4, {0xa8, 2, 0, 0}}),
           "next idle tick sends only heartbeat, never unknown or cancelled actuator work");
    reopened.send(ack(4, 0x01)); f.app.run_once();
    const auto fresh = f.app.disable_servos(1);
    expect(fresh.sequence == 5, "only explicit new decision submits continuous next sequence");
    f.app.run_once();
    expect(reopened.receive() == Codec::encodeWire({0x11, 5, {1, 0}}), "fresh explicit command reaches wire");
}

void abort_and_queue_admission()
{
    link_core::LinkCoreConfig config;
    config.ordinary_queue_capacity = 1;
    Fixture f(config);
    f.activate();
    const auto first = f.app.enable_servos(1);
    const auto second = f.app.neutral_servos(1);
    const auto full = f.app.disable_servos(1);
    expect(first.status == CommandSubmitStatus::Submitted && second.status == CommandSubmitStatus::Submitted &&
           full.status == CommandSubmitStatus::QueueFull && !full.sequence, "QueueFull is distinct from admission");
    const auto result = f.app.abort();
    expect(find_event(result.events, LinkEventType::RequestOutcomeUnknown) &&
           find_event(result.events, LinkEventType::RequestCancelled), "explicit abort returns original outcomes");
    expect(f.app.abort().events.empty(), "repeated abort has no invented success");
}

void transport_admission_rejection()
{
    Fixture f({}, 1);
    f.activate();
    auto &runtime_ref = application::detail::OnboardApplicationTestAccess::runtime(f.app);
    runtime::detail::LinkRuntimeTestAccess::poller(runtime_ref, readable_only_poll);
    // The due heartbeat owns the only software transport frame slot. Restrict
    // this one readiness notification to RX so the frame cannot drain yet.
    now_ms = 675;
    f.pty.send(Codec::encodeWire({0x20, 1, {1}}));
    f.app.run_once();
    const auto rejected = f.app.enable_servos(1);
    expect(rejected.status == CommandSubmitStatus::TransportRejected && !rejected.sequence,
           "transport ownership rejection is distinct from application admission");
    runtime::detail::LinkRuntimeTestAccess::poller(runtime_ref, ::poll);
}

void runtime_error_results_are_preserved()
{
    Fixture f;
    f.activate();
    auto &runtime_ref = application::detail::OnboardApplicationTestAccess::runtime(f.app);
    runtime::detail::LinkRuntimeTestAccess::poller(runtime_ref, failing_poll);
    poll_error = EINTR;
    const auto interrupted = f.app.run_once();
    expect(interrupted.status == runtime::RuntimeStatus::Interrupted && interrupted.error_number == EINTR &&
           f.app.link_state() == link_core::LinkState::Active,
           "Interrupted and errno pass through without application retry");
    const auto command = f.app.neutral_servos(1);
    expect(command.status == CommandSubmitStatus::Submitted, "admission after interruption remains active");
    poll_error = ENOMEM;
    const auto fatal = f.app.run_once();
    expect(fatal.status == runtime::RuntimeStatus::PollFatal && fatal.error_number == ENOMEM &&
           find_event(fatal.events, LinkEventType::RequestOutcomeUnknown),
           "PollFatal preserves errno and uncertain request outcome");
    expect(f.app.run_once().status == runtime::RuntimeStatus::NeedsOpen,
           "application reports fatal state without automatic reconnect");
}
}

int main()
{
    try {
        validation_and_typed_commands();
        telemetry_interleave_and_malformed();
        loss_explicit_reopen_no_replay();
        abort_and_queue_admission();
        transport_admission_rejection();
        runtime_error_results_are_preserved();
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    if (rbp2_test::failures) { return EXIT_FAILURE; }
    std::cout << "All onboard application PTY tests passed\n";
    return EXIT_SUCCESS;
}
#endif
