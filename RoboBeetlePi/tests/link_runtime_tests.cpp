#if !__has_include("robobeetle/runtime/link_runtime.hpp")
#include <cstdio>
int main()
{
    std::fprintf(stderr, "FAIL: Slice 5 LinkRuntime is not implemented\n");
    return 1;
}
#else
#include "test_support.hpp"
#include "robobeetle/runtime/link_runtime.hpp"
#include "robobeetle/protocol/codec.hpp"

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace robobeetle::runtime::detail {
struct LinkRuntimeTestAccess {
    static void inject(LinkRuntime &r, TimeMs (*clock)(),
                       int (*poller)(pollfd *, nfds_t, int))
    { r.clock_fn_ = clock; r.poll_fn_ = poller; }
    static session::SerialSession &session(LinkRuntime &r) { return r.session_; }
    static int timeout(std::optional<TimeMs> deadline, TimeMs now)
    { return LinkRuntime::poll_timeout(deadline, now); }
};
}
namespace robobeetle::session::detail {
struct SerialSessionTestAccess {
    using Step = SerialSession::TeardownStep;
    static transport::PosixSerialTransport &port(SerialSession &s) { return s.transport_; }
    static void trace(SerialSession &s, void (*fn)(const SerialSession &, Step))
    { s.trace_fn_ = fn; }
};
}
namespace robobeetle::transport::detail {
struct PosixSerialTestAccess {
    static void inject(PosixSerialTransport &p,
                       ssize_t (*reader)(int, void *, std::size_t),
                       ssize_t (*writer)(int, const void *, std::size_t))
    { p.read_fn_ = reader; p.write_fn_ = writer; }
};
}
namespace rbp2_test { int failures = 0; }

namespace {
using namespace robobeetle;
using rbp2_test::expect;
using runtime::LinkRuntime;
using runtime::RuntimeStatus;
using runtime::TimeMs;
using session::SessionState;
using link_core::LinkEventType;
using protocol::Bytes;
using protocol::Codec;
using Access = runtime::detail::LinkRuntimeTestAccess;
using SessionAccess = session::detail::SerialSessionTestAccess;
using PortAccess = transport::detail::PosixSerialTestAccess;

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
    ~Pty() { ::close(master); }
};

struct Script {
    TimeMs now{0};
    std::vector<TimeMs> clock_samples;
    std::size_t clock_index{0};
    std::vector<char> operations;
    std::optional<TimeMs> after_poll;
    std::optional<TimeMs> after_read;
    int poll_result{0};
    int poll_error{0};
    short revents{0};
    int read_error{EAGAIN};
    std::vector<int> fds;
    std::vector<short> interests;
    std::vector<int> timeouts;
    std::vector<char> io;
    std::vector<SessionAccess::Step> teardown;
    Bytes incoming;
    Bytes sent;
};
Script *active = nullptr;
TimeMs fake_clock()
{
    active->operations.push_back('C');
    if (active->clock_index < active->clock_samples.size()) {
        active->now = active->clock_samples[active->clock_index++];
    }
    return active->now;
}
int fake_poll(pollfd *fds, nfds_t count, int timeout)
{
    active->operations.push_back('P');
    expect(count == 1 && fds && fds[0].fd >= 0, "poll receives one valid fd");
    active->fds.push_back(fds[0].fd);
    active->interests.push_back(fds[0].events);
    active->timeouts.push_back(timeout);
    fds[0].revents = active->revents;
    if (active->after_poll) { active->now = *active->after_poll; }
    errno = active->poll_error;
    return active->poll_result;
}
ssize_t fake_read(int, void *buffer, std::size_t capacity)
{
    active->operations.push_back('R');
    active->io.push_back('R');
    if (active->after_read) { active->now = *active->after_read; }
    if (active->incoming.empty()) { errno = active->read_error; return -1; }
    const auto n = std::min(capacity, active->incoming.size());
    std::memcpy(buffer, active->incoming.data(), n);
    active->incoming.erase(active->incoming.begin(), active->incoming.begin() + n);
    return static_cast<ssize_t>(n);
}
ssize_t fake_write(int, const void *buffer, std::size_t size)
{
    active->operations.push_back('W');
    active->io.push_back('W');
    const auto *bytes = static_cast<const std::uint8_t *>(buffer);
    active->sent.insert(active->sent.end(), bytes, bytes + size);
    return static_cast<ssize_t>(size);
}
void teardown_trace(const session::SerialSession &, SessionAccess::Step step)
{ active->teardown.push_back(step); }
std::size_t count(const std::vector<link_core::LinkEvent> &events, LinkEventType type)
{
    return static_cast<std::size_t>(std::count_if(events.begin(), events.end(),
        [type](const auto &event) { return event.type == type; }));
}

struct Fixture {
    Pty pty;
    Script script;
    LinkRuntime runtime;
    explicit Fixture(link_core::LinkCoreConfig config = {}) : runtime(config)
    {
        active = &script;
        Access::inject(runtime, fake_clock, fake_poll);
        auto &session = Access::session(runtime);
        SessionAccess::trace(session, teardown_trace);
        PortAccess::inject(SessionAccess::port(session), fake_read, fake_write);
    }
    ~Fixture() { SessionAccess::trace(Access::session(runtime), nullptr); }
    void open() { expect(runtime.open(pty.path.c_str()) == 0, "runtime opens PTY"); }
    runtime::RuntimeResult run(TimeMs now, short ready = 0, int result = 1)
    {
        script.now = now;
        script.revents = ready;
        script.poll_result = result;
        return runtime.run_once();
    }
    void online()
    {
        open();
        run(575, POLLOUT); // raw zero, then post-poll tick queues first heartbeat
        expect(runtime.state() == SessionState::Online, "fixture Online");
        run(575, POLLOUT); // physically drain heartbeat
    }
    void activate()
    {
        online();
        script.incoming = Codec::encodeWire({0x02, 0x8000, {0, 0, 0x01, 0}});
        run(576, POLLIN);
        expect(runtime.link_state() == link_core::LinkState::Active, "ACK activates runtime");
    }
};

void opening_quiet_and_write_interest()
{
    Fixture f;
    expect(f.runtime.run_once().status == RuntimeStatus::NeedsOpen && f.script.fds.empty(),
           "A: closed returns NeedsOpen without poll");
    f.open();
    expect(f.run(0, 0, 0).status == RuntimeStatus::Timeout && f.script.timeouts.back() == 575,
           "B: quiet deadline determines timeout");
    expect(f.script.interests.back() == POLLIN && f.script.io.empty(), "D: quiet never requests POLLOUT");
    auto result = f.run(575, POLLOUT);
    expect(f.script.interests.back() == (POLLIN | POLLOUT) && f.script.sent == Bytes{0},
           "C/E: due entry tick queues raw zero before constructing poll interests");
    expect(f.script.timeouts.back() == -1, "Q: resync valid fd without deadline permits infinite poll wait");
    expect(count(result.events, LinkEventType::HeartbeatDispatched) == 1 && f.runtime.wants_write(),
           "S: post-poll timer queues first heartbeat");
    f.run(575, POLLOUT);
    expect(f.script.interests.back() == (POLLIN | POLLOUT), "E/S: heartbeat requests POLLOUT next iteration");
    const auto writes = f.script.io.size();
    f.run(576, 0, 0);
    expect(f.script.interests.back() == POLLIN && f.script.io.size() == writes && !f.runtime.wants_write(),
           "D/F: drained Online queue removes POLLOUT and avoids writable busy loop");
}

void timeout_conversion()
{
    expect(Access::timeout(100, 100) == 0 && Access::timeout(99, 100) == 0,
           "O: due and past deadlines yield zero");
    expect(Access::timeout(101, 100) == 1, "O: future boundary preserves millisecond");
    expect(Access::timeout(std::numeric_limits<TimeMs>::max(), 1) == INT_MAX &&
           Access::timeout(static_cast<TimeMs>(INT_MAX) + 100, 100) == INT_MAX,
           "P: large unsigned deltas clamp without narrowing overflow");
    expect(Access::timeout(std::nullopt, 100) == -1, "Q: absent deadline yields -1");
}

void readable_order_and_events()
{
    Fixture f;
    f.activate();
    const auto submitted = f.runtime.submit_request(0x14, {1});
    expect(submitted.status == link_core::SubmitStatus::Accepted, "wrapper admits ordinary work");
    f.script.incoming = Codec::encodeWire({0x20, 42, {1, 2}});
    f.script.incoming.resize(1024, 0); // one notification may consume at most 512
    f.script.io.clear();
    auto result = f.run(580, POLLIN | POLLOUT);
    expect(f.script.interests.back() == (POLLIN | POLLOUT), "E: ordinary bytes request POLLOUT");
    expect(f.script.io == std::vector<char>{'R', 'W'} && f.script.incoming.size() == 512,
           "G/H: one read, then one write, with unconsumed RX left for next iteration");
    expect(count(result.events, LinkEventType::FrameReceived) == 1, "V: frame event delivered exactly once");
    result = f.run(581, POLLIN);
    expect(count(result.events, LinkEventType::FrameReceived) == 0, "V: next iteration does not replay events");
}

void assert_aborted(const Fixture &f)
{
    using Step = SessionAccess::Step;
    expect(f.script.teardown == std::vector<Step>{Step::AdmissionClosed, Step::Aborted,
           Step::TxDropped, Step::TxFlushed, Step::Closed, Step::ReopenRequired},
           "I/J/K/L/N: unified session teardown order exactly once");
    expect(f.runtime.state() == SessionState::ReopenRequired && f.runtime.native_fd() == -1,
           "unified abort closes fd");
}

void readiness_errors()
{
    for (short error : {POLLERR, POLLHUP, POLLNVAL}) {
        Fixture f;
        f.activate();
        f.runtime.submit_request(0x14, {1});
        f.runtime.submit_request(0x10, {2});
        f.script.io.clear();
        const auto result = f.run(580, static_cast<short>(error | POLLIN | POLLOUT));
        expect(result.status == RuntimeStatus::SessionLost && f.script.io.empty(),
               "J/K/L: error readiness aborts before all readable/writable services");
        expect(result.error_number == (error == POLLNVAL ? EBADF : EIO), "readiness reason retained");
        expect(result.events.size() == 3 && result.events[0].type == LinkEventType::RequestOutcomeUnknown &&
               result.events[1].type == LinkEventType::RequestCancelled && result.events[2].type == LinkEventType::StateChanged,
               "V: pending Unknown then queued Cancelled then Lost, exactly once");
        assert_aborted(f);
        expect(f.runtime.abort().empty() && f.runtime.run_once().events.empty(), "V: abort and next run do not repeat outcomes");
    }
    Fixture f;
    f.activate();
    f.runtime.submit_request(0x14, {1});
    f.script.io.clear();
    f.script.read_error = EIO;
    const auto result = f.run(580, POLLIN | POLLOUT);
    expect(result.status == RuntimeStatus::SessionLost && f.script.io == std::vector<char>{'R'},
           "I: readable loss skips writable on closed fd");
    assert_aborted(f);
}

void poll_errors()
{
    Fixture f;
    f.online();
    const int fd = f.runtime.native_fd();
    f.script.poll_error = EINTR;
    const auto before = f.script.fds.size();
    auto result = f.run(580, 0, -1);
    expect(result.status == RuntimeStatus::Interrupted && result.error_number == EINTR &&
           f.runtime.state() == SessionState::Online && f.runtime.native_fd() == fd &&
           f.script.teardown.empty() && f.script.fds.size() == before + 1,
           "M: EINTR returns once, no retry or teardown");
    f.script.poll_error = ENOMEM;
    result = f.run(581, 0, -1);
    expect(result.status == RuntimeStatus::PollFatal && result.error_number == ENOMEM,
           "N: other poll error aborts and preserves original errno");
    assert_aborted(f);
}

void timer_ordering_and_reopen()
{
    {
        Fixture f;
        f.open();
        f.script.after_poll = 575;
        auto result = f.run(0, 0, 0);
        expect(result.status == RuntimeStatus::Timeout && f.runtime.wants_write() &&
               f.runtime.state() == SessionState::Resynchronizing,
               "B: timeout samples fresh time and advances quiet deadline");
    }
    {
        Fixture f;
        f.online();
        f.script.incoming = Codec::encodeWire({0x20, 5, {}});
        f.script.after_read = 675;
        const auto result = f.run(674, POLLIN);
        expect(result.events.size() == 2 && result.events[0].type == LinkEventType::FrameReceived &&
               result.events[1].type == LinkEventType::HeartbeatDispatched && f.runtime.wants_write(),
               "R/S/V: early readiness delivers RX before post-read timer event; timer is not starved");
    }
    {
        Fixture f;
        f.online();
        f.script.incoming = Codec::encodeWire({0x20, 6, {}});
        f.script.after_read = 1025;
        const auto result = f.run(600, POLLIN);
        expect(result.status == RuntimeStatus::SessionLost && result.events.front().type == LinkEventType::FrameReceived,
               "R: fresh post-read tick evaluates liveness even with continuous RX");
        assert_aborted(f);
    }
    {
        Fixture f;
        f.online();
        const auto polls = f.script.fds.size();
        const int old_fd = f.runtime.native_fd();
        const auto result = f.run(1025);
        expect(result.status == RuntimeStatus::NeedsOpen && f.script.fds.size() == polls &&
               count(result.events, LinkEventType::StateChanged) >= 1,
               "T: pre-poll Lost returns NeedsOpen with events without polling closed fd");
        // Occupy the released descriptor so reopen must acquire a different one.
        const int reserved = ::open("/dev/null", O_RDONLY | O_CLOEXEC);
        expect(reserved == old_fd, "U: occupy old fd to expose stale fd caching");
        f.open();
        const int new_fd = f.runtime.native_fd();
        f.run(1026, 0, 0);
        expect(new_fd != old_fd && f.script.fds.back() == new_fd, "U: each run rereads reopened native fd");
        ::close(reserved);
    }
}

void deadline_crossed_during_pre_poll_work()
{
    Fixture f;
    f.online();
    // Next heartbeat is due at 675. Pre-tick sees 674, but time spent in
    // pre-poll work/scheduling crosses that deadline before poll can block.
    f.script.clock_samples = {674, 676, 676, 677};
    f.script.operations.clear();
    f.script.incoming = Codec::encodeWire({0x20, 8, {}});
    const auto polls = f.script.fds.size();
    const auto result = f.run(674, POLLIN);
    expect(f.script.fds.size() == polls + 1 && f.script.timeouts.back() == 0,
           "O regression: crossed deadline uses fresh pre-poll time, never stale positive timeout");
    expect(f.script.interests.back() == POLLIN,
           "O regression: pre-tick at 674 precedes timeout sampling and has no queued heartbeat");
    expect(f.script.operations == std::vector<char>{'C', 'C', 'P', 'C', 'R', 'C'},
           "O regression: pre-tick clock, timeout clock, one poll, readiness clock/read, post-tick clock");
    expect(result.events.size() == 2 && result.events[0].type == LinkEventType::FrameReceived &&
           result.events[1].type == LinkEventType::HeartbeatDispatched && f.runtime.wants_write(),
           "O regression: readiness precedes post-tick timer event without delaying the due timer");
}

void real_pty_runtime()
{
    Pty pty;
    Script clock;
    active = &clock;
    LinkRuntime runtime;
    Access::inject(runtime, fake_clock, ::poll);
    expect(runtime.open(pty.path.c_str()) == 0, "W: runtime opens real PTY");
    // Actual PTY readiness at 574 avoids waiting 575ms; production poll/I/O and
    // session lifecycle are used, with only the monotonic clock substituted.
    clock.now = 574;
    const std::uint8_t stale = 42;
    expect(::write(pty.master, &stale, 1) == 1, "W: pre-deadline readiness setup");
    runtime.run_once();
    pollfd master{pty.master, POLLIN, 0};
    expect(::poll(&master, 1, 0) == 0 && runtime.state() == SessionState::SafetyQuiet,
           "W: actual wire remains quiet at 574");
    clock.now = 575;
    const auto raw = runtime.run_once();
    expect(count(raw.events, LinkEventType::HeartbeatDispatched) == 1, "W: raw drain precedes first heartbeat scheduling");
    runtime.run_once();
    const auto expected = rbp2_test::concat(Bytes{0}, Codec::encodeWire({0x01, 0, {0, 0, 0, 0}}));
    Bytes received;
    std::uint8_t buffer[256]{};
    for (std::size_t attempt = 0; attempt < expected.size() && received.size() < expected.size(); ++attempt) {
        master.revents = 0;
        if (::poll(&master, 1, 1000) != 1) { break; }
        const auto n = ::read(pty.master, buffer, sizeof(buffer));
        if (n <= 0) { break; }
        received.insert(received.end(), buffer, buffer + n);
    }
    expect(received == expected, "W: real poll and PTY see exactly raw zero followed by first heartbeat");
    const auto frame = Codec::encodeWire({0x20, 7, {0, 0x11, 0x13}});
    expect(::write(pty.master, frame.data(), frame.size()) == static_cast<ssize_t>(frame.size()), "W: PTY RX setup");
    const auto result = runtime.run_once();
    expect(count(result.events, LinkEventType::FrameReceived) == 1, "W: real poll drives real receive once");
}
}

int main()
{
    try {
        opening_quiet_and_write_interest();
        timeout_conversion();
        readable_order_and_events();
        readiness_errors();
        poll_errors();
        timer_ordering_and_reopen();
        deadline_crossed_during_pre_poll_work();
        real_pty_runtime();
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    if (rbp2_test::failures != 0) { return EXIT_FAILURE; }
    std::cout << "All link runtime tests passed (A-W)\n";
    return EXIT_SUCCESS;
}
#endif
