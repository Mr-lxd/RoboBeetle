#if !__has_include("robobeetle/session/serial_session.hpp")
#include <cstdio>
int main()
{
    std::fprintf(stderr, "FAIL: Slice 4 SerialSession lifecycle is not implemented\n");
    return 1;
}
#else
#include "test_support.hpp"
#include "robobeetle/session/serial_session.hpp"
#include "robobeetle/protocol/codec.hpp"
#include "robobeetle/protocol/message_types.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <new>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace robobeetle::transport::detail {
struct PosixSerialTestAccess {
    static void inject(PosixSerialTransport &port,
                       ssize_t (*reader)(int, void *, std::size_t),
                       ssize_t (*writer)(int, const void *, std::size_t))
    {
        port.read_fn_ = reader;
        port.write_fn_ = writer;
    }
    static std::size_t offset(const PosixSerialTransport &port)
    { return port.tx_queue_.front_offset(); }
};
}
namespace robobeetle::session::detail {
struct SerialSessionTestAccess {
    using Step = SerialSession::TeardownStep;
    static transport::PosixSerialTransport &port(SerialSession &s) { return s.transport_; }
    static const transport::PosixSerialTransport &port(const SerialSession &s) { return s.transport_; }
    static const link_core::LinkCore &link(const SerialSession &s) { return s.link_; }
    static void inject_abort(SerialSession &s,
        std::vector<link_core::LinkEvent> (*abort)(link_core::LinkCore &, TimeMs))
    { s.abort_fn_ = abort; }
    static std::optional<TimeMs> safe_after(const SerialSession &s) { return s.safe_after_; }
    static void inject(SerialSession &s, int (*flush)(int, int),
                       void (*trace)(const SerialSession &, Step))
    { s.flush_fn_ = flush; s.trace_fn_ = trace; }
};
}
namespace rbp2_test { int failures = 0; }

namespace {
using rbp2_test::expect;
using namespace robobeetle;
using session::SerialSession;
using session::SessionState;
using link_core::TimeMs;
using link_core::LinkEvent;
using link_core::LinkEventType;
using link_core::LinkState;
using link_core::SubmitStatus;
using protocol::Bytes;
using protocol::Codec;
using protocol::Frame;
using Access = session::detail::SerialSessionTestAccess;
using PortAccess = transport::detail::PosixSerialTestAccess;

struct Pty {
    int master{-1};
    std::string path;
    Pty()
    {
        master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (master < 0) { throw std::runtime_error("posix_openpt failed"); }
        if (::grantpt(master) != 0 || ::unlockpt(master) != 0) {
            ::close(master);
            throw std::runtime_error("PTY setup failed");
        }
        const char *name = ::ptsname(master);
        if (!name) { ::close(master); throw std::runtime_error("ptsname failed"); }
        path = name;
    }
    ~Pty() { ::close(master); }
};

struct Step { ssize_t count; int error; };
struct Script {
    Bytes incoming;
    Bytes sent;
    std::vector<Bytes> writes;
    std::vector<Step> tx;
    std::size_t tx_index{0};
    std::size_t rx_index{0};
    std::size_t reads{0};
    std::size_t chunk{512};
    int read_error{EAGAIN};
    int flush_error{0};
    std::vector<int> flushes;
    std::vector<Access::Step> trace;
    int closing_fd{-1};
    bool abort_throws{false};
    bool trace_throws{false};
};
Script *active = nullptr;

ssize_t scripted_write(int, const void *data, std::size_t size)
{
    const auto *bytes = static_cast<const std::uint8_t *>(data);
    active->writes.emplace_back(bytes, bytes + size);
    Step step{static_cast<ssize_t>(size), 0};
    if (active->tx_index < active->tx.size()) { step = active->tx[active->tx_index++]; }
    if (step.count > 0) {
        if (static_cast<std::size_t>(step.count) > size) { throw std::runtime_error("bad TX script"); }
        active->sent.insert(active->sent.end(), bytes, bytes + step.count);
    }
    errno = step.error;
    return step.count;
}

ssize_t scripted_read(int, void *data, std::size_t capacity)
{
    ++active->reads;
    const auto count = std::min({capacity, active->chunk,
                                active->incoming.size() - active->rx_index});
    if (count != 0) {
        std::memcpy(data, active->incoming.data() + active->rx_index, count);
        active->rx_index += count;
        return static_cast<ssize_t>(count);
    }
    errno = active->read_error;
    return active->read_error == 0 ? 0 : -1;
}

int scripted_flush(int fd, int selector)
{
    expect(::fcntl(fd, F_GETFD) >= 0, "flush occurs while fd is still open");
    active->flushes.push_back(selector);
    if (active->flush_error) { errno = active->flush_error; return -1; }
    return ::tcflush(fd, selector);
}

void trace_teardown(const SerialSession &s, Access::Step step)
{
    active->trace.push_back(step);
    expect(!Access::port(s).accepting_new_tx(), "J/K/L: gate closed at every teardown step");
    if (step == Access::Step::AdmissionClosed) {
        active->closing_fd = s.native_fd();
    }
    if (step == Access::Step::Aborted || step == Access::Step::TxDropped ||
        step == Access::Step::TxFlushed) {
        if (!active->abort_throws) {
            expect(s.link_state() == LinkState::Lost, "J/K/L: abort precedes drop/flush");
        }
        expect(s.native_fd() == active->closing_fd && s.native_fd() >= 0,
               "J/K/L: fd remains owned through abort/drop/flush");
    }
    if (step == Access::Step::TxDropped || step == Access::Step::TxFlushed) {
        expect(!Access::port(s).has_pending_tx(), "J/K/L: queue dropped before kernel flush");
    }
    if (step == Access::Step::Closed || step == Access::Step::ReopenRequired) {
        expect(s.native_fd() == -1, "J/K/L: close precedes reopen-required");
        errno = 0;
        expect(::fcntl(active->closing_fd, F_GETFD) < 0 && errno == EBADF,
               "J/K/L: descriptor actually closed");
    }
    if (active->trace_throws) { throw std::bad_alloc{}; }
}

std::size_t count(const std::vector<LinkEvent> &events, LinkEventType type)
{
    return static_cast<std::size_t>(std::count_if(events.begin(), events.end(),
        [type](const LinkEvent &event) { return event.type == type; }));
}

struct Fixture {
    Pty pty;
    Script script;
    SerialSession session;
    explicit Fixture(link_core::LinkCoreConfig config = {}) : session(config)
    {
        active = &script;
        Access::inject(session, scripted_flush, trace_teardown);
        PortAccess::inject(Access::port(session), scripted_read, scripted_write);
    }
    ~Fixture()
    {
        // Destruction still flushes/closes; no callback may outlive its script.
        Access::inject(session, ::tcflush, nullptr);
        active = nullptr;
    }
    void open(TimeMs now = 0)
    { expect(session.retry_open(pty.path.c_str(), now) == 0, "PTY session opens"); }
    void online()
    {
        open();
        session.tick(575);
        session.service_writable(575);
        expect(session.state() == SessionState::Online, "fixture is Online after raw zero drains");
    }
    void activate()
    {
        online();
        session.tick(575);
        session.service_writable(575);
        const Frame heartbeat = Codec::decodeWire(rbp2_test::without_delimiter(script.writes.back())).frame;
        script.incoming = Codec::encodeWire({0x02, 0x8000,
            {static_cast<std::uint8_t>(heartbeat.sequence),
             static_cast<std::uint8_t>(heartbeat.sequence >> 8), 0x01, 0}});
        session.service_readable(576);
        expect(session.link_state() == LinkState::Active, "ACK activates real LinkCore");
    }
};

void quiet_and_resynchronization()
{
    Fixture f;
    expect(!f.session.next_wakeup_ms(), "closed does not invent a retry timer");
    f.open(100);
    expect(f.session.native_fd() >= 0 && f.session.state() == SessionState::SafetyQuiet &&
           !Access::port(f.session).accepting_new_tx(), "A: open immediately disables admission");
    for (TimeMs now : {100U, 101U, 674U}) {
        expect(f.session.tick(now).empty(), "B/C: before 575 no LinkCore poll events");
        f.session.service_writable(now);
        expect(f.session.submit_request(0x14, {}, now).status == SubmitStatus::NotActive,
               "quiet cannot admit an ordinary request");
    }
    expect(f.script.sent.empty() && f.script.writes.empty() && f.script.flushes.empty() &&
           Access::link(f.session).next_sequence() == 0 && !Access::link(f.session).next_wakeup_ms(),
           "A/B/C: 574ms quiet has no raw zero, heartbeat or started LinkCore");
    expect(f.session.next_wakeup_ms() == 675, "U: only session quiet deadline exposed");
    const Bytes stale{9, 8, 7};
    expect(::write(f.pty.master, stale.data(), stale.size()) == 3, "D: stale kernel RX setup");
    pollfd ready{f.session.native_fd(), POLLIN, 0};
    expect(::poll(&ready, 1, 1000) == 1, "D: stale bytes reached slave");
    f.session.tick(675);
    expect(f.session.state() == SessionState::Resynchronizing && f.session.wants_write() &&
           !Access::port(f.session).accepting_new_tx(), "B/D: exact 575 queues resynchronization");
    expect(f.script.flushes == std::vector<int>{TCIFLUSH}, "D: stale kernel RX discarded once");
    ready.revents = 0;
    expect(::poll(&ready, 1, 0) == 0, "D: stale kernel RX is actually gone");
    expect(!f.session.next_wakeup_ms(), "U: resync has no fabricated heartbeat deadline");
    f.script.tx = {{0, 0}, {-1, EAGAIN}, {-1, EINTR}, {-1, EWOULDBLOCK}, {1, 0}};
    for (int i = 0; i < 3; ++i) {
        f.session.service_writable(675 + i);
        f.session.tick(675 + i);
        expect(f.session.state() == SessionState::Resynchronizing &&
               !Access::link(f.session).next_wakeup_ms() &&
               !Access::port(f.session).accepting_new_tx(), "E: zero/EAGAIN/EINTR cannot start LinkCore");
        expect(f.session.submit_request(0x14, {}, 677).status == SubmitStatus::NotActive,
               "E: resync rejects ordinary work");
    }
    f.session.service_writable(678);
    expect(f.script.sent == Bytes{0} && f.session.state() == SessionState::Online &&
           Access::port(f.session).accepting_new_tx(), "F: raw zero drains before Online/start/admission");
    expect(f.session.next_wakeup_ms() == 678, "F/U: first heartbeat due only after raw completion");
    expect(std::all_of(f.script.writes.begin(), f.script.writes.end(),
           [](const Bytes &bytes) { return bytes == Bytes{0}; }), "D/E: exactly standalone raw zero retried");
    expect(f.session.submit_request(0x14, {}, 678).status == SubmitStatus::NotActive,
           "Online Unconfirmed still respects Active gate");
    auto events = f.session.tick(678);
    expect(count(events, LinkEventType::HeartbeatDispatched) == 1, "F: first normal heartbeat accepted");
    f.session.service_writable(678);
    expect(f.script.sent.front() == 0 && f.script.sent.size() > 1 &&
           Codec::decodeWire(rbp2_test::without_delimiter(f.script.writes.back())).ok(),
           "G: raw zero physically precedes a valid Protocol V2 heartbeat");
    expect(f.session.next_wakeup_ms() == 778, "U: Online selects earliest LinkCore deadline");
}

void receive_and_read_bound()
{
    Fixture f;
    f.online();
    const Frame frame{0x20, 42, {0, 0xFF, 0x11, 0x13, 0x0D, 0x0A}};
    f.script.incoming = Codec::encodeWire(frame);
    f.script.chunk = 2;
    std::vector<LinkEvent> events;
    while (f.script.rx_index < f.script.incoming.size()) {
        auto part = f.session.service_readable(580);
        events.insert(events.end(), part.begin(), part.end());
    }
    expect(count(events, LinkEventType::FrameReceived) == 1 &&
           count(events, LinkEventType::DecodeError) == 0, "H: fragmented raw RX reaches decoder once");
    const auto received = std::find_if(events.begin(), events.end(),
        [](const LinkEvent &event) { return event.type == LinkEventType::FrameReceived; });
    expect(received != events.end() && received->frame.payload == frame.payload &&
           received->frame.sequence == frame.sequence, "H: exact unmodified frame payload and sequence");
    expect(f.session.service_readable(581).empty() && f.session.state() == SessionState::Online,
           "I: EAGAIN is normal and does not redeliver RX");
    f.script.incoming.insert(f.script.incoming.end(), 16384, 1);
    f.script.chunk = 512;
    const auto before = f.script.reads;
    f.session.service_readable(582);
    expect(f.script.reads - before == 1, "H: one readable notification performs one successful read");
    const auto consumed = f.script.rx_index;
    f.session.service_readable(583);
    expect(f.script.reads - before == 2 && f.script.rx_index == consumed + 512,
           "H: remaining burst requires another readable notification");
}

void exact_full_read_does_not_probe_eof()
{
    Fixture f;
    f.online();
    const auto frame = Codec::encodeWire({0x20, 512, {0, 0x11, 0x13, 0xFF}});
    f.script.incoming.assign(512 - frame.size(), 0);
    f.script.incoming.insert(f.script.incoming.end(), frame.begin(), frame.end());
    f.script.chunk = 512;
    // Reproduces VMIN=0: after exactly one full buffer, an unready second read
    // returns zero. That zero must never be requested in this readiness round.
    f.script.read_error = 0;
    const auto events = f.session.service_readable(580);
    expect(f.script.reads == 1 && f.script.rx_index == 512,
           "H regression: exact-512 Progress never makes a speculative second read");
    expect(f.session.state() == SessionState::Online && f.script.trace.empty(),
           "H regression: exact-full healthy burst does not trigger EOF teardown");
    expect(count(events, LinkEventType::FrameReceived) == 1,
           "H regression: exact-full burst delivers its valid frame once");
}

void stale_reads_do_not_activate()
{
    Fixture f;
    f.open();
    f.script.incoming = Codec::encodeWire({0x20, 1, {1, 2}});
    expect(f.session.service_readable(574).empty() && !Access::link(f.session).next_wakeup_ms(),
           "quiet RX is discarded without LinkCore.receive");
    f.session.tick(575);
    f.script.incoming.insert(f.script.incoming.end(), {1, 2, 3, 0});
    expect(f.session.service_readable(575).empty() && !Access::link(f.session).next_wakeup_ms(),
           "resync RX is discarded without LinkCore.receive");
}

void assert_teardown(const Fixture &f)
{
    expect(f.script.trace == std::vector<Access::Step>{Access::Step::AdmissionClosed,
               Access::Step::Aborted, Access::Step::TxDropped, Access::Step::TxFlushed,
               Access::Step::Closed, Access::Step::ReopenRequired},
           "J/K/L: exact unified gate -> abort -> drop -> TCOFLUSH -> close -> reopen order");
    expect(f.script.flushes == std::vector<int>{TCIFLUSH, TCOFLUSH} &&
           f.session.state() == SessionState::ReopenRequired && !f.session.wants_write() &&
           !f.session.next_wakeup_ms(), "J/K/L: kernel TX flushed once; closed has no busy retry timer");
}

void losses_and_no_replay()
{
    // J/K/L exercise the same teardown under fatal read, fatal write, liveness.
    for (int cause = 0; cause < 4; ++cause) {
        link_core::LinkCoreConfig config;
        config.ack_timeout_ms = 2000;
        Fixture f(config);
        f.activate();
        auto pending = f.session.submit_request(0x14, {0xA1, 0xB2}, 580);
        auto queued = f.session.submit_request(0x10, {3}, 580);
        expect(pending.status == SubmitStatus::Accepted && queued.status == SubmitStatus::Accepted,
               "N/O: pending and queued ordinary setup");
        f.script.tx = {{2, 0}};
        f.session.service_writable(580);
        expect(PortAccess::offset(Access::port(f.session)) == 2, "M: ordinary front genuinely partially written");
        f.session.tick(675); // heartbeat successor in the software queue
        const auto next_sequence = Access::link(f.session).next_sequence();
        const TimeMs lost_at = cause == 2 ? 1026 : 700;
        std::vector<LinkEvent> events;
        if (cause == 0 || cause == 3) {
            f.script.read_error = cause == 0 ? EIO : 0;
            events = f.session.service_readable(lost_at);
        } else if (cause == 1) {
            f.script.tx.push_back({-1, EIO});
            events = f.session.service_writable(lost_at);
        } else {
            events = f.session.tick(lost_at);
        }
        assert_teardown(f);
        expect(count(events, LinkEventType::RequestOutcomeUnknown) == 1 &&
               count(events, LinkEventType::RequestCancelled) == 1,
               "N/O: pending Unknown, queued Cancelled exactly once");
        expect(!Access::link(f.session).ordinary_request_in_flight() &&
               Access::link(f.session).queued_ordinary_count() == 0 &&
               !Access::port(f.session).has_pending_tx(), "M/N/O: all stale software work removed");
        expect(f.session.abort(lost_at + 10).empty() && f.session.tick(lost_at + 10).empty() &&
               f.session.service_readable(lost_at + 10).empty() &&
               f.session.service_writable(lost_at + 10).empty(), "T: repeat Lost/abort has no duplicate outcomes");
        expect(f.script.trace.size() == 6, "T: repeated abort does not repeat teardown or reset quiet");
        expect(f.session.submit_request(0x14, {}, lost_at + 10).status == SubmitStatus::NotActive,
               "reopen-required cannot accept work");
        f.script.read_error = EAGAIN;
        f.open(lost_at + 20);
        expect(f.session.next_wakeup_ms() == lost_at + 575, "P/T: reopen preserves loss deadline");
        const auto sent_before = f.script.sent.size();
        f.session.tick(lost_at + 574);
        f.session.service_writable(lost_at + 574);
        expect(f.script.sent.size() == sent_before && f.session.state() == SessionState::SafetyQuiet &&
               !Access::port(f.session).accepting_new_tx(), "P: early reopened fd cannot TX/start");
        f.session.tick(lost_at + 575);
        f.session.service_writable(lost_at + 575);
        expect(f.script.sent.size() == sent_before + 1 && f.script.sent.back() == 0,
               "M: next fd emits raw zero only, no stale remainder/successor/replay");
        f.session.tick(lost_at + 575);
        f.session.service_writable(lost_at + 575);
        const auto restarted = Codec::decodeWire(rbp2_test::without_delimiter(f.script.writes.back()));
        expect(restarted.ok() && restarted.frame.message_type == 0x01 &&
               restarted.frame.sequence == next_sequence, "N/O: restart sends only heartbeat, continuous sequence");
    }
}

std::size_t fd_count()
{
    auto *directory = ::opendir("/proc/self/fd");
    if (!directory) { throw std::runtime_error("cannot count fds"); }
    std::size_t result = 0;
    while (::readdir(directory)) { ++result; }
    ::closedir(directory);
    return result;
}

void late_reopen_and_failures()
{
    {
        Fixture f;
        f.online();
        f.session.abort(600);
        f.open(1200);
        expect(f.session.state() == SessionState::SafetyQuiet && f.script.sent == Bytes{0},
               "Q: late open still cannot bypass resync");
        f.session.tick(1200);
        expect(f.session.state() == SessionState::Resynchronizing && !Access::port(f.session).accepting_new_tx(),
               "Q: late retry may enter resync but cannot start LinkCore yet");
        f.session.service_writable(1200);
        expect(f.script.sent == Bytes({0, 0}) && f.session.next_wakeup_ms() == 1200,
               "Q: restart begins only after new raw zero");
    }
    {
        Fixture f;
        const auto before = fd_count();
        for (int i = 0; i < 8; ++i) {
            expect(f.session.retry_open("/dev/null/missing", 0) == ENOTDIR, "R: explicit open error");
            expect(f.session.retry_open("/dev/null", 0) == ENOTTY, "R: explicit configure error");
            expect(f.session.native_fd() == -1 && f.session.state() == SessionState::ReopenRequired &&
                   !f.session.next_wakeup_ms(), "R: no half-open fd or auto retry timer");
            f.session.tick(10000);
        }
        expect(fd_count() == before && f.script.writes.empty(), "R: no fd leak or automatic I/O retry");
    }
    for (int failure : {EIO, EINTR}) {
        Fixture f;
        f.online();
        f.script.flush_error = failure;
        f.script.read_error = EIO;
        f.session.service_readable(600);
        assert_teardown(f);
        expect(f.session.last_error() == failure, "S: failed TCOFLUSH is surfaced, including EINTR");
        expect(f.session.tick(10000).empty() && !f.session.next_wakeup_ms() &&
               f.session.link_state() == LinkState::Lost, "S: flush failure does not retry or start LinkCore");
    }
    {
        Fixture f;
        f.open();
        f.script.flush_error = EIO;
        f.session.tick(575);
        expect(f.session.state() == SessionState::ReopenRequired && f.session.native_fd() == -1 &&
               f.script.writes.empty() && f.session.last_error() == EIO,
               "failed initial RX flush closes safely without raw zero or start");
    }
}

void monotonic_and_deadlines()
{
    {
        Fixture f;
        const auto maximum = std::numeric_limits<TimeMs>::max();
        f.open(maximum - 100);
        expect(f.session.next_wakeup_ms() == maximum, "quiet deadline addition saturates");
        f.session.tick(maximum - 1);
        expect(f.script.writes.empty() && f.session.state() == SessionState::SafetyQuiet,
               "overflow never permits early transmission");
        f.session.tick(maximum);
        expect(f.session.state() == SessionState::Resynchronizing, "saturated deadline reaches resync");
    }
    {
        Fixture f;
        f.online();
        f.session.tick(800);
        f.session.abort(700); // defensive clamp: loss cannot travel backwards
        f.open(750);
        expect(f.session.next_wakeup_ms() == 1375, "monotonic session timestamps clamp backwards inputs");
    }
    {
        link_core::LinkCoreConfig config;
        config.heartbeat_interval_ms = 1000;
        config.ack_timeout_ms = 200;
        config.liveness_timeout_ms = 5000;
        Fixture f(config);
        f.activate();
        expect(f.session.next_wakeup_ms() == 1575, "U: acknowledged heartbeat leaves next heartbeat deadline");
        f.session.submit_request(0x14, {}, 600);
        expect(f.session.next_wakeup_ms() == 800, "U: ordinary ACK deadline wins over heartbeat");
    }
}

std::vector<LinkEvent> throwing_abort(link_core::LinkCore &, TimeMs)
{
    throw std::bad_alloc{};
}

std::vector<LinkEvent> throwing_abort_after_outcomes(link_core::LinkCore &link, TimeMs now_ms)
{
    link.abort_session(now_ms);
    throw std::bad_alloc{};
}

void exception_safe_teardown()
{
    for (const int failure : {0, 1, 2}) {
        Fixture f;
        f.activate();
        f.session.submit_request(0x14, {0xA1, 0xB2}, 580);
        f.session.submit_request(0x10, {3}, 580);
        f.script.tx = {{2, 0}};
        f.session.service_writable(580);
        expect(PortAccess::offset(Access::port(f.session)) == 2,
               "exception cleanup starts with a partial physical frame");
        f.script.abort_throws = true;
        f.script.trace_throws = failure != 0;
        Access::inject_abort(f.session, failure == 2 ? throwing_abort_after_outcomes : throwing_abort);
        bool propagated = false;
        try { f.session.abort(600); }
        catch (const std::bad_alloc &) { propagated = true; }
        expect(propagated, "explicit abort reports logical outcome-allocation failure");
        expect(f.script.trace == std::vector<Access::Step>{Access::Step::AdmissionClosed,
                   Access::Step::TxDropped, Access::Step::TxFlushed, Access::Step::Closed,
                   Access::Step::ReopenRequired},
               "throwing logical abort still drops, flushes before close and enters reopen-required");
        expect(f.script.flushes == std::vector<int>{TCIFLUSH, TCOFLUSH} &&
                   f.session.native_fd() == -1 && !Access::port(f.session).has_pending_tx() &&
                   !Access::port(f.session).accepting_new_tx() &&
                   f.session.state() == SessionState::ReopenRequired &&
                   Access::safe_after(f.session) == 1175 && !f.session.next_wakeup_ms(),
               "logical/trace exceptions cannot leave open fd, stale TX, admission or a busy timer");
        expect(f.session.last_error() == ENOMEM, "allocation failure remains visible after cleanup");
        expect(f.session.abort(601).empty(), "closed abort remains idempotent after exception");
    }
    {
        Fixture f;
        f.activate();
        f.session.submit_request(0x14, {1}, 580);
        f.session.submit_request(0x10, {3}, 580);
        f.script.trace_throws = true;
        const auto events = f.session.abort(600);
        assert_teardown(f);
        expect(count(events, LinkEventType::RequestOutcomeUnknown) == 1 &&
                   count(events, LinkEventType::RequestCancelled) == 1,
               "throwing diagnostics cannot change normal logical abort outcomes");
    }
    // Check destructor containment separately: reaching these assertions proves
    // an abort exception did not escape the noexcept destructor and terminate.
    Pty pty;
    Script script;
    active = &script;
    int fd = -1;
    {
        SerialSession session;
        Access::inject(session, scripted_flush, trace_teardown);
        expect(session.retry_open(pty.path.c_str(), 0) == 0, "exception destructor opens fd");
        fd = session.native_fd();
        session.tick(575); // Queue the raw byte; destructor must discard it.
        script.abort_throws = true;
        script.trace_throws = true;
        Access::inject_abort(session, throwing_abort);
    }
    errno = 0;
    expect(::fcntl(fd, F_GETFD) < 0 && errno == EBADF &&
               script.flushes == std::vector<int>{TCIFLUSH, TCOFLUSH} &&
               script.trace.back() == Access::Step::ReopenRequired,
           "noexcept destructor contains abort/trace failures after flush/drop/close");
    active = nullptr;
}

void real_pty_wire_order()
{
    Pty pty;
    SerialSession session;
    expect(session.retry_open(pty.path.c_str(), 0) == 0, "real session PTY open");
    session.tick(574);
    pollfd ready{pty.master, POLLIN, 0};
    expect(::poll(&ready, 1, 0) == 0, "A/C: actual fd quiet before deadline");
    session.tick(575);
    session.service_writable(575);
    session.tick(575);
    session.service_writable(575);
    const auto expected = rbp2_test::concat(Bytes{0}, Codec::encodeWire({0x01, 0, {0, 0, 0, 0}}));
    Bytes received;
    std::uint8_t buffer[256]{};
    for (std::size_t attempt = 0; attempt < expected.size() && received.size() < expected.size(); ++attempt) {
        ready.revents = 0;
        if (::poll(&ready, 1, 1000) != 1) { break; }
        const auto length = ::read(pty.master, buffer, sizeof(buffer));
        if (length <= 0) { break; }
        received.insert(received.end(), buffer, buffer + length);
    }
    expect(received == expected, "G: actual PTY sees exactly raw zero then first heartbeat");
    const auto incoming = Codec::encodeWire({0x20, 77, {0, 0x11, 0x13, 0xFF}});
    expect(::write(pty.master, incoming.data(), incoming.size()) == static_cast<ssize_t>(incoming.size()),
           "H: actual PTY receive setup");
    ready = {session.native_fd(), POLLIN, 0};
    expect(::poll(&ready, 1, 1000) == 1, "H: actual PTY input is readable");
    const auto events = session.service_readable(576);
    expect(count(events, LinkEventType::FrameReceived) == 1 && session.state() == SessionState::Online,
           "H: real short read delivered once without speculative no-data Fatal");
    const int fd = session.native_fd();
    session.abort(600);
    errno = 0;
    expect(::fcntl(fd, F_GETFD) < 0 && errno == EBADF, "real teardown releases fd");
    int destructor_fd = -1;
    {
        SerialSession owned;
        expect(owned.retry_open(pty.path.c_str(), 0) == 0, "destructor lifecycle setup");
        destructor_fd = owned.native_fd();
    }
    errno = 0;
    expect(::fcntl(destructor_fd, F_GETFD) < 0 && errno == EBADF, "destructor flushes then releases owned fd");
}
}

int main()
{
    try {
        quiet_and_resynchronization();
        receive_and_read_bound();
        exact_full_read_does_not_probe_eof();
        stale_reads_do_not_activate();
        losses_and_no_replay();
        late_reopen_and_failures();
        monotonic_and_deadlines();
        exception_safe_teardown();
        real_pty_wire_order();
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    if (rbp2_test::failures != 0) { return EXIT_FAILURE; }
    std::cout << "All serial session tests passed (A-U)\n";
    return EXIT_SUCCESS;
}
#endif
