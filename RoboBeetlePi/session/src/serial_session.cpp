#include "robobeetle/session/serial_session.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <iterator>
#include <limits>
#include <new>
#include <termios.h>

namespace robobeetle::session {
namespace {
constexpr TimeMs safety_quiet_ms = 575U;

TimeMs quiet_deadline(TimeMs now_ms)
{
    const auto maximum = std::numeric_limits<TimeMs>::max();
    return now_ms > maximum - safety_quiet_ms ? maximum : now_ms + safety_quiet_ms;
}

void append(std::vector<link_core::LinkEvent> &events,
            std::vector<link_core::LinkEvent> additional)
{
    events.insert(events.end(), std::make_move_iterator(additional.begin()),
                  std::make_move_iterator(additional.end()));
}
} // namespace

SerialSession::SerialSession(link_core::LinkCoreConfig config,
                             std::size_t max_tx_frames, std::size_t max_tx_bytes)
    : transport_(max_tx_frames, max_tx_bytes), link_(transport_, config), flush_fn_(::tcflush),
      abort_fn_([](link_core::LinkCore &link, TimeMs now_ms) { return link.abort_session(now_ms); })
{
}

SerialSession::~SerialSession() noexcept
{
    // Explicit abort lets the owner collect outcomes before destruction.
    // Destruction still follows the same flush-before-close lifecycle.
    try { abort(last_now_ms_); }
    catch (...) {
        // teardown has already completed non-allocating physical cleanup;
        // outcome allocation failure must never escape destruction.
    }
}

TimeMs SerialSession::observe_time(TimeMs now_ms) noexcept
{
    last_now_ms_ = std::max(last_now_ms_, now_ms);
    return last_now_ms_;
}

int SerialSession::retry_open(const char *device_path, TimeMs now_ms)
{
    now_ms = observe_time(now_ms);
    if (state_ != SessionState::ReopenRequired) { return EBUSY; }
    if (!safe_after_) { safe_after_ = quiet_deadline(now_ms); }
    last_error_ = transport_.open(device_path);
    transport_.set_accept_new_tx(false);
    if (last_error_ == 0) { state_ = SessionState::SafetyQuiet; }
    return last_error_;
}

std::vector<link_core::LinkEvent> SerialSession::tick(TimeMs now_ms)
{
    now_ms = observe_time(now_ms);
    if (state_ == SessionState::SafetyQuiet && now_ms >= *safe_after_) {
        // Old receive fragments must not enter the new LinkCore decoder.
        if (flush_fn_(transport_.native_fd(), TCIFLUSH) != 0) {
            return teardown(now_ms, errno);
        }
        // The transport admits whole byte sequences, including this standalone
        // delimiter. Only this single-owner method can temporarily open the
        // gate: no LinkCore method is called while the raw byte is pending.
        transport_.set_accept_new_tx(true);
        bool accepted = false;
        try {
            accepted = transport_.write({0x00});
        } catch (...) {
            transport_.set_accept_new_tx(false);
            throw;
        }
        transport_.set_accept_new_tx(false);
        if (!accepted) { return teardown(now_ms, ENOBUFS); }
        state_ = SessionState::Resynchronizing;
        return {};
    }
    if (state_ != SessionState::Online) { return {}; }
    auto events = link_.poll(now_ms);
    if (link_.state() == link_core::LinkState::Lost) { append(events, teardown(now_ms, 0)); }
    return events;
}

std::vector<link_core::LinkEvent> SerialSession::service_writable(TimeMs now_ms)
{
    now_ms = observe_time(now_ms);
    if (state_ != SessionState::Resynchronizing && state_ != SessionState::Online) { return {}; }
    const auto result = transport_.pump_tx();
    if (result.status == transport::IoStatus::Fatal) { return teardown(now_ms, result.error_number); }
    if (state_ == SessionState::Resynchronizing && !transport_.has_pending_tx()) {
        if (!link_.start(now_ms)) { return teardown(now_ms, EPROTO); }
        transport_.set_accept_new_tx(true);
        state_ = SessionState::Online;
    }
    return {};
}

std::vector<link_core::LinkEvent> SerialSession::service_readable(TimeMs now_ms)
{
    now_ms = observe_time(now_ms);
    std::vector<link_core::LinkEvent> events;
    if (state_ == SessionState::ReopenRequired) { return events; }
    std::array<std::uint8_t, 512U> buffer{};
    const auto result = transport_.read_some(buffer.data(), buffer.size());
    if (result.status == transport::IoStatus::Fatal) {
        return teardown(now_ms, result.error_number);
    }
    if (result.status == transport::IoStatus::Progress && state_ == SessionState::Online) {
        events = link_.receive(protocol::Bytes(buffer.begin(),
            buffer.begin() + result.bytes_processed), now_ms);
        if (link_.state() == link_core::LinkState::Lost) {
            append(events, teardown(now_ms, 0));
        }
    }
    // Even a full buffer does not prove another byte is ready. With VMIN=0,
    // a speculative next read may return zero (Fatal by transport contract).
    // Every successful read ends this round; the caller must renew readiness.
    return events;
}

std::vector<link_core::LinkEvent> SerialSession::abort(TimeMs now_ms)
{
    return teardown(observe_time(now_ms), 0);
}

std::vector<link_core::LinkEvent> SerialSession::teardown(TimeMs now_ms, int error_number)
{
    if (state_ == SessionState::ReopenRequired) { return {}; }
    last_error_ = error_number;
    transport_.set_accept_new_tx(false);
    trace(TeardownStep::AdmissionClosed);
    try {
        auto events = abort_fn_(link_, now_ms);
        trace(TeardownStep::Aborted);
        finish_teardown(now_ms);
        return events;
    } catch (const std::bad_alloc &) {
        last_error_ = ENOMEM;
        finish_teardown(now_ms);
        throw;
    } catch (...) {
        last_error_ = EIO;
        finish_teardown(now_ms);
        throw;
    }
}

void SerialSession::finish_teardown(TimeMs now_ms) noexcept
{
    // This physical tail performs no allocation. It runs after logical abort
    // both on success and during exception unwinding. Diagnostic callbacks
    // and private syscall seams cannot prevent release of the owned fd.
    transport_.drop_pending_tx();
    trace(TeardownStep::TxDropped);
    // Never tcdrain or ordinary-operation flush: discard kernel TX exactly at
    // the session boundary while the owned descriptor is still open. A failed
    // flush is surfaced and still closes the fd; retry requires a new open.
    try {
        if (flush_fn_(transport_.native_fd(), TCOFLUSH) != 0) { last_error_ = errno; }
    } catch (...) {
        last_error_ = EIO;
    }
    trace(TeardownStep::TxFlushed);
    transport_.close();
    trace(TeardownStep::Closed);
    state_ = SessionState::ReopenRequired;
    safe_after_ = quiet_deadline(now_ms);
    trace(TeardownStep::ReopenRequired);
}

bool SerialSession::submit_latest_setpoint(const protocol::Bytes &payload)
{
    if (state_ != SessionState::Online)
        return false;
    return link_.submit_latest_setpoint(payload);
}

void SerialSession::clear_latest_setpoint()
{
    link_.clear_latest_setpoint();
}

link_core::SubmitResult SerialSession::submit_request(protocol::Byte request_type,
                                                      const protocol::Bytes &payload,
                                                      TimeMs now_ms)
{
    now_ms = observe_time(now_ms);
    if (state_ != SessionState::Online) { return {link_core::SubmitStatus::NotActive, std::nullopt}; }
    return link_.submit_request(request_type, payload, now_ms);
}

bool SerialSession::wants_write() const noexcept
{
    return (state_ == SessionState::Resynchronizing || state_ == SessionState::Online) &&
           transport_.has_pending_tx();
}

std::optional<TimeMs> SerialSession::next_wakeup_ms() const
{
    if (state_ == SessionState::SafetyQuiet) { return safe_after_; }
    if (state_ == SessionState::Online) { return link_.next_wakeup_ms(); }
    // Closed needs an explicit retry, resync needs write readiness; neither
    // should manufacture a timer that makes an event loop spin.
    return std::nullopt;
}

void SerialSession::trace(TeardownStep step) const noexcept
{
    try { if (trace_fn_) { trace_fn_(*this, step); } }
    catch (...) { /* Diagnostics must never interfere with physical cleanup. */ }
}
} // namespace robobeetle::session
