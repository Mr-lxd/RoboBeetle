#include "robobeetle/runtime/link_runtime.hpp"

#include <cerrno>
#include <chrono>
#include <climits>
#include <iterator>

namespace robobeetle::runtime {
namespace {
void append(std::vector<link_core::LinkEvent> &events,
            std::vector<link_core::LinkEvent> additional)
{
    events.insert(events.end(), std::make_move_iterator(additional.begin()),
                  std::make_move_iterator(additional.end()));
}
}

LinkRuntime::LinkRuntime(link_core::LinkCoreConfig config,
                         std::size_t max_tx_frames, std::size_t max_tx_bytes)
    : session_(config, max_tx_frames, max_tx_bytes), clock_fn_(monotonic_now), poll_fn_(::poll)
{
}

TimeMs LinkRuntime::monotonic_now()
{
    static_assert(std::chrono::steady_clock::is_steady, "runtime requires a monotonic clock");
    return static_cast<TimeMs>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

int LinkRuntime::poll_timeout(std::optional<TimeMs> deadline, TimeMs now_ms) noexcept
{
    if (!deadline) { return -1; }
    if (*deadline <= now_ms) { return 0; }
    const auto delta = *deadline - now_ms;
    return delta > static_cast<TimeMs>(INT_MAX) ? INT_MAX : static_cast<int>(delta);
}

bool LinkRuntime::needs_open() const noexcept
{
    return session_.state() == session::SessionState::ReopenRequired || session_.native_fd() < 0;
}

int LinkRuntime::open(const char *device_path)
{
    return session_.retry_open(device_path, clock_fn_());
}

bool LinkRuntime::submit_latest_setpoint(const protocol::Bytes &payload)
{
    return session_.submit_latest_setpoint(payload);
}

void LinkRuntime::clear_latest_setpoint()
{
    session_.clear_latest_setpoint();
}

link_core::SubmitResult LinkRuntime::submit_request(protocol::Byte request_type,
                                                   const protocol::Bytes &payload)
{
    return session_.submit_request(request_type, payload, clock_fn_());
}

std::vector<link_core::LinkEvent> LinkRuntime::abort()
{
    return session_.abort(clock_fn_());
}

RuntimeResult LinkRuntime::run_once()
{
    RuntimeResult result;
    const auto before_poll = clock_fn_();
    append(result.events, session_.tick(before_poll));
    if (needs_open()) {
        result.status = RuntimeStatus::NeedsOpen;
        result.error_number = session_.last_error();
        return result;
    }

    // Reacquire the descriptor and interest after tick: tick can queue TX or
    // tear down the session. Never cache a descriptor across iterations.
    pollfd descriptor{session_.native_fd(), POLLIN, 0};
    if (session_.wants_write()) { descriptor.events |= POLLOUT; }
    // Pre-tick work or scheduling may cross a deadline. Compute the remaining
    // wait using fresh time so an already-due timer cannot incur another delay.
    const auto timeout_now = clock_fn_();
    const int ready = poll_fn_(&descriptor, 1, poll_timeout(session_.next_wakeup_ms(), timeout_now));
    if (ready < 0) {
        const int error = errno; // abort/flush must not replace the poll errno
        result.error_number = error;
        if (error == EINTR) {
            result.status = RuntimeStatus::Interrupted;
            return result; // caller decides when to run again; never silently retry
        }
        append(result.events, session_.abort(clock_fn_()));
        result.status = RuntimeStatus::PollFatal;
        return result;
    }

    const auto after_poll = clock_fn_();
    if (ready > 0) {
        if (descriptor.revents & (POLLNVAL | POLLERR | POLLHUP)) {
            // NVAL identifies an invalid descriptor. ERR/HUP have no errno;
            // report EIO, including HUP+POLLIN without speculative final reads.
            result.error_number = (descriptor.revents & POLLNVAL) ? EBADF : EIO;
            append(result.events, session_.abort(after_poll));
            result.status = RuntimeStatus::SessionLost;
            return result;
        }
        if (descriptor.revents & POLLIN) {
            append(result.events, session_.service_readable(after_poll));
        }
        // RX may have closed the session. Re-read lifecycle/fd before writing;
        // only service POLLOUT if it was requested and bytes still need TX.
        if (!needs_open() && (descriptor.events & POLLOUT) &&
            (descriptor.revents & POLLOUT) && session_.wants_write()) {
            append(result.events, session_.service_writable(after_poll));
        }
    }

    // Fresh time after bounded readiness work prevents a continuous RX stream
    // from starving timers. RX awakened before the deadline is handled first.
    append(result.events, session_.tick(clock_fn_()));
    if (needs_open()) {
        result.status = RuntimeStatus::SessionLost;
        result.error_number = session_.last_error();
    } else {
        result.status = ready == 0 ? RuntimeStatus::Timeout : RuntimeStatus::Progress;
    }
    return result;
}

} // namespace robobeetle::runtime
