#pragma once

#if !defined(__linux__)
#error "LinkRuntime is supported only on Linux"
#endif

#include "robobeetle/session/serial_session.hpp"

#include <poll.h>

namespace robobeetle::runtime {

using TimeMs = link_core::TimeMs;
namespace detail { struct LinkRuntimeTestAccess; }

enum class RuntimeStatus { Progress, Timeout, Interrupted, NeedsOpen, SessionLost, PollFatal };

struct RuntimeResult {
    RuntimeStatus status{RuntimeStatus::Progress};
    std::vector<link_core::LinkEvent> events;
    int error_number{0};
};

// Single owner/thread. Exactly one readiness iteration; the application owns
// repetition and explicit reopening. All timestamps use the monotonic clock.
class LinkRuntime final {
public:
    explicit LinkRuntime(link_core::LinkCoreConfig config = {},
                         std::size_t max_tx_frames = 64U,
                         std::size_t max_tx_bytes = 65536U);

    int open(const char *device_path);
    bool submit_latest_setpoint(const protocol::Bytes &payload);
    void clear_latest_setpoint();

    link_core::SubmitResult submit_request(protocol::Byte request_type,
                                           const protocol::Bytes &payload);
    std::vector<link_core::LinkEvent> abort();
    RuntimeResult run_once();

    [[nodiscard]] session::SessionState state() const noexcept { return session_.state(); }
    [[nodiscard]] link_core::LinkState link_state() const { return session_.link_state(); }
    // Borrowed diagnostics only; the runtime alone polls this descriptor.
    [[nodiscard]] int native_fd() const noexcept { return session_.native_fd(); }
    [[nodiscard]] bool wants_write() const noexcept { return session_.wants_write(); }
    [[nodiscard]] std::optional<TimeMs> next_wakeup_ms() const { return session_.next_wakeup_ms(); }

private:
    friend struct detail::LinkRuntimeTestAccess;
    static TimeMs monotonic_now();
    static int poll_timeout(std::optional<TimeMs> deadline, TimeMs now_ms) noexcept;
    bool needs_open() const noexcept;

    session::SerialSession session_;
    TimeMs (*clock_fn_)();
    int (*poll_fn_)(pollfd *, nfds_t, int);
};

} // namespace robobeetle::runtime
