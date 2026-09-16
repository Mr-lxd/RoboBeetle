#pragma once

#if !defined(__linux__)
#error "SerialSession is supported only on Linux"
#endif

#include "robobeetle/link_core/link_core.hpp"
#include "robobeetle/transport/posix_serial_transport.hpp"

namespace robobeetle::session {

using TimeMs = link_core::TimeMs;
namespace detail { struct SerialSessionTestAccess; }

enum class SessionState { ReopenRequired, SafetyQuiet, Resynchronizing, Online };

// Single owner, single thread. The caller supplies monotonic milliseconds and
// readiness notifications. This class has no event loop or reconnect policy.
class SerialSession final {
public:
    explicit SerialSession(link_core::LinkCoreConfig config = {},
                           std::size_t max_tx_frames = 64U,
                           std::size_t max_tx_bytes = 65536U);
    ~SerialSession() noexcept;
    SerialSession(const SerialSession &) = delete;
    SerialSession &operator=(const SerialSession &) = delete;
    SerialSession(SerialSession &&) = delete;
    SerialSession &operator=(SerialSession &&) = delete;

    // Explicit open/retry only. Returns zero or errno; an owned fd gives EBUSY.
    // First attempt anchors the initial quiet deadline; retries after loss keep
    // that loss's deadline. Success owns a gated fd in SafetyQuiet, not Online.
    int retry_open(const char *device_path, TimeMs now_ms);

    // Tick advances quiet/resync setup or polls Online LinkCore. A writable
    // notification pumps at most one front frame. No LinkCore poll/start or
    // ordinary admission occurs before the standalone raw zero leaves the
    // software queue. Queue completion does not claim UART last-bit completion.
    std::vector<link_core::LinkEvent> tick(TimeMs now_ms);
    std::vector<link_core::LinkEvent> service_writable(TimeMs now_ms);
    // One read_some call per readable notification, including full-buffer
    // Progress. More input requires a fresh readiness notification. Pre-Online
    // input is discarded. As in
    // PosixSerialTransport, read()==0 is Fatal (including VMIN=0 no-data zero);
    // callers must supply actual readability, not speculative read polling.
    std::vector<link_core::LinkEvent> service_readable(TimeMs now_ms);

    // Explicit disconnect shares the fatal/liveness teardown and returns
    // pending Unknown / queued Cancelled outcomes. Repeated abort is a no-op.
    // Logical outcome allocation may throw, but physical cleanup still
    // completes before propagation. Destruction contains all such exceptions.
    std::vector<link_core::LinkEvent> abort(TimeMs now_ms);
    link_core::SubmitResult submit_request(protocol::Byte request_type,
                                           const protocol::Bytes &payload,
                                           TimeMs now_ms);

    [[nodiscard]] SessionState state() const noexcept { return state_; }
    [[nodiscard]] link_core::LinkState link_state() const { return link_.state(); }
    // Borrowed only for readiness registration; do not close or mutate flags.
    [[nodiscard]] int native_fd() const noexcept { return transport_.native_fd(); }
    [[nodiscard]] bool wants_write() const noexcept;
    [[nodiscard]] std::optional<TimeMs> next_wakeup_ms() const;
    // Most recent open/fatal/flush errno, or ENOMEM/EIO on logical abort failure.
    // Zero can mean read EOF or no error;
    // use state() to distinguish a failed session. TCOFLUSH failure wins over
    // the initiating I/O errno and remains visible until an explicit retry.
    [[nodiscard]] int last_error() const noexcept { return last_error_; }

private:
    friend struct detail::SerialSessionTestAccess;
    enum class TeardownStep { AdmissionClosed, Aborted, TxDropped, TxFlushed, Closed, ReopenRequired };
    TimeMs observe_time(TimeMs now_ms) noexcept;
    std::vector<link_core::LinkEvent> teardown(TimeMs now_ms, int error_number);
    void finish_teardown(TimeMs now_ms) noexcept;
    void trace(TeardownStep step) const noexcept;

    transport::PosixSerialTransport transport_;
    link_core::LinkCore link_;
    SessionState state_{SessionState::ReopenRequired};
    std::optional<TimeMs> safe_after_;
    TimeMs last_now_ms_{0U};
    int last_error_{0};
    // Private syscall and ordering seams; no testing API leaks into LinkCore.
    int (*flush_fn_)(int, int);
    std::vector<link_core::LinkEvent> (*abort_fn_)(link_core::LinkCore &, TimeMs);
    void (*trace_fn_)(const SerialSession &, TeardownStep){nullptr};
};

} // namespace robobeetle::session
