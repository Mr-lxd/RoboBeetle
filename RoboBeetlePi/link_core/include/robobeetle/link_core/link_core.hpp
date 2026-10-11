#pragma once

#include "robobeetle/link_core/link_config.hpp"
#include "robobeetle/link_core/link_events.hpp"
#include "robobeetle/protocol/stream_decoder.hpp"
#include "robobeetle/transport/transport.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

namespace robobeetle::link_core {

using TimeMs = std::uint64_t;

class LinkCore final {
public:
    explicit LinkCore(transport::Transport &transport,
                      LinkCoreConfig config = {});

    // Starts or restarts the logical LinkCore session. Sequence allocation
    // and process uptime remain continuous across a restart; pending wire
    // correlations are local state and are cleared.
    // Returns false when a restart is attempted before Lost. An accepted
    // request remains owned by the current session in that case.
    bool start(TimeMs now_ms);

    // Deterministic scheduler tick. It never sleeps or reads a wall clock.
    std::vector<LinkEvent> poll(TimeMs now_ms);

    // Feeds every received byte through the single Protocol V2 stream
    // decoder and returns nonblocking diagnostic/completion events.
    std::vector<LinkEvent> receive(const protocol::Bytes &bytes,
                                   TimeMs now_ms);

    // Immediately aborts the logical session without attempting transport
    // I/O. Pending ordinary work becomes OutcomeUnknown, queued ordinary
    // work is cancelled, and the session enters Lost. The sequence allocator
    // remains continuous and repeated aborts are idempotent.
    std::vector<LinkEvent> abort_session(TimeMs now_ms);

    // Returns the earliest absolute monotonic timestamp at which poll() must
    // run again. This is a read-only scheduler seam; it performs no I/O.
    [[nodiscard]] std::optional<TimeMs> next_wakeup_ms() const;

    // Queues an ordinary ACKed request. At most one ordinary request is
    // transmitted at a time; later requests use the bounded FIFO.
    // Its ACK timeout is an end-to-end budget anchored when Transport accepts
    // the complete wire frame, not when physical UART transmission finishes.
    // Returns a deterministic rejection reason or the allocated request
    // sequence when the request is accepted for immediate dispatch or
    // bounded FIFO queueing.
    SubmitResult submit_request(
        protocol::Byte request_type,
        const protocol::Bytes &payload,
        TimeMs now_ms);

    bool submit_latest_setpoint(const protocol::Bytes &payload);
    void clear_latest_setpoint()
    {
        latest_setpoint_.reset();
    }

    [[nodiscard]] LinkState state() const { return state_; }
    [[nodiscard]] std::size_t queued_ordinary_count() const
    {
        return ordinary_queue_.size();
    }
    [[nodiscard]] bool ordinary_request_in_flight() const
    {
        return pending_ordinary_.has_value();
    }
    [[nodiscard]] std::uint16_t next_sequence() const
    {
        return next_sequence_;
    }

private:
    enum class RequestKind {
        Ordinary,
        Heartbeat,
    };

    enum class HistoryStatus {
        Completed,
        TimedOut,
    };

    struct QueuedOrdinary {
        protocol::Frame frame;
        protocol::Bytes wire;
    };

    struct PendingOrdinary {
        protocol::Frame frame;
        protocol::Bytes wire;
        TimeMs deadline{0U};
    };

    struct HeartbeatRecord {
        protocol::Frame frame;
        TimeMs deadline{0U};
        bool acknowledged{false};
        bool timed_out{false};
        bool recovery_candidate{false};
    };

    struct CorrelationHistory {
        std::uint16_t sequence{0U};
        protocol::Byte request_type{0U};
        RequestKind kind{RequestKind::Ordinary};
        HistoryStatus status{HistoryStatus::Completed};
    };

    std::uint16_t allocate_sequence();
    bool sequence_in_use(std::uint16_t sequence) const;
    bool dispatch_ordinary(QueuedOrdinary request,
                           TimeMs now_ms,
                           std::vector<LinkEvent> &events);
    void dispatch_due_heartbeat(TimeMs now_ms,
                                std::vector<LinkEvent> &events);
    void process_timeouts(TimeMs now_ms, std::vector<LinkEvent> &events);
    void process_liveness(TimeMs now_ms, std::vector<LinkEvent> &events);
    void trim_heartbeat_history();
    void handle_frame(const protocol::Frame &frame,
                      TimeMs now_ms,
                      std::vector<LinkEvent> &events);
    void handle_ack(const protocol::Frame &frame,
                    TimeMs now_ms,
                    std::vector<LinkEvent> &events);
    void complete_ordinary(const protocol::Frame &ack,
                           std::uint16_t request_sequence,
                           protocol::Byte result,
                           TimeMs now_ms,
                           std::vector<LinkEvent> &events);
    void complete_heartbeat(HeartbeatRecord &heartbeat,
                            protocol::Byte result,
                            TimeMs now_ms,
                            std::vector<LinkEvent> &events);
    void cancel_ordinary_queue(std::vector<LinkEvent> &events);
    void enter_lost(TimeMs now_ms, std::vector<LinkEvent> &events);
    void set_state(LinkState state, std::vector<LinkEvent> &events);
    void remember_correlation(std::uint16_t sequence,
                              protocol::Byte request_type,
                              RequestKind kind,
                              HistoryStatus status);
    const CorrelationHistory *find_history(std::uint16_t sequence,
                                           protocol::Byte request_type) const;
    bool has_history_sequence(std::uint16_t sequence) const;

    transport::Transport &transport_;
    LinkCoreConfig config_;
    protocol::StreamDecoder decoder_;
    LinkState state_{LinkState::Unconfirmed};
    std::uint16_t next_sequence_{0U};
    bool started_{false};
    TimeMs process_start_ms_{0U};
    TimeMs next_heartbeat_due_ms_{0U};
    std::optional<TimeMs> first_heartbeat_dispatch_ms_;
    std::optional<TimeMs> last_good_heartbeat_ack_ms_;
    std::optional<protocol::Bytes> latest_setpoint_;
    std::optional<PendingOrdinary> pending_ordinary_;
    std::deque<QueuedOrdinary> ordinary_queue_;
    std::deque<HeartbeatRecord> heartbeat_history_;
    std::deque<CorrelationHistory> correlation_history_;
    bool recovery_required_{false};
};

} // namespace robobeetle::link_core
