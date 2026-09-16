#include "robobeetle/link_core/link_core.hpp"

#include "robobeetle/protocol/message_types.hpp"

#include <algorithm>
#include <limits>

namespace robobeetle::link_core {
namespace {

using protocol::Byte;
using protocol::Bytes;
using protocol::Codec;
using protocol::Frame;
using protocol::MessageType;

constexpr Byte heartbeat_type = static_cast<Byte>(MessageType::Heartbeat);
constexpr Byte ack_type = static_cast<Byte>(MessageType::Ack);

void append_le32(Bytes &bytes, std::uint32_t value)
{
    bytes.push_back(static_cast<Byte>(value & 0xFFU));
    bytes.push_back(static_cast<Byte>((value >> 8U) & 0xFFU));
    bytes.push_back(static_cast<Byte>((value >> 16U) & 0xFFU));
    bytes.push_back(static_cast<Byte>((value >> 24U) & 0xFFU));
}

std::uint16_t read_le16(const Bytes &bytes)
{
    return static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(bytes[0]) |
        (static_cast<std::uint16_t>(bytes[1]) << 8U));
}

LinkEvent make_simple_event(LinkEventType type)
{
    LinkEvent event;
    event.type = type;
    return event;
}

} // namespace

LinkCore::LinkCore(transport::Transport &transport, LinkCoreConfig config)
    : transport_(transport), config_(config), next_sequence_(config.initial_sequence)
{
    if (config_.heartbeat_interval_ms == 0U) {
        config_.heartbeat_interval_ms = 1U;
    }
    if (config_.ack_timeout_ms == 0U) {
        config_.ack_timeout_ms = 1U;
    }
    if (config_.liveness_timeout_ms == 0U) {
        config_.liveness_timeout_ms = config_.ack_timeout_ms;
    }
    if (config_.heartbeat_history_capacity == 0U) {
        config_.heartbeat_history_capacity = 1U;
    }
    if (config_.correlation_history_capacity == 0U) {
        config_.correlation_history_capacity = 1U;
    }
}

bool LinkCore::start(TimeMs now_ms)
{
    if (started_ && state_ != LinkState::Lost) {
        return false;
    }

    if (!started_) {
        process_start_ms_ = now_ms;
        started_ = true;
    }

    decoder_.reset();
    state_ = LinkState::Unconfirmed;
    pending_ordinary_.reset();
    ordinary_queue_.clear();
    heartbeat_history_.clear();
    correlation_history_.clear();
    first_heartbeat_dispatch_ms_.reset();
    last_good_heartbeat_ack_ms_.reset();
    next_heartbeat_due_ms_ = now_ms;
    recovery_required_ = false;
    return true;
}

std::vector<LinkEvent> LinkCore::poll(TimeMs now_ms)
{
    if (!started_) {
        start(now_ms);
    }

    std::vector<LinkEvent> events;
    process_timeouts(now_ms, events);
    process_liveness(now_ms, events);
    if (state_ != LinkState::Lost) {
        dispatch_due_heartbeat(now_ms, events);
    }
    return events;
}

std::vector<LinkEvent> LinkCore::receive(const Bytes &bytes, TimeMs now_ms)
{
    if (!started_) {
        start(now_ms);
    }

    std::vector<LinkEvent> events;
    const auto decoded = decoder_.feed(bytes);
    for (const auto &result : decoded) {
        if (!result.ok()) {
            LinkEvent event = make_simple_event(LinkEventType::DecodeError);
            event.decode_error = result.error;
            events.push_back(event);
            continue;
        }
        handle_frame(result.frame, now_ms, events);
    }
    return events;
}

std::vector<LinkEvent> LinkCore::abort_session(TimeMs now_ms)
{
    std::vector<LinkEvent> events;
    if (state_ == LinkState::Lost) {
        return events;
    }
    enter_lost(now_ms, events);
    return events;
}

std::optional<TimeMs> LinkCore::next_wakeup_ms() const
{
    if (!started_ || state_ == LinkState::Lost) {
        return std::nullopt;
    }

    std::optional<TimeMs> earliest;
    const auto consider = [&earliest](TimeMs deadline) {
        if (!earliest.has_value() || deadline < earliest.value()) {
            earliest = deadline;
        }
    };

    consider(next_heartbeat_due_ms_);
    if (pending_ordinary_.has_value()) {
        consider(pending_ordinary_->deadline);
    }
    for (const auto &heartbeat : heartbeat_history_) {
        if (!heartbeat.acknowledged && !heartbeat.timed_out) {
            consider(heartbeat.deadline);
        }
    }

    if (first_heartbeat_dispatch_ms_.has_value()) {
        const TimeMs reference = last_good_heartbeat_ack_ms_.value_or(
            *first_heartbeat_dispatch_ms_);
        const TimeMs timeout = config_.liveness_timeout_ms;
        const TimeMs deadline =
            reference > std::numeric_limits<TimeMs>::max() - timeout
                ? std::numeric_limits<TimeMs>::max()
                : reference + timeout;
        consider(deadline);
    }

    return earliest;
}

SubmitResult LinkCore::submit_request(Byte request_type,
                                      const Bytes &payload,
                                      TimeMs now_ms)
{
    if (!started_) {
        start(now_ms);
    }
    if (payload.size() > Codec::MaxPayloadSize) {
        return {SubmitStatus::PayloadTooLarge, std::nullopt};
    }
    if (state_ != LinkState::Active) {
        return {SubmitStatus::NotActive, std::nullopt};
    }

    QueuedOrdinary request;
    request.frame.message_type = request_type;
    request.frame.sequence = allocate_sequence();
    request.frame.payload = payload;
    request.wire = Codec::encodeWire(request.frame);
    if (request.wire.empty()) {
        return {SubmitStatus::PayloadTooLarge, std::nullopt};
    }

    if (pending_ordinary_.has_value()) {
        if (ordinary_queue_.size() >= config_.ordinary_queue_capacity) {
            return {SubmitStatus::QueueFull, std::nullopt};
        }
        const auto sequence = request.frame.sequence;
        ordinary_queue_.push_back(std::move(request));
        return {SubmitStatus::Accepted, sequence};
    }

    std::vector<LinkEvent> dispatch_events;
    const auto sequence = request.frame.sequence;
    if (!dispatch_ordinary(std::move(request), now_ms, dispatch_events)) {
        return {SubmitStatus::TransportRejected, std::nullopt};
    }
    return {SubmitStatus::Accepted, sequence};
}

std::uint16_t LinkCore::allocate_sequence()
{
    constexpr std::size_t sequence_space =
        static_cast<std::size_t>(std::numeric_limits<std::uint16_t>::max()) + 1U;
    for (std::size_t attempt = 0U; attempt < sequence_space; ++attempt) {
        const std::uint16_t candidate = next_sequence_;
        next_sequence_ = static_cast<std::uint16_t>(next_sequence_ + 1U);
        if (!sequence_in_use(candidate)) {
            return candidate;
        }
    }
    // The configured histories are bounded, so this is unreachable in normal
    // operation. Returning the next value keeps the API deterministic if a
    // caller deliberately exhausts all 16-bit sequence values.
    return next_sequence_;
}

bool LinkCore::sequence_in_use(std::uint16_t sequence) const
{
    if (pending_ordinary_.has_value() &&
        pending_ordinary_->frame.sequence == sequence) {
        return true;
    }
    for (const auto &queued : ordinary_queue_) {
        if (queued.frame.sequence == sequence) {
            return true;
        }
    }
    for (const auto &heartbeat : heartbeat_history_) {
        if (heartbeat.frame.sequence == sequence) {
            return true;
        }
    }
    return has_history_sequence(sequence);
}

bool LinkCore::dispatch_ordinary(QueuedOrdinary request,
                                 TimeMs now_ms,
                                 std::vector<LinkEvent> &events)
{
    if (!transport_.write(request.wire)) {
        LinkEvent event = make_simple_event(LinkEventType::TransportWriteFailed);
        event.frame = request.frame;
        events.push_back(event);

        LinkEvent cancelled = make_simple_event(
            LinkEventType::RequestCancelled);
        cancelled.outcome.kind = OutcomeKind::Cancelled;
        cancelled.outcome.sequence = request.frame.sequence;
        cancelled.outcome.request_type = request.frame.message_type;
        events.push_back(cancelled);
        cancel_ordinary_queue(events);
        set_state(LinkState::Degraded, events);
        return false;
    }

    PendingOrdinary pending;
    pending.frame = request.frame;
    pending.wire = request.wire;
    // Transport ownership acceptance is the end-to-end ACK timeout anchor;
    // physical UART completion is intentionally outside this phase's API.
    pending.deadline = now_ms + config_.ack_timeout_ms;
    pending_ordinary_ = std::move(pending);

    LinkEvent event = make_simple_event(LinkEventType::OrdinaryDispatched);
    event.frame = request.frame;
    events.push_back(event);
    return true;
}

void LinkCore::dispatch_due_heartbeat(TimeMs now_ms,
                                      std::vector<LinkEvent> &events)
{
    if (now_ms < next_heartbeat_due_ms_) {
        return;
    }

    Frame heartbeat;
    heartbeat.message_type = heartbeat_type;
    heartbeat.sequence = allocate_sequence();
    heartbeat.payload.reserve(4U);
    const auto uptime_ms = static_cast<std::uint32_t>(
        static_cast<std::uint64_t>(now_ms - process_start_ms_) & 0xFFFFFFFFULL);
    append_le32(heartbeat.payload, uptime_ms);
    const Bytes wire = Codec::encodeWire(heartbeat);
    if (wire.empty()) {
        return;
    }

    if (transport_.write(wire)) {
        // Transport ownership acceptance anchors this Heartbeat's ACK deadline and,
        // until the first successful Heartbeat ACK, the first Heartbeat ownership
        // acceptance is the initial liveness reference.
        // Successful Heartbeat ACKs refresh liveness at ACK receive time.
        // Physical UART completion is never a timeout anchor.
        HeartbeatRecord record;
        record.frame = heartbeat;
        record.deadline = now_ms + config_.ack_timeout_ms;
        record.recovery_candidate =
            state_ == LinkState::Degraded && recovery_required_;
        heartbeat_history_.push_back(std::move(record));
        trim_heartbeat_history();
        if (!first_heartbeat_dispatch_ms_.has_value()) {
            first_heartbeat_dispatch_ms_ = now_ms;
        }
        LinkEvent event = make_simple_event(LinkEventType::HeartbeatDispatched);
        event.frame = heartbeat;
        events.push_back(event);
    } else {
        LinkEvent event = make_simple_event(LinkEventType::TransportWriteFailed);
        event.frame = heartbeat;
        events.push_back(event);
    }

    next_heartbeat_due_ms_ = now_ms + config_.heartbeat_interval_ms;
}

void LinkCore::process_timeouts(TimeMs now_ms,
                                std::vector<LinkEvent> &events)
{
    if (pending_ordinary_.has_value() &&
        now_ms >= pending_ordinary_->deadline) {
        const PendingOrdinary timed_out = *pending_ordinary_;
        pending_ordinary_.reset();
        remember_correlation(timed_out.frame.sequence,
                             timed_out.frame.message_type,
                             RequestKind::Ordinary,
                             HistoryStatus::TimedOut);

        LinkEvent event = make_simple_event(
            LinkEventType::RequestOutcomeUnknown);
        event.outcome.kind = OutcomeKind::OutcomeUnknown;
        event.outcome.sequence = timed_out.frame.sequence;
        event.outcome.request_type = timed_out.frame.message_type;
        events.push_back(event);
        cancel_ordinary_queue(events);
        if (state_ != LinkState::Lost) {
            set_state(LinkState::Degraded, events);
        }
    }

    for (auto &heartbeat : heartbeat_history_) {
        if (!heartbeat.acknowledged && !heartbeat.timed_out &&
            now_ms >= heartbeat.deadline) {
            heartbeat.timed_out = true;
        }
    }
    trim_heartbeat_history();
}

void LinkCore::process_liveness(TimeMs now_ms,
                                std::vector<LinkEvent> &events)
{
    if (state_ == LinkState::Lost || !first_heartbeat_dispatch_ms_.has_value()) {
        return;
    }

    const TimeMs reference = last_good_heartbeat_ack_ms_.value_or(
        *first_heartbeat_dispatch_ms_);
    if (now_ms >= reference &&
        now_ms - reference >= config_.liveness_timeout_ms) {
        enter_lost(now_ms, events);
    }
}

void LinkCore::handle_frame(const Frame &frame,
                            TimeMs now_ms,
                            std::vector<LinkEvent> &events)
{
    if (frame.message_type == ack_type) {
        handle_ack(frame, now_ms, events);
        return;
    }

    LinkEvent event = make_simple_event(
        protocol::is_known_message_type(frame.message_type)
            ? LinkEventType::FrameReceived
            : LinkEventType::UnknownMessage);
    event.frame = frame;
    events.push_back(event);
}

void LinkCore::handle_ack(const Frame &frame,
                          TimeMs now_ms,
                          std::vector<LinkEvent> &events)
{
    if (frame.payload.size() != 4U) {
        LinkEvent event = make_simple_event(LinkEventType::AckMalformed);
        event.frame = frame;
        events.push_back(event);
        return;
    }

    const std::uint16_t request_sequence = read_le16(frame.payload);
    const Byte request_type = frame.payload[2];
    const Byte result = frame.payload[3];

    if (state_ == LinkState::Lost) {
        LinkEvent event = make_simple_event(LinkEventType::AckIgnored);
        event.ack_disposition = AckDisposition::Late;
        event.frame = frame;
        events.push_back(event);
        return;
    }

    if (pending_ordinary_.has_value() &&
        pending_ordinary_->frame.sequence == request_sequence) {
        if (pending_ordinary_->frame.message_type != request_type) {
            LinkEvent event = make_simple_event(LinkEventType::AckIgnored);
            event.ack_disposition = AckDisposition::TypeMismatch;
            event.frame = frame;
            events.push_back(event);
            return;
        }
        complete_ordinary(frame, request_sequence, result, now_ms, events);
        return;
    }

    for (auto &heartbeat : heartbeat_history_) {
        if (heartbeat.frame.sequence != request_sequence) {
            continue;
        }
        if (request_type != heartbeat_type) {
            LinkEvent event = make_simple_event(LinkEventType::AckIgnored);
            event.ack_disposition = AckDisposition::TypeMismatch;
            event.frame = frame;
            events.push_back(event);
            return;
        }
        if (heartbeat.acknowledged) {
            LinkEvent event = make_simple_event(LinkEventType::AckIgnored);
            event.ack_disposition = AckDisposition::Duplicate;
            event.frame = frame;
            events.push_back(event);
            return;
        }
        if (heartbeat.timed_out) {
            LinkEvent event = make_simple_event(LinkEventType::AckIgnored);
            event.ack_disposition = AckDisposition::Late;
            event.frame = frame;
            events.push_back(event);
            return;
        }
        complete_heartbeat(heartbeat, result, now_ms, events);
        return;
    }

    if (const auto *history = find_history(request_sequence, request_type);
        history != nullptr) {
        LinkEvent event = make_simple_event(LinkEventType::AckIgnored);
        event.ack_disposition = history->status == HistoryStatus::TimedOut
            ? AckDisposition::Late
            : AckDisposition::Duplicate;
        event.frame = frame;
        events.push_back(event);
        return;
    }

    for (const auto &history : correlation_history_) {
        if (history.sequence == request_sequence) {
            LinkEvent event = make_simple_event(LinkEventType::AckIgnored);
            event.ack_disposition = AckDisposition::TypeMismatch;
            event.frame = frame;
            events.push_back(event);
            return;
        }
    }

    LinkEvent event = make_simple_event(LinkEventType::AckIgnored);
    event.ack_disposition = AckDisposition::Unmatched;
    event.frame = frame;
    events.push_back(event);
}

void LinkCore::complete_ordinary(const Frame &ack,
                                 std::uint16_t request_sequence,
                                 Byte result,
                                 TimeMs now_ms,
                                 std::vector<LinkEvent> &events)
{
    const PendingOrdinary completed = *pending_ordinary_;
    pending_ordinary_.reset();
    remember_correlation(request_sequence,
                         completed.frame.message_type,
                         RequestKind::Ordinary,
                         HistoryStatus::Completed);

    LinkEvent event = make_simple_event(
        result == 0U ? LinkEventType::RequestAccepted
                     : LinkEventType::RequestRejected);
    event.frame = ack;
    event.outcome.kind = result == 0U ? OutcomeKind::Accepted
                                      : OutcomeKind::Rejected;
    event.outcome.sequence = request_sequence;
    event.outcome.request_type = completed.frame.message_type;
    event.outcome.result = result;
    events.push_back(event);

    if (!ordinary_queue_.empty() && state_ == LinkState::Active) {
        QueuedOrdinary next = std::move(ordinary_queue_.front());
        ordinary_queue_.pop_front();
        dispatch_ordinary(std::move(next), now_ms, events);
    }
}

void LinkCore::complete_heartbeat(HeartbeatRecord &heartbeat,
                                  Byte result,
                                  TimeMs now_ms,
                                  std::vector<LinkEvent> &events)
{
    heartbeat.acknowledged = true;
    LinkEvent event = make_simple_event(
        result == 0U ? LinkEventType::RequestAccepted
                     : LinkEventType::RequestRejected);
    event.outcome.kind = result == 0U ? OutcomeKind::Accepted
                                      : OutcomeKind::Rejected;
    event.outcome.sequence = heartbeat.frame.sequence;
    event.outcome.request_type = heartbeat.frame.message_type;
    event.outcome.result = result;
    events.push_back(event);

    if (result == 0U) {
        const bool recovery_ack = state_ != LinkState::Degraded ||
                                   heartbeat.recovery_candidate;
        if (state_ != LinkState::Lost && recovery_ack) {
            last_good_heartbeat_ack_ms_ = now_ms;
            set_state(LinkState::Active, events);
        }
    } else if (state_ == LinkState::Active) {
        set_state(LinkState::Degraded, events);
    }
    trim_heartbeat_history();
}

void LinkCore::cancel_ordinary_queue(std::vector<LinkEvent> &events)
{
    while (!ordinary_queue_.empty()) {
        const QueuedOrdinary cancelled = std::move(ordinary_queue_.front());
        ordinary_queue_.pop_front();
        LinkEvent event = make_simple_event(LinkEventType::RequestCancelled);
        event.outcome.kind = OutcomeKind::Cancelled;
        event.outcome.sequence = cancelled.frame.sequence;
        event.outcome.request_type = cancelled.frame.message_type;
        events.push_back(event);
    }
}

void LinkCore::enter_lost(TimeMs now_ms, std::vector<LinkEvent> &events)
{
    if (pending_ordinary_.has_value()) {
        const PendingOrdinary uncertain = *pending_ordinary_;
        pending_ordinary_.reset();
        remember_correlation(uncertain.frame.sequence,
                             uncertain.frame.message_type,
                             RequestKind::Ordinary,
                             HistoryStatus::TimedOut);
        LinkEvent event = make_simple_event(
            LinkEventType::RequestOutcomeUnknown);
        event.outcome.kind = OutcomeKind::OutcomeUnknown;
        event.outcome.sequence = uncertain.frame.sequence;
        event.outcome.request_type = uncertain.frame.message_type;
        events.push_back(event);
    }
    cancel_ordinary_queue(events);
    for (auto &heartbeat : heartbeat_history_) {
        if (!heartbeat.acknowledged) {
            heartbeat.timed_out = true;
        }
    }
    trim_heartbeat_history();
    (void)now_ms;
    set_state(LinkState::Lost, events);
}

void LinkCore::set_state(LinkState state, std::vector<LinkEvent> &events)
{
    if (state_ == state) {
        return;
    }
    if (state == LinkState::Degraded) {
        recovery_required_ = true;
        cancel_ordinary_queue(events);
    } else if (state == LinkState::Active) {
        recovery_required_ = false;
    }
    state_ = state;
    LinkEvent event = make_simple_event(LinkEventType::StateChanged);
    event.state = state;
    events.push_back(event);
}

void LinkCore::trim_heartbeat_history()
{
    while (heartbeat_history_.size() > config_.heartbeat_history_capacity) {
        const auto retired = std::find_if(
            heartbeat_history_.begin(), heartbeat_history_.end(),
            [](const HeartbeatRecord &heartbeat) {
                return heartbeat.acknowledged || heartbeat.timed_out;
            });
        if (retired == heartbeat_history_.end()) {
            return;
        }
        heartbeat_history_.erase(retired);
    }
}

void LinkCore::remember_correlation(std::uint16_t sequence,
                                    Byte request_type,
                                    RequestKind kind,
                                    HistoryStatus status)
{
    CorrelationHistory entry;
    entry.sequence = sequence;
    entry.request_type = request_type;
    entry.kind = kind;
    entry.status = status;
    correlation_history_.push_back(entry);
    while (correlation_history_.size() > config_.correlation_history_capacity) {
        correlation_history_.pop_front();
    }
}

const LinkCore::CorrelationHistory *LinkCore::find_history(
    std::uint16_t sequence,
    Byte request_type) const
{
    for (auto it = correlation_history_.rbegin();
         it != correlation_history_.rend();
         ++it) {
        if (it->sequence == sequence && it->request_type == request_type) {
            return &*it;
        }
    }
    return nullptr;
}

bool LinkCore::has_history_sequence(std::uint16_t sequence) const
{
    return std::any_of(correlation_history_.begin(),
                       correlation_history_.end(),
                       [sequence](const CorrelationHistory &entry) {
                           return entry.sequence == sequence;
                       });
}

} // namespace robobeetle::link_core
