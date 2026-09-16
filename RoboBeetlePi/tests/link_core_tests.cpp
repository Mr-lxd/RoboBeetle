#include "test_support.hpp"

#include "fake_clock.hpp"
#include "fake_transport.hpp"

#include "robobeetle/link_core/link_core.hpp"
#include "robobeetle/protocol/codec.hpp"
#include "robobeetle/protocol/message_types.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace rbp2_test {
namespace {

using robobeetle::link_core::AckDisposition;
using robobeetle::link_core::LinkCore;
using robobeetle::link_core::LinkCoreConfig;
using robobeetle::link_core::LinkEvent;
using robobeetle::link_core::LinkEventType;
using robobeetle::link_core::LinkState;
using robobeetle::link_core::OutcomeKind;
using robobeetle::link_core::SubmitResult;
using robobeetle::link_core::SubmitStatus;
using robobeetle::link_core::TimeMs;
using robobeetle::protocol::Byte;
using robobeetle::protocol::Bytes;
using robobeetle::protocol::Codec;
using robobeetle::protocol::Frame;
using robobeetle::protocol::MessageType;

Bytes frame_wire(const Frame &frame)
{
    return Codec::encodeWire(frame);
}

Bytes ack_wire(std::uint16_t request_sequence,
               Byte request_type,
               Byte result = 0U,
               std::uint16_t ack_sequence = 0x8000U)
{
    return frame_wire({static_cast<Byte>(MessageType::Ack), ack_sequence,
                       {static_cast<Byte>(request_sequence & 0xFFU),
                        static_cast<Byte>((request_sequence >> 8U) & 0xFFU),
                        request_type, result}});
}

Frame decode_write(const FakeTransport &transport, std::size_t index)
{
    const auto &wire = transport.writes().at(index);
    const auto decoded = Codec::decodeWire(without_delimiter(wire));
    expect(decoded.ok(), "FakeTransport write must contain a valid frame");
    return decoded.frame;
}

bool has_event(const std::vector<LinkEvent> &events, LinkEventType type)
{
    return std::any_of(events.begin(), events.end(),
                       [type](const LinkEvent &event) {
                           return event.type == type;
                       });
}

bool accepted(const SubmitResult &result)
{
    return result.status == SubmitStatus::Accepted &&
           result.sequence.has_value();
}

bool activate_link(LinkCore &link,
                   FakeTransport &transport,
                   TimeMs start_ms = 0U)
{
    link.start(start_ms);
    const auto dispatch_events = link.poll(start_ms);
    expect(has_event(dispatch_events, LinkEventType::HeartbeatDispatched),
           "activation must dispatch an initial Heartbeat");
    if (transport.writes().empty()) {
        expect(false, "activation must write an initial Heartbeat");
        return false;
    }
    const Frame heartbeat = decode_write(transport,
                                         transport.writes().size() - 1U);
    const auto ack_events = link.receive(
        ack_wire(heartbeat.sequence, heartbeat.message_type), start_ms + 1U);
    const bool active = has_event(ack_events, LinkEventType::RequestAccepted) &&
                        link.state() == LinkState::Active;
    expect(active, "matching initial Heartbeat ACK must activate LinkCore");
    return active;
}

const LinkEvent *find_event(const std::vector<LinkEvent> &events,
                            LinkEventType type)
{
    const auto it = std::find_if(events.begin(), events.end(),
                                 [type](const LinkEvent &event) {
                                     return event.type == type;
                                 });
    return it == events.end() ? nullptr : &*it;
}

LinkCoreConfig quiet_config()
{
    LinkCoreConfig config;
    config.heartbeat_interval_ms = 1000U;
    config.ack_timeout_ms = 200U;
    config.liveness_timeout_ms = 10000U;
    config.ordinary_queue_capacity = 2U;
    return config;
}

void acknowledge_heartbeat(LinkCore &link,
                           FakeTransport &transport,
                           TimeMs now_ms,
                           std::size_t write_index)
{
    const Frame heartbeat = decode_write(transport, write_index);
    expect(heartbeat.message_type == static_cast<Byte>(MessageType::Heartbeat),
           "expected Heartbeat write before ACK");
    const auto events = link.receive(
        ack_wire(heartbeat.sequence, heartbeat.message_type), now_ms);
    expect(has_event(events, LinkEventType::RequestAccepted),
           "matching Heartbeat ACK must complete the Heartbeat");
}

void test_heartbeat_schedule_and_wrap()
{
    FakeTransport transport;
    LinkCoreConfig config = quiet_config();
    config.heartbeat_interval_ms = 100U;
    config.initial_sequence = 0xFFFFU;
    LinkCore link(transport, config);
    FakeClock clock;
    link.start(clock.now_ms());

    const auto first_events = link.poll(clock.now_ms());
    expect(first_events.size() == 1U &&
               first_events.front().type == LinkEventType::HeartbeatDispatched,
           "first poll must dispatch the initial Heartbeat");
    expect(transport.writes().size() == 1U,
           "initial Heartbeat must use the transport write path");
    const Frame first = decode_write(transport, 0U);
    expect(first.sequence == 0xFFFFU,
           "Heartbeat must use the configured initial sequence");
    expect(first.payload == bytes({0U, 0U, 0U, 0U}),
           "Heartbeat uptime must be monotonic process uptime in LE");

    clock.set(99U);
    expect(link.poll(clock.now_ms()).empty(),
           "Heartbeat must not be due before its interval");
    clock.set(100U);
    const auto second_events = link.poll(clock.now_ms());
    expect(has_event(second_events, LinkEventType::HeartbeatDispatched) &&
               transport.writes().size() == 2U,
           "Heartbeat must be due exactly at its interval");
    const Frame second = decode_write(transport, 1U);
    expect(second.sequence == 0U,
           "Heartbeat sequence must wrap from ffff to zero");
    expect(second.payload == bytes({100U, 0U, 0U, 0U}),
           "Heartbeat uptime must use the injected monotonic timestamp");

    clock.set(200U);
    expect(has_event(link.poll(clock.now_ms()),
                     LinkEventType::HeartbeatDispatched) &&
               transport.writes().size() == 3U,
           "next Heartbeat must be scheduled at the next fixed interval");
}

void test_heartbeat_uptime_wrap()
{
    FakeTransport transport;
    LinkCoreConfig config = quiet_config();
    config.heartbeat_interval_ms = 1U;
    config.liveness_timeout_ms = std::numeric_limits<TimeMs>::max();
    LinkCore link(transport, config);
    FakeClock clock(0U);
    link.start(clock.now_ms());
    link.poll(clock.now_ms());

    clock.set(0x100000005ULL);
    link.poll(clock.now_ms());
    const Frame wrapped = decode_write(transport, 1U);
    expect(wrapped.payload == bytes({5U, 0U, 0U, 0U}),
           "Heartbeat uptime must wrap modulo uint32 without wall clock");
}

void test_ack_correlation_and_results()
{
    FakeTransport transport;
    LinkCoreConfig config = quiet_config();
    LinkCore link(transport, config);
    expect(activate_link(link, transport),
           "ACK correlation test requires an active LinkCore");

    const auto request_identity = link.submit_request(
        0x10U, bytes({0x01U, 0U}), 2U);
    expect(accepted(request_identity),
           "ordinary request must be accepted for dispatch");
    const Frame request = decode_write(transport, 1U);
    if (accepted(request_identity)) {
        expect(request_identity.sequence.value() == request.sequence,
               "returned request identity must equal the wire sequence");
    }

    const auto wrong_sequence = link.receive(
        ack_wire(static_cast<std::uint16_t>(request.sequence + 1U),
                 request.message_type), 2U);
    expect(has_event(wrong_sequence, LinkEventType::AckIgnored) &&
               !has_event(wrong_sequence, LinkEventType::RequestAccepted),
           "wrong-sequence ACK must be ignored without completion");

    const auto wrong_type = link.receive(
        ack_wire(request.sequence, 0x11U), 3U);
    expect(has_event(wrong_type, LinkEventType::AckIgnored) &&
               wrong_type.front().ack_disposition == AckDisposition::TypeMismatch,
           "wrong-type ACK must be ignored without completing the request");

    const auto malformed = link.receive(
        frame_wire({static_cast<Byte>(MessageType::Ack), 0x8001U,
                    bytes({static_cast<Byte>(request.sequence & 0xFFU),
                           static_cast<Byte>((request.sequence >> 8U) & 0xFFU),
                           request.message_type})}),
        4U);
    expect(has_event(malformed, LinkEventType::AckMalformed),
           "malformed ACK payload must be reported");

    const auto rejected = link.receive(
        ack_wire(request.sequence, request.message_type, 5U), 5U);
    const auto *rejected_event = find_event(rejected, LinkEventType::RequestRejected);
    expect(rejected_event != nullptr,
           "matching non-OK ACK must complete as rejected");
    if (rejected_event != nullptr) {
        expect(rejected_event->outcome.kind == OutcomeKind::Rejected &&
                   rejected_event->outcome.result == 5U &&
                   rejected_event->outcome.sequence == request.sequence,
               "non-OK ACK result and sequence must be preserved");
    }
}

void test_timeout_and_stale_ack()
{
    FakeTransport transport;
    LinkCoreConfig config = quiet_config();
    config.ack_timeout_ms = 20U;
    config.ordinary_queue_capacity = 2U;
    LinkCore link(transport, config);
    expect(activate_link(link, transport),
           "timeout test requires an active LinkCore");

    const auto request_identity = link.submit_request(
        0x12U, bytes({0x01U}), 2U);
    expect(accepted(request_identity),
           "ordinary request must start the timeout test");
    const Frame request = decode_write(transport, 1U);
    const auto queued_identity_b = link.submit_request(
        0x13U, bytes({0x02U}), 2U);
    expect(accepted(queued_identity_b),
           "second ordinary request must be queued");
    const auto queued_identity_c = link.submit_request(
        0x14U, bytes({0x03U}), 2U);
    expect(accepted(queued_identity_c),
           "third ordinary request must be queued");
    expect(link.queued_ordinary_count() == 2U,
           "ordinary queue must retain bounded unsent work");

    const auto before_deadline_events = link.poll(21U);
    expect(!has_event(before_deadline_events,
                      LinkEventType::RequestOutcomeUnknown),
           "request must not time out before deadline");
    const auto timeout_events = link.poll(22U);
    const auto *unknown = find_event(timeout_events,
                                     LinkEventType::RequestOutcomeUnknown);
    expect(unknown != nullptr,
           "request must time out exactly at its configured deadline");
    if (unknown != nullptr) {
        expect(unknown->outcome.kind == OutcomeKind::OutcomeUnknown &&
                   unknown->outcome.sequence == request.sequence,
               "timeout must produce OutcomeUnknown for the sent request");
    }
    expect(has_event(timeout_events, LinkEventType::RequestCancelled) &&
               link.queued_ordinary_count() == 0U,
           "terminal timeout must cancel queued ordinary work");
    expect(link.poll(23U).empty(),
           "a timed-out request must emit its terminal outcome only once");

    const auto stale = link.receive(
        ack_wire(request.sequence, request.message_type), 23U);
    expect(has_event(stale, LinkEventType::AckIgnored) &&
               stale.front().ack_disposition == AckDisposition::Late &&
               !has_event(stale, LinkEventType::RequestAccepted),
           "late ACK must not resurrect a timed-out request");
}

void test_receive_stream_and_telemetry()
{
    FakeTransport transport;
    LinkCoreConfig config = quiet_config();
    LinkCore link(transport, config);
    expect(activate_link(link, transport),
           "receive stream test requires an active LinkCore");
    const auto request_identity = link.submit_request(
        0x10U, bytes({0x01U, 0U}), 2U);
    expect(accepted(request_identity),
           "receive stream test must create an ordinary request");
    const Frame request = decode_write(transport, 1U);
    const Bytes ack = ack_wire(request.sequence, request.message_type);
    const Bytes telemetry = frame_wire({
        static_cast<Byte>(MessageType::LeakStatus), 77U, bytes({1U})});

    const auto fragmented_first = link.receive(
        Bytes(ack.begin(), ack.begin() + 2), 2U);
    expect(!has_event(fragmented_first, LinkEventType::RequestAccepted),
           "fragmented ACK must not complete before its delimiter");
    const auto fragmented_second = link.receive(
        concat(Bytes(ack.begin() + 2, ack.end()), telemetry), 3U);
    expect(has_event(fragmented_second, LinkEventType::RequestAccepted) &&
               has_event(fragmented_second, LinkEventType::FrameReceived),
           "ACK and telemetry in one stream chunk must both be processed");

    FakeTransport malformed_transport;
    LinkCore malformed_link(malformed_transport, config);
    expect(activate_link(malformed_link, malformed_transport),
           "malformed recovery test requires an active LinkCore");
    const auto malformed_identity = malformed_link.submit_request(
        0x10U, bytes({0x01U, 0U}), 2U);
    expect(accepted(malformed_identity),
           "malformed recovery test must create an ordinary request");
    const Frame malformed_request = decode_write(malformed_transport, 1U);
    Bytes bad = ack_wire(malformed_request.sequence,
                         malformed_request.message_type);
    bad[bad.size() - 2U] ^= 0x01U;
    const auto malformed_recovered = malformed_link.receive(
        concat(bad, ack_wire(malformed_request.sequence,
                             malformed_request.message_type,
                             0U, 0x8001U)),
        2U);
    expect(malformed_recovered.size() == 2U &&
               malformed_recovered[0].type == LinkEventType::DecodeError &&
               malformed_recovered[1].type == LinkEventType::RequestAccepted,
           "malformed frame must not prevent the following valid ACK");

    FakeTransport unknown_transport;
    LinkCore unknown_link(unknown_transport, config);
    expect(activate_link(unknown_link, unknown_transport),
           "unknown-type test requires an active LinkCore");
    const auto unknown_identity = unknown_link.submit_request(
        0x10U, bytes({0x01U, 0U}), 2U);
    expect(accepted(unknown_identity),
           "unknown-type test must create an ordinary request");
    const Frame unknown_request = decode_write(unknown_transport, 1U);
    const Bytes unknown = frame_wire({0x7FU, 99U, bytes({0xAAU})});
    const auto unknown_events = unknown_link.receive(
        concat(unknown, ack_wire(unknown_request.sequence,
                                  unknown_request.message_type,
                                  0U, 0x8002U)),
        2U);
    expect(unknown_events.size() == 2U &&
               unknown_events[0].type == LinkEventType::UnknownMessage &&
               unknown_events[1].type == LinkEventType::RequestAccepted,
           "valid unknown message must be diagnostic and preserve ACK flow");
}

void test_state_and_heartbeat_independence()
{
    FakeTransport transport;
    LinkCoreConfig config = quiet_config();
    config.heartbeat_interval_ms = 100U;
    config.ack_timeout_ms = 20U;
    config.liveness_timeout_ms = 450U;
    config.ordinary_queue_capacity = 2U;
    LinkCore link(transport, config);
    expect(link.state() == LinkState::Unconfirmed,
           "LinkCore must begin without confirmed communication");
    link.start(0U);
    link.poll(0U);
    acknowledge_heartbeat(link, transport, 1U, 0U);
    expect(link.state() == LinkState::Active,
           "matching Heartbeat ACK must confirm communication");

    const auto ordinary_identity = link.submit_request(
        0x10U, bytes({0x01U, 0U}), 2U);
    expect(accepted(ordinary_identity),
           "ordinary timeout must be independent of Heartbeat lane");
    const auto ordinary = decode_write(transport, 1U);
    const auto degraded_events = link.poll(22U);
    expect(has_event(degraded_events, LinkEventType::RequestOutcomeUnknown) &&
               link.state() == LinkState::Degraded,
           "ordinary timeout must expose OutcomeUnknown and degrade locally");
    (void)ordinary;

    // Heartbeats continue while an ordinary request is pending and recover a
    // degraded link without replaying the timed-out ordinary request.
    const auto heartbeat_events = link.poll(100U);
    expect(has_event(heartbeat_events, LinkEventType::HeartbeatDispatched),
           "Heartbeat lane must continue independently of ordinary timeout");
    acknowledge_heartbeat(link, transport, 101U, 2U);
    expect(link.state() == LinkState::Active,
           "later healthy Heartbeat communication must recover Degraded");

    FakeTransport lost_transport;
    LinkCore lost_link(lost_transport, config);
    lost_link.start(0U);
    lost_link.poll(0U);
    const auto lost_events = lost_link.poll(450U);
    expect(has_event(lost_events, LinkEventType::StateChanged) &&
               lost_link.state() == LinkState::Lost,
           "missing Heartbeat ACKs must enter Lost at the configured deadline");
    expect(lost_transport.writes().size() == 1U,
           "Lost state must stop new Heartbeat dispatch until explicit restart");
    const auto stale_after_lost = lost_link.receive(
        ack_wire(decode_write(lost_transport, 0U).sequence,
                 static_cast<Byte>(MessageType::Heartbeat)),
        451U);
    expect(has_event(stale_after_lost, LinkEventType::AckIgnored) &&
               lost_link.state() == LinkState::Lost,
           "ACKs received while Lost must not revive local liveness");

    lost_link.start(500U);
    const auto recovery_events = lost_link.poll(500U);
    expect(has_event(recovery_events, LinkEventType::HeartbeatDispatched),
           "explicit restart must permit a fresh Heartbeat recovery attempt");
    acknowledge_heartbeat(lost_link, lost_transport, 501U, 1U);
    expect(lost_link.state() == LinkState::Active,
           "fresh matching Heartbeat ACK must recover Lost state");
}

void test_transport_write_failure_terminates_queue()
{
    FakeTransport transport;
    LinkCoreConfig config = quiet_config();
    config.ordinary_queue_capacity = 2U;
    LinkCore link(transport, config);
    expect(activate_link(link, transport),
           "write-failure test requires an active LinkCore");

    const auto identity_a = link.submit_request(0x20U, bytes({0x01U}), 2U);
    const auto identity_b = link.submit_request(0x21U, bytes({0x02U}), 2U);
    const auto identity_c = link.submit_request(0x22U, bytes({0x03U}), 2U);
    expect(accepted(identity_a) && accepted(identity_b) &&
               accepted(identity_c),
           "A, B, and C must be accepted before the transport failure");
    const Frame request_a = decode_write(transport, 1U);

    transport.set_write_succeeds(false);
    const auto events = link.receive(
        ack_wire(request_a.sequence, request_a.message_type), 3U);
    const LinkEvent *failed = find_event(events,
                                         LinkEventType::TransportWriteFailed);
    const LinkEvent *cancelled_b = find_event(events,
                                              LinkEventType::RequestCancelled);
    expect(failed != nullptr,
           "transport refusal must be observable as a write failure");
    expect(cancelled_b != nullptr &&
               cancelled_b->outcome.sequence ==
                   identity_b.sequence.value_or(0xFFFFU) &&
               cancelled_b->outcome.kind == OutcomeKind::Cancelled,
           "queued request B must receive an explicit terminal cancellation");
    const bool cancelled_c = std::any_of(
        events.begin(), events.end(), [&](const LinkEvent &event) {
            return event.type == LinkEventType::RequestCancelled &&
                   event.outcome.sequence ==
                       identity_c.sequence.value_or(0xFFFFU);
        });
    expect(cancelled_c,
           "remaining queued request C must receive a terminal cancellation");
    expect(std::count_if(events.begin(), events.end(),
                         [](const LinkEvent &event) {
                             return event.type == LinkEventType::RequestCancelled;
                         }) == 2,
           "remaining queued ordinary work must be cancelled deterministically");
    expect(!link.ordinary_request_in_flight() &&
               link.queued_ordinary_count() == 0U &&
               link.state() == LinkState::Degraded,
           "transport refusal must not leave a queue or pending-request zombie");
    (void)identity_c;
}

void test_abort_session_contract()
{
    FakeTransport transport;
    LinkCoreConfig config = quiet_config();
    config.heartbeat_interval_ms = 1000U;
    LinkCore link(transport, config);
    expect(activate_link(link, transport),
           "abort-session test requires an active LinkCore");

    const auto identity_a = link.submit_request(
        0x60U, bytes({0x01U}), 2U);
    const auto identity_b = link.submit_request(
        0x61U, bytes({0x02U}), 2U);
    const auto identity_c = link.submit_request(
        0x62U, bytes({0x03U}), 2U);
    expect(accepted(identity_a) && accepted(identity_b) &&
               accepted(identity_c) && link.ordinary_request_in_flight() &&
               link.queued_ordinary_count() == 2U,
           "abort-session test must have one pending and two queued requests");

    const std::size_t writes_before_abort = transport.writes().size();
    const std::uint16_t next_sequence_before_abort = link.next_sequence();
    const auto abort_events = link.abort_session(3U);
    const auto unknown_count = std::count_if(
        abort_events.begin(), abort_events.end(),
        [](const LinkEvent &event) {
            return event.type == LinkEventType::RequestOutcomeUnknown;
        });
    const auto cancelled_count = std::count_if(
        abort_events.begin(), abort_events.end(),
        [](const LinkEvent &event) {
            return event.type == LinkEventType::RequestCancelled;
        });
    expect(unknown_count == 1U && cancelled_count == 2U,
           "abort-session must terminate pending and queued ordinary work once");
    expect(link.state() == LinkState::Lost &&
               !link.ordinary_request_in_flight() &&
               link.queued_ordinary_count() == 0U,
           "abort-session must enter Lost and clear ordinary work");
    expect(transport.writes().size() == writes_before_abort,
           "abort-session must not write to the transport");
    expect(link.next_sequence() == next_sequence_before_abort,
           "abort-session must not allocate a new sequence");

    const auto repeated_abort_events = link.abort_session(4U);
    expect(std::count_if(repeated_abort_events.begin(),
                         repeated_abort_events.end(),
                         [](const LinkEvent &event) {
                             return event.type == LinkEventType::RequestOutcomeUnknown ||
                                    event.type == LinkEventType::RequestCancelled;
                         }) == 0U,
           "repeated abort-session must not duplicate terminal outcomes");

    expect(link.start(5U),
           "explicit restart after abort must remain available to the session owner");
    link.poll(5U);
    const Frame restarted_heartbeat = decode_write(
        transport, transport.writes().size() - 1U);
    expect(restarted_heartbeat.sequence == next_sequence_before_abort,
           "restart after abort must preserve sequence allocation continuity");
}

void test_next_wakeup_contract()
{
    FakeTransport initial_transport;
    LinkCoreConfig initial_config = quiet_config();
    initial_config.heartbeat_interval_ms = 50U;
    initial_config.ack_timeout_ms = 20U;
    initial_config.liveness_timeout_ms = 100U;
    LinkCore initial(initial_transport, initial_config);
    initial.start(100U);
    const auto initial_due = initial.next_wakeup_ms();
    expect(initial_due.has_value() && initial_due.value() == 100U,
           "initial session must schedule Heartbeat immediately");
    initial.poll(100U);
    const auto initial_heartbeat_deadline = initial.next_wakeup_ms();
    expect(initial_heartbeat_deadline.has_value() &&
               initial_heartbeat_deadline.value() == 120U,
           "Heartbeat ACK deadline must be the earliest wake after acceptance");
    const Frame initial_heartbeat = decode_write(initial_transport, 0U);
    initial.receive(ack_wire(initial_heartbeat.sequence,
                             initial_heartbeat.message_type),
                    105U);
    const auto after_heartbeat_ack = initial.next_wakeup_ms();
    expect(after_heartbeat_ack.has_value() && after_heartbeat_ack.value() == 150U,
           "matching Heartbeat ACK must remove its deadline and expose next Heartbeat");

    FakeTransport ordinary_transport;
    LinkCoreConfig ordinary_config = quiet_config();
    ordinary_config.heartbeat_interval_ms = 1000U;
    ordinary_config.ack_timeout_ms = 20U;
    ordinary_config.liveness_timeout_ms = 50U;
    LinkCore ordinary(ordinary_transport, ordinary_config);
    expect(activate_link(ordinary, ordinary_transport),
           "next-wakeup ordinary test requires an active LinkCore");
    const auto liveness_due = ordinary.next_wakeup_ms();
    expect(liveness_due.has_value() && liveness_due.value() == 51U,
           "liveness deadline must participate after a successful Heartbeat ACK");
    const auto ordinary_identity = ordinary.submit_request(
        0x63U, bytes({0x04U}), 2U);
    expect(accepted(ordinary_identity),
           "ordinary request must be accepted for its deadline test");
    const auto ordinary_due = ordinary.next_wakeup_ms();
    expect(ordinary_due.has_value() && ordinary_due.value() == 22U,
           "ordinary ACK deadline must participate in next wakeup");
    const Frame ordinary_frame = decode_write(ordinary_transport, 1U);
    const auto ordinary_ack_events = ordinary.receive(
        ack_wire(ordinary_frame.sequence, ordinary_frame.message_type), 10U);
    expect(has_event(ordinary_ack_events, LinkEventType::RequestAccepted),
           "ordinary ACK must complete before checking its removed deadline");
    const auto after_ordinary_ack = ordinary.next_wakeup_ms();
    expect(after_ordinary_ack.has_value() && after_ordinary_ack.value() == 51U,
           "completed ordinary ACK must remove its deadline");

    FakeTransport retired_transport;
    LinkCoreConfig retired_config = quiet_config();
    retired_config.heartbeat_interval_ms = 1000U;
    retired_config.ack_timeout_ms = 20U;
    retired_config.liveness_timeout_ms = 1000U;
    LinkCore retired(retired_transport, retired_config);
    retired.start(0U);
    retired.poll(0U);
    retired.poll(20U);
    const auto retired_due = retired.next_wakeup_ms();
    expect(retired_due.has_value() && retired_due.value() == 1000U,
           "retired Heartbeat correlation must not create a wakeup");

    FakeTransport lost_transport;
    LinkCoreConfig lost_config = quiet_config();
    lost_config.heartbeat_interval_ms = 1000U;
    lost_config.ack_timeout_ms = 20U;
    lost_config.liveness_timeout_ms = 20U;
    LinkCore lost(lost_transport, lost_config);
    lost.start(0U);
    lost.poll(0U);
    lost.poll(20U);
    expect(lost.state() == LinkState::Lost && !lost.next_wakeup_ms().has_value(),
           "Lost LinkCore must expose no scheduling wakeup");

    FakeTransport boundary_transport;
    LinkCoreConfig boundary_config = quiet_config();
    boundary_config.heartbeat_interval_ms = 1000U;
    boundary_config.ack_timeout_ms = 20U;
    boundary_config.liveness_timeout_ms = 1000U;
    LinkCore boundary(boundary_transport, boundary_config);
    expect(activate_link(boundary, boundary_transport),
           "deadline-boundary test requires an active LinkCore");
    const auto boundary_identity = boundary.submit_request(
        0x64U, bytes({0x05U}), 2U);
    expect(accepted(boundary_identity),
           "deadline-boundary ordinary request must be accepted");
    const Frame boundary_frame = decode_write(boundary_transport, 1U);
    const auto boundary_timeout = boundary.poll(22U);
    expect(has_event(boundary_timeout, LinkEventType::RequestOutcomeUnknown),
           "poll at an exact ACK deadline must expire the request");
    const auto boundary_ack = boundary.receive(
        ack_wire(boundary_frame.sequence, boundary_frame.message_type), 22U);
    expect(has_event(boundary_ack, LinkEventType::AckIgnored) &&
               boundary_ack.front().ack_disposition == AckDisposition::Late,
           "ACK received at an expired deadline must remain late");
}

void test_degraded_recovery_barrier()
{
    FakeTransport transport;
    LinkCoreConfig config = quiet_config();
    config.heartbeat_interval_ms = 10U;
    config.ack_timeout_ms = 20U;
    config.liveness_timeout_ms = 1000U;
    LinkCore link(transport, config);
    expect(activate_link(link, transport),
           "recovery barrier test requires an active LinkCore");

    const auto first_recovery_events = link.poll(10U);
    expect(has_event(first_recovery_events,
                     LinkEventType::HeartbeatDispatched),
           "H1 must be dispatched before the ordinary timeout");
    const Frame h1 = decode_write(transport, 1U);
    const auto ordinary_identity = link.submit_request(
        0x23U, bytes({0x01U}), 11U);
    expect(accepted(ordinary_identity),
           "ordinary request must be accepted while Active");
    const auto timeout_events = link.poll(31U);
    expect(has_event(timeout_events, LinkEventType::RequestOutcomeUnknown) &&
               link.state() == LinkState::Degraded,
           "ordinary timeout must enter Degraded with an unknown outcome");

    const auto late_h1 = link.receive(
        ack_wire(h1.sequence, h1.message_type), 32U);
    expect(has_event(late_h1, LinkEventType::AckIgnored) &&
               link.state() == LinkState::Degraded,
           "H1 dispatched before Degraded must not clear the recovery barrier");

    const std::size_t h2_index = transport.writes().size() - 1U;
    const Frame h2 = decode_write(transport, h2_index);
    expect(h2.sequence != h1.sequence,
           "recovery Heartbeat must use a fresh sequence");
    const auto h2_ack = link.receive(
        ack_wire(h2.sequence, h2.message_type), 33U);
    expect(has_event(h2_ack, LinkEventType::RequestAccepted) &&
               link.state() == LinkState::Active,
           "only a Heartbeat dispatched after Degraded may restore Active");
    (void)ordinary_identity;
}

void test_submission_state_gate_and_restart()
{
    FakeTransport transport;
    LinkCoreConfig config = quiet_config();
    config.ack_timeout_ms = 20U;
    LinkCore link(transport, config);
    const auto unconfirmed_identity = link.submit_request(
        0x30U, bytes({0x01U}), 0U);
    expect(unconfirmed_identity.status == SubmitStatus::NotActive &&
               !unconfirmed_identity.sequence.has_value() &&
               transport.writes().empty(),
           "ordinary requests must not transmit while Unconfirmed");
    expect(activate_link(link, transport),
           "state-gate test requires Heartbeat confirmation");

    const auto active_identity = link.submit_request(
        0x31U, bytes({0x02U}), 2U);
    expect(accepted(active_identity) && transport.writes().size() == 2U,
           "ordinary requests must transmit while Active");
    expect(!link.start(3U) && link.ordinary_request_in_flight(),
           "restart while Active must be rejected without discarding work");

    link.poll(22U);
    expect(link.state() == LinkState::Degraded,
           "ordinary timeout must establish the Degraded gate");
    const std::size_t writes_before_degraded_submit = transport.writes().size();
    const auto degraded_identity = link.submit_request(
        0x32U, bytes({0x03U}), 23U);
    expect(degraded_identity.status == SubmitStatus::NotActive &&
               !degraded_identity.sequence.has_value() &&
               transport.writes().size() == writes_before_degraded_submit,
           "ordinary requests must not transmit while Degraded");
}

void test_live_heartbeat_correlations_not_evicted()
{
    FakeTransport transport;
    LinkCoreConfig config = quiet_config();
    config.heartbeat_interval_ms = 1U;
    config.ack_timeout_ms = 200U;
    config.liveness_timeout_ms = std::numeric_limits<TimeMs>::max();
    config.heartbeat_history_capacity = 8U;
    LinkCore link(transport, config);
    link.start(0U);
    link.poll(0U);
    for (TimeMs now = 1U; now <= 8U; ++now) {
        link.poll(now);
    }
    expect(transport.writes().size() == 9U,
           "short Heartbeat interval must dispatch every due Heartbeat");
    const Frame first = decode_write(transport, 0U);
    const auto first_ack = link.receive(
        ack_wire(first.sequence, first.message_type), 9U);
    expect(has_event(first_ack, LinkEventType::RequestAccepted),
           "a live Heartbeat ACK must remain correlatable past history capacity");
}

void test_degraded_cancels_queued_work()
{
    FakeTransport transport;
    LinkCoreConfig config = quiet_config();
    config.heartbeat_interval_ms = 10U;
    config.ack_timeout_ms = 100U;
    config.liveness_timeout_ms = 1000U;
    config.ordinary_queue_capacity = 2U;
    LinkCore link(transport, config);
    expect(activate_link(link, transport),
           "Degraded FIFO test requires an active LinkCore");

    const auto identity_a = link.submit_request(0x40U, bytes({0x01U}), 2U);
    const auto identity_b = link.submit_request(0x41U, bytes({0x02U}), 2U);
    const auto identity_c = link.submit_request(0x42U, bytes({0x03U}), 2U);
    expect(accepted(identity_a) && accepted(identity_b) &&
               accepted(identity_c),
           "A, B, and C must be accepted before Degraded transition");
    const Frame request_a = decode_write(transport, 1U);

    const auto h1_events = link.poll(10U);
    expect(has_event(h1_events, LinkEventType::HeartbeatDispatched),
           "H1 must be dispatched while the link is Active");
    const Frame h1 = decode_write(transport, 2U);
    const auto rejected_h1 = link.receive(
        ack_wire(h1.sequence, h1.message_type, 1U), 11U);
    expect(link.state() == LinkState::Degraded,
           "non-OK Heartbeat ACK must enter Degraded");
    expect(std::count_if(rejected_h1.begin(), rejected_h1.end(),
                         [](const LinkEvent &event) {
                             return event.type == LinkEventType::RequestCancelled;
                         }) == 2,
           "entering Degraded must cancel all unsent ordinary work");
    expect(link.queued_ordinary_count() == 0U &&
               transport.writes().size() == 3U,
           "Degraded transition must leave no queued work to transmit");

    const auto completed_a = link.receive(
        ack_wire(request_a.sequence, request_a.message_type, 0U, 0x9001U),
        12U);
    const LinkEvent *a_outcome = find_event(
        completed_a, LinkEventType::RequestAccepted);
    expect(a_outcome != nullptr &&
               a_outcome->outcome.sequence == identity_a.sequence.value_or(
                   0xFFFFU),
           "in-flight A must receive only its own terminal outcome");
    expect(transport.writes().size() == 3U &&
               link.state() == LinkState::Degraded,
           "A completion must not dispatch B while Degraded");

    const auto h2_events = link.poll(20U);
    expect(has_event(h2_events, LinkEventType::HeartbeatDispatched),
           "Degraded link must dispatch a fresh recovery Heartbeat");
    const Frame h2 = decode_write(transport, 3U);
    const auto h2_ack = link.receive(
        ack_wire(h2.sequence, h2.message_type), 21U);
    expect(has_event(h2_ack, LinkEventType::RequestAccepted) &&
               link.state() == LinkState::Active,
           "post-Degraded Heartbeat ACK must restore Active");

    const auto identity_d = link.submit_request(0x43U, bytes({0x04U}), 22U);
    expect(accepted(identity_d) && transport.writes().size() == 5U,
           "only newly submitted work after recovery may transmit");
    (void)identity_b;
    (void)identity_c;
}

void test_submit_rejection_statuses()
{
    FakeTransport transport;
    LinkCoreConfig config = quiet_config();
    config.ordinary_queue_capacity = 1U;
    LinkCore link(transport, config);
    const auto not_active = link.submit_request(
        0x50U, bytes({0x01U}), 0U);
    expect(not_active.status == SubmitStatus::NotActive &&
               !not_active.sequence.has_value() && transport.writes().empty(),
           "ordinary submit while Unconfirmed must report NotActive");
    expect(activate_link(link, transport),
           "submission status test requires an active LinkCore");

    const Bytes oversized(Codec::MaxPayloadSize + 1U, 0U);
    const std::size_t writes_before_oversized = transport.writes().size();
    const auto payload_too_large = link.submit_request(
        0x51U, oversized, 2U);
    expect(payload_too_large.status == SubmitStatus::PayloadTooLarge &&
               !payload_too_large.sequence.has_value() &&
               transport.writes().size() == writes_before_oversized,
           "oversized payload must report PayloadTooLarge without transmission");

    const auto identity_a = link.submit_request(0x52U, bytes({0x02U}), 2U);
    const auto identity_b = link.submit_request(0x53U, bytes({0x03U}), 2U);
    const std::size_t writes_before_full = transport.writes().size();
    const auto queue_full = link.submit_request(0x54U, bytes({0x04U}), 2U);
    expect(accepted(identity_a) && accepted(identity_b) &&
               queue_full.status == SubmitStatus::QueueFull &&
               !queue_full.sequence.has_value() &&
               transport.writes().size() == writes_before_full,
           "full ordinary FIFO must report QueueFull without transmission");

    FakeTransport rejected_transport;
    LinkCore rejected_link(rejected_transport, config);
    expect(activate_link(rejected_link, rejected_transport),
           "transport rejection status test requires an active LinkCore");
    rejected_transport.set_write_succeeds(false);
    const std::size_t writes_before_rejection =
        rejected_transport.writes().size();
    const auto transport_rejected = rejected_link.submit_request(
        0x55U, bytes({0x05U}), 2U);
    expect(transport_rejected.status == SubmitStatus::TransportRejected &&
               !transport_rejected.sequence.has_value() &&
               rejected_transport.writes().size() == writes_before_rejection &&
               rejected_link.state() == LinkState::Degraded &&
               !rejected_link.ordinary_request_in_flight(),
           "immediate transport refusal must be diagnosable and Degraded");
}

} // namespace

void test_link_core_contract()
{
    test_heartbeat_schedule_and_wrap();
    test_heartbeat_uptime_wrap();
    test_ack_correlation_and_results();
    test_timeout_and_stale_ack();
    test_receive_stream_and_telemetry();
    test_state_and_heartbeat_independence();
    test_transport_write_failure_terminates_queue();
    test_degraded_recovery_barrier();
    test_submission_state_gate_and_restart();
    test_live_heartbeat_correlations_not_evicted();
    test_degraded_cancels_queued_work();
    test_submit_rejection_statuses();
    test_abort_session_contract();
    test_next_wakeup_contract();
}

} // namespace rbp2_test
