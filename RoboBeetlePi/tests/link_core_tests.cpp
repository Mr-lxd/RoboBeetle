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
using robobeetle::link_core::CommandOutcome;
using robobeetle::link_core::LinkCore;
using robobeetle::link_core::LinkCoreConfig;
using robobeetle::link_core::LinkEvent;
using robobeetle::link_core::LinkEventType;
using robobeetle::link_core::LinkState;
using robobeetle::link_core::OutcomeKind;
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
    link.start(0U);
    link.poll(0U);

    expect(link.submit_request(0x10U, bytes({0x01U, 0U}), 1U),
           "ordinary request must be accepted for dispatch");
    const Frame request = decode_write(transport, 1U);

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
    link.start(0U);

    expect(link.submit_request(0x12U, bytes({0x01U}), 0U),
           "ordinary request must start the timeout test");
    const Frame request = decode_write(transport, 0U);
    expect(link.submit_request(0x13U, bytes({0x02U}), 0U),
           "second ordinary request must be queued");
    expect(link.submit_request(0x14U, bytes({0x03U}), 0U),
           "third ordinary request must be queued");
    expect(link.queued_ordinary_count() == 2U,
           "ordinary queue must retain bounded unsent work");

    const auto before_deadline_events = link.poll(19U);
    expect(!has_event(before_deadline_events,
                      LinkEventType::RequestOutcomeUnknown),
           "request must not time out before deadline");
    const auto timeout_events = link.poll(20U);
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
    expect(link.poll(21U).empty(),
           "a timed-out request must emit its terminal outcome only once");

    const auto stale = link.receive(
        ack_wire(request.sequence, request.message_type), 22U);
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
    link.start(0U);
    link.poll(0U);
    expect(link.submit_request(0x10U, bytes({0x01U, 0U}), 1U),
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
    malformed_link.start(0U);
    malformed_link.poll(0U);
    expect(malformed_link.submit_request(0x10U, bytes({0x01U, 0U}), 1U),
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
    unknown_link.start(0U);
    unknown_link.poll(0U);
    expect(unknown_link.submit_request(0x10U, bytes({0x01U, 0U}), 1U),
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

    expect(link.submit_request(0x10U, bytes({0x01U, 0U}), 2U),
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

} // namespace

void test_link_core_contract()
{
    test_heartbeat_schedule_and_wrap();
    test_heartbeat_uptime_wrap();
    test_ack_correlation_and_results();
    test_timeout_and_stale_ack();
    test_receive_stream_and_telemetry();
    test_state_and_heartbeat_independence();
}

} // namespace rbp2_test
