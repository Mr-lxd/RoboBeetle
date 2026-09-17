#include "test_support.hpp"
#include "robobeetle/gateway/gateway_owner.hpp"
#include "robobeetle/gateway/tcp_adapter.hpp"
#include "robobeetle/gateway/rbrp_codec.hpp"
#include "robobeetle/protocol/codec.hpp"

#include <arpa/inet.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <netinet/in.h>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace rbp2_test { int failures = 0; }

namespace robobeetle::gateway::detail {
struct GatewayOwnerTestAccess {
    static void queue_connection(GatewayOwner &owner, ControlSourceId source)
    {
        owner.on_source_connected(source);
    }

    static std::vector<ControlSourceId>
    pending_connections(const GatewayOwner &owner)
    {
        std::lock_guard<std::mutex> lock(owner.bridge_mutex_);
        return {owner.pending_connections_.begin(),
                owner.pending_connections_.end()};
    }
};
} // namespace robobeetle::gateway::detail

namespace {

using namespace robobeetle::gateway;
using robobeetle::protocol::Codec;
using robobeetle::protocol::Frame;
using rbp2_test::expect;

struct CallbackState {
    mutable std::mutex mutex;
    std::condition_variable changed;
    std::vector<ControlSourceId> connected;
    std::vector<RemoteEnvelope> inbound;
    std::vector<SourceLostSignal> lost;
    std::size_t diagnostics{0};
    bool accept_inbound{true};

    TcpAdapterCallbacks callbacks()
    {
        return TcpAdapterCallbacks{
            [this](ControlSourceId source) {
                std::lock_guard<std::mutex> lock(mutex);
                connected.push_back(source);
                changed.notify_all();
            },
            [this](const RemoteEnvelope &envelope) {
                std::lock_guard<std::mutex> lock(mutex);
                if (!accept_inbound) {
                    return false;
                }
                inbound.push_back(envelope);
                changed.notify_all();
                return true;
            },
            [this](const SourceLostSignal &signal) {
                std::lock_guard<std::mutex> lock(mutex);
                lost.push_back(signal);
                changed.notify_all();
            },
            [this](const char *) {
                std::lock_guard<std::mutex> lock(mutex);
                ++diagnostics;
                changed.notify_all();
            },
        };
    }

    template<class Predicate>
    bool wait_for(Predicate predicate, int milliseconds = 1000)
    {
        std::unique_lock<std::mutex> lock(mutex);
        return changed.wait_for(lock, std::chrono::milliseconds(milliseconds),
                                std::move(predicate));
    }

    ControlSourceId last_source() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        return connected.empty() ? 0U : connected.back();
    }

    std::size_t inbound_count() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        return inbound.size();
    }

    std::size_t lost_count() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        return lost.size();
    }

    SourceLostSignal last_lost() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        return lost.back();
    }
};

struct BarrierApplication final : GatewayApplicationPort {
    std::size_t abort_calls{0};
    GatewayApplicationSessionState session{
        GatewayApplicationSessionState::ReopenRequired};
    GatewayApplicationLinkState link{GatewayApplicationLinkState::Unconfirmed};

    int open() override
    {
        session = GatewayApplicationSessionState::SafetyQuiet;
        return 0;
    }

    GatewayApplicationRunResult run_once() override { return {}; }

    GatewayApplicationAbortResult abort() override
    {
        ++abort_calls;
        session = GatewayApplicationSessionState::ReopenRequired;
        link = GatewayApplicationLinkState::Unconfirmed;
        return {};
    }

    GatewayApplicationSubmitResult
    submit(const RobotCommand &) override
    {
        return {GatewayApplicationSubmitStatus::NotActive, std::nullopt};
    }

    GatewayApplicationSessionState session_state() const noexcept override
    {
        return session;
    }

    GatewayApplicationLinkState link_state() const override { return link; }
};

int connect_loopback(std::uint16_t port)
{
    const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        throw std::runtime_error("client socket failed");
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr) != 1 ||
        ::connect(fd, reinterpret_cast<sockaddr *>(&address),
                   sizeof(address)) != 0) {
        ::close(fd);
        throw std::runtime_error("loopback connect failed");
    }
    return fd;
}

void send_all(int fd, const Bytes &bytes)
{
    std::size_t offset = 0U;
    while (offset < bytes.size()) {
        const auto sent = ::send(fd, bytes.data() + offset,
                                 bytes.size() - offset, MSG_NOSIGNAL);
        if (sent < 0 && errno == EINTR) {
            continue;
        }
        if (sent <= 0) {
            throw std::runtime_error("client send failed");
        }
        offset += static_cast<std::size_t>(sent);
    }
}

Bytes receive_exact(int fd, std::size_t size, int timeout_ms = 1000)
{
    Bytes result;
    result.reserve(size);
    std::array<Byte, 256U> buffer{};
    while (result.size() < size) {
        pollfd ready{fd, POLLIN, 0};
        const int polled = ::poll(&ready, 1, timeout_ms);
        if (polled < 0 && errno == EINTR) {
            continue;
        }
        if (polled <= 0 ||
            (ready.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            throw std::runtime_error("client receive timeout");
        }
        const auto received =
            ::recv(fd, buffer.data(), buffer.size(), 0);
        if (received <= 0) {
            throw std::runtime_error("client receive EOF");
        }
        result.insert(result.end(), buffer.begin(),
                      buffer.begin() + received);
    }
    return result;
}

bool peer_closed(int fd, int timeout_ms = 1000)
{
    pollfd ready{fd, POLLIN | POLLHUP, 0};
    const int polled = ::poll(&ready, 1, timeout_ms);
    if (polled <= 0) {
        return false;
    }
    if ((ready.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
        return true;
    }
    std::uint8_t byte = 0U;
    const auto received = ::recv(fd, &byte, 1U, MSG_DONTWAIT);
    return received == 0;
}

GatewayMessage hello_reply()
{
    return GatewayMessage{1U, HelloReply{0U, 512U, 250U, 1000U}};
}

Bytes protocol_ack(std::uint16_t sequence, std::uint8_t type,
                   std::uint8_t result = 0U);

void one_source_fragmented_input_and_source_addressed_output()
{
    CallbackState callbacks;
    TcpAdapter adapter("127.0.0.1", 0U, callbacks.callbacks());
    expect(adapter.start() == 0, "TCP adapter starts a loopback listener");

    const int client = connect_loopback(adapter.bound_port());
    expect(callbacks.wait_for([&callbacks] {
               return !callbacks.connected.empty();
           }), "first connection receives a nonzero source ID");
    const auto source = callbacks.last_source();
    expect(source != 0U && adapter.current_source_id() == source,
           "current TCP connection has one source generation");

    const auto hello = encode_frame(
        RbrpMessageKind::Hello, 42U, Bytes{0U, 0U});
    for (const auto byte : hello.wire) {
        send_all(client, Bytes{byte});
    }
    expect(callbacks.wait_for([&callbacks] {
               return callbacks.inbound_count() == 1U;
           }), "fragmented RBRP is reconstructed by the worker");

    {
        std::lock_guard<std::mutex> lock(callbacks.mutex);
        expect(callbacks.inbound.front().source == source &&
                   callbacks.inbound.front().message.request_id == 42U &&
                   callbacks.inbound.front().received_at_ms != 0U,
               "inbound envelope carries source and trusted receive timestamp");
    }

    const GatewayOutbound outbound{source, hello_reply()};
    expect(adapter.publish(outbound),
           "owner output accepts a source-addressed critical reply");
    const auto wire = receive_exact(client, 16U + 8U);
    const auto expected = encode_gateway_message(outbound.message);
    expect(wire == expected.wire,
           "worker serializes and sends the source-addressed output");

    ::shutdown(client, SHUT_RDWR);
    ::close(client);
    expect(callbacks.wait_for([&callbacks] {
               return callbacks.lost_count() >= 1U;
           }), "client EOF produces a SourceLost signal");
    expect(callbacks.last_lost().source == source &&
               callbacks.last_lost().reason == SourceLostReason::Disconnected,
           "disconnect SourceLost is tagged with the old source");
    adapter.stop();
}

void wrong_direction_frame_closes_source_as_fatal_protocol()
{
    CallbackState callbacks;
    TcpAdapter adapter("127.0.0.1", 0U, callbacks.callbacks());
    expect(adapter.start() == 0,
           "TCP adapter starts for wrong-direction protocol coverage");
    const int client = connect_loopback(adapter.bound_port());
    expect(callbacks.wait_for([&callbacks] {
               return callbacks.connected.size() == 1U;
           }),
           "wrong-direction test source connects");
    const auto source = callbacks.last_source();

    const auto server_frame =
        encode_frame(RbrpMessageKind::HelloReply, 77U, Bytes(8U, 0U));
    send_all(client, server_frame.wire);
    expect(callbacks.wait_for([&callbacks] {
               return callbacks.lost_count() >= 1U;
           }),
           "wrong-direction frame closes the TCP source");
    expect(callbacks.last_lost().source == source &&
               callbacks.last_lost().reason == SourceLostReason::FatalProtocol &&
               callbacks.inbound_count() == 0U,
           "wrong-direction frame produces only a FatalProtocol SourceLost");

    ::shutdown(client, SHUT_RDWR);
    ::close(client);
    adapter.stop();
}

void second_source_is_closed_and_ids_do_not_inherit()
{
    CallbackState callbacks;
    TcpAdapter adapter("127.0.0.1", 0U, callbacks.callbacks());
    expect(adapter.start() == 0, "TCP adapter starts for source isolation");
    const int first = connect_loopback(adapter.bound_port());
    expect(callbacks.wait_for([&callbacks] {
               return callbacks.connected.size() == 1U;
           }), "first source is registered");
    const auto first_source = callbacks.last_source();

    const int second = connect_loopback(adapter.bound_port());
    expect(peer_closed(second),
           "second accepted connection is immediately closed");
    ::close(second);

    ::shutdown(first, SHUT_RDWR);
    ::close(first);
    expect(callbacks.wait_for([&callbacks] {
               return callbacks.lost_count() >= 1U;
           }), "first source close is observed");

    const int replacement = connect_loopback(adapter.bound_port());
    expect(callbacks.wait_for([&callbacks] {
               return callbacks.connected.size() == 2U;
           }), "replacement connection receives a new source ID");
    const auto replacement_source = callbacks.last_source();
    expect(replacement_source > first_source &&
               replacement_source != first_source,
           "new source ID is increasing and does not inherit generation state");

    const auto stale = GatewayOutbound{first_source, hello_reply()};
    expect(adapter.publish(stale),
           "stale owner output is safely dropped rather than rerouted");
    const auto fresh = GatewayOutbound{replacement_source, hello_reply()};
    expect(adapter.publish(fresh),
           "current owner output remains sendable");
    const auto fresh_wire = receive_exact(replacement, 24U);
    expect(fresh_wire == encode_gateway_message(fresh.message).wire,
           "new source receives only its own output");

    ::shutdown(replacement, SHUT_RDWR);
    ::close(replacement);
    adapter.stop();
}

void queued_replacement_survives_same_poll_generation_rollover()
{
    CallbackState callbacks;
    TcpAdapter adapter("127.0.0.1", 0U, callbacks.callbacks());
    expect(adapter.start() == 0,
           "TCP adapter starts for deterministic generation rollover");
    const int first = connect_loopback(adapter.bound_port());
    expect(callbacks.wait_for([&callbacks] {
               return callbacks.connected.size() == 1U;
           }),
           "source A is registered before the replacement is queued");
    const auto source_a = callbacks.last_source();
    expect(adapter.publish(GatewayOutbound{source_a, hello_reply()}),
           "source A has a pending critical output during rollover setup");

    const int replacement = connect_loopback(adapter.bound_port());
    ::shutdown(first, SHUT_RDWR);
    ::close(first);
    expect(callbacks.wait_for([&callbacks] {
               return callbacks.connected.size() == 2U;
           }),
           "source B waiting in the listener survives A client revents");
    const auto source_b = callbacks.last_source();
    expect(source_b != 0U && source_b > source_a &&
               adapter.current_source_id() == source_b,
           "replacement source receives a fresh increasing generation ID");

    send_all(replacement,
             encode_frame(RbrpMessageKind::Hello, 1U, {0U, 0U}).wire);
    expect(callbacks.wait_for([&callbacks] {
               return callbacks.inbound.size() == 1U;
           }),
           "the replacement source remains readable after A rollover");
    {
        std::lock_guard<std::mutex> lock(callbacks.mutex);
        expect(callbacks.inbound.front().source == source_b,
               "old A client revents cannot retag B inbound data");
    }

    ::shutdown(replacement, SHUT_RDWR);
    ::close(replacement);
    adapter.stop();
}

void stale_poll_revents_are_rejected_by_generation()
{
    const detail::TcpClientPollSnapshot old_a{
        41, 700U, static_cast<short>(POLLHUP | POLLIN | POLLOUT)};
    expect(!detail::tcp_client_poll_snapshot_is_current(old_a, 42, 701U),
           "old A HUP/POLLIN/POLLOUT revents are discarded for current B");

    const detail::TcpClientPollSnapshot current_b{
        42, 701U, static_cast<short>(POLLIN)};
    expect(detail::tcp_client_poll_snapshot_is_current(current_b, 42, 701U),
           "current B revents remain eligible after generation rollover");
}

void inbound_delivery_barrier_preserves_timely_heartbeat()
{
    std::mutex inbound_mutex;
    std::condition_variable inbound_changed;
    bool connected = false;
    bool entered = false;
    bool released = false;
    bool enqueued = false;
    std::optional<RemoteEnvelope> captured;
    std::mutex bridge_mutex;
    std::optional<RemoteEnvelope> pending;

    TcpAdapter adapter(
        "127.0.0.1", 0U,
        TcpAdapterCallbacks{
            [&](ControlSourceId) {
                std::lock_guard<std::mutex> lock(inbound_mutex);
                connected = true;
                inbound_changed.notify_all();
            },
            [&](const RemoteEnvelope &envelope) {
                std::unique_lock<std::mutex> lock(inbound_mutex);
                captured = envelope;
                entered = true;
                inbound_changed.notify_all();
                inbound_changed.wait(lock, [&] { return released; });
                {
                    std::lock_guard<std::mutex> bridge_lock(bridge_mutex);
                    pending = envelope;
                }
                enqueued = true;
                inbound_changed.notify_all();
                return true;
            },
            [](const SourceLostSignal &) {},
            [](const char *) {},
        });
    expect(adapter.start() == 0,
           "TCP adapter starts for deterministic ingress barrier coverage");
    const int client = connect_loopback(adapter.bound_port());
    {
        std::unique_lock<std::mutex> lock(inbound_mutex);
        expect(inbound_changed.wait_for(
                   lock, std::chrono::seconds(1), [&] { return connected; }),
               "ingress barrier source connects before the heartbeat");
    }

    send_all(client,
             encode_frame(RbrpMessageKind::ControlHeartbeat, 1U, {}).wire);
    RemoteEnvelope heartbeat_envelope;
    {
        std::unique_lock<std::mutex> lock(inbound_mutex);
        expect(inbound_changed.wait_for(
                   lock, std::chrono::seconds(1), [&] { return entered; }),
               "worker marks decoded heartbeat delivery before enqueue returns");
        if (captured.has_value()) {
            heartbeat_envelope = *captured;
        }
    }
    expect(adapter.inbound_delivery_in_flight(),
           "in-flight delivery remains visible while enqueue is blocked");

    BarrierApplication application;
    GatewayTimeMs fake_now = heartbeat_envelope.received_at_ms - 999U;
    std::vector<GatewayOutbound> outputs;
    ControlGatewayCore core(
        application,
        GatewayCoreCallbacks{
            [&](const GatewayOutbound &output) {
                outputs.push_back(output);
                return true;
            },
            [](const CloseSourceSignal &) { return true; },
            [](const char *) {},
            [&fake_now] { return fake_now; },
        });
    const auto source = heartbeat_envelope.source;
    core.source_connected(source);
    core.process(RemoteEnvelope{source, fake_now,
                                RemoteMessage{RbrpMessageKind::Hello, 10U,
                                              HelloRequest{0U}}},
                 fake_now);
    core.process(RemoteEnvelope{source, fake_now,
                                RemoteMessage{RbrpMessageKind::AcquireControl,
                                              11U, AcquireControlRequest{}}},
                 fake_now);
    const auto check_now = heartbeat_envelope.received_at_ms + 1U;
    bool evaluated = false;
    {
        std::lock_guard<std::mutex> bridge_lock(bridge_mutex);
        if (!pending.has_value() && !adapter.inbound_delivery_in_flight()) {
            core.check_time(check_now);
            evaluated = true;
        }
    }
    expect(!evaluated && core.authority_state() == AuthorityState::Owned,
           "owner lease boundary does not expire while decoded delivery is in flight");

    {
        std::lock_guard<std::mutex> lock(inbound_mutex);
        released = true;
        inbound_changed.notify_all();
    }
    {
        std::unique_lock<std::mutex> lock(inbound_mutex);
        expect(inbound_changed.wait_for(
                   lock, std::chrono::seconds(1), [&] { return enqueued; }),
               "blocked heartbeat completes enqueue after the lease boundary");
    }
    {
        std::lock_guard<std::mutex> bridge_lock(bridge_mutex);
        expect(pending.has_value() &&
                   pending->received_at_ms == heartbeat_envelope.received_at_ms,
               "the queued envelope keeps its pre-boundary trusted timestamp");
        if (pending.has_value()) {
            core.process(*pending, check_now);
        }
    }
    expect(core.authority_state() == AuthorityState::Owned &&
               application.abort_calls == 0U,
           "a timely heartbeat remains authoritative after delayed enqueue");

    ::shutdown(client, SHUT_RDWR);
    ::close(client);
    adapter.stop();
}

void coalesced_recv_batch_shares_one_trusted_timestamp()
{
    const auto command =
        encode_frame(RbrpMessageKind::CommandRequest, 20U, {0x06U});
    const auto heartbeat =
        encode_frame(RbrpMessageKind::ControlHeartbeat, 21U, {});
    Bytes batch = command.wire;
    batch.insert(batch.end(), heartbeat.wire.begin(), heartbeat.wire.end());
    std::atomic<std::size_t> inbound_in_flight{0U};
    std::mutex batch_mutex;
    std::condition_variable batch_changed;
    bool first_delivery_entered = false;
    bool release_first_delivery = false;
    bool barrier_active_on_second = false;
    bool close_called = false;
    bool process_result = false;
    std::size_t now_calls = 0U;
    std::vector<RemoteEnvelope> captured;
    RbrpDecoder decoder;

    std::thread worker([&] {
        process_result = detail::process_tcp_inbound_batch(
            decoder, inbound_in_flight, 51U, batch.data(), batch.size(),
            [&] {
                ++now_calls;
                return 1999U;
            },
            [&](const RemoteEnvelope &envelope) {
                std::unique_lock<std::mutex> lock(batch_mutex);
                captured.push_back(envelope);
                if (captured.size() == 1U) {
                    first_delivery_entered = true;
                    batch_changed.notify_all();
                    batch_changed.wait(lock,
                                       [&] { return release_first_delivery; });
                }
                if (captured.size() == 2U) {
                    barrier_active_on_second =
                        inbound_in_flight.load() == 1U;
                }
                batch_changed.notify_all();
                return true;
            },
            [](const char *) {},
            [&](SourceLostReason) { close_called = true; });
        std::lock_guard<std::mutex> lock(batch_mutex);
        batch_changed.notify_all();
    });

    {
        std::unique_lock<std::mutex> lock(batch_mutex);
        expect(batch_changed.wait_for(
                   lock, std::chrono::seconds(1),
                   [&] { return first_delivery_entered; }),
               "deterministic decode batch reaches its first delivery");
        expect(inbound_in_flight.load() == 1U,
               "inbound barrier is active during the first batch delivery");
        release_first_delivery = true;
        batch_changed.notify_all();
    }
    worker.join();

    expect(process_result && !close_called &&
               inbound_in_flight.load() == 0U && captured.size() == 2U &&
               now_calls == 1U &&
               captured[0].received_at_ms == captured[1].received_at_ms &&
               barrier_active_on_second,
           "one decoder.feed batch delivers both frames under one barrier and timestamp");

    BarrierApplication application;
    GatewayTimeMs fake_now = 1000U;
    std::vector<GatewayOutbound> outputs;
    ControlGatewayCore core(
        application,
        GatewayCoreCallbacks{
            [&](const GatewayOutbound &output) {
                outputs.push_back(output);
                return true;
            },
            [](const CloseSourceSignal &) { return true; },
            [](const char *) {},
            [&fake_now] { return fake_now; },
        });
    core.source_connected(51U);
    core.process(RemoteEnvelope{51U, fake_now,
                                RemoteMessage{RbrpMessageKind::Hello, 10U,
                                              HelloRequest{0U}}},
                 fake_now);
    core.process(RemoteEnvelope{51U, fake_now,
                                RemoteMessage{RbrpMessageKind::AcquireControl,
                                              11U, AcquireControlRequest{}}},
                 fake_now);

    bool authority_survived_batch = false;
    if (captured.size() == 2U) {
        core.process(captured[0], fake_now);
        core.process(captured[1], fake_now);
        authority_survived_batch =
            core.authority_state() == AuthorityState::Owned &&
            application.abort_calls == 0U &&
            core.lease_deadline_ms() == captured[1].received_at_ms + 1000U;
    }
    expect(authority_survived_batch,
           "a coalesced ordinary message and heartbeat cannot false-expire authority");

}

void newest_source_registration_replaces_pending_older_generation()
{
    GatewayOwner owner("/unused", "127.0.0.1", 0U);
    detail::GatewayOwnerTestAccess::queue_connection(owner, 70U);
    detail::GatewayOwnerTestAccess::queue_connection(owner, 71U);
    const auto pending =
        detail::GatewayOwnerTestAccess::pending_connections(owner);
    expect(pending.size() == 1U && pending.front() == 71U,
           "a newer source registration coalesces the older pending generation");
}

void stale_close_and_old_telemetry_never_affect_new_source()
{
    CallbackState callbacks;
    TcpAdapter adapter("127.0.0.1", 0U, callbacks.callbacks());
    expect(adapter.start() == 0, "TCP adapter starts for stale control test");
    const int first = connect_loopback(adapter.bound_port());
    expect(callbacks.wait_for([&callbacks] {
               return callbacks.connected.size() == 1U;
           }), "source A connects");
    const auto source_a = callbacks.last_source();

    ::shutdown(first, SHUT_RDWR);
    ::close(first);
    expect(callbacks.wait_for([&callbacks] {
               return callbacks.lost_count() >= 1U;
           }), "source A disconnects");

    const int second = connect_loopback(adapter.bound_port());
    expect(callbacks.wait_for([&callbacks] {
               return callbacks.connected.size() == 2U;
           }), "source B connects");
    const auto source_b = callbacks.last_source();

    expect(adapter.close_source(
               CloseSourceSignal{source_a, CloseSourceReason::CriticalTxFailure}),
           "stale CloseSource is accepted into the control path");
    expect(adapter.publish(GatewayOutbound{source_a, hello_reply()}),
           "stale A output is dropped without touching B");
    expect(adapter.publish(GatewayOutbound{source_b, hello_reply()}),
           "B output is accepted after stale A controls");
    expect(receive_exact(second, 24U) ==
               encode_gateway_message(hello_reply()).wire,
           "B receives no stale A output or stale close");

    ::shutdown(second, SHUT_RDWR);
    ::close(second);
    adapter.stop();
}

void inbound_queue_exhaustion_is_source_loss()
{
    CallbackState callbacks;
    callbacks.accept_inbound = false;
    TcpAdapter adapter("127.0.0.1", 0U, callbacks.callbacks());
    expect(adapter.start() == 0, "TCP adapter starts for inbound bound test");
    const int client = connect_loopback(adapter.bound_port());
    expect(callbacks.wait_for([&callbacks] {
               return callbacks.connected.size() == 1U;
           }), "source connects before inbound exhaustion");

    const auto hello = encode_frame(
        RbrpMessageKind::Hello, 1U, Bytes{0U, 0U});
    send_all(client, hello.wire);
    expect(callbacks.wait_for([&callbacks] {
               return callbacks.lost_count() >= 1U;
           }), "failed inbound enqueue creates high-priority SourceLost");
    expect(callbacks.last_lost().reason ==
               SourceLostReason::InboundQueueExhausted,
           "inbound queue exhaustion carries its explicit source-loss reason");
    ::close(client);
    adapter.stop();
}

void critical_bound_partial_telemetry_and_shutdown_contract()
{
    expect(TcpAdapter::kMaxInboundMessages == 32U &&
               TcpAdapter::kMaxInboundPayloadBytes == 16U * 1024U &&
               TcpAdapter::kMaxCriticalFrames == 32U &&
               TcpAdapter::kMaxCriticalBytes == 32U * 1024U &&
               TcpAdapter::kTelemetrySlotCount == 3U,
           "TCP bridge exposes the frozen bounded resource contract");

    CallbackState callbacks;
    TcpAdapter adapter("127.0.0.1", 0U, callbacks.callbacks());
    expect(adapter.start() == 0, "TCP adapter starts for bounded output test");
    const int client = connect_loopback(adapter.bound_port());
    expect(callbacks.wait_for([&callbacks] {
               return callbacks.connected.size() == 1U;
           }), "source connects before output bound test");
    const auto source = callbacks.last_source();

    const auto telemetry = GatewayOutbound{
        source, GatewayMessage{
                    0U, GatewayLeakTelemetry{7U, LeakState::Dry}}};
    const auto newer = GatewayOutbound{
        source, GatewayMessage{
                    0U, GatewayLeakTelemetry{8U, LeakState::Wet}}};
    expect(adapter.publish(telemetry) && adapter.publish(newer),
           "latest telemetry can replace an unsent same-kind value");
    const auto first_wire = receive_exact(client, 19U);
    RbrpDecoder decoder;
    std::vector<RbrpFrame> frames;
    expect(decoder.feed(first_wire.data(), first_wire.size(), frames) ==
               RbrpFeedStatus::Ok &&
               frames.size() == 1U &&
               (frames.front().payload == Bytes{7U, 0U, 1U} ||
                frames.front().payload == Bytes{8U, 0U, 2U}),
           "telemetry output is a complete source-addressed RBRP frame");

    for (std::size_t i = 0U; i < 10000U; ++i) {
        if (!adapter.publish(
                GatewayOutbound{source, GatewayMessage{
                                           i + 1U,
                                           ServiceErrorMessage{
                                               ServiceErrorCode::NotAuthority,
                                               RbrpMessageKind::CommandRequest,
                                               static_cast<std::uint32_t>(i)}}})) {
            break;
        }
        if (i == 9999U) {
            expect(false, "critical output eventually reports bounded exhaustion");
        }
    }

    ::shutdown(client, SHUT_RDWR);
    ::close(client);
    adapter.stop();
}

struct ProtocolPty {
    int master{-1};
    std::string path;
    Bytes buffered;

    ProtocolPty()
    {
        master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (master < 0 || ::grantpt(master) != 0 || ::unlockpt(master) != 0 ||
            ::ptsname(master) == nullptr) {
            if (master >= 0) {
                ::close(master);
            }
            throw std::runtime_error("protocol PTY setup failed");
        }
        path = ::ptsname(master);
    }

    ~ProtocolPty()
    {
        close_master();
    }

    void close_master() noexcept
    {
        if (master >= 0) {
            ::close(master);
            master = -1;
        }
    }

    void send(const Bytes &wire) const
    {
        std::size_t offset = 0U;
        while (offset < wire.size()) {
            const auto written =
                ::write(master, wire.data() + offset, wire.size() - offset);
            if (written < 0 && errno == EINTR) {
                continue;
            }
            if (written <= 0) {
                throw std::runtime_error("protocol PTY write failed");
            }
            offset += static_cast<std::size_t>(written);
        }
    }

    Bytes next_wire(int timeout_ms = 2000)
    {
        for (;;) {
            const auto delimiter =
                std::find(buffered.begin(), buffered.end(), Byte{0U});
            if (delimiter != buffered.end()) {
                const auto end = delimiter + 1;
                Bytes frame(buffered.begin(), end);
                buffered.erase(buffered.begin(), end);
                return frame;
            }

            pollfd ready{master, POLLIN, 0};
            const int polled = ::poll(&ready, 1, timeout_ms);
            if (polled < 0 && errno == EINTR) {
                continue;
            }
            if (polled <= 0 ||
                (ready.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                throw std::runtime_error("protocol PTY receive timeout");
            }
            std::array<Byte, 512U> bytes{};
            const auto received =
                ::read(master, bytes.data(), bytes.size());
            if (received < 0 && errno == EINTR) {
                continue;
            }
            if (received <= 0) {
                throw std::runtime_error("protocol PTY receive EOF");
            }
            buffered.insert(buffered.end(), bytes.begin(),
                            bytes.begin() + received);
        }
    }

    Frame next_frame(int timeout_ms = 2000)
    {
        for (;;) {
            const auto wire = next_wire(timeout_ms);
            if (wire.size() == 1U && wire[0] == 0U) {
                continue;
            }
            Bytes body(wire.begin(), wire.end() - 1);
            const auto decoded = Codec::decodeWire(body);
            if (!decoded.ok()) {
                throw std::runtime_error("malformed Protocol V2 output");
            }
            return decoded.frame;
        }
    }

    Frame next_frame_of_type(std::uint8_t type)
    {
        for (;;) {
            const auto frame = next_frame();
            if (frame.message_type == type) {
                return frame;
            }
        }
    }

    bool no_command_for(int timeout_ms)
    {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(timeout_ms);
        for (;;) {
            if (master < 0) {
                return true;
            }
            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline) {
                return true;
            }
            const auto remaining = static_cast<int>(std::min(
                static_cast<long long>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        deadline - now)
                        .count()),
                200LL));
            pollfd ready{master, POLLIN, 0};
            const int polled = ::poll(&ready, 1, remaining);
            if (polled < 0 && errno == EINTR) {
                continue;
            }
            if (polled <= 0 ||
                (ready.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                return true;
            }
            const auto wire = next_wire(remaining);
            if (wire.size() == 1U && wire[0] == 0U) {
                continue;
            }
            Bytes body(wire.begin(), wire.end() - 1);
            const auto decoded = Codec::decodeWire(body);
            if (!decoded.ok()) {
                throw std::runtime_error("malformed Protocol V2 output");
            }
            if (decoded.frame.message_type == 0x01U) {
                send(protocol_ack(decoded.frame.sequence, 0x01U));
                continue;
            }
            return false;
        }
    }
};

Bytes protocol_ack(std::uint16_t sequence, std::uint8_t type,
                   std::uint8_t result)
{
    return Codec::encodeWire(Frame{
        0x02U, sequence,
        {static_cast<std::uint8_t>(sequence & 0xffU),
         static_cast<std::uint8_t>(sequence >> 8U), type, result}});
}

struct RbrpSocketReader {
    int fd;
    RbrpDecoder decoder;
    std::vector<RbrpFrame> pending;

    RbrpFrame next(int timeout_ms = 2000)
    {
        for (;;) {
            if (!pending.empty()) {
                RbrpFrame frame = std::move(pending.front());
                pending.erase(pending.begin());
                return frame;
            }
            pollfd ready{fd, POLLIN, 0};
            const int polled = ::poll(&ready, 1, timeout_ms);
            if (polled < 0 && errno == EINTR) {
                continue;
            }
            if (polled <= 0 ||
                (ready.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                throw std::runtime_error("RBRP client receive timeout");
            }
            std::array<Byte, 1024U> bytes{};
            const auto received =
                ::recv(fd, bytes.data(), bytes.size(), 0);
            if (received <= 0) {
                throw std::runtime_error("RBRP client receive EOF");
            }
            if (decoder.feed(bytes.data(), static_cast<std::size_t>(received),
                             pending) == RbrpFeedStatus::Fatal) {
                throw std::runtime_error("gateway emitted fatal RBRP");
            }
        }
    }

    RbrpFrame next_kind(RbrpMessageKind kind)
    {
        for (;;) {
            auto frame = next();
            if (frame.kind == kind) {
                return frame;
            }
        }
    }

    std::optional<RbrpFrame> try_next_kind(RbrpMessageKind kind,
                                            int timeout_ms)
    {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(timeout_ms);
        for (;;) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline) {
                return std::nullopt;
            }
            const auto remaining = static_cast<int>(std::max(
                1LL,
                static_cast<long long>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        deadline - now)
                        .count())));
            RbrpFrame frame;
            try {
                frame = next(remaining);
            } catch (const std::exception &) {
                return std::nullopt;
            }
            if (frame.kind == kind) {
                return frame;
            }
        }
    }
};

std::uint16_t le16(const Bytes &bytes, std::size_t offset)
{
    return static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(bytes[offset]) |
        (static_cast<std::uint16_t>(bytes[offset + 1U]) << 8U));
}

void send_remote(int fd, RbrpMessageKind kind, RequestId request_id,
                 const Bytes &payload)
{
    const auto encoded = encode_frame(kind, request_id, payload);
    if (encoded.status != RbrpEncodeStatus::Ok) {
        throw std::runtime_error("invalid test RBRP frame");
    }
    send_all(fd, encoded.wire);
}

Bytes protocol_frame(std::uint8_t type, std::uint16_t sequence,
                     const Bytes &payload)
{
    return Codec::encodeWire(Frame{type, sequence, payload});
}

void acknowledge_heartbeats_for(ProtocolPty &pty, int timeout_ms)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto remaining = static_cast<int>(std::max(
            1LL,
            static_cast<long long>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    deadline - std::chrono::steady_clock::now())
                    .count())));
        Frame frame;
        try {
            frame = pty.next_frame(remaining);
        } catch (const std::exception &) {
            return;
        }
        if (frame.message_type != 0x01U) {
            throw std::runtime_error("unexpected UART frame while leasing");
        }
        pty.send(protocol_ack(frame.sequence, 0x01U));
    }
}

struct RealOwnerFixture {
    ProtocolPty pty;
    GatewayOwner owner;
    int client{-1};
    std::optional<RbrpSocketReader> reader;

    RealOwnerFixture()
        : owner(pty.path, "127.0.0.1", 0U)
    {
        if (owner.start() != 0) {
            throw std::runtime_error("GatewayOwner start failed");
        }
        connect_client();
    }

    ~RealOwnerFixture()
    {
        disconnect_client();
        owner.stop();
    }

    void connect_client()
    {
        client = connect_loopback(owner.bound_port());
        reader.emplace(client);
    }

    void disconnect_client() noexcept
    {
        reader.reset();
        if (client >= 0) {
            ::shutdown(client, SHUT_RDWR);
            ::close(client);
            client = -1;
        }
    }

    RbrpFrame next_kind(RbrpMessageKind kind)
    {
        return reader->next_kind(kind);
    }

    void hello_acquire(RequestId hello_id, RequestId acquire_id)
    {
        send_remote(client, RbrpMessageKind::Hello, hello_id, {0U, 0U});
        const auto hello = next_kind(RbrpMessageKind::HelloReply);
        expect(hello.request_id == hello_id && hello.payload.size() == 8U,
               "real owner completes Hello with the negotiated payload");

        send_remote(client, RbrpMessageKind::AcquireControl, acquire_id, {});
        const auto acquire = next_kind(RbrpMessageKind::AcquireReply);
        expect(acquire.request_id == acquire_id && acquire.payload.size() == 12U &&
                   acquire.payload[0] == 0U && acquire.payload[1] == 2U,
               "real owner grants authority only after explicit Acquire");
        (void)next_kind(RbrpMessageKind::ControlState);
    }

    std::uint16_t activate()
    {
        const auto raw_zero = pty.next_wire();
        expect(raw_zero == Bytes{0U},
               "real owner preserves the 575-ms safety-quiet boundary");
        const auto heartbeat = pty.next_frame();
        expect(heartbeat.message_type == 0x01U,
               "real owner emits a heartbeat before any post-acquire command");
        if (heartbeat.message_type == 0x01U) {
            pty.send(protocol_ack(heartbeat.sequence, 0x01U));
        }
        const auto state = next_kind(RbrpMessageKind::ControlState);
        expect(state.request_id == 0U && state.payload.size() == 8U &&
                   state.payload[2] ==
                       static_cast<Byte>(GatewayApplicationLinkState::Active),
               "real owner publishes Link Active before remote commands proceed");
        return heartbeat.sequence;
    }
};

RbrpFrame submit_remote_command(RealOwnerFixture &fixture,
                                RequestId request_id, const Bytes &payload)
{
    send_remote(fixture.client, RbrpMessageKind::CommandRequest, request_id,
                payload);
    const auto submitted = fixture.next_kind(RbrpMessageKind::CommandSubmitted);
    expect(submitted.request_id == request_id && submitted.payload.size() == 4U,
           "real owner returns CommandSubmitted for a typed command");
    return submitted;
}

Frame next_uart_command(ProtocolPty &pty, std::uint8_t message_type)
{
    for (;;) {
        const auto frame = pty.next_frame();
        if (frame.message_type == 0x01U) {
            pty.send(protocol_ack(frame.sequence, 0x01U));
            continue;
        }
        if (frame.message_type == message_type) {
            return frame;
        }
        throw std::runtime_error("unexpected Protocol V2 command frame");
    }
}

void real_owner_rejected_ack_and_telemetry_forwarding()
{
    RealOwnerFixture fixture;
    fixture.hello_acquire(1U, 2U);
    (void)fixture.activate();

    const auto submitted = submit_remote_command(
        fixture, 10U, {0x02U, 0x01U, 0x00U});
    expect(submitted.payload[0] == 0U && submitted.payload[1] == 1U,
           "a valid command is admitted before its final ACK");
    const auto command = next_uart_command(fixture.pty, 0x11U);
    expect(command.payload == Bytes{0x01U, 0x00U},
           "real owner preserves the typed DisableServos payload");
    fixture.pty.send(protocol_ack(command.sequence, 0x11U, 7U));
    const auto rejected = fixture.next_kind(RbrpMessageKind::CommandOutcome);
    expect(rejected.request_id == 10U && rejected.payload.size() == 5U &&
               rejected.payload[0] == 1U && rejected.payload[1] == 2U &&
               le16(rejected.payload, 2U) == command.sequence &&
               rejected.payload[4] == 7U,
           "a rejected STM32 ACK becomes a source-correlated outcome");

    fixture.pty.send(protocol_frame(
        0x20U, 55U, {2U}));
    const auto leak = fixture.next_kind(RbrpMessageKind::LeakTelemetry);
    expect(leak.request_id == 0U && leak.payload == Bytes{55U, 0U, 2U},
           "real owner forwards typed Leak telemetry with its sequence");

    Bytes imu(56U, 0U);
    imu[0] = 1U;
    imu[1] = 0x07U;
    fixture.pty.send(protocol_frame(0x21U, 56U, imu));
    const auto imu_wire = fixture.next_kind(RbrpMessageKind::ImuTelemetry);
    expect(imu_wire.request_id == 0U && imu_wire.payload.size() == 58U &&
               le16(imu_wire.payload, 0U) == 56U &&
               imu_wire.payload[2] == 1U && imu_wire.payload[3] == 0x07U,
           "real owner forwards the typed IMU telemetry schema");

    Bytes depth(38U, 0U);
    depth[0] = 1U;
    depth[1] = 0x03U;
    fixture.pty.send(protocol_frame(0x22U, 57U, depth));
    const auto depth_wire = fixture.next_kind(RbrpMessageKind::DepthTelemetry);
    expect(depth_wire.request_id == 0U && depth_wire.payload.size() == 40U &&
               le16(depth_wire.payload, 0U) == 57U &&
               depth_wire.payload[2] == 1U && depth_wire.payload[3] == 0x03U,
           "real owner forwards the typed Depth telemetry schema");
}

void real_owner_queued_heartbeat_does_not_false_expire()
{
    RealOwnerFixture fixture;
    fixture.hello_acquire(60U, 61U);
    (void)fixture.activate();

    std::atomic<bool> stop_acknowledger{false};
    std::thread uart_acknowledger([&fixture, &stop_acknowledger] {
        while (!stop_acknowledger.load()) {
            try {
                const auto frame = fixture.pty.next_frame(100);
                if (frame.message_type == 0x01U) {
                    fixture.pty.send(protocol_ack(frame.sequence, 0x01U));
                }
            } catch (const std::exception &) {
                // A short poll timeout is expected between lower-layer
                // heartbeat frames; keep servicing until the stress window.
            }
        }
    });

    RequestId request_id = 100U;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(1500);
    while (std::chrono::steady_clock::now() < deadline) {
        send_remote(fixture.client, RbrpMessageKind::ControlHeartbeat,
                    request_id++, {});
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }

    stop_acknowledger.store(true);
    uart_acknowledger.join();
    const auto submitted = submit_remote_command(fixture, 200U, {0x06U});
    expect(submitted.payload[0] == 0U && submitted.payload[1] == 1U,
           "heartbeats queued while owner work runs do not false-expire the lease");
}

void real_owner_sustained_inbound_does_not_starve_pty()
{
    RealOwnerFixture fixture;
    fixture.hello_acquire(70U, 71U);
    (void)fixture.activate();

    std::atomic<bool> stop_traffic{false};
    std::atomic<bool> traffic_failed{false};
    std::thread traffic([&] {
        RequestId request_id = 300U;
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(2200);
        auto next_send = std::chrono::steady_clock::now();
        while (!stop_traffic.load() &&
               std::chrono::steady_clock::now() < deadline) {
            try {
                send_remote(fixture.client,
                            RbrpMessageKind::ControlHeartbeat, request_id++, {});
            } catch (const std::exception &) {
                traffic_failed.store(true);
                return;
            }
            next_send += std::chrono::milliseconds(20);
            std::this_thread::sleep_until(next_send);
        }
    });

    std::size_t onboard_heartbeats = 0U;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(2200);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto remaining = static_cast<int>(std::max(
            1LL,
            std::min(200LL,
                     static_cast<long long>(
                         std::chrono::duration_cast<std::chrono::milliseconds>(
                             deadline - std::chrono::steady_clock::now())
                             .count()))));
        Frame frame;
        try {
            frame = fixture.pty.next_frame(remaining);
        } catch (const std::exception &) {
            continue;
        }
        if (frame.message_type != 0x01U) {
            throw std::runtime_error(
                "unexpected UART frame during independent ingress stress");
        }
        fixture.pty.send(protocol_ack(frame.sequence, 0x01U));
        ++onboard_heartbeats;
    }

    stop_traffic.store(true);
    traffic.join();

    expect(onboard_heartbeats >= 3U && !traffic_failed.load(),
           "independent bounded inbound traffic still services multiple Protocol V2 heartbeats");
    const auto submitted = submit_remote_command(
        fixture, 400U, {0x06U});
    expect(submitted.payload[0] == 0U && submitted.payload[1] == 1U,
           "authority remains usable after sustained inbound traffic");
}

void real_owner_nonheartbeat_traffic_does_not_keep_lease_alive()
{
    RealOwnerFixture fixture;
    fixture.hello_acquire(80U, 81U);
    (void)fixture.activate();
    fixture.reader.reset();

    std::atomic<bool> stop_uart{false};
    std::atomic<bool> uart_failed{false};
    std::thread uart_acknowledger([&fixture, &stop_uart, &uart_failed] {
        while (!stop_uart.load()) {
            try {
                const auto frame = fixture.pty.next_frame(100);
                if (frame.message_type == 0x01U) {
                    fixture.pty.send(protocol_ack(frame.sequence, 0x01U));
                } else {
                    uart_failed.store(true);
                    return;
                }
            } catch (const std::exception &) {
                // A short poll timeout is expected between lower-layer
                // heartbeat frames; keep servicing until the stress window.
            }
        }
    });

    RbrpSocketReader response_reader{fixture.client};
    std::mutex response_mutex;
    std::condition_variable response_changed;
    std::atomic<bool> stop_responses{false};
    bool lease_expired = false;
    bool link_lost = false;
    std::thread responses([&] {
        while (!stop_responses.load()) {
            try {
                const auto frame = response_reader.next(100);
                if (frame.kind != RbrpMessageKind::ControlState ||
                    frame.payload.size() < 4U) {
                    continue;
                }
                std::lock_guard<std::mutex> lock(response_mutex);
                if (frame.payload[3] ==
                    static_cast<Byte>(GatewayStateReason::LeaseExpired)) {
                    lease_expired = true;
                }
                if (frame.payload[3] ==
                    static_cast<Byte>(GatewayStateReason::LinkLost)) {
                    link_lost = true;
                }
                response_changed.notify_all();
            } catch (const std::exception &) {
                // The reader is intentionally independent of the bounded
                // producer so TCP responses cannot become its backpressure.
            }
        }
    });

    std::atomic<bool> traffic_failed{false};
    std::atomic<bool> stop_traffic{false};
    std::thread traffic([&] {
        RequestId request_id = 500U;
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(1600);
        auto next_send = std::chrono::steady_clock::now();
        while (!stop_traffic.load() &&
               std::chrono::steady_clock::now() < deadline) {
            try {
                send_remote(fixture.client, RbrpMessageKind::Hello,
                            request_id++, {0U, 0U});
            } catch (const std::exception &) {
                traffic_failed.store(true);
                return;
            }
            next_send += std::chrono::milliseconds(20);
            std::this_thread::sleep_until(next_send);
        }
    });

    bool observed_lease_expired = false;
    {
        std::unique_lock<std::mutex> lock(response_mutex);
        observed_lease_expired = response_changed.wait_for(
            lock, std::chrono::milliseconds(2500),
            [&] { return lease_expired; });
    }

    stop_traffic.store(true);
    traffic.join();
    stop_responses.store(true);
    ::shutdown(fixture.client, SHUT_RDWR);
    responses.join();
    stop_uart.store(true);
    uart_acknowledger.join();

    {
        std::lock_guard<std::mutex> lock(response_mutex);
        expect(observed_lease_expired && lease_expired && !link_lost &&
                   !traffic_failed.load() && !uart_failed.load(),
               "independent duplicate Hello traffic cannot keep authority alive or cause LinkLost");
    }
}

void real_owner_wrong_direction_revokes_authority_without_uart_command()
{
    RealOwnerFixture fixture;
    fixture.hello_acquire(90U, 91U);
    (void)fixture.activate();

    const auto server_frame =
        encode_frame(RbrpMessageKind::HelloReply, 92U, Bytes(8U, 0U));
    send_all(fixture.client, server_frame.wire);
    expect(peer_closed(fixture.client),
           "wrong-direction TCP input closes source A peer");
    expect(fixture.pty.no_command_for(200),
           "wrong-direction TCP input does not synthesize a UART actuator command");

    fixture.disconnect_client();
    fixture.connect_client();
    send_remote(fixture.client, RbrpMessageKind::CommandRequest, 100U,
                Bytes{0x06U});
    const auto before_hello = fixture.next_kind(RbrpMessageKind::ServiceError);
    expect(before_hello.request_id == 100U && before_hello.payload.size() == 8U &&
               le16(before_hello.payload, 0U) ==
                   static_cast<std::uint16_t>(ServiceErrorCode::NotHello),
           "replacement source B does not inherit usable authority before Hello");

    fixture.hello_acquire(101U, 102U);
}

void real_owner_lease_expiry_aborts_without_synthesized_command()
{
    RealOwnerFixture fixture;
    fixture.hello_acquire(20U, 21U);
    (void)fixture.activate();

    acknowledge_heartbeats_for(fixture.pty, 1400);
    const auto state = fixture.next_kind(RbrpMessageKind::ControlState);
    expect(state.request_id == 0U && state.payload.size() == 8U &&
               state.payload[0] == 0U &&
               state.payload[3] == static_cast<Byte>(GatewayStateReason::LeaseExpired),
           "gateway lease expiry revokes authority while TCP remains open");
    expect(fixture.pty.no_command_for(200),
           "lease expiry reaches abort without synthesizing a UART command");
}

void real_owner_uart_loss_and_tcp_survival()
{
    RealOwnerFixture fixture;
    fixture.hello_acquire(30U, 31U);
    (void)fixture.activate();
    fixture.pty.close_master();

    const auto state = fixture.next_kind(RbrpMessageKind::ControlState);
    expect(state.request_id == 0U && state.payload.size() == 8U &&
               state.payload[0] == 0U &&
               state.payload[3] == static_cast<Byte>(GatewayStateReason::LinkLost),
           "UART/PTTY loss revokes authority without closing TCP generation");

    send_remote(fixture.client, RbrpMessageKind::ControlHeartbeat, 32U, {});
    const auto not_authority = fixture.next_kind(RbrpMessageKind::ServiceError);
    expect(not_authority.request_id == 32U && not_authority.payload.size() == 8U &&
               le16(not_authority.payload, 0U) ==
                   static_cast<std::uint16_t>(ServiceErrorCode::NotAuthority),
           "TCP remains responsive after UART loss but cannot command");
}

void real_owner_disconnect_reconnect_isolation_and_no_replay()
{
    RealOwnerFixture fixture;
    fixture.hello_acquire(40U, 41U);
    const auto heartbeat_a = fixture.activate();

    const auto pending = submit_remote_command(
        fixture, 42U, {0x03U, 0x01U, 0x2cU, 0x01U});
    const auto pending_wire = next_uart_command(fixture.pty, 0x13U);
    expect(pending.payload[0] == 0U && pending.payload[1] == 1U &&
               le16(pending.payload, 2U) == pending_wire.sequence,
           "A command remains pending after UART ownership acceptance");

    const auto queued = submit_remote_command(
        fixture, 43U, {0x02U, 0x01U, 0x00U});
    expect(queued.payload[0] == 0U && queued.payload[1] == 1U,
           "a second command is admitted to the bounded ordinary queue");
    expect(fixture.pty.no_command_for(120),
           "queued command has no UART bytes before the first ACK");

    fixture.disconnect_client();
    fixture.connect_client();
    send_remote(fixture.client, RbrpMessageKind::CommandRequest, 100U,
                {0x06U});
    const auto before_hello = fixture.next_kind(RbrpMessageKind::ServiceError);
    expect(before_hello.request_id == 100U && before_hello.payload.size() == 8U &&
               le16(before_hello.payload, 0U) ==
                   static_cast<std::uint16_t>(ServiceErrorCode::NotHello),
           "a replacement TCP source must perform Hello again");

    fixture.hello_acquire(101U, 102U);
    const auto heartbeat_b = fixture.activate();
    expect(heartbeat_b != heartbeat_a && heartbeat_b != pending_wire.sequence,
           "reconnect preserves STM32 sequence continuity without reuse");

    fixture.pty.send(protocol_ack(pending_wire.sequence, 0x13U));
    expect(!fixture.reader->try_next_kind(RbrpMessageKind::CommandOutcome, 150),
           "late A ACK cannot create an outcome for replacement source B");

    const auto replacement = submit_remote_command(
        fixture, 103U, {0x03U, 0x02U, 0x90U, 0x01U});
    const auto replacement_wire = next_uart_command(fixture.pty, 0x13U);
    expect(replacement.payload[0] == 0U && replacement.payload[1] == 1U &&
               replacement_wire.sequence == le16(replacement.payload, 2U) &&
               replacement_wire.sequence != pending_wire.sequence,
           "replacement source receives only its new command, never A replay");
    fixture.pty.send(protocol_ack(replacement_wire.sequence, 0x13U));
    const auto outcome = fixture.next_kind(RbrpMessageKind::CommandOutcome);
    expect(outcome.request_id == 103U && outcome.payload[0] == 0U,
           "replacement source completes only its own command");

    fixture.disconnect_client();
    fixture.connect_client();
    send_remote(fixture.client, RbrpMessageKind::CommandRequest, 104U,
                {0x06U});
    const auto before_hello_c = fixture.next_kind(RbrpMessageKind::ServiceError);
    expect(before_hello_c.request_id == 104U &&
               le16(before_hello_c.payload, 0U) ==
                   static_cast<std::uint16_t>(ServiceErrorCode::NotHello),
           "source C is not usable until its own Hello after B disconnects");
    fixture.hello_acquire(105U, 106U);
    const auto heartbeat_c = fixture.activate();
    expect(heartbeat_c != heartbeat_b && heartbeat_c != replacement_wire.sequence,
           "source C completes registration after B loss without replaying B state");
}

void real_owner_pty_lifecycle()
{
    ProtocolPty pty;
    GatewayOwner owner(pty.path, "127.0.0.1", 0U);
    expect(owner.start() == 0, "GatewayOwner starts the real TCP/PTY stack");

    const int client = connect_loopback(owner.bound_port());
    RbrpSocketReader reader{client};

    send_all(client, encode_frame(
                         RbrpMessageKind::Hello, 1U, Bytes{0U, 0U})
                         .wire);
    const auto hello = reader.next_kind(RbrpMessageKind::HelloReply);
    expect(hello.request_id == 1U && hello.payload.size() == 8U &&
               le16(hello.payload, 0U) == 0U &&
               le16(hello.payload, 2U) == 512U,
           "real owner returns an exact HelloReply");

    send_all(client, encode_frame(
                         RbrpMessageKind::AcquireControl, 2U, {}).wire);
    const auto acquire = reader.next_kind(RbrpMessageKind::AcquireReply);
    expect(acquire.request_id == 2U && acquire.payload.size() == 12U &&
               acquire.payload[0] == 0U && acquire.payload[1] == 2U,
           "real owner performs explicit Acquire and grants authority");
    (void)reader.next_kind(RbrpMessageKind::ControlState);

    const auto raw_zero = pty.next_wire();
    expect(raw_zero == Bytes{0U},
           "real owner preserves the 575-ms raw resynchronization boundary");
    const auto heartbeat = pty.next_frame_of_type(0x01U);
    expect(heartbeat.sequence == 0U && heartbeat.payload.size() == 4U,
           "real owner emits the first Protocol V2 heartbeat");
    pty.send(protocol_ack(heartbeat.sequence, 0x01U));
    const auto active = reader.next_kind(RbrpMessageKind::ControlState);
    expect(active.request_id == 0U && active.payload.size() == 8U &&
               active.payload[2] ==
                   static_cast<Byte>(GatewayApplicationLinkState::Active),
           "real owner waits for an explicit Active state before commands");

    send_all(client, encode_frame(
                         RbrpMessageKind::CommandRequest, 100U,
                         Bytes{0x03U, 0x00U, 0x00U, 0x00U})
                         .wire);
    const auto submitted =
        reader.next_kind(RbrpMessageKind::CommandSubmitted);
    expect(submitted.request_id == 100U && submitted.payload.size() == 4U &&
               submitted.payload[0] == 0U && submitted.payload[1] == 1U,
           "real owner separates command admission from final outcome");
    const auto command_sequence = le16(submitted.payload, 2U);

    const auto command_wire = pty.next_frame_of_type(0x13U);
    expect(command_wire.sequence == command_sequence &&
               command_wire.payload == Bytes{1U, 0U, 0U, 0U},
           "real owner maps the typed remote command to Protocol V2");
    pty.send(protocol_ack(command_sequence, 0x13U));

    const auto outcome = reader.next_kind(RbrpMessageKind::CommandOutcome);
    expect(outcome.request_id == 100U && outcome.payload.size() == 5U &&
               outcome.payload[0] == 0U && outcome.payload[1] == 3U &&
               le16(outcome.payload, 2U) == command_sequence &&
               outcome.payload[4] == 0U,
           "real owner maps the STM32 ACK to a final CommandOutcome");

    ::shutdown(client, SHUT_RDWR);
    ::close(client);
    owner.stop();
    expect(!owner.started(),
           "normal owner shutdown joins the worker and owner threads");
}

} // namespace

int main()
{
    try {
        one_source_fragmented_input_and_source_addressed_output();
        wrong_direction_frame_closes_source_as_fatal_protocol();
        second_source_is_closed_and_ids_do_not_inherit();
        queued_replacement_survives_same_poll_generation_rollover();
        stale_poll_revents_are_rejected_by_generation();
        inbound_delivery_barrier_preserves_timely_heartbeat();
        coalesced_recv_batch_shares_one_trusted_timestamp();
        newest_source_registration_replaces_pending_older_generation();
        stale_close_and_old_telemetry_never_affect_new_source();
        inbound_queue_exhaustion_is_source_loss();
        critical_bound_partial_telemetry_and_shutdown_contract();
        real_owner_pty_lifecycle();
        real_owner_rejected_ack_and_telemetry_forwarding();
        real_owner_queued_heartbeat_does_not_false_expire();
        real_owner_sustained_inbound_does_not_starve_pty();
        real_owner_nonheartbeat_traffic_does_not_keep_lease_alive();
        real_owner_wrong_direction_revokes_authority_without_uart_command();
        real_owner_lease_expiry_aborts_without_synthesized_command();
        real_owner_uart_loss_and_tcp_survival();
        real_owner_disconnect_reconnect_isolation_and_no_replay();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return EXIT_FAILURE;
    }

    if (rbp2_test::failures == 0) {
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}
