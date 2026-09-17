#include "test_support.hpp"
#include "robobeetle/gateway/tcp_adapter.hpp"
#include "robobeetle/gateway/rbrp_codec.hpp"

#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <netinet/in.h>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace rbp2_test { int failures = 0; }

namespace {

using namespace robobeetle::gateway;
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

} // namespace

int main()
{
    try {
        one_source_fragmented_input_and_source_addressed_output();
        second_source_is_closed_and_ids_do_not_inherit();
        stale_close_and_old_telemetry_never_affect_new_source();
        inbound_queue_exhaustion_is_source_loss();
        critical_bound_partial_telemetry_and_shutdown_contract();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return EXIT_FAILURE;
    }

    if (rbp2_test::failures == 0) {
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}
