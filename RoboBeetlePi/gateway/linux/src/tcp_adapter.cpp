#include "robobeetle/gateway/tcp_adapter.hpp"

#include "robobeetle/gateway/rbrp_codec.hpp"

#include <algorithm>
#include <atomic>
#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <netinet/in.h>
#include <mutex>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <thread>
#include <type_traits>
#include <unistd.h>
#include <utility>

namespace robobeetle::gateway {
namespace {

constexpr std::size_t kCloseSignalLimit = 8U;
constexpr std::size_t kReadBufferSize = 4096U;

bool set_nonblocking(int fd) noexcept
{
    const int flags = ::fcntl(fd, F_GETFL, 0);
    return flags >= 0 && ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

GatewayTimeMs monotonic_now() noexcept
{
    using Clock = std::chrono::steady_clock;
    static_assert(Clock::is_steady, "gateway requires a monotonic clock");
    return static_cast<GatewayTimeMs>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now().time_since_epoch())
            .count());
}

bool is_telemetry(const GatewayMessagePayload &payload) noexcept
{
    return std::holds_alternative<GatewayLeakTelemetry>(payload) ||
           std::holds_alternative<GatewayImuTelemetry>(payload) ||
           std::holds_alternative<GatewayDepthTelemetry>(payload);
}

std::size_t telemetry_slot(const GatewayMessagePayload &payload) noexcept
{
    if (std::holds_alternative<GatewayLeakTelemetry>(payload)) {
        return 0U;
    }
    if (std::holds_alternative<GatewayImuTelemetry>(payload)) {
        return 1U;
    }
    return 2U;
}

} // namespace

namespace detail {
namespace {

struct InboundDeliveryBatchScope {
    std::atomic<std::size_t> &counter;

    explicit InboundDeliveryBatchScope(std::atomic<std::size_t> &value)
        : counter(value)
    {
        counter.fetch_add(1U);
    }

    ~InboundDeliveryBatchScope() { counter.fetch_sub(1U); }

    InboundDeliveryBatchScope(const InboundDeliveryBatchScope &) = delete;
    InboundDeliveryBatchScope &operator=(const InboundDeliveryBatchScope &) =
        delete;
};

} // namespace

bool process_tcp_inbound_batch(
    RbrpDecoder &decoder, std::atomic<std::size_t> &inbound_in_flight,
    ControlSourceId source, const Byte *data, std::size_t size,
    const std::function<GatewayTimeMs()> &now_ms,
    const TcpInboundBatchEnqueue &enqueue,
    const TcpInboundBatchDiagnostic &diagnostic,
    const TcpInboundBatchClose &close_current)
{
    InboundDeliveryBatchScope delivery_scope{inbound_in_flight};
    std::vector<RbrpFrame> frames;
    if (decoder.feed(data, size, frames) == RbrpFeedStatus::Fatal) {
        if (close_current) {
            close_current(SourceLostReason::FatalProtocol);
        }
        return false;
    }

    const auto received_at_ms = now_ms ? now_ms() : 0U;
    for (auto &frame : frames) {
        const auto decoded = decode_remote_message(frame);
        if (decoded.status == RbrpMessageDecodeStatus::WrongDirection) {
            if (close_current) {
                close_current(SourceLostReason::FatalProtocol);
            }
            return false;
        }
        if (!decoded.message) {
            if (diagnostic) {
                diagnostic("well-framed RBRP message rejected by semantic decoder");
            }
            continue;
        }
        const RemoteEnvelope envelope{source, received_at_ms, *decoded.message};
        if (!enqueue || !enqueue(envelope)) {
            if (close_current) {
                close_current(SourceLostReason::InboundQueueExhausted);
            }
            return false;
        }
    }
    return true;
}

} // namespace detail

struct TcpAdapter::Impl {
    struct PendingFrame {
        ControlSourceId source{0};
        Bytes wire;
        std::size_t offset{0};
        bool telemetry{false};
        std::size_t telemetry_slot{0};
    };

    std::string bind_address;
    std::uint16_t requested_port{0};
    TcpAdapterCallbacks callbacks;

    mutable std::mutex mutex;
    std::thread worker;
    int listener_fd{-1};
    int event_fd{-1};
    int client_fd{-1};
    std::uint16_t actual_port{0};
    ControlSourceId current_source{0};
    ControlSourceId next_source{0};
    bool started{false};
    bool stopping{false};

    std::deque<PendingFrame> critical;
    std::size_t critical_bytes{0};
    TcpAdapterStats stats{};
    std::array<std::optional<PendingFrame>,
               TcpAdapter::kTelemetrySlotCount>
        telemetry;
    std::deque<CloseSourceSignal> close_signals;

    RbrpDecoder decoder;
    std::optional<PendingFrame> in_flight;
    std::atomic<std::size_t> inbound_deliveries_in_flight{0U};

    Impl(std::string address, std::uint16_t port,
         TcpAdapterCallbacks configured_callbacks)
        : bind_address(std::move(address)), requested_port(port),
          callbacks(std::move(configured_callbacks))
    {
    }

    GatewayTimeMs now_ms() const noexcept
    {
        if (callbacks.now_ms) {
            return callbacks.now_ms();
        }
        return monotonic_now();
    }

    void report(const char *message) noexcept
    {
        try {
            if (callbacks.diagnostic) {
                callbacks.diagnostic(message);
            }
        } catch (...) {
        }
    }

    void notify_worker() noexcept
    {
        const std::uint64_t value = 1U;
        for (;;) {
            const auto written =
                ::write(event_fd, &value, sizeof(value));
            if (written == static_cast<ssize_t>(sizeof(value)) ||
                (written < 0 && errno == EAGAIN)) {
                return;
            }
            if (written < 0 && errno == EINTR) {
                continue;
            }
            return;
        }
    }

    ControlSourceId allocate_source()
    {
        ++next_source;
        if (next_source == 0U) {
            ++next_source;
        }
        return next_source;
    }

    void clear_source_state_locked()
    {
        critical.clear();
        critical_bytes = 0U;
        for (auto &slot : telemetry) {
            slot.reset();
        }
        in_flight.reset();
        decoder.reset();
    }

    void close_fd(int &fd) noexcept
    {
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }

    int open_listener()
    {
        const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
        if (fd < 0) {
            return errno;
        }
        int reuse = 1;
        (void)::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse,
                           sizeof(reuse));
        if (!set_nonblocking(fd)) {
            const int error = errno;
            ::close(fd);
            return error == 0 ? EIO : error;
        }

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(requested_port);
        if (::inet_pton(AF_INET, bind_address.c_str(), &address.sin_addr) !=
            1) {
            ::close(fd);
            return EINVAL;
        }
        if (::bind(fd, reinterpret_cast<const sockaddr *>(&address),
                   sizeof(address)) != 0 ||
            ::listen(fd, 1) != 0) {
            const int error = errno;
            ::close(fd);
            return error == 0 ? EIO : error;
        }

        sockaddr_in bound{};
        socklen_t length = sizeof(bound);
        if (::getsockname(fd, reinterpret_cast<sockaddr *>(&bound), &length) !=
            0) {
            const int error = errno;
            ::close(fd);
            return error == 0 ? EIO : error;
        }
        listener_fd = fd;
        actual_port = ntohs(bound.sin_port);
        return 0;
    }

    void invoke_source_connected(ControlSourceId source) noexcept
    {
        try {
            if (callbacks.source_connected) {
                callbacks.source_connected(source);
            }
        } catch (...) {
            report("source-connected callback threw");
        }
    }

    void invoke_source_lost(const SourceLostSignal &signal) noexcept
    {
        try {
            if (callbacks.source_lost) {
                callbacks.source_lost(signal);
            }
        } catch (...) {
            report("source-lost callback threw");
        }
    }

    bool invoke_inbound(const RemoteEnvelope &envelope) noexcept
    {
        try {
            return callbacks.enqueue_inbound &&
                   callbacks.enqueue_inbound(envelope);
        } catch (...) {
            report("inbound callback threw");
            return false;
        }
    }

    void close_current(bool notify_owner, SourceLostReason reason)
    {
        int fd = -1;
        ControlSourceId source = 0U;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (client_fd < 0) {
                return;
            }
            fd = client_fd;
            client_fd = -1;
            source = current_source;
            current_source = 0U;
            clear_source_state_locked();
        }
        ::shutdown(fd, SHUT_RDWR);
        ::close(fd);
        if (notify_owner && source != 0U) {
            invoke_source_lost(SourceLostSignal{source, reason});
        }
    }

    void accept_connections()
    {
        for (;;) {
            sockaddr_in peer{};
            socklen_t length = sizeof(peer);
            const int fd = ::accept4(
                listener_fd, reinterpret_cast<sockaddr *>(&peer), &length,
                SOCK_NONBLOCK | SOCK_CLOEXEC);
            if (fd < 0) {
                if (errno == EINTR) {
                    continue;
                }
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    return;
                }
                report("TCP accept failed");
                return;
            }

            bool accepted = false;
            ControlSourceId source = 0U;
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (client_fd < 0 && !stopping) {
                    client_fd = fd;
                    clear_source_state_locked();
                    source = allocate_source();
                    current_source = source;
                    accepted = true;
                }
            }
            if (accepted) {
                invoke_source_connected(source);
            } else {
                ::shutdown(fd, SHUT_RDWR);
                ::close(fd);
            }
        }
    }

    void read_client()
    {
        int fd = -1;
        ControlSourceId source = 0U;
        {
            std::lock_guard<std::mutex> lock(mutex);
            fd = client_fd;
            source = current_source;
        }
        if (fd < 0 || source == 0U) {
            return;
        }

        std::array<Byte, kReadBufferSize> buffer{};
        for (;;) {
            const auto received =
                ::recv(fd, buffer.data(), buffer.size(), MSG_DONTWAIT);
            if (received > 0) {
                const auto source_remains = detail::process_tcp_inbound_batch(
                    decoder, inbound_deliveries_in_flight, source,
                    buffer.data(), static_cast<std::size_t>(received),
                    [this] { return now_ms(); },
                    [this](const RemoteEnvelope &envelope) {
                        return invoke_inbound(envelope);
                    },
                    [this](const char *message) { report(message); },
                    [this](SourceLostReason reason) {
                        close_current(true, reason);
                    });
                if (!source_remains) {
                    return;
                }
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    if (client_fd != fd || current_source != source) {
                        return;
                    }
                }
                continue;
            }
            if (received == 0) {
                close_current(true, SourceLostReason::Disconnected);
                return;
            }
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return;
            }
            close_current(true, SourceLostReason::Disconnected);
            return;
        }
    }

    void choose_next_frame()
    {
        if (in_flight.has_value()) {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex);
        if (client_fd < 0 || current_source == 0U) {
            return;
        }
        if (!critical.empty()) {
            in_flight = std::move(critical.front());
            critical.pop_front();
            critical_bytes -= in_flight->wire.size();
            return;
        }
        for (auto &slot : telemetry) {
            if (slot.has_value()) {
                in_flight = std::move(*slot);
                slot.reset();
                return;
            }
        }
    }

    void write_client()
    {
        choose_next_frame();
        if (!in_flight.has_value()) {
            return;
        }

        int fd = -1;
        {
            std::lock_guard<std::mutex> lock(mutex);
            fd = client_fd;
        }
        if (fd < 0) {
            in_flight.reset();
            return;
        }

        for (;;) {
            const auto remaining = in_flight->wire.size() -
                                   in_flight->offset;
            const auto sent = ::send(
                fd, in_flight->wire.data() + in_flight->offset, remaining,
                MSG_NOSIGNAL | MSG_DONTWAIT);
            if (sent > 0) {
                in_flight->offset += static_cast<std::size_t>(sent);
                if (in_flight->offset == in_flight->wire.size()) {
                    in_flight.reset();
                }
                return;
            }
            if (sent < 0 && errno == EINTR) {
                continue;
            }
            if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                return;
            }
            in_flight.reset();
            close_current(true, SourceLostReason::CriticalTxFailure);
            return;
        }
    }

    void drain_eventfd()
    {
        std::uint64_t value = 0U;
        for (;;) {
            const auto received = ::read(event_fd, &value, sizeof(value));
            if (received == static_cast<ssize_t>(sizeof(value))) {
                continue;
            }
            if (received < 0 && errno == EINTR) {
                continue;
            }
            return;
        }
    }

    void process_close_signals()
    {
        for (;;) {
            CloseSourceSignal signal;
            bool stale = false;
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (close_signals.empty()) {
                    return;
                }
                signal = close_signals.front();
                close_signals.pop_front();
                stale = signal.source == 0U ||
                        signal.source != current_source;
            }
            if (stale) {
                report("stale CloseSource ignored");
                continue;
            }
            close_current(false, SourceLostReason::Disconnected);
        }
    }

    void run()
    {
        for (;;) {
            pollfd descriptors[3] = {};
            int polled_client_fd = -1;
            ControlSourceId polled_source_id = 0U;
            {
                std::lock_guard<std::mutex> lock(mutex);
                descriptors[0] = pollfd{listener_fd, POLLIN, 0};
                descriptors[1] = pollfd{client_fd, POLLIN, 0};
                polled_client_fd = client_fd;
                polled_source_id = current_source;
                if (client_fd >= 0 &&
                    (in_flight.has_value() || !critical.empty() ||
                     std::any_of(telemetry.begin(), telemetry.end(),
                                 [](const auto &slot) {
                                     return slot.has_value();
                                 }))) {
                    descriptors[1].events |= POLLOUT;
                }
                descriptors[2] = pollfd{event_fd, POLLIN, 0};
            }

            const int ready = ::poll(descriptors, 3, -1);
            if (ready < 0) {
                if (errno == EINTR) {
                    continue;
                }
                report("TCP poll failed");
                close_current(true, SourceLostReason::CriticalTxFailure);
                break;
            }

            if ((descriptors[2].revents & POLLIN) != 0) {
                drain_eventfd();
                process_close_signals();
            }
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (stopping) {
                    break;
                }
            }

            bool client_snapshot_is_current = false;
            const detail::TcpClientPollSnapshot client_snapshot{
                polled_client_fd, polled_source_id, descriptors[1].revents};
            {
                std::lock_guard<std::mutex> lock(mutex);
                client_snapshot_is_current =
                    detail::tcp_client_poll_snapshot_is_current(
                        client_snapshot, client_fd, current_source);
            }
            if (client_snapshot_is_current) {
                const auto client_revents = client_snapshot.revents;
                if ((client_revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                    close_current(true, SourceLostReason::Disconnected);
                } else {
                    if ((client_revents & POLLOUT) != 0) {
                        write_client();
                    }
                    if ((client_revents & POLLIN) != 0) {
                        read_client();
                    }
                }
            }
            if ((descriptors[0].revents & POLLIN) != 0) {
                accept_connections();
            }
        }

        close_current(false, SourceLostReason::Shutdown);
        close_fd(listener_fd);
        close_fd(event_fd);
        std::lock_guard<std::mutex> lock(mutex);
        started = false;
        stopping = false;
        actual_port = 0U;
    }
};

TcpAdapter::TcpAdapter(std::string bind_address, std::uint16_t port,
                       TcpAdapterCallbacks callbacks)
    : impl_(std::make_unique<Impl>(std::move(bind_address), port,
                                   std::move(callbacks)))
{
}

TcpAdapter::~TcpAdapter()
{
    stop();
}

int TcpAdapter::start()
{
    auto &impl = *impl_;
    {
        std::lock_guard<std::mutex> lock(impl.mutex);
        if (impl.started) {
            return EALREADY;
        }
        impl.stats = {};
    }

    const int listener_error = impl.open_listener();
    if (listener_error != 0) {
        return listener_error;
    }
    impl.event_fd = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (impl.event_fd < 0) {
        const int error = errno;
        impl.close_fd(impl.listener_fd);
        impl.actual_port = 0U;
        return error == 0 ? EIO : error;
    }

    {
        std::lock_guard<std::mutex> lock(impl.mutex);
        impl.started = true;
        impl.stopping = false;
    }
    try {
        impl.worker = std::thread([&impl] { impl.run(); });
    } catch (...) {
        {
            std::lock_guard<std::mutex> lock(impl.mutex);
            impl.stopping = true;
        }
        impl.notify_worker();
        impl.close_fd(impl.event_fd);
        impl.close_fd(impl.listener_fd);
        impl.actual_port = 0U;
        std::lock_guard<std::mutex> lock(impl.mutex);
        impl.started = false;
        throw;
    }
    return 0;
}

void TcpAdapter::stop() noexcept
{
    auto &impl = *impl_;
    {
        std::lock_guard<std::mutex> lock(impl.mutex);
        if (!impl.started) {
            if (impl.worker.joinable()) {
                impl.worker.join();
            }
            return;
        }
        impl.stopping = true;
    }
    impl.notify_worker();
    if (impl.worker.joinable()) {
        impl.worker.join();
    }
}

bool TcpAdapter::publish(const GatewayOutbound &output)
{
    auto &impl = *impl_;
    const auto encoded = encode_gateway_message(output.message);
    if (encoded.status != RbrpEncodeStatus::Ok) {
        return false;
    }

    bool stale = false;
    {
        std::lock_guard<std::mutex> lock(impl.mutex);
        if (!impl.started || impl.stopping) {
            return false;
        }
        if (output.source == 0U || output.source != impl.current_source) {
            stale = true;
        } else {
            Impl::PendingFrame pending{
                output.source, encoded.wire, 0U,
                is_telemetry(output.message.payload), 0U};
            if (pending.telemetry) {
                pending.telemetry_slot =
                    telemetry_slot(output.message.payload);
                if (impl.telemetry[pending.telemetry_slot].has_value()) {
                    ++impl.stats.telemetry_replacements;
                }
                impl.telemetry[pending.telemetry_slot] = std::move(pending);
            } else {
                if (impl.critical.size() >= TcpAdapter::kMaxCriticalFrames ||
                    impl.critical_bytes + encoded.wire.size() >
                        TcpAdapter::kMaxCriticalBytes) {
                    ++impl.stats.critical_overflow_count;
                    return false;
                }
                impl.critical_bytes += encoded.wire.size();
                impl.critical.push_back(std::move(pending));
                impl.stats.critical_frames_high_water = std::max(
                    impl.stats.critical_frames_high_water,
                    impl.critical.size());
                impl.stats.critical_bytes_high_water = std::max(
                    impl.stats.critical_bytes_high_water,
                    impl.critical_bytes);
            }
        }
    }
    if (stale) {
        impl.report("stale GatewayOutbound dropped");
        return true;
    }
    impl.notify_worker();
    return true;
}

bool TcpAdapter::close_source(const CloseSourceSignal &signal)
{
    auto &impl = *impl_;
    {
        std::lock_guard<std::mutex> lock(impl.mutex);
        if (!impl.started || impl.stopping ||
            impl.close_signals.size() >= kCloseSignalLimit) {
            return false;
        }
        impl.close_signals.push_back(signal);
    }
    impl.notify_worker();
    return true;
}

std::uint16_t TcpAdapter::bound_port() const noexcept
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->actual_port;
}

ControlSourceId TcpAdapter::current_source_id() const noexcept
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->current_source;
}

bool TcpAdapter::inbound_delivery_in_flight() const noexcept
{
    return impl_->inbound_deliveries_in_flight.load() != 0U;
}

TcpAdapterStats TcpAdapter::stats() const noexcept
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->stats;
}

} // namespace robobeetle::gateway
