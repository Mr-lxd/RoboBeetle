#pragma once

#if !defined(__linux__)
#error "TcpAdapter is supported only on Linux"
#endif

#include "robobeetle/gateway/gateway_types.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace robobeetle::gateway {

struct TcpAdapterCallbacks {
    std::function<void(ControlSourceId)> source_connected;
    std::function<bool(const RemoteEnvelope &)> enqueue_inbound;
    std::function<void(const SourceLostSignal &)> source_lost;
    std::function<void(const char *)> diagnostic;
};

class TcpAdapter final {
public:
    static constexpr std::size_t kMaxInboundMessages = 32U;
    static constexpr std::size_t kMaxInboundPayloadBytes = 16U * 1024U;
    static constexpr std::size_t kMaxCriticalFrames = 32U;
    static constexpr std::size_t kMaxCriticalBytes = 32U * 1024U;
    static constexpr std::size_t kTelemetrySlotCount = 3U;

    TcpAdapter(std::string bind_address, std::uint16_t port,
               TcpAdapterCallbacks callbacks);
    ~TcpAdapter();

    TcpAdapter(const TcpAdapter &) = delete;
    TcpAdapter &operator=(const TcpAdapter &) = delete;
    TcpAdapter(TcpAdapter &&) = delete;
    TcpAdapter &operator=(TcpAdapter &&) = delete;

    int start();
    void stop() noexcept;

    // Returns false only when the critical owner-to-worker channel cannot
    // accept the item. Stale source-addressed output is a successful drop.
    bool publish(const GatewayOutbound &output);
    bool close_source(const CloseSourceSignal &signal);

    [[nodiscard]] std::uint16_t bound_port() const noexcept;
    [[nodiscard]] ControlSourceId current_source_id() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace robobeetle::gateway
