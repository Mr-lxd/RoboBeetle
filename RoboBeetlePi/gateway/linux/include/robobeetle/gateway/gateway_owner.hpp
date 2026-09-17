#pragma once

#if !defined(__linux__)
#error "GatewayOwner is supported only on Linux"
#endif

#include "robobeetle/gateway/control_gateway_core.hpp"
#include "robobeetle/gateway/linux_onboard_application_port.hpp"
#include "robobeetle/gateway/tcp_adapter.hpp"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace robobeetle::gateway {

namespace detail {
struct GatewayOwnerTestAccess;
}

class GatewayOwner final {
public:
    GatewayOwner(std::string device_path, std::string bind_address,
                 std::uint16_t port,
                 link_core::LinkCoreConfig config = {},
                 std::size_t max_tx_frames = 64U,
                 std::size_t max_tx_bytes = 65536U);
    ~GatewayOwner();

    GatewayOwner(const GatewayOwner &) = delete;
    GatewayOwner &operator=(const GatewayOwner &) = delete;
    GatewayOwner(GatewayOwner &&) = delete;
    GatewayOwner &operator=(GatewayOwner &&) = delete;

    int start();
    void stop() noexcept;

    [[nodiscard]] std::uint16_t bound_port() const noexcept;
    [[nodiscard]] bool started() const noexcept;

private:
    friend struct detail::GatewayOwnerTestAccess;

    static GatewayTimeMs monotonic_now() noexcept;
    static std::size_t remote_payload_size(const RemoteMessage &message) noexcept;

    void owner_loop();
    void iteration();
    void drain_bridge_until_quiet_and_check_time(
        GatewayTimeMs owner_now_ms);
    void notify_owner() noexcept;
    void on_source_connected(ControlSourceId source) noexcept;
    bool enqueue_inbound(const RemoteEnvelope &envelope) noexcept;
    void on_source_lost(const SourceLostSignal &signal) noexcept;
    void on_diagnostic(const char *) noexcept;

    mutable std::mutex bridge_mutex_;
    std::condition_variable bridge_changed_;
    std::deque<ControlSourceId> pending_connections_;
    std::deque<SourceLostSignal> pending_source_losses_;
    std::deque<RemoteEnvelope> pending_inbound_;
    std::size_t pending_inbound_bytes_{0};
    bool stop_requested_{false};
    bool started_{false};
    bool shutdown_consumed_{false};

    LinuxOnboardApplicationPort application_;
    TcpAdapter tcp_;
    ControlGatewayCore core_;
    std::thread owner_thread_;
};

} // namespace robobeetle::gateway
