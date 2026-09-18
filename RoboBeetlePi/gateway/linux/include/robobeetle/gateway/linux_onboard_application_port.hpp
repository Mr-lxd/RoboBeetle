#pragma once

#if !defined(__linux__)
#error "LinuxOnboardApplicationPort is supported only on Linux"
#endif

#include "robobeetle/application/onboard_application.hpp"
#include "robobeetle/gateway/gateway_application_port.hpp"

#include <string>

namespace robobeetle::gateway {

namespace detail {
struct LinuxOnboardApplicationPortTestAccess;
}

class LinuxOnboardApplicationPort final : public GatewayApplicationPort {
public:
    explicit LinuxOnboardApplicationPort(
        std::string device_path,
        link_core::LinkCoreConfig config = {},
        std::size_t max_tx_frames = 64U,
        std::size_t max_tx_bytes = 65536U);

    int open() override;
    GatewayApplicationRunResult run_once() override;
    GatewayApplicationAbortResult abort() override;
    GatewayApplicationSubmitResult
    submit(const RobotCommand &command) override;

    [[nodiscard]] GatewayApplicationSessionState
    session_state() const noexcept override;
    [[nodiscard]] GatewayApplicationLinkState link_state() const override;

    [[nodiscard]] const std::string &device_path() const noexcept
    {
        return device_path_;
    }

private:
    friend struct detail::LinuxOnboardApplicationPortTestAccess;

    static GatewayApplicationSessionState
    map_session_state(session::SessionState state) noexcept;
    static GatewayApplicationLinkState
    map_link_state(link_core::LinkState state) noexcept;
    static GatewayApplicationRunStatus
    map_run_status(runtime::RuntimeStatus status) noexcept;

    GatewayApplicationSubmitResult
    map_submit_result(const application::CommandSubmitResult &result) const;
    std::vector<GatewayApplicationEvent>
    map_events(const std::vector<application::ApplicationEvent> &events) const;

    std::string device_path_;
    application::OnboardApplication application_;
};

} // namespace robobeetle::gateway
