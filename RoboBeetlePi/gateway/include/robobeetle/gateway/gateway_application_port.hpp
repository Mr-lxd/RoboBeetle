#pragma once

#include "robobeetle/gateway/gateway_types.hpp"

namespace robobeetle::gateway {

class GatewayApplicationPort {
public:
    virtual ~GatewayApplicationPort() = default;

    virtual int open() = 0;
    virtual GatewayApplicationRunResult run_once() = 0;
    virtual GatewayApplicationAbortResult abort() = 0;
    virtual GatewayApplicationSubmitResult
    submit(const RobotCommand &command) = 0;

    [[nodiscard]] virtual GatewayApplicationSessionState
    session_state() const noexcept = 0;
    [[nodiscard]] virtual GatewayApplicationLinkState link_state() const = 0;
};

} // namespace robobeetle::gateway
