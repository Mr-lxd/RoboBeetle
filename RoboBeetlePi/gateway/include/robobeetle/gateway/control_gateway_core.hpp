#pragma once

#include "robobeetle/gateway/gateway_application_port.hpp"

#include <cstddef>
#include <functional>
#include <unordered_map>
#include <vector>

namespace robobeetle::gateway {

struct GatewayCoreCallbacks {
    std::function<bool(const GatewayOutbound &)> publish;
    std::function<bool(const CloseSourceSignal &)> close_source;
    std::function<void(const char *)> diagnostic;
};

class ControlGatewayCore final {
public:
    ControlGatewayCore(GatewayApplicationPort &application,
                       GatewayCoreCallbacks callbacks);

    void source_connected(ControlSourceId source);
    void process(const RemoteEnvelope &envelope, GatewayTimeMs owner_now_ms);
    void source_lost(const SourceLostSignal &signal,
                    GatewayTimeMs owner_now_ms);
    void consume_application_run_result(
        const GatewayApplicationRunResult &result,
        GatewayTimeMs owner_now_ms);
    void consume_application_abort_result(
        const GatewayApplicationAbortResult &result,
        GatewayTimeMs owner_now_ms);
    void check_time(GatewayTimeMs owner_now_ms);
    void network_output_failed(ControlSourceId source,
                               GatewayStateReason reason,
                               GatewayTimeMs owner_now_ms);
    void shutdown(GatewayTimeMs owner_now_ms);

    [[nodiscard]] AuthorityState authority_state() const noexcept
    {
        return authority_;
    }

    [[nodiscard]] ControlSourceId current_source_id() const noexcept
    {
        return current_source_;
    }

    [[nodiscard]] bool source_present() const noexcept
    {
        return source_present_;
    }

    [[nodiscard]] bool hello_complete() const noexcept
    {
        return hello_complete_;
    }

    [[nodiscard]] GatewayTimeMs lease_deadline_ms() const noexcept
    {
        return lease_deadline_ms_;
    }

    [[nodiscard]] std::size_t outstanding_count() const noexcept
    {
        return request_to_sequence_.size();
    }

    [[nodiscard]] std::size_t diagnostic_count() const noexcept
    {
        return diagnostic_count_;
    }

    [[nodiscard]] bool accepting_commands() const noexcept
    {
        return accepting_commands_;
    }

private:
    struct Correlation {
        ControlSourceId source{0};
        RequestId request_id{0};
        RobotCommandKind command_kind{RobotCommandKind::StopMotion};
        std::uint16_t sequence{0};
    };

    void emit(const GatewayOutbound &output, GatewayTimeMs owner_now_ms);
    void emit_error(ControlSourceId source, RequestId request_id,
                    RbrpMessageKind related_kind, ServiceErrorCode code,
                    std::uint32_t detail, GatewayTimeMs owner_now_ms);
    void emit_state(ControlSourceId source, GatewayStateReason reason,
                    GatewayTimeMs owner_now_ms);
    void diagnostic(const char *message);
    void mark_completed(RequestId request_id);
    [[nodiscard]] bool request_id_is_duplicate(RequestId request_id) const;
    [[nodiscard]] bool valid_command(const RobotCommand &command) const;
    [[nodiscard]] bool is_backward(const RobotCommand &command) const;
    [[nodiscard]] CommandSubmittedStatus
    submitted_status(GatewayApplicationSubmitStatus status) const;
    void handle_payload(const RemoteEnvelope &envelope,
                        GatewayTimeMs owner_now_ms);
    void handle_command(const RemoteEnvelope &envelope,
                        const CommandRequest &request,
                        GatewayTimeMs owner_now_ms);
    void consume_events(const std::vector<GatewayApplicationEvent> &events,
                        GatewayTimeMs owner_now_ms);
    void consume_event(const GatewayApplicationEvent &event,
                       GatewayTimeMs owner_now_ms);
    void terminalize_outstanding(GatewayCommandOutcome outcome,
                                 GatewayTimeMs owner_now_ms);
    void revoke(ControlSourceId source, GatewayStateReason reason,
                GatewayTimeMs owner_now_ms, bool close_source);
    void abort_once(GatewayTimeMs owner_now_ms);
    void close_source(ControlSourceId source, CloseSourceReason reason);

    GatewayApplicationPort &application_;
    GatewayCoreCallbacks callbacks_;
    ControlSourceId current_source_{0};
    bool source_present_{false};
    bool hello_complete_{false};
    bool accepting_commands_{true};
    AuthorityState authority_{AuthorityState::Unowned};
    GatewayTimeMs lease_deadline_ms_{0};
    bool abort_called_{false};
    bool handling_output_failure_{false};
    GatewayTimeMs last_now_ms_{0};

    std::vector<RequestId> recent_request_ids_;
    std::unordered_map<RequestId, std::uint16_t> request_to_sequence_;
    std::unordered_map<std::uint16_t, Correlation> correlations_;
    std::size_t diagnostic_count_{0};
};

} // namespace robobeetle::gateway
