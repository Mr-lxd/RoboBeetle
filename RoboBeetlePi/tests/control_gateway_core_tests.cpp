#include "test_support.hpp"
#include "robobeetle/gateway/control_gateway_core.hpp"

#include <cstdint>
#include <cstdlib>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace rbp2_test { int failures = 0; }

namespace {

using namespace robobeetle::gateway;
using rbp2_test::expect;

struct FakeGatewayApplicationPort final : GatewayApplicationPort {
    int open_result{0};
    std::size_t open_calls{0};
    std::size_t abort_calls{0};
    std::size_t run_calls{0};
    std::size_t submit_calls{0};
    GatewayApplicationSessionState session{
        GatewayApplicationSessionState::ReopenRequired};
    GatewayApplicationLinkState link{GatewayApplicationLinkState::Unconfirmed};
    GatewayApplicationSubmitResult submit_result{
        GatewayApplicationSubmitStatus::NotActive, std::nullopt};
    GatewayApplicationRunResult run_result;
    GatewayApplicationAbortResult abort_result;
    std::vector<RobotCommand> submitted_commands;

    int open() override
    {
        ++open_calls;
        if (open_result == 0) {
            session = GatewayApplicationSessionState::SafetyQuiet;
        }
        return open_result;
    }

    GatewayApplicationRunResult run_once() override
    {
        ++run_calls;
        return std::move(run_result);
    }

    GatewayApplicationAbortResult abort() override
    {
        ++abort_calls;
        session = GatewayApplicationSessionState::ReopenRequired;
        link = GatewayApplicationLinkState::Unconfirmed;
        return std::move(abort_result);
    }

    GatewayApplicationSubmitResult
    submit(const RobotCommand &command) override
    {
        ++submit_calls;
        submitted_commands.push_back(command);
        return submit_result;
    }

    GatewayApplicationSessionState session_state() const noexcept override
    {
        return session;
    }

    GatewayApplicationLinkState link_state() const override
    {
        return link;
    }
};

struct OutputSink {
    std::vector<GatewayOutbound> outputs;
    std::vector<CloseSourceSignal> closes;
    std::size_t diagnostics{0};
    bool accept_outputs{true};
    bool accept_closes{true};

    GatewayCoreCallbacks callbacks()
    {
        return GatewayCoreCallbacks{
            [this](const GatewayOutbound &output) {
                if (!accept_outputs) {
                    return false;
                }
                outputs.push_back(output);
                return true;
            },
            [this](const CloseSourceSignal &signal) {
                if (!accept_closes) {
                    return false;
                }
                closes.push_back(signal);
                return true;
            },
            [this](const char *) { ++diagnostics; },
        };
    }

    void clear_outputs()
    {
        outputs.clear();
    }
};

RemoteEnvelope envelope(ControlSourceId source, GatewayTimeMs received_at,
                        RemoteMessage message)
{
    return RemoteEnvelope{source, received_at, std::move(message)};
}

RemoteMessage hello(RequestId request_id, std::uint16_t capabilities = 0U)
{
    return RemoteMessage{RbrpMessageKind::Hello, request_id,
                          HelloRequest{capabilities}};
}

RemoteMessage acquire(RequestId request_id)
{
    return RemoteMessage{RbrpMessageKind::AcquireControl, request_id,
                          AcquireControlRequest{}};
}

RemoteMessage heartbeat(RequestId request_id, GatewayTimeMs received_at)
{
    return RemoteMessage{RbrpMessageKind::ControlHeartbeat, request_id,
                          ControlHeartbeatRequest{}};
}

RemoteMessage release(RequestId request_id)
{
    return RemoteMessage{RbrpMessageKind::ReleaseControl, request_id,
                          ReleaseControlRequest{}};
}

RemoteMessage command(RequestId request_id, RobotCommandKind kind,
                      std::optional<RobotCommand> value)
{
    return RemoteMessage{
        RbrpMessageKind::CommandRequest, request_id,
        CommandRequest{static_cast<Byte>(kind), std::move(value)}};
}

template<class T>
const T *find_output(const OutputSink &sink, ControlSourceId source,
                     RequestId request_id)
{
    for (const auto &output : sink.outputs) {
        if (output.source != source ||
            output.message.request_id != request_id) {
            continue;
        }
        if (const auto *message = std::get_if<T>(&output.message.payload)) {
            return message;
        }
    }
    return nullptr;
}

template<class T>
const T *find_any_output(const OutputSink &sink, ControlSourceId source)
{
    for (const auto &output : sink.outputs) {
        if (output.source != source) {
            continue;
        }
        if (const auto *message = std::get_if<T>(&output.message.payload)) {
            return message;
        }
    }
    return nullptr;
}

void hello_is_required_and_acquire_is_explicit()
{
    FakeGatewayApplicationPort application;
    OutputSink sink;
    ControlGatewayCore core(application, sink.callbacks());
    core.source_connected(7U);

    core.process(envelope(7U, 10U, acquire(1U)), 10U);
    const auto *not_hello =
        find_output<ServiceErrorMessage>(sink, 7U, 1U);
    expect(not_hello != nullptr &&
               not_hello->error_code == ServiceErrorCode::NotHello,
           "Acquire before Hello is a semantic ServiceError");

    sink.clear_outputs();
    core.process(envelope(7U, 20U, hello(2U)), 20U);
    expect(core.hello_complete() && find_output<HelloReply>(sink, 7U, 2U),
           "valid Hello completes the source lifecycle");

    sink.clear_outputs();
    core.process(envelope(7U, 30U, acquire(3U)), 30U);
    const auto *reply = find_output<AcquireReply>(sink, 7U, 3U);
    expect(application.open_calls == 1U && core.authority_state() ==
               AuthorityState::Owned && reply != nullptr &&
               reply->result == AcquireResult::Granted &&
               reply->authority_state == AuthorityState::Owned,
           "Acquire performs exactly one explicit open before granting authority");
}

void acquire_failure_and_invalid_state_do_not_adopt_sessions()
{
    FakeGatewayApplicationPort failed_application;
    failed_application.open_result = 13;
    OutputSink failed_sink;
    ControlGatewayCore failed_core(failed_application, failed_sink.callbacks());
    failed_core.source_connected(1U);
    failed_core.process(envelope(1U, 1U, hello(1U)), 1U);
    failed_sink.clear_outputs();
    failed_core.process(envelope(1U, 2U, acquire(2U)), 2U);
    const auto *failed =
        find_output<AcquireReply>(failed_sink, 1U, 2U);
    expect(failed_application.open_calls == 1U &&
               failed_core.authority_state() == AuthorityState::Unowned &&
               failed != nullptr && failed->result == AcquireResult::OpenFailed &&
               failed->detail == 13U,
           "open failure returns OpenFailed with errno and no retry/adoption");

    FakeGatewayApplicationPort already_open;
    already_open.session = GatewayApplicationSessionState::SafetyQuiet;
    OutputSink already_open_sink;
    ControlGatewayCore already_open_core(already_open,
                                         already_open_sink.callbacks());
    already_open_core.source_connected(2U);
    already_open_core.process(envelope(2U, 1U, hello(1U)), 1U);
    already_open_sink.clear_outputs();
    already_open_core.process(envelope(2U, 2U, acquire(2U)), 2U);
    const auto *invalid =
        find_output<AcquireReply>(already_open_sink, 2U, 2U);
    expect(already_open.open_calls == 0U && invalid != nullptr &&
               invalid->result == AcquireResult::InvalidState &&
               already_open_core.authority_state() == AuthorityState::Unowned,
           "Acquire never silently adopts a SafetyQuiet session");
}

void lease_uses_grant_time_and_trusted_receive_time()
{
    FakeGatewayApplicationPort application;
    OutputSink sink;
    ControlGatewayCore core(application, sink.callbacks());
    core.source_connected(4U);
    core.process(envelope(4U, 1U, hello(1U)), 1U);
    core.process(envelope(4U, 2U, acquire(2U)), 500U);
    expect(core.lease_deadline_ms() == 1500U,
           "initial lease starts at owner-side successful grant time");

    core.process(envelope(4U, 900U, heartbeat(3U, 900U)), 1600U);
    expect(core.authority_state() == AuthorityState::Owned &&
               core.lease_deadline_ms() == 1900U,
           "timely heartbeat refresh uses trusted receive time, not processing time");

    core.check_time(1600U);
    expect(core.authority_state() == AuthorityState::Owned,
           "timely heartbeat processed after the old deadline stays authoritative");

    core.process(envelope(4U, 1900U, heartbeat(4U, 1900U)), 1900U);
    core.check_time(1900U);
    expect(core.authority_state() == AuthorityState::Unowned &&
               application.abort_calls == 1U,
           "heartbeat at the deadline cannot revive authority");
}

void backlog_does_not_create_an_eight_message_lease_budget()
{
    FakeGatewayApplicationPort application;
    OutputSink sink;
    ControlGatewayCore core(application, sink.callbacks());
    core.source_connected(5U);
    core.process(envelope(5U, 1U, hello(1U)), 1U);
    core.process(envelope(5U, 2U, acquire(2U)), 10U);

    for (RequestId id = 3U; id < 23U; ++id) {
        core.process(envelope(
                         5U, 100U + id,
                         command(id, RobotCommandKind::StopMotion,
                                 StopMotion{})),
                     100U + id);
    }
    core.process(envelope(5U, 900U, heartbeat(23U, 900U)), 1200U);
    expect(core.authority_state() == AuthorityState::Owned &&
               core.lease_deadline_ms() == 1900U,
           "a bounded ordinary backlog does not hide a timely heartbeat");
}

void release_and_source_loss_abort_exactly_once()
{
    FakeGatewayApplicationPort application;
    application.submit_result = {
        GatewayApplicationSubmitStatus::Submitted, 813U};
    OutputSink sink;
    ControlGatewayCore core(application, sink.callbacks());
    core.source_connected(8U);
    core.process(envelope(8U, 1U, hello(1U)), 1U);
    core.process(envelope(8U, 2U, acquire(2U)), 2U);
    sink.clear_outputs();
    core.process(envelope(
                     8U, 3U, command(3U, RobotCommandKind::SetServoAngle,
                                     SetServoAngle{4U, -2})),
                 3U);
    expect(application.submit_calls == 1U &&
               core.outstanding_count() == 1U,
           "a submitted command remains outstanding for its final outcome");
    sink.clear_outputs();

    core.process(envelope(8U, 4U, release(4U)), 4U);
    const auto *unknown =
        find_output<GatewayCommandOutcomeMessage>(sink, 8U, 3U);
    expect(application.abort_calls == 1U &&
               core.authority_state() == AuthorityState::Unowned &&
               unknown != nullptr &&
               unknown->event.outcome == GatewayCommandOutcome::OutcomeUnknown,
           "Release revokes, aborts, and terminalizes pending commands");

    core.source_lost(SourceLostSignal{8U, SourceLostReason::Disconnected}, 5U);
    expect(application.abort_calls == 1U,
           "repeated source loss does not call abort twice");
}

void source_generation_isolation_prevents_stale_loss()
{
    FakeGatewayApplicationPort application;
    OutputSink sink;
    ControlGatewayCore core(application, sink.callbacks());
    core.source_connected(10U);
    core.process(envelope(10U, 1U, hello(1U)), 1U);
    core.process(envelope(10U, 2U, acquire(2U)), 2U);
    core.source_lost(SourceLostSignal{10U, SourceLostReason::Disconnected}, 3U);
    expect(core.current_source_id() == 10U &&
               core.source_present() == false,
           "source loss retires the old source without inventing a new one");

    core.source_connected(11U);
    core.process(envelope(11U, 4U, hello(1U)), 4U);
    core.process(envelope(11U, 5U, acquire(2U)), 5U);
    const auto aborts_before_stale = application.abort_calls;
    core.source_lost(SourceLostSignal{10U, SourceLostReason::Disconnected}, 6U);
    expect(core.current_source_id() == 11U &&
               core.authority_state() == AuthorityState::Owned &&
               application.abort_calls == aborts_before_stale &&
               sink.diagnostics > 0U,
           "late SourceLost for A cannot revoke current source B");
}

void request_ids_are_scoped_and_duplicate_safe()
{
    FakeGatewayApplicationPort application;
    application.submit_result = {
        GatewayApplicationSubmitStatus::Submitted, 901U};
    OutputSink sink;
    ControlGatewayCore core(application, sink.callbacks());
    core.source_connected(12U);

    core.process(envelope(12U, 1U, hello(9U)), 1U);
    sink.clear_outputs();
    core.process(envelope(12U, 2U, hello(9U)), 2U);
    const auto *already_hello =
        find_output<ServiceErrorMessage>(sink, 12U, 9U);
    expect(already_hello != nullptr &&
               already_hello->error_code == ServiceErrorCode::DuplicateRequestId,
           "recent completed request IDs cannot be reused");

    sink.clear_outputs();
    core.process(envelope(12U, 3U, acquire(0U)), 3U);
    const auto *zero =
        find_output<ServiceErrorMessage>(sink, 12U, 0U);
    expect(zero != nullptr && zero->error_code ==
               ServiceErrorCode::InvalidRequestId,
           "request ID zero is rejected without execution");

    core.process(envelope(12U, 4U, acquire(10U)), 4U);
    sink.clear_outputs();
    core.process(envelope(
                     12U, 5U, command(20U, RobotCommandKind::SetServoAngle,
                                      SetServoAngle{1U, 10})),
                 5U);
    core.process(envelope(
                     12U, 6U, command(20U, RobotCommandKind::SetServoAngle,
                                      SetServoAngle{1U, 11})),
                 6U);
    const auto *duplicate =
        find_output<ServiceErrorMessage>(sink, 12U, 20U);
    expect(application.submit_calls == 1U && duplicate != nullptr &&
               duplicate->error_code == ServiceErrorCode::DuplicateRequestId,
           "duplicate outstanding command IDs never re-execute");

    sink.clear_outputs();
    core.consume_application_run_result(
        GatewayApplicationRunResult{
            GatewayApplicationRunStatus::Progress,
            {GatewayCommandOutcomeEvent{
                GatewayCommandOutcome::Accepted, 901U, 0U}},
            0},
        7U);
    core.process(envelope(
                     12U, 8U, command(20U, RobotCommandKind::SetServoAngle,
                                      SetServoAngle{1U, 12})),
                 8U);
    const auto *recent =
        find_output<ServiceErrorMessage>(sink, 12U, 20U);
    expect(recent != nullptr &&
               recent->error_code == ServiceErrorCode::DuplicateRequestId,
           "completed command IDs remain protected by recent history");
}

void local_rejections_and_backward_never_submit()
{
    FakeGatewayApplicationPort application;
    application.submit_result = {
        GatewayApplicationSubmitStatus::Submitted, 100U};
    OutputSink sink;
    ControlGatewayCore core(application, sink.callbacks());
    core.source_connected(13U);
    core.process(envelope(13U, 1U, hello(1U)), 1U);
    core.process(envelope(13U, 2U, acquire(2U)), 2U);
    sink.clear_outputs();

    core.process(envelope(
                     13U, 3U, command(3U, RobotCommandKind::EnableServos,
                                      EnableServos{0U})),
                 3U);
    const auto *invalid =
        find_output<CommandSubmittedMessage>(sink, 13U, 3U);
    expect(application.submit_calls == 0U && invalid != nullptr &&
               invalid->status == CommandSubmittedStatus::InvalidArgument &&
               !invalid->sequence.has_value(),
           "invalid typed parameters are locally rejected without submit");

    sink.clear_outputs();
    core.process(envelope(
                     13U, 4U, command(4U, RobotCommandKind::StartMotion,
                                      StartMotion{MotionMode::Backward})),
                 4U);
    const auto *backward =
        find_output<CommandSubmittedMessage>(sink, 13U, 4U);
    expect(application.submit_calls == 0U && backward != nullptr &&
               backward->status == CommandSubmittedStatus::PendingQualification &&
               !backward->sequence.has_value(),
           "Backward remains PendingQualification with zero application submit");

    application.submit_result = {
        GatewayApplicationSubmitStatus::NotActive, std::nullopt};
    sink.clear_outputs();
    core.process(envelope(
                     13U, 5U, command(5U, RobotCommandKind::StopMotion,
                                      StopMotion{})),
                 5U);
    const auto *not_active =
        find_output<CommandSubmittedMessage>(sink, 13U, 5U);
    expect(application.submit_calls == 1U && not_active != nullptr &&
               not_active->status == CommandSubmittedStatus::NotActive &&
               !not_active->sequence.has_value(),
           "commands before Link Active retain typed NotActive semantics");
}

void final_outcomes_correlate_only_by_live_sequence()
{
    FakeGatewayApplicationPort application;
    application.submit_result = {
        GatewayApplicationSubmitStatus::Submitted, 813U};
    OutputSink sink;
    ControlGatewayCore core(application, sink.callbacks());
    core.source_connected(14U);
    core.process(envelope(14U, 1U, hello(1U)), 1U);
    core.process(envelope(14U, 2U, acquire(2U)), 2U);
    sink.clear_outputs();
    core.process(envelope(
                     14U, 3U, command(3U, RobotCommandKind::SetServoAngle,
                                      SetServoAngle{4U, 200})),
                 3U);
    sink.clear_outputs();

    core.consume_application_run_result(
        GatewayApplicationRunResult{
            GatewayApplicationRunStatus::Progress,
            {GatewayCommandOutcomeEvent{
                GatewayCommandOutcome::Accepted, 813U, 0U}},
            0},
        4U);
    const auto *accepted =
        find_output<GatewayCommandOutcomeMessage>(sink, 14U, 3U);
    expect(accepted != nullptr &&
               accepted->event.outcome == GatewayCommandOutcome::Accepted &&
               accepted->command_kind == RobotCommandKind::SetServoAngle &&
               core.outstanding_count() == 0U,
           "Accepted outcome uses live sequence correlation and original request ID");

    sink.clear_outputs();
    core.consume_application_run_result(
        GatewayApplicationRunResult{
            GatewayApplicationRunStatus::Progress,
            {GatewayCommandOutcomeEvent{
                GatewayCommandOutcome::Rejected, 813U, 7U}},
            0},
        5U);
    expect(sink.outputs.empty() && core.diagnostic_count() > 0U,
           "stale sequence outcome is diagnostic only");

    application.submit_result = {
        GatewayApplicationSubmitStatus::Submitted, 814U};
    core.process(envelope(
                     14U, 6U, command(6U, RobotCommandKind::StopMotion,
                                      StopMotion{})),
                 6U);
    core.consume_application_run_result(
        GatewayApplicationRunResult{
            GatewayApplicationRunStatus::Progress,
            {GatewayCommandOutcomeEvent{
                GatewayCommandOutcome::Rejected, 814U, 3U}},
            0},
        7U);
    const auto *rejected =
        find_output<GatewayCommandOutcomeMessage>(sink, 14U, 6U);
    expect(rejected != nullptr &&
               rejected->event.result == 3U &&
               rejected->event.outcome == GatewayCommandOutcome::Rejected,
           "Rejected outcome forwards the application result byte");

    application.submit_result = {
        GatewayApplicationSubmitStatus::Submitted, 815U};
    core.process(envelope(
                     14U, 8U, command(8U, RobotCommandKind::StopMotion,
                                      StopMotion{})),
                 8U);
    core.consume_application_run_result(
        GatewayApplicationRunResult{
            GatewayApplicationRunStatus::Progress,
            {GatewayCommandOutcomeEvent{
                GatewayCommandOutcome::OutcomeUnknown, 815U, 0U}},
            0},
        9U);
    const auto *unknown =
        find_output<GatewayCommandOutcomeMessage>(sink, 14U, 8U);
    expect(unknown != nullptr &&
               unknown->event.outcome == GatewayCommandOutcome::OutcomeUnknown,
           "OutcomeUnknown remains an explicit non-success outcome");

    application.submit_result = {
        GatewayApplicationSubmitStatus::Submitted, 816U};
    core.process(envelope(
                     14U, 10U, command(10U, RobotCommandKind::StopMotion,
                                       StopMotion{})),
                 10U);
    core.consume_application_run_result(
        GatewayApplicationRunResult{
            GatewayApplicationRunStatus::Progress,
            {GatewayCommandOutcomeEvent{
                GatewayCommandOutcome::Cancelled, 816U, 0U}},
            0},
        11U);
    const auto *cancelled =
        find_output<GatewayCommandOutcomeMessage>(sink, 14U, 10U);
    expect(cancelled != nullptr &&
               cancelled->event.outcome == GatewayCommandOutcome::Cancelled,
           "Cancelled remains an explicit final outcome");
}

void session_link_loss_and_network_failure_use_abort_path()
{
    FakeGatewayApplicationPort lost_application;
    OutputSink lost_sink;
    ControlGatewayCore lost_core(lost_application, lost_sink.callbacks());
    lost_core.source_connected(15U);
    lost_core.process(envelope(15U, 1U, hello(1U)), 1U);
    lost_core.process(envelope(15U, 2U, acquire(2U)), 2U);
    lost_application.run_result.status = GatewayApplicationRunStatus::SessionLost;
    lost_core.consume_application_run_result(lost_application.run_once(), 3U);
    expect(lost_application.abort_calls == 1U &&
               lost_core.authority_state() == AuthorityState::Unowned,
           "SessionLost revokes authority through exactly one abort");

    FakeGatewayApplicationPort network_application;
    OutputSink network_sink;
    ControlGatewayCore network_core(network_application,
                                    network_sink.callbacks());
    network_core.source_connected(16U);
    network_core.process(envelope(16U, 1U, hello(1U)), 1U);
    network_core.process(envelope(16U, 2U, acquire(2U)), 2U);
    network_core.network_output_failed(
        16U, GatewayStateReason::CriticalTxFailure, 3U);
    expect(network_application.abort_calls == 1U &&
               network_core.authority_state() == AuthorityState::Unowned &&
               network_sink.closes.size() == 1U &&
               network_sink.closes[0].source == 16U,
           "critical output failure revokes, aborts, and closes its source");
}

void shutdown_is_graceful_and_does_not_synthesize_actuator_commands()
{
    FakeGatewayApplicationPort application;
    OutputSink sink;
    ControlGatewayCore core(application, sink.callbacks());
    core.source_connected(17U);
    core.process(envelope(17U, 1U, hello(1U)), 1U);
    core.process(envelope(17U, 2U, acquire(2U)), 2U);
    sink.clear_outputs();

    core.shutdown(3U);
    expect(application.abort_calls == 1U &&
               core.authority_state() == AuthorityState::Unowned &&
               !core.accepting_commands() &&
               sink.closes.size() == 1U &&
               sink.closes[0].source == 17U,
           "shutdown aborts and closes the source without a guessed stop command");

    for (const auto &output : sink.outputs) {
        expect(!std::holds_alternative<CommandSubmittedMessage>(
                   output.message.payload),
               "shutdown does not synthesize CommandSubmitted actuator traffic");
    }
}

} // namespace

int main()
{
    hello_is_required_and_acquire_is_explicit();
    acquire_failure_and_invalid_state_do_not_adopt_sessions();
    lease_uses_grant_time_and_trusted_receive_time();
    backlog_does_not_create_an_eight_message_lease_budget();
    release_and_source_loss_abort_exactly_once();
    source_generation_isolation_prevents_stale_loss();
    request_ids_are_scoped_and_duplicate_safe();
    local_rejections_and_backward_never_submit();
    final_outcomes_correlate_only_by_live_sequence();
    session_link_loss_and_network_failure_use_abort_path();
    shutdown_is_graceful_and_does_not_synthesize_actuator_commands();

    if (rbp2_test::failures == 0) {
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}
