#pragma once
#include "controller/IConsoleController.h"
#include "vision/VisualDiagnosticSession.h"
#include "vision/VisualDispatchStateMachine.h"
#include "vision/VisualExecutionInfo.h"
#include <functional>
#include <set>

namespace rb::vision
{
// Separate proportional executor. The owner supplies monotonic evaluations and
// forwards ordinary command outcomes; discrete dispatch has no dependency here.
class VisualProportionalDispatchStateMachine
{
  public:
    VisualProportionalDispatchStateMachine(IConsoleController *controller, VisualDispatchConfig config);
    std::function<void(const VisualDispatchRecord &)> record;
    std::function<void(const VisualApproachCsvInfo &)> event;
    VisualApproachConfig config{};
    IConsoleController::ProportionalConfig q{};
    bool dryRun{false}, channelBusy{false};
    bool continuous{false};
    ArmReason eligibility(const VisualDispatchInput &, const std::optional<VisualDiagnosticSnapshot> &) const;
    ArmReason arm(const VisualDispatchInput &, const std::optional<VisualDiagnosticSnapshot> &);
    void evaluate(const VisualDispatchInput &, const std::optional<VisualDiagnosticSnapshot> &);
    void terminal(quint32, CommandTerminalResult, quint8, qint64, qint64 now);
    void disarm(qint64 now);
    void telemetry(qint64 now);
    bool armed() const { return armed_; }
    bool stopAwaiting() const { return stopAwaiting_; }
    bool stopTimeoutAlert() const { return stopTimeoutCount_ >= 3; }
    std::optional<CommandTerminalResult> operatorStopResult() const { return operatorResult_; }
    std::optional<ProposedCommand> currentMode() const
    {
        return streaming_ ? std::optional{mode_} : std::nullopt;
    }
    DepthEnvelopeState envelope() const { return envelope_; }
    VisualApproachCsvInfo info() const { return info_; }

  private:
    void beginStop(qint64 now, bool disarm, bool operatorStop = false);
    void submitStop(qint64 now);
    void start(qint64 now);
    void stream(qint64 now);
    void note(const QString &event, qint64 now, const QString &detail = {});
    void dispatch(std::optional<quint32> id, ProposedCommand command, const QString &result, qint64 now,
                  quint8 raw = 0xff, qint64 rtt = -1);
    IConsoleController *controller_;
    VisualDispatchConfig dispatchConfig_;
    VisualApproachMemory area_;
    VisualApproachCsvInfo info_;
    DepthEnvelopeMemory depth_;
    DepthEnvelopeState envelope_{DepthEnvelopeState::Unavailable};
    bool armed_{false}, streaming_{false}, stopAwaiting_{false}, stopEpisode_{false}, operatorStop_{false};
    std::optional<quint32> startId_;
    std::set<quint32> stopIds_;
    std::optional<qint64> lastStopAccepted_, lastSwitch_, noTargetSince_, lastAcquiredFrame_, lastTime_;
    qint64 startSent_{0}, stopSent_{0};
    unsigned stopTimeoutCount_{0};
    std::optional<CommandTerminalResult> operatorResult_;
    ProposedCommand mode_{ProposedCommand::Forward};
    VisualSetpoint setpoint_{};
    qint16 submittedPitch_{0};
};
} // namespace rb::vision
