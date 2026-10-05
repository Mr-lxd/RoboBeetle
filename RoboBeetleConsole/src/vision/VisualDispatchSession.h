#pragma once
#include "controller/IConsoleController.h"
#include "vision/VisualDiagnosticSession.h"
#include "vision/VisualDispatchStateMachine.h"
#include <QHash>

namespace rb::vision {
// Independent operator-facing readiness conditions. Every field is evaluated on
// its own so the checklist can show all outstanding items at once.
struct AutoFollowReadiness {
    bool linkAndControl{false}; // Remote RBRP link up and control authority held.
    bool servosReady{false};    // Required servos enabled and their pose known.
    bool tracking{false};       // Diagnostic snapshot reports TRACKING.
    [[nodiscard]] int missingCount() const
    {
        return (linkAndControl ? 0 : 1) + (servosReady ? 0 : 1) + (tracking ? 0 : 1);
    }
    [[nodiscard]] bool allReady() const { return missingCount() == 0; }
};
struct VisualDispatchRecord {
    qint64 nowMs{0};
    quint64 policyId{0};
    std::optional<quint32> wireId;
    ProposedCommand command{ProposedCommand::Stop};
    QString result;
    quint8 rawResult{0xff};
    qint64 ackRttMs{-1};
};
class VisualDispatchSession final : public QObject, private VisualCommandSendPort {
    Q_OBJECT
public:
    using SnapshotProvider = std::function<std::optional<VisualDiagnosticSnapshot>()>;
    using NowMs = std::function<qint64()>;
    VisualDispatchSession(IConsoleController *controller, SnapshotProvider snapshot, NowMs now = {}, VisualDispatchConfig config = {}, QObject *parent = nullptr);
    VisualDispatchSession(IConsoleController *controller, VisualDiagnosticSession *diagnostic, VisualDispatchConfig config = {}, QObject *parent = nullptr);
    void setFeatureEnabled(bool enabled);
    ArmReason arm();
    void disarm();
    void manualInput(ManualInputKind kind);
    void timerTick();
    // Test seam: stops the 50 ms evaluation timer so a caller can drive the
    // session deterministically with timerTick(). Never changes policy state.
    void setTimerEnabled(bool enabled) { enabled ? timer_.start() : timer_.stop(); }
    [[nodiscard]] bool timerEnabled() const { return timer_.isActive(); }
    bool featureEnabled() const { return enabled_; }
    bool armed() const { return machine_.armed(); }
    ArmReason armReason() const { return reason_; }
    std::optional<ProposedCommand> currentMode() const;
    bool stopTimeoutAlert() const { return machine_.stopTimeoutAlert(); }
    std::optional<CommandTerminalResult> operatorStopResult() const { return operatorStopResult_; }
    QString poseMismatch() const { return poseMismatch_; }
    std::optional<int> confirmedTurnSign() const { return confirmedSign_; }
    // Read-only checklist for the operator surface. Unlike eligibility(), every
    // condition is reported independently and no state is mutated.
    [[nodiscard]] AutoFollowReadiness readiness() const;
signals:
    void statusChanged();
    void dispatchRecorded(const rb::vision::VisualDispatchRecord &record);
private:
    struct Association { quint64 policyId; ProposedCommand command; bool operatorStop; };
    void send(const DispatchRequest &request) override;
    void submit(quint64 policyId, ProposedCommand command, bool operatorStop);
    void terminal(quint32 id, CommandTerminalResult result, quint8 raw, qint64 rtt);
    void clearAssociations(bool includeOperator = false);
    void diagnosticChanged(const VisualDiagnosticSnapshot &snapshot);
    VisualDispatchInput input(const VisualDiagnosticSnapshot *snapshot = nullptr) const;
    IConsoleController *controller_;
    SnapshotProvider snapshot_;
    NowMs now_;
    QElapsedTimer clock_;
    QTimer timer_;
    VisualDispatchStateMachine machine_;
    quint16 requiredServoMask_;
    bool enabled_{false};
    bool submitting_{false};
    bool evaluating_{false};
    ArmReason reason_{ArmReason::NotTracking};
    std::optional<int> confirmedSign_{VisualPolicyConfig::configuredTurnSign};
    std::optional<CommandTerminalResult> operatorStopResult_;
    QString poseMismatch_;
    QHash<quint32, Association> associations_;
};
}
Q_DECLARE_METATYPE(rb::vision::VisualDispatchRecord)
