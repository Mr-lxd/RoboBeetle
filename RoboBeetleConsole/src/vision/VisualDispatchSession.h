#pragma once
#include "controller/IConsoleController.h"
#include "vision/VisualDiagnosticSession.h"
#include "vision/VisualDispatchStateMachine.h"
#include <QHash>

namespace rb::vision {
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
    void selectTurnSign(std::optional<int> sign);
    void confirmTurnSign();
    void manualInput(ManualInputKind kind);
    void timerTick();
    bool featureEnabled() const { return enabled_; }
    bool armed() const { return machine_.armed(); }
    ArmReason armReason() const { return reason_; }
    std::optional<ProposedCommand> currentMode() const;
    bool stopTimeoutAlert() const { return machine_.stopTimeoutAlert(); }
    std::optional<CommandTerminalResult> operatorStopResult() const { return operatorStopResult_; }
    QString poseMismatch() const { return poseMismatch_; }
    std::optional<int> selectedTurnSign() const { return selectedSign_; }
    std::optional<int> confirmedTurnSign() const { return confirmedSign_; }
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
    std::optional<int> selectedSign_, confirmedSign_;
    std::optional<CommandTerminalResult> operatorStopResult_;
    QString poseMismatch_;
    QHash<quint32, Association> associations_;
};
}
Q_DECLARE_METATYPE(rb::vision::VisualDispatchRecord)
