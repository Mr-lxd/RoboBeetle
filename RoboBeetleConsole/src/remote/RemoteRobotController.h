#pragma once

#include "controller/IConsoleController.h"
#include "remote/RbrpClientSession.h"

#include <QHash>
#include <QTimer>

#include <optional>

namespace rb {

class RemoteRobotController final : public IConsoleController {
    Q_OBJECT

public:
    explicit RemoteRobotController(QObject *parent = nullptr);

    [[nodiscard]] ConsoleBackendKind backendKind() const noexcept override
    {
        return ConsoleBackendKind::RemoteRbrp;
    }

    void refreshSerialPorts() override;
    void connectController(const ConsoleConnectionConfiguration &configuration) override;
    void disconnectController() override;
    void shutdown() override;

    bool acquireControl() override;
    bool releaseControl() override;
    bool enableServo(ServoId id) override;
    bool disableServo(ServoId id) override;
    bool disableAll() override;
    bool setServoPwm(ServoId id, quint16 pulseUs) override;
    bool setServoAngle(ServoId id, qint16 angleCentidegrees) override;
    bool neutralServo(ServoId id) override;
    bool startMotion(MotionMode mode) override;
    bool stopMotion() override;
    bool setGaitBackend(GaitBackend backend) override;

    [[nodiscard]] bool isConnected() const override { return session_.isConnected(); }
    [[nodiscard]] bool canAcquireControl() const override { return session_.canAcquireControl(); }
    [[nodiscard]] bool isControlActive() const override { return session_.isControlActive(); }
    [[nodiscard]] ControlAuthorityState authorityState() const override
    {
        return session_.authorityState();
    }
    [[nodiscard]] bool supportsRawPwm() const noexcept override { return true; }
    [[nodiscard]] bool isServoSupported(ServoId id) const override;
    [[nodiscard]] bool isServoEnabled(ServoId id) const override;
    [[nodiscard]] bool isServoDisablePending(ServoId id) const override;
    [[nodiscard]] LeakState leakState() const override { return leakState_; }
    [[nodiscard]] const ImuMonitorState &imuState() const override { return imuState_; }
    [[nodiscard]] const DepthMonitorState &depthState() const override { return depthState_; }
    [[nodiscard]] ProtocolMonitor monitor() const override { return monitor_; }
    [[nodiscard]] MotionState motionState() const override { return motionState_; }
    [[nodiscard]] MotionMode motionMode() const override { return motionMode_; }
    [[nodiscard]] std::optional<GaitBackend> confirmedGaitBackend() const override
    {
        return confirmedGaitBackend_;
    }
    [[nodiscard]] std::optional<GaitBackend> requestedGaitBackend() const override
    {
        return pendingGaitBackend_;
    }
    [[nodiscard]] bool isGaitBackendChangePending() const override
    {
        return pendingGaitBackend_.has_value();
    }
    [[nodiscard]] bool isMotionActive() const override;
    [[nodiscard]] bool isMotionReady(MotionMode mode) const override;
    [[nodiscard]] bool isMotionTransitioning() const override
    {
        return motionModeTransitionTimer_.isActive();
    }

private:
    struct PendingCommand {
        robobeetle::gateway::RobotCommandKind kind{
            robobeetle::gateway::RobotCommandKind::StopMotion};
        quint16 servoMask{0};
        std::optional<MotionMode> motionMode;
        std::optional<GaitBackend> gaitBackend;
        std::optional<quint16> submittedSequence;
        bool superseded{false};
        qint64 sentAtMs{0};
    };

    std::optional<quint32>
    submitCommand(robobeetle::gateway::RobotCommandKind kind,
                  const QByteArray &payload, PendingCommand pending);
    void handleFrame(quint8 kind, quint32 requestId, const QByteArray &payload);
    void handleCommandSubmitted(quint32 requestId, const QByteArray &payload);
    void handleCommandOutcome(quint32 requestId, const QByteArray &payload);
    void handleLeakTelemetry(const QByteArray &payload);
    void handleImuTelemetry(const QByteArray &payload);
    void handleDepthTelemetry(const QByteArray &payload);
    void handleServiceError(quint32 requestId, const QByteArray &payload);

    void applyAcceptedCommand(const PendingCommand &pending);
    void terminalizePending(quint32 requestId, const QString &status);
    void supersedePendingMotionStarts();
    void supersedePendingForDisable(quint16 affectedMask);
    void failClosedControlState(const QString &reason);
    void resetTelemetry();
    void refreshTelemetryStaleness();

    void setEnabledMask(quint16 mask);
    void setDisablePendingMask(quint16 mask);
    void setLeakState(LeakState state);
    void setMotionState(MotionState state, MotionMode mode);
    void updateMonitor(const QString &status = {});
    void noteTxFrame(const QByteArray &wire);
    void noteRxFrame(quint8 kind, quint32 requestId, const QByteArray &payload);

    static QByteArray maskPayload(quint16 mask);
    static qint64 nowMs();
    static QString authorityText(ControlAuthorityState state, bool active);

    RbrpClientSession session_;
    quint16 enabledMask_{0};
    quint16 disablePendingMask_{0};
    LeakState leakState_{LeakState::Unknown};
    qint64 lastLeakTelemetryAtMs_{-1};
    ImuMonitorState imuState_;
    DepthMonitorState depthState_;
    MotionState motionState_{MotionState::Stopped};
    MotionMode motionMode_{MotionMode::Stop};
    std::optional<GaitBackend> confirmedGaitBackend_;
    std::optional<GaitBackend> pendingGaitBackend_;
    QHash<quint32, PendingCommand> pending_;
    QTimer motionStopTimer_;
    QTimer motionModeTransitionTimer_;
    QTimer telemetryTimer_;
    ProtocolMonitor monitor_;
    bool wasControlActive_{false};
    bool userReleasePending_{false};
};

} // namespace rb
