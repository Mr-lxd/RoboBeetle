#pragma once

#include "controller/IConsoleController.h"
#include "protocol/StreamDecoder.h"
#include "robot/DepthMonitor.h"
#include "robot/LeakStatus.h"
#include "robot/ImuMonitor.h"
#include "robot/RobotCommand.h"
#include "robot/ServoDescriptor.h"
#include "transport/ITransport.h"

#include <QHash>
#include <QObject>
#include <QQueue>
#include <QStringList>
#include <QTimer>

#include <functional>
#include <optional>

namespace rb {

enum class LinkProfile {
    DirectUart,
    Apc220HalfDuplex,
};

inline constexpr qsizetype kApc220CommandQueueCapacity = 8;
inline constexpr int kLeakTelemetryRefreshIntervalMs = 500;
inline constexpr int kLeakTelemetryStaleOpportunities = 3;
inline constexpr int kDefaultLeakTelemetryStaleTimeoutMs =
    kLeakTelemetryRefreshIntervalMs * kLeakTelemetryStaleOpportunities;

struct RobotControllerConfig {
    int heartbeatIntervalMs{100};
    int ackTimeoutMs{200};
    int maxRetries{3};
    // Conservative host-link hard liveness budget measured from heartbeat
    // dispatch. The legacy-named profile leaves an explicit margin below the
    // Firmware watchdog; DirectUart ignores this field.
    int heartbeatSafetyBudgetMs{490};
    // Three Firmware telemetry refresh opportunities (3 x 500 ms) are
    // required before a connected Console treats leak state as stale.
    int leakTelemetryStaleTimeoutMs{kDefaultLeakTelemetryStaleTimeoutMs};
    quint16 supportedServoMask{SupportedServoMask};
    LinkProfile linkProfile{LinkProfile::DirectUart};

    static RobotControllerConfig bringUpProvisional();
    static RobotControllerConfig apc220Provisional();
};

struct MotionRequest {
    MotionMode mode{MotionMode::Stop};
    MotionAction action{MotionAction::Stop};
};

class RobotController final : public IConsoleController {
    Q_OBJECT

public:
    using PortDiscovery = std::function<QStringList()>;

    explicit RobotController(ITransport *transport,
                             RobotControllerConfig config,
                             PortDiscovery portDiscovery = {},
                             QObject *parent = nullptr);

    [[nodiscard]] ConsoleBackendKind backendKind() const noexcept override
    {
        return ConsoleBackendKind::DirectSerial;
    }
    void refreshSerialPorts() override;
    void connectController(const ConsoleConnectionConfiguration &configuration) override;
    void disconnectController() override;
    void connectTransport(const TransportConfiguration &configuration);
    void disconnectTransport();
    void shutdown() override;

    [[nodiscard]] bool canAcquireControl() const override { return false; }
    bool acquireControl() override { return isConnected(); }
    bool releaseControl() override { return false; }
    bool enableServo(ServoId id) override;
    bool disableServo(ServoId id) override;
    bool disableAll() override;
    bool setServoPwm(ServoId id, quint16 pulseUs) override;
    bool setServoAngle(ServoId id, qint16 angleCentidegrees) override;
    bool neutralServo(ServoId id) override;
    bool startMotion(MotionMode mode) override;
    bool stopMotion() override;
    bool setGaitBackend(GaitBackend backend) override;
    bool setFrontRearCoordination(FrontRearCoordination coordination) override;

    [[nodiscard]] bool isConnected() const override { return state_ == TransportState::Connected; }
    [[nodiscard]] bool isControlActive() const override { return isConnected(); }
    [[nodiscard]] ControlAuthorityState authorityState() const override
    {
        return isConnected() ? ControlAuthorityState::Owned
                             : ControlAuthorityState::Unowned;
    }
    [[nodiscard]] bool supportsRawPwm() const noexcept override { return true; }
    [[nodiscard]] bool isServoSupported(ServoId id) const override;
    [[nodiscard]] bool isServoEnabled(ServoId id) const override;
    [[nodiscard]] bool isServoDisablePending(ServoId id) const override;
    [[nodiscard]] LeakState leakState() const override { return leakState_; }
    [[nodiscard]] const ImuMonitorState &imuState() const override { return imuMonitor_.state(); }
    [[nodiscard]] ImuMonitor *imuMonitor() { return &imuMonitor_; }
    [[nodiscard]] const ImuMonitor *imuMonitor() const { return &imuMonitor_; }
    [[nodiscard]] const DepthMonitorState &depthState() const override { return depthMonitor_.state(); }
    [[nodiscard]] DepthMonitor *depthMonitor() { return &depthMonitor_; }
    [[nodiscard]] const DepthMonitor *depthMonitor() const { return &depthMonitor_; }
    [[nodiscard]] RobotControllerConfig config() const { return config_; }
    [[nodiscard]] ProtocolMonitor monitor() const override { return monitor_; }
    [[nodiscard]] MotionState motionState() const override { return motionState_; }
    [[nodiscard]] MotionMode motionMode() const override { return motionMode_; }
    [[nodiscard]] std::optional<GaitBackend> confirmedGaitBackend() const
    {
        return confirmedGaitBackend_;
    }
    [[nodiscard]] std::optional<GaitBackend> requestedGaitBackend() const
    {
        return pendingGaitBackend_;
    }
    [[nodiscard]] bool isGaitBackendChangePending() const override
    {
        return pendingGaitBackend_.has_value();
    }
    [[nodiscard]] std::optional<FrontRearCoordination>
    confirmedFrontRearCoordination() const override
    {
        return confirmedFrontRearCoordination_;
    }
    [[nodiscard]] std::optional<FrontRearCoordination>
    requestedFrontRearCoordination() const override
    {
        return pendingFrontRearCoordination_;
    }
    [[nodiscard]] bool isFrontRearCoordinationChangePending() const override
    {
        return pendingFrontRearCoordination_.has_value();
    }
    [[nodiscard]] bool isMotionActive() const override;
    [[nodiscard]] bool isMotionReady(MotionMode mode) const override;
    [[nodiscard]] bool isMotionTransitioning() const override
    {
        return motionModeTransitionTimer_.isActive();
    }
    [[nodiscard]] qsizetype queuedCommandCount() const
    {
        return commandQueue_.size() + priorityCommandQueue_.size()
            + motionStopCommandQueue_.size();
    }

private:
    struct PendingRequest {
        quint16 sequence{0};
        QByteArray frame;
        MessageType type;
        quint16 servoMask{0};
        qint64 sentAtMs{0};
        int retries{0};
        std::optional<MotionRequest> motionRequest;
        bool motionCancelled{false};
        bool cancelled{false};
        std::optional<GaitBackend> gaitBackendRequest;
        std::optional<FrontRearCoordination> frontRearCoordinationRequest;
    };

    struct QueuedCommand {
        MessageType type;
        QByteArray payload;
        quint16 affectedMask{0};
        std::optional<MotionRequest> motionRequest;
        std::optional<GaitBackend> gaitBackendRequest;
        std::optional<FrontRearCoordination> frontRearCoordinationRequest;
    };

    bool sendCommand(MessageType type, const QByteArray &payload, quint16 affectedMask = 0,
                     bool expectAck = true,
                     std::optional<MotionRequest> motionRequest = std::nullopt,
                     std::optional<GaitBackend> gaitBackendRequest = std::nullopt,
                     std::optional<FrontRearCoordination> frontRearCoordinationRequest = std::nullopt);
    void sendHeartbeat();
    bool dispatchApc220Command(const QueuedCommand &command);
    bool dispatchApc220Retry(quint16 sequence);
    void pumpApc220Scheduler();
    void refreshApc220HeartbeatDue();
    bool canStartApc220OrdinaryExchange() const;
    void dispatchApc220Heartbeat();
    void processIncoming(const QByteArray &bytes);
    void handlePacket(const Packet &packet);
    void handleAck(const Packet &packet);
    void handleLeakStatus(const Packet &packet);
    void handleImuSnapshot(const Packet &packet);
    void handleDepthSnapshot(const Packet &packet);
    void checkTimeouts();
    void refreshLeakTelemetryStaleness(qint64 now);
    void updateMonitor();
    void resetSchedulerState();
    void dropQueuedCommand(const QueuedCommand &command);
    void clearQueuedCommandsForDisable(quint16 affectedMask);
    void cancelQueuedMotionRequests();
    void cancelPendingMotionRequests();
    [[nodiscard]] bool hasPendingMotionWork() const;
    [[nodiscard]] bool hasPendingMotionStop() const;
    [[nodiscard]] bool hasPendingMotionStart() const;
    void cancelPendingDirectActuatorRequests();
    void failClosedDirectActuators();
    void failClosedApc220Actuators();
    void failClosedMotionState();
    void setMotionState(MotionState state, MotionMode mode);
    void setEnabledMask(quint16 mask);
    void setDisablePendingMask(quint16 mask);
    void setLeakState(LeakState state);
    void clearGaitBackendPending();
    void clearGaitBackendOutstanding();
    void clearFrontRearCoordinationPending();
    void clearFrontRearCoordinationOutstanding();
    void markApc220LivenessLost();
    void noteWriteFailure(const QString &context);
    bool rejectUnsupportedServo(ServoId id, const QString &command);
    [[nodiscard]] quint16 motionProtectionMask() const;
    static QByteArray maskPayload(quint16 mask);
    static qint64 nowMs();
    static bool isMotionCommand(MessageType type);
    static bool isActuatorCommand(MessageType type);

    ITransport *transport_;
    RobotControllerConfig config_;
    PortDiscovery portDiscovery_;
    StreamDecoder decoder_;
    TransportState state_{TransportState::Disconnected};
    quint16 nextSequence_{1};
    quint16 enabledMask_{0};
    quint16 disablePendingMask_{0};
    quint16 motionOwnedMask_{0};
    quint16 motionTransitionOwnedMask_{0};
    QHash<quint16, PendingRequest> pending_;
    QQueue<QueuedCommand> priorityCommandQueue_;
    // A graceful Motion STOP outranks ordinary work but remains below the
    // safety-priority Disable queue.  Keeping a dedicated lane also makes
    // STOP supersession explicit instead of relying on FIFO ordering.
    QQueue<QueuedCommand> motionStopCommandQueue_;
    QQueue<QueuedCommand> commandQueue_;
    std::optional<PendingRequest> deferredRetry_;
    bool heartbeatDue_{false};
    bool heartbeatReady_{false};
    bool actuatorFailClosed_{false};
    qint64 nextHeartbeatDueAtMs_{0};
    qint64 nextHeartbeatSafetyDeadlineAtMs_{0};
    QTimer heartbeatTimer_;
    QTimer retryTimer_;
    QTimer motionStopTimer_;
    QTimer motionModeTransitionTimer_;
    ProtocolMonitor monitor_;
    LeakState leakState_{LeakState::Unknown};
    qint64 lastLeakTelemetryAtMs_{-1};
    ImuMonitor imuMonitor_;
    DepthMonitor depthMonitor_;
    MotionState motionState_{MotionState::Stopped};
    MotionMode motionMode_{MotionMode::Stop};
    std::optional<GaitBackend> confirmedGaitBackend_;
    std::optional<GaitBackend> pendingGaitBackend_;
    std::optional<FrontRearCoordination> confirmedFrontRearCoordination_;
    std::optional<FrontRearCoordination> pendingFrontRearCoordination_;
};

} // namespace rb
