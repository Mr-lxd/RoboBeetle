#pragma once

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

struct ProtocolMonitor {
    quint64 txPacketCount{0};
    quint64 rxPacketCount{0};
    quint64 crcErrorCount{0};
    quint64 timeoutCount{0};
    qint64 lastAckRttMs{-1};
    QString ackStatus{QStringLiteral("Idle")};
};

class RobotController final : public QObject {
    Q_OBJECT

public:
    using PortDiscovery = std::function<QStringList()>;

    explicit RobotController(ITransport *transport,
                             RobotControllerConfig config,
                             PortDiscovery portDiscovery = {},
                             QObject *parent = nullptr);

    void refreshSerialPorts();
    void connectTransport(const TransportConfiguration &configuration);
    void disconnectTransport();
    void shutdown();

    bool enableServo(ServoId id);
    bool disableServo(ServoId id);
    bool disableAll();
    bool setServoPwm(ServoId id, quint16 pulseUs);
    bool setServoAngle(ServoId id, qint16 angleCentidegrees);
    bool neutralServo(ServoId id);

    [[nodiscard]] bool isConnected() const { return state_ == TransportState::Connected; }
    [[nodiscard]] bool isServoSupported(ServoId id) const;
    [[nodiscard]] bool isServoEnabled(ServoId id) const;
    [[nodiscard]] bool isServoDisablePending(ServoId id) const;
    [[nodiscard]] LeakState leakState() const { return leakState_; }
    [[nodiscard]] const ImuMonitorState &imuState() const { return imuMonitor_.state(); }
    [[nodiscard]] ImuMonitor *imuMonitor() { return &imuMonitor_; }
    [[nodiscard]] const ImuMonitor *imuMonitor() const { return &imuMonitor_; }
    [[nodiscard]] const DepthMonitorState &depthState() const { return depthMonitor_.state(); }
    [[nodiscard]] DepthMonitor *depthMonitor() { return &depthMonitor_; }
    [[nodiscard]] const DepthMonitor *depthMonitor() const { return &depthMonitor_; }
    [[nodiscard]] RobotControllerConfig config() const { return config_; }
    [[nodiscard]] ProtocolMonitor monitor() const { return monitor_; }
    [[nodiscard]] qsizetype queuedCommandCount() const
    {
        return commandQueue_.size() + priorityCommandQueue_.size();
    }

signals:
    void serialPortsChanged(const QStringList &ports);
    void connectionStateChanged(rb::TransportState state);
    void servoStateChanged(int servoIndex, bool enabled);
    void servoDisablePendingChanged(int servoIndex, bool pending);
    void leakStateChanged(rb::LeakState state);
    void protocolMonitorChanged(const rb::ProtocolMonitor &monitor);
    void txHexChanged(const QString &hex);
    void rxHexChanged(const QString &hex);
    void logMessage(const QString &message);

private:
    struct PendingRequest {
        quint16 sequence{0};
        QByteArray frame;
        MessageType type;
        quint16 servoMask{0};
        qint64 sentAtMs{0};
        int retries{0};
    };

    struct QueuedCommand {
        MessageType type;
        QByteArray payload;
        quint16 affectedMask{0};
    };

    bool sendCommand(MessageType type, const QByteArray &payload, quint16 affectedMask = 0,
                     bool expectAck = true);
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
    void clearQueuedCommandsForDisable(quint16 affectedMask);
    void failClosedApc220Actuators();
    void setEnabledMask(quint16 mask);
    void setDisablePendingMask(quint16 mask);
    void setLeakState(LeakState state);
    void markApc220LivenessLost();
    void noteWriteFailure(const QString &context);
    bool rejectUnsupportedServo(ServoId id, const QString &command);
    static QByteArray maskPayload(quint16 mask);
    static qint64 nowMs();

    ITransport *transport_;
    RobotControllerConfig config_;
    PortDiscovery portDiscovery_;
    StreamDecoder decoder_;
    TransportState state_{TransportState::Disconnected};
    quint16 nextSequence_{1};
    quint16 enabledMask_{0};
    quint16 disablePendingMask_{0};
    QHash<quint16, PendingRequest> pending_;
    QQueue<QueuedCommand> priorityCommandQueue_;
    QQueue<QueuedCommand> commandQueue_;
    std::optional<PendingRequest> deferredRetry_;
    bool heartbeatDue_{false};
    bool heartbeatReady_{false};
    bool actuatorFailClosed_{false};
    qint64 nextHeartbeatDueAtMs_{0};
    qint64 nextHeartbeatSafetyDeadlineAtMs_{0};
    QTimer heartbeatTimer_;
    QTimer retryTimer_;
    ProtocolMonitor monitor_;
    LeakState leakState_{LeakState::Unknown};
    qint64 lastLeakTelemetryAtMs_{-1};
    ImuMonitor imuMonitor_;
    DepthMonitor depthMonitor_;
};

} // namespace rb

Q_DECLARE_METATYPE(rb::ProtocolMonitor)
