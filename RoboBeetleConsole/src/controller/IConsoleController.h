#pragma once

#include "robot/DepthControl.h"
#include "robot/DepthMonitor.h"
#include "robot/ImuMonitor.h"
#include "robot/LeakStatus.h"
#include "robot/RobotCommand.h"
#include "transport/ITransport.h"

#include <QObject>
#include <QString>
#include <QStringList>

#include <optional>

namespace rb {

enum class ConsoleBackendKind {
    RemoteRbrp,
    DirectSerial,
};

enum class ControlAuthorityState {
    Unowned,
    Acquiring,
    Owned,
};

enum class CommandTerminalResult { Ok, Busy, Rejected, OutcomeUnknown };

struct ConsoleConnectionConfiguration {
    QString endpoint;
    quint16 tcpPort{47000};
    qint32 baudRate{9600};
};

struct ProtocolMonitor {
    quint64 txPacketCount{0};
    quint64 rxPacketCount{0};
    quint64 crcErrorCount{0};
    quint64 timeoutCount{0};
    qint64 lastAckRttMs{-1};
    QString ackStatus{QStringLiteral("Idle")};
};

class IConsoleController : public QObject {
    Q_OBJECT

public:
    explicit IConsoleController(QObject *parent = nullptr) : QObject(parent) {}
    ~IConsoleController() override = default;

    [[nodiscard]] virtual ConsoleBackendKind backendKind() const noexcept = 0;
    virtual void refreshSerialPorts() = 0;
    virtual void connectController(const ConsoleConnectionConfiguration &configuration) = 0;
    virtual void disconnectController() = 0;
    virtual void shutdown() = 0;

    [[nodiscard]] virtual bool canAcquireControl() const = 0;
    virtual bool acquireControl() = 0;
    virtual bool releaseControl() = 0;
    virtual bool enableServo(ServoId id) = 0;
    virtual bool disableServo(ServoId id) = 0;
    virtual bool disableAll() = 0;
    virtual bool setServoPwm(ServoId id, quint16 pulseUs) = 0;
    virtual bool setServoAngle(ServoId id, qint16 angleCentidegrees) = 0;
    virtual bool neutralServo(ServoId id) = 0;
    virtual bool startMotion(MotionMode mode) = 0;
    virtual bool stopMotion() = 0;
    virtual std::optional<quint32> submitVisualMotion(MotionMode) { return std::nullopt; }
    [[nodiscard]] virtual quint16 inferredPoseKnownMask() const { return 0; }
    [[nodiscard]] virtual std::optional<DepthControlSample> controlDepthSample() const { return std::nullopt; }
    virtual bool zeroDepth(QString *error = nullptr)
    {
        if (error) *error = QStringLiteral("unsupported");
        return false;
    }
    [[nodiscard]] virtual std::optional<qint64> depthZeroedAtMs() const { return std::nullopt; }
    virtual bool setGaitBackend(GaitBackend backend) = 0;
    virtual bool setFrontRearCoordination(FrontRearCoordination coordination) = 0;

    [[nodiscard]] virtual bool isConnected() const = 0;
    [[nodiscard]] virtual bool isControlActive() const = 0;
    [[nodiscard]] virtual ControlAuthorityState authorityState() const = 0;
    [[nodiscard]] virtual bool supportsRawPwm() const noexcept = 0;
    [[nodiscard]] virtual bool isServoSupported(ServoId id) const = 0;
    [[nodiscard]] virtual bool isServoEnabled(ServoId id) const = 0;
    [[nodiscard]] virtual bool isServoDisablePending(ServoId id) const = 0;
    [[nodiscard]] virtual LeakState leakState() const = 0;
    [[nodiscard]] virtual const ImuMonitorState &imuState() const = 0;
    [[nodiscard]] virtual const DepthMonitorState &depthState() const = 0;
    [[nodiscard]] virtual ProtocolMonitor monitor() const = 0;
    [[nodiscard]] virtual MotionState motionState() const = 0;
    [[nodiscard]] virtual MotionMode motionMode() const = 0;
    [[nodiscard]] virtual std::optional<GaitBackend> confirmedGaitBackend() const = 0;
    [[nodiscard]] virtual std::optional<GaitBackend> requestedGaitBackend() const = 0;
    [[nodiscard]] virtual bool isGaitBackendChangePending() const = 0;
    [[nodiscard]] virtual std::optional<FrontRearCoordination>
    confirmedFrontRearCoordination() const = 0;
    [[nodiscard]] virtual std::optional<FrontRearCoordination>
    requestedFrontRearCoordination() const = 0;
    [[nodiscard]] virtual bool isFrontRearCoordinationChangePending() const = 0;
    [[nodiscard]] virtual bool isMotionActive() const = 0;
    [[nodiscard]] virtual bool isMotionReady(MotionMode mode) const = 0;
    [[nodiscard]] virtual bool isMotionTransitioning() const = 0;

signals:
    void commandTerminal(quint32 requestId, rb::CommandTerminalResult result, quint8 rawResult, qint64 rttMs);
    void serialPortsChanged(const QStringList &ports);
    void connectionStateChanged(rb::TransportState state);
    void controlAvailabilityChanged();
    void authorityStateChanged(rb::ControlAuthorityState state, bool active);
    void servoStateChanged(int servoIndex, bool enabled);
    void servoDisablePendingChanged(int servoIndex, bool pending);
    void leakStateChanged(rb::LeakState state);
    void imuStateChanged();
    void depthStateChanged();
    void controlDepthSampleChanged(); // 每收到一个深度样本、归零或清除时发出
    void motionStateChanged(rb::MotionState state, rb::MotionMode mode);
    void gaitBackendStateChanged();
    void frontRearCoordinationStateChanged();
    void protocolMonitorChanged(const rb::ProtocolMonitor &monitor);
    void txHexChanged(const QString &hex);
    void rxHexChanged(const QString &hex);
    void logMessage(const QString &message);
};

} // namespace rb

Q_DECLARE_METATYPE(rb::ControlAuthorityState)
Q_DECLARE_METATYPE(rb::ProtocolMonitor)
Q_DECLARE_METATYPE(rb::MotionState)
Q_DECLARE_METATYPE(rb::MotionMode)
Q_DECLARE_METATYPE(rb::GaitBackend)
Q_DECLARE_METATYPE(rb::FrontRearCoordination)

Q_DECLARE_METATYPE(rb::CommandTerminalResult)
