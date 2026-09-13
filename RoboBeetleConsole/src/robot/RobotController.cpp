#include "robot/RobotController.h"

#include "protocol/PacketCodec.h"

#include <QDateTime>

#include <chrono>
#include <optional>
#include <utility>

namespace rb {
namespace {

quint16 readLe16(const QByteArray &data, qsizetype offset)
{
    return static_cast<quint16>(static_cast<quint8>(data[offset])
        | (static_cast<quint16>(static_cast<quint8>(data[offset + 1])) << 8U));
}

void appendLe16(QByteArray &data, quint16 value)
{
    data.append(static_cast<char>(value & 0xffU));
    data.append(static_cast<char>((value >> 8U) & 0xffU));
}

QByteArray motionPayload(MotionMode mode, MotionAction action)
{
    QByteArray payload;
    payload.append(static_cast<char>(1));
    payload.append(static_cast<char>(mode));
    payload.append(static_cast<char>(action));
    return payload;
}

QString ackResultText(quint8 value)
{
    switch (static_cast<AckResult>(value)) {
    case AckResult::Ok: return QStringLiteral("OK");
    case AckResult::InvalidPayload: return QStringLiteral("InvalidPayload");
    case AckResult::HostNotAlive: return QStringLiteral("HostNotAlive");
    case AckResult::UnsupportedServo: return QStringLiteral("UnsupportedServo");
    case AckResult::ServoNotEnabled: return QStringLiteral("ServoNotEnabled");
    case AckResult::OutOfRange: return QStringLiteral("OutOfRange");
    case AckResult::HardwareFailure: return QStringLiteral("HardwareFailure");
    case AckResult::Busy: return QStringLiteral("Busy");
    }
    return QStringLiteral("UnknownResult");
}

bool isServoActuatorCommand(MessageType type)
{
    switch (type) {
    case MessageType::ServoEnable:
    case MessageType::SetServoPwm:
    case MessageType::SetServoAngle:
    case MessageType::Neutral:
        return true;
    case MessageType::Heartbeat:
    case MessageType::Ack:
    case MessageType::Error:
    case MessageType::ServoDisable:
    case MessageType::LeakStatus:
    case MessageType::ImuSnapshot:
    case MessageType::DepthSnapshot:
        return false;
    }
    return false;
}

bool isMotionMessage(MessageType type)
{
    return type == MessageType::SetMotionMode;
}

} // namespace

RobotControllerConfig RobotControllerConfig::bringUpProvisional()
{
    return {};
}

RobotControllerConfig RobotControllerConfig::apc220Provisional()
{
    RobotControllerConfig config = bringUpProvisional();
    config.linkProfile = LinkProfile::Apc220HalfDuplex;
    config.heartbeatIntervalMs = 250;
    config.ackTimeoutMs = 250;
    config.heartbeatSafetyBudgetMs = 490;
    return config;
}

RobotController::RobotController(ITransport *transport,
                                 RobotControllerConfig config,
                                 PortDiscovery portDiscovery,
                                 QObject *parent)
    : QObject(parent),
      transport_(transport),
      config_(config),
      portDiscovery_(std::move(portDiscovery)),
      imuMonitor_(this),
      depthMonitor_(this)
{
    Q_ASSERT(transport_ != nullptr);
    heartbeatTimer_.setInterval(config_.heartbeatIntervalMs);
    retryTimer_.setInterval(qMax(10, config_.ackTimeoutMs / 4));
    motionStopTimer_.setSingleShot(true);

    connect(&heartbeatTimer_, &QTimer::timeout, this, &RobotController::sendHeartbeat);
    connect(&retryTimer_, &QTimer::timeout, this, &RobotController::checkTimeouts);
    connect(&motionStopTimer_, &QTimer::timeout, this, [this] {
        if (motionState_ == MotionState::Stopping) {
            setMotionState(MotionState::Stopped, MotionMode::Stop);
        }
    });
    connect(transport_, &ITransport::bytesReceived, this, &RobotController::processIncoming);
    connect(transport_, &ITransport::errorOccurred, this, [this](const QString &message) {
        emit logMessage(QStringLiteral("Transport error: %1").arg(message));
        if (config_.linkProfile != LinkProfile::Apc220HalfDuplex) {
            return;
        }

        const bool wasConnected = state_ == TransportState::Connected;
        const bool wasAlreadyError = state_ == TransportState::Error;
        heartbeatTimer_.stop();
        retryTimer_.stop();
        state_ = TransportState::Error;
        resetSchedulerState();
        failClosedMotionState();
        monitor_.ackStatus = QStringLiteral("Transport error");
        if (wasConnected) {
            emit logMessage(QStringLiteral(
                "Safety notice: transport was lost; Disable All could not be delivered"));
        }
        updateMonitor();
        if (!wasAlreadyError) {
            emit connectionStateChanged(TransportState::Error);
        }
    });
    connect(transport_, &ITransport::stateChanged, this, [this](TransportState state) {
        const bool wasConnected = state_ == TransportState::Connected;
        state_ = state;
        imuMonitor_.handleTransportState(state);
        depthMonitor_.handleTransportState(state);
        if (state == TransportState::Connected) {
            resetSchedulerState();
            setMotionState(MotionState::Stopped, MotionMode::Stop);
            heartbeatReady_ = config_.linkProfile != LinkProfile::Apc220HalfDuplex;
            heartbeatDue_ = config_.linkProfile == LinkProfile::Apc220HalfDuplex;
            nextHeartbeatDueAtMs_ = config_.linkProfile == LinkProfile::Apc220HalfDuplex
                ? nowMs()
                : 0;
            nextHeartbeatSafetyDeadlineAtMs_ = config_.linkProfile == LinkProfile::Apc220HalfDuplex
                ? nowMs()
                : 0;
            heartbeatTimer_.start();
            retryTimer_.start();
            monitor_.ackStatus = QStringLiteral("Connected; awaiting commands");
        } else if (state == TransportState::Closing
                   || state == TransportState::Disconnected || state == TransportState::Error) {
            heartbeatTimer_.stop();
            retryTimer_.stop();
            resetSchedulerState();
            failClosedMotionState();
            monitor_.ackStatus = state == TransportState::Error
                ? QStringLiteral("Transport error")
                : QStringLiteral("Disconnected");
            if (state == TransportState::Error && wasConnected) {
                emit logMessage(QStringLiteral(
                    "Safety notice: transport was lost; Disable All could not be delivered"));
            }
        }
        updateMonitor();
        if (state_ == state) {
            if (state == TransportState::Connected
                && config_.linkProfile == LinkProfile::Apc220HalfDuplex) {
                pumpApc220Scheduler();
            }
            if (state_ == state) {
                emit connectionStateChanged(state);
            }
        }
    });
}

void RobotController::refreshSerialPorts()
{
    emit serialPortsChanged(portDiscovery_ ? portDiscovery_() : QStringList{});
}

void RobotController::connectTransport(const TransportConfiguration &configuration)
{
    transport_->open(configuration);
}

void RobotController::disconnectTransport()
{
    if (isConnected()) {
        failClosedMotionState();
        const quint16 supportedMask = config_.supportedServoMask;
        const QByteArray payload = maskPayload(supportedMask);
        if (!sendCommand(MessageType::ServoDisable, payload, supportedMask, false)) {
            noteWriteFailure(QStringLiteral("Disable All during disconnect"));
        } else {
            emit logMessage(QStringLiteral("Disable All sent before disconnect; ACK not awaited"));
        }
    } else {
        emit logMessage(QStringLiteral("Disable All could not be sent: transport is not connected"));
    }
    resetSchedulerState();
    transport_->close();
}

void RobotController::shutdown()
{
    disconnectTransport();
}

bool RobotController::enableServo(ServoId id)
{
    if (rejectUnsupportedServo(id, QStringLiteral("Servo Enable"))) {
        return false;
    }
    return sendCommand(MessageType::ServoEnable, maskPayload(servoMask(id)), servoMask(id));
}

bool RobotController::disableServo(ServoId id)
{
    if (rejectUnsupportedServo(id, QStringLiteral("Servo Disable"))) {
        return false;
    }
    const quint16 mask = servoMask(id);
    if ((disablePendingMask_ & mask) != 0U) {
        emit logMessage(QStringLiteral("Servo Disable already awaiting ACK for servo %1")
                            .arg(static_cast<quint8>(id)));
        return false;
    }
    const bool markPendingBeforeSend = config_.linkProfile == LinkProfile::Apc220HalfDuplex;
    if (markPendingBeforeSend) {
        setDisablePendingMask(static_cast<quint16>(disablePendingMask_ | mask));
    }
    failClosedMotionState();
    if (!sendCommand(MessageType::ServoDisable, maskPayload(mask), mask)) {
        if (markPendingBeforeSend) {
            setDisablePendingMask(static_cast<quint16>(disablePendingMask_ & ~mask));
        }
        return false;
    }
    if (!markPendingBeforeSend) {
        setDisablePendingMask(static_cast<quint16>(disablePendingMask_ | mask));
    }
    return true;
}

bool RobotController::disableAll()
{
    const quint16 mask = config_.supportedServoMask;
    if ((disablePendingMask_ & mask) != 0U) {
        emit logMessage(QStringLiteral("Disable All already awaiting ACK"));
        return false;
    }
    const bool markPendingBeforeSend = config_.linkProfile == LinkProfile::Apc220HalfDuplex;
    if (markPendingBeforeSend) {
        setDisablePendingMask(static_cast<quint16>(disablePendingMask_ | mask));
    }
    failClosedMotionState();
    if (!sendCommand(MessageType::ServoDisable, maskPayload(mask), mask)) {
        if (markPendingBeforeSend) {
            setDisablePendingMask(static_cast<quint16>(disablePendingMask_ & ~mask));
        }
        return false;
    }
    if (!markPendingBeforeSend) {
        setDisablePendingMask(static_cast<quint16>(disablePendingMask_ | mask));
    }
    return true;
}

bool RobotController::setServoPwm(ServoId id, quint16 pulseUs)
{
    if (isMotionActive()) {
        emit logMessage(QStringLiteral("Set PWM rejected: Motion owns actuators (BUSY)"));
        return false;
    }
    if (rejectUnsupportedServo(id, QStringLiteral("Set PWM"))) {
        return false;
    }
    const ServoDescriptor *descriptor = servoDescriptor(id);
    if (descriptor == nullptr) {
        return false;
    }
    if (!isServoEnabled(id)) {
        emit logMessage(QStringLiteral("Set PWM rejected: %1 is not enabled and acknowledged")
                            .arg(QString::fromLatin1(descriptor->semanticName)));
        return false;
    }
    if (isServoDisablePending(id)) {
        emit logMessage(QStringLiteral("Set PWM rejected: %1 disable is awaiting ACK")
                            .arg(QString::fromLatin1(descriptor->semanticName)));
        return false;
    }
    if (pulseUs < descriptor->commandMinPwmUs || pulseUs > descriptor->commandMaxPwmUs) {
        emit logMessage(QStringLiteral("Set PWM rejected: %1 us is outside %2 command range %3-%4 us")
                            .arg(pulseUs)
                            .arg(QString::fromLatin1(descriptor->semanticName))
                            .arg(descriptor->commandMinPwmUs)
                            .arg(descriptor->commandMaxPwmUs));
        return false;
    }

    QByteArray payload;
    payload.append(static_cast<char>(1));
    payload.append(static_cast<char>(id));
    appendLe16(payload, pulseUs);
    return sendCommand(MessageType::SetServoPwm, payload, servoMask(id));
}

bool RobotController::setServoAngle(ServoId id, qint16 angleCentidegrees)
{
    if (isMotionActive()) {
        emit logMessage(QStringLiteral("Set Angle rejected: Motion owns actuators (BUSY)"));
        return false;
    }
    if (rejectUnsupportedServo(id, QStringLiteral("Set Angle"))) {
        return false;
    }
    const ServoDescriptor *descriptor = servoDescriptor(id);
    if ((descriptor == nullptr) || !descriptor->angleSupported) {
        emit logMessage(QStringLiteral("Set Angle rejected: %1 has no angle capability")
                            .arg(descriptor == nullptr
                                     ? QStringLiteral("unknown servo")
                                     : QString::fromLatin1(descriptor->semanticName)));
        return false;
    }
    if (!isServoEnabled(id)) {
        emit logMessage(QStringLiteral("Set Angle rejected: %1 is not enabled and acknowledged")
                            .arg(QString::fromLatin1(descriptor->semanticName)));
        return false;
    }
    if (isServoDisablePending(id)) {
        emit logMessage(QStringLiteral("Set Angle rejected: %1 disable is awaiting ACK")
                            .arg(QString::fromLatin1(descriptor->semanticName)));
        return false;
    }
    if (angleCentidegrees < descriptor->commandMinAngleCdeg
        || angleCentidegrees > descriptor->commandMaxAngleCdeg) {
        emit logMessage(QStringLiteral("Set Angle rejected: %1 cdeg is outside %2 command range %3-%4 cdeg")
                            .arg(angleCentidegrees)
                            .arg(QString::fromLatin1(descriptor->semanticName))
                            .arg(descriptor->commandMinAngleCdeg)
                            .arg(descriptor->commandMaxAngleCdeg));
        return false;
    }

    QByteArray payload;
    payload.append(static_cast<char>(1));
    payload.append(static_cast<char>(id));
    appendLe16(payload, static_cast<quint16>(angleCentidegrees));
    return sendCommand(MessageType::SetServoAngle, payload, servoMask(id));
}

bool RobotController::neutralServo(ServoId id)
{
    if (isMotionActive()) {
        emit logMessage(QStringLiteral("Neutral rejected: Motion owns actuators (BUSY)"));
        return false;
    }
    if (rejectUnsupportedServo(id, QStringLiteral("Neutral"))) {
        return false;
    }
    const ServoDescriptor *descriptor = servoDescriptor(id);
    if (descriptor == nullptr) {
        return false;
    }
    if (!isServoEnabled(id)) {
        emit logMessage(QStringLiteral("Neutral rejected: %1 is not enabled and acknowledged")
                            .arg(QString::fromLatin1(descriptor->semanticName)));
        return false;
    }
    if (isServoDisablePending(id)) {
        emit logMessage(QStringLiteral("Neutral rejected: %1 disable is awaiting ACK")
                            .arg(QString::fromLatin1(descriptor->semanticName)));
        return false;
    }
    return sendCommand(MessageType::Neutral, maskPayload(servoMask(id)), servoMask(id));
}

bool RobotController::startMotion(MotionMode mode)
{
    if (!isConnected()) {
        emit logMessage(QStringLiteral("Motion START rejected: transport is not connected"));
        return false;
    }
    if (!isValidMotionMode(mode) || mode == MotionMode::Stop) {
        emit logMessage(QStringLiteral("Motion START rejected: invalid mode"));
        return false;
    }
    if (motionState_ == MotionState::Stopping) {
        emit logMessage(QStringLiteral("Motion START rejected: STOPPING (BUSY)"));
        return false;
    }
    if (!isMotionReady(mode)) {
        emit logMessage(QStringLiteral("Motion START rejected: required Servo channels are not enabled"));
        return false;
    }

    return sendCommand(
        MessageType::SetMotionMode,
        motionPayload(mode, MotionAction::Start),
        motionRequiredServoMask(mode),
        true,
        MotionRequest{mode, MotionAction::Start});
}

bool RobotController::stopMotion()
{
    if (!isConnected()) {
        emit logMessage(QStringLiteral("Motion STOP rejected: transport is not connected"));
        return false;
    }
    if (motionState_ == MotionState::Stopping) {
        return true;
    }
    if (motionState_ == MotionState::Stopped || motionState_ == MotionState::Faulted) {
        return true;
    }

    return sendCommand(
        MessageType::SetMotionMode,
        motionPayload(MotionMode::Stop, MotionAction::Stop),
        SupportedServoMask,
        true,
        MotionRequest{MotionMode::Stop, MotionAction::Stop});
}

bool RobotController::isMotionReady(MotionMode mode) const
{
    if (!isConnected() || !isValidMotionMode(mode) || mode == MotionMode::Stop) {
        return false;
    }
    const quint16 requiredMask = motionRequiredServoMask(mode);
    return (config_.supportedServoMask & requiredMask) == requiredMask
        && (enabledMask_ & requiredMask) == requiredMask
        && (disablePendingMask_ & requiredMask) == 0U;
}

bool RobotController::isServoEnabled(ServoId id) const
{
    const ServoDescriptor *descriptor = servoDescriptor(id);
    return descriptor != nullptr && (enabledMask_ & descriptor->mask) != 0U;
}

bool RobotController::isServoDisablePending(ServoId id) const
{
    const ServoDescriptor *descriptor = servoDescriptor(id);
    return descriptor != nullptr && (disablePendingMask_ & descriptor->mask) != 0U;
}

bool RobotController::isServoSupported(ServoId id) const
{
    const ServoDescriptor *descriptor = servoDescriptor(id);
    return descriptor != nullptr && descriptor->supported
        && (config_.supportedServoMask & descriptor->mask) != 0U;
}

bool RobotController::sendCommand(MessageType type,
                                  const QByteArray &payload,
                                  quint16 affectedMask,
                                  bool expectAck,
                                  std::optional<MotionRequest> motionRequest)
{
    if (!isConnected()) {
        emit logMessage(QStringLiteral("Command rejected: transport is not connected"));
        return false;
    }

    if (isMotionActive() && isServoActuatorCommand(type)) {
        emit logMessage(QStringLiteral("Command rejected: Motion owns actuators (BUSY)"));
        return false;
    }

    if (config_.linkProfile == LinkProfile::Apc220HalfDuplex && expectAck) {
        if (type == MessageType::ServoEnable && actuatorFailClosed_ && !heartbeatReady_) {
            monitor_.ackStatus = QStringLiteral(
                "Host-link liveness recovering; Enable rejected");
            emit logMessage(monitor_.ackStatus);
            updateMonitor();
            return false;
        }

        refreshApc220HeartbeatDue();
        const QueuedCommand command{type, payload, affectedMask, motionRequest};
        const bool isSafetyDisable = type == MessageType::ServoDisable;
        const bool ordinaryAdmissionBlocked = !isSafetyDisable
            && !canStartApc220OrdinaryExchange();
        if (isSafetyDisable) {
            clearQueuedCommandsForDisable(affectedMask);
        }
        if (!heartbeatReady_ || !pending_.isEmpty() || heartbeatDue_ || ordinaryAdmissionBlocked
            || deferredRetry_.has_value() || !priorityCommandQueue_.isEmpty()
            || !commandQueue_.isEmpty()) {
            // A safety Disable must always have room.  Any ordinary entries
            // that survived affected-mask filtering are evicted before the
            // bounded-queue check; a full priority queue is still rejected.
            while (isSafetyDisable && queuedCommandCount() >= kApc220CommandQueueCapacity
                   && !commandQueue_.isEmpty()) {
                commandQueue_.dequeue();
            }
            if (queuedCommandCount() >= kApc220CommandQueueCapacity) {
                emit logMessage(QStringLiteral(
                    "Command rejected: host-link command queue is full (%1)")
                                    .arg(kApc220CommandQueueCapacity));
                return false;
            }
            if (isSafetyDisable) {
                priorityCommandQueue_.enqueue(command);
            } else {
                commandQueue_.enqueue(command);
            }
            monitor_.ackStatus = QStringLiteral("Queued %1message 0x%2 (%3/%4)")
                                     .arg(isSafetyDisable ? QStringLiteral("priority ")
                                                          : QString())
                                     .arg(static_cast<quint8>(type), 2, 16, QLatin1Char('0'))
                                     .arg(queuedCommandCount())
                                     .arg(kApc220CommandQueueCapacity);
            updateMonitor();
            if (isSafetyDisable || ordinaryAdmissionBlocked) {
                pumpApc220Scheduler();
            }
            return true;
        }
        return dispatchApc220Command(command);
    }

    const quint16 sequence = nextSequence_++;
    const QByteArray frame = PacketCodec::encodeWire({type, sequence, payload});
    if (frame.isEmpty() || !transport_->write(frame)) {
        noteWriteFailure(QStringLiteral("message 0x%1 sequence %2")
                             .arg(static_cast<quint8>(type), 2, 16, QLatin1Char('0'))
                             .arg(sequence));
        return false;
    }

    ++monitor_.txPacketCount;
    emit txHexChanged(QString::fromLatin1(frame.toHex(' ').toUpper()));
    if (expectAck) {
        pending_.insert(sequence,
                        {sequence, frame, type, affectedMask, nowMs(), 0,
                         motionRequest});
        monitor_.ackStatus = QStringLiteral("Waiting for ACK seq=%1").arg(sequence);
    }
    updateMonitor();
    return true;
}

bool RobotController::dispatchApc220Command(const QueuedCommand &command)
{
    const quint16 sequence = nextSequence_++;
    const QByteArray frame = PacketCodec::encodeWire({command.type, sequence, command.payload});
    if (frame.isEmpty()) {
        noteWriteFailure(QStringLiteral("message 0x%1 sequence %2")
                             .arg(static_cast<quint8>(command.type), 2, 16, QLatin1Char('0'))
                             .arg(sequence));
        return false;
    }

    pending_.insert(sequence,
                    {sequence, frame, command.type, command.affectedMask, nowMs(), 0,
                     command.motionRequest});
    const bool writeSucceeded = transport_->write(frame);
    const bool stillConnected = isConnected();
    if (!writeSucceeded || !stillConnected) {
        if (pending_.contains(sequence)) {
            pending_.remove(sequence);
        }
        noteWriteFailure(QStringLiteral("message 0x%1 sequence %2")
                             .arg(static_cast<quint8>(command.type), 2, 16, QLatin1Char('0'))
                             .arg(sequence));
        return false;
    }

    if (pending_.contains(sequence)) {
        pending_[sequence].sentAtMs = nowMs();
        monitor_.ackStatus = QStringLiteral("Waiting for ACK seq=%1").arg(sequence);
    }
    ++monitor_.txPacketCount;
    emit txHexChanged(QString::fromLatin1(frame.toHex(' ').toUpper()));
    updateMonitor();
    return true;
}

bool RobotController::dispatchApc220Retry(quint16 sequence)
{
    const auto it = pending_.find(sequence);
    if (it == pending_.end()) {
        return false;
    }

    const PendingRequest request = it.value();
    const int nextRetry = request.retries + 1;
    const bool writeSucceeded = transport_->write(request.frame);
    auto current = pending_.find(sequence);
    if (current == pending_.end()) {
        // A synchronous transport error may reset the scheduler while write() is
        // still unwinding. Do not touch the invalidated hash entry.
        return false;
    }

    current->retries = nextRetry;
    current->sentAtMs = nowMs();
    if (!writeSucceeded) {
        const int retryCount = current->retries;
        noteWriteFailure(QStringLiteral("retry seq=%1").arg(sequence));
        emit logMessage(QStringLiteral("Retry %1/%2 seq=%3 failed")
                            .arg(retryCount)
                            .arg(config_.maxRetries)
                            .arg(sequence));
        updateMonitor();
        return false;
    }

    const QByteArray frame = current->frame;
    const int retryCount = current->retries;
    ++monitor_.txPacketCount;
    emit txHexChanged(QString::fromLatin1(frame.toHex(' ').toUpper()));
    emit logMessage(QStringLiteral("Retry %1/%2 seq=%3")
                        .arg(retryCount)
                        .arg(config_.maxRetries)
                        .arg(sequence));
    updateMonitor();
    return true;
}

void RobotController::pumpApc220Scheduler()
{
    if (config_.linkProfile != LinkProfile::Apc220HalfDuplex) {
        return;
    }
    refreshApc220HeartbeatDue();
    if (!isConnected()
        || !pending_.isEmpty()) {
        return;
    }

    if (!heartbeatReady_) {
        if (!heartbeatDue_) {
            return;
        }
        dispatchApc220Heartbeat();
        return;
    }

    if (heartbeatDue_) {
        dispatchApc220Heartbeat();
        return;
    }

    if (!priorityCommandQueue_.isEmpty()) {
        const QueuedCommand command = priorityCommandQueue_.dequeue();
        if (!dispatchApc220Command(command)) {
            if (isConnected() && command.type == MessageType::ServoDisable) {
                setDisablePendingMask(
                    static_cast<quint16>(disablePendingMask_ & ~command.affectedMask));
            }
            emit logMessage(QStringLiteral(
                "Host-link priority command dropped after write failure (message 0x%1)")
                                .arg(static_cast<quint8>(command.type), 2, 16, QLatin1Char('0')));
        }
        return;
    }

    // The soft target is allowed to slip while an exchange is in flight, but
    // do not start ordinary work when its worst-case timeout/polling window
    // would cross the hard heartbeat safety boundary.  Send a heartbeat now
    // and release the ordinary queue only after that exchange completes.
    const bool hasOrdinaryWork = deferredRetry_.has_value() || !commandQueue_.isEmpty();
    if (hasOrdinaryWork && !canStartApc220OrdinaryExchange()) {
        heartbeatDue_ = true;
        dispatchApc220Heartbeat();
        return;
    }

    if (deferredRetry_.has_value()) {
        PendingRequest request = *deferredRetry_;
        deferredRetry_.reset();
        pending_.insert(request.sequence, request);
        dispatchApc220Retry(request.sequence);
        return;
    }

    if (!commandQueue_.isEmpty()) {
        const QueuedCommand command = commandQueue_.dequeue();
        if (!dispatchApc220Command(command)) {
            if (isConnected() && command.type == MessageType::ServoDisable) {
                setDisablePendingMask(
                    static_cast<quint16>(disablePendingMask_ & ~command.affectedMask));
            }
            emit logMessage(QStringLiteral(
                "Host-link command dropped after write failure (message 0x%1)")
                                .arg(static_cast<quint8>(command.type), 2, 16, QLatin1Char('0')));
        }
    }
}

void RobotController::sendHeartbeat()
{
    if (config_.linkProfile == LinkProfile::Apc220HalfDuplex) {
        if (!pending_.isEmpty()
            && pending_.begin()->type == MessageType::Heartbeat) {
            return;
        }
        refreshApc220HeartbeatDue();
        if (!heartbeatDue_ && nextHeartbeatDueAtMs_ > 0) {
            return;
        }
        heartbeatDue_ = true;
        pumpApc220Scheduler();
        return;
    }
    const quint32 uptime = static_cast<quint32>(QDateTime::currentMSecsSinceEpoch() & 0xffffffffU);
    QByteArray payload;
    payload.append(static_cast<char>(uptime & 0xffU));
    payload.append(static_cast<char>((uptime >> 8U) & 0xffU));
    payload.append(static_cast<char>((uptime >> 16U) & 0xffU));
    payload.append(static_cast<char>((uptime >> 24U) & 0xffU));
    sendCommand(MessageType::Heartbeat, payload);
}

void RobotController::dispatchApc220Heartbeat()
{
    QByteArray payload;
    const quint32 uptime = static_cast<quint32>(QDateTime::currentMSecsSinceEpoch()
                                                 & 0xffffffffU);
    payload.append(static_cast<char>(uptime & 0xffU));
    payload.append(static_cast<char>((uptime >> 8U) & 0xffU));
    payload.append(static_cast<char>((uptime >> 16U) & 0xffU));
    payload.append(static_cast<char>((uptime >> 24U) & 0xffU));

    // Both deadlines are anchored at the wire dispatch time.  ACK handling
    // may confirm liveness and record RTT, but never moves either deadline.
    const qint64 dispatchAtMs = nowMs();
    heartbeatDue_ = false;
    nextHeartbeatDueAtMs_ = dispatchAtMs + qMax(0, config_.heartbeatIntervalMs);
    nextHeartbeatSafetyDeadlineAtMs_ = config_.heartbeatSafetyBudgetMs > 0
        ? dispatchAtMs + config_.heartbeatSafetyBudgetMs
        : 0;
    if (!dispatchApc220Command({MessageType::Heartbeat, payload, 0})) {
        heartbeatDue_ = true;
        nextHeartbeatDueAtMs_ = dispatchAtMs;
        nextHeartbeatSafetyDeadlineAtMs_ = dispatchAtMs;
    } else if (!heartbeatReady_ && pending_.isEmpty()) {
        // A synchronous rejected/mismatched ACK consumed the request during
        // write(); keep liveness due without moving the deadline to ACK time.
        heartbeatDue_ = true;
    }
}

void RobotController::refreshApc220HeartbeatDue()
{
    const qint64 now = nowMs();
    if (config_.linkProfile == LinkProfile::Apc220HalfDuplex
        && ((nextHeartbeatDueAtMs_ > 0 && now >= nextHeartbeatDueAtMs_)
            || (nextHeartbeatSafetyDeadlineAtMs_ > 0
                && now >= nextHeartbeatSafetyDeadlineAtMs_))) {
        heartbeatDue_ = true;
    }
}

bool RobotController::canStartApc220OrdinaryExchange() const
{
    if (config_.linkProfile != LinkProfile::Apc220HalfDuplex
        || nextHeartbeatSafetyDeadlineAtMs_ <= 0
        || config_.heartbeatSafetyBudgetMs <= 0) {
        return true;
    }

    // A timeout is only observed on the retry timer, so reserve one polling
    // interval in addition to the configured ACK timeout.  The conservative
    // host-link profile's 490 ms hard budget therefore leaves an explicit 10 ms below
    // Firmware's strict >500 ms watchdog boundary.
    const qint64 worstExchangeMs = qMax(0, config_.ackTimeoutMs)
        + qMax(0, retryTimer_.interval());
    return nowMs() + worstExchangeMs <= nextHeartbeatSafetyDeadlineAtMs_;
}

void RobotController::processIncoming(const QByteArray &bytes)
{
    emit rxHexChanged(QString::fromLatin1(bytes.toHex(' ').toUpper()));
    const QVector<DecodeResult> results = decoder_.feed(bytes);
    for (const DecodeResult &result : results) {
        if (!result.ok()) {
            if (result.error == DecodeError::CrcMismatch) {
                ++monitor_.crcErrorCount;
            }
            emit logMessage(QStringLiteral("RX rejected: %1 (%2)")
                                .arg(decodeErrorText(result.error), result.detail));
            continue;
        }
        ++monitor_.rxPacketCount;
        handlePacket(result.packet);
    }
    updateMonitor();
}

void RobotController::handlePacket(const Packet &packet)
{
    if (packet.type == MessageType::ImuSnapshot) {
        handleImuSnapshot(packet);
        return;
    }
    if (packet.type == MessageType::DepthSnapshot) {
        handleDepthSnapshot(packet);
        return;
    }
    if (packet.type == MessageType::LeakStatus) {
        handleLeakStatus(packet);
        return;
    }
    if (packet.type == MessageType::Ack) {
        handleAck(packet);
        return;
    }
    if (packet.type == MessageType::Error) {
        if (packet.payload.size() >= 5) {
            const quint16 sequence = readLe16(packet.payload, 0);
            const auto requestType = static_cast<MessageType>(
                static_cast<quint8>(packet.payload[2]));
            std::optional<PendingRequest> request;
            const auto it = pending_.find(sequence);
            if (it != pending_.end()) {
                request = it.value();
            } else if (deferredRetry_.has_value()
                       && deferredRetry_->sequence == sequence) {
                request = *deferredRetry_;
            }
            if (request.has_value() && request->type != requestType) {
                monitor_.ackStatus = QStringLiteral("Error type mismatch seq=%1")
                                         .arg(sequence);
                emit logMessage(monitor_.ackStatus);
                return;
            }
            if (!request.has_value()) {
                const quint16 errorCode = readLe16(packet.payload, 3);
                monitor_.ackStatus = QStringLiteral("Unmatched Error seq=%1 code=%2")
                                         .arg(sequence)
                                         .arg(errorCode);
                emit logMessage(monitor_.ackStatus);
                return;
            }
            if (it != pending_.end()) {
                pending_.erase(it);
            } else {
                deferredRetry_.reset();
            }
            if (request->type == MessageType::ServoDisable) {
                setDisablePendingMask(
                    static_cast<quint16>(disablePendingMask_ & ~request->servoMask));
            }
            const quint16 errorCode = readLe16(packet.payload, 3);
            monitor_.ackStatus = QStringLiteral("Error seq=%1 code=%2").arg(sequence).arg(errorCode);
            if (request->type == MessageType::Heartbeat) {
                heartbeatReady_ = false;
                heartbeatDue_ = true;
                markApc220LivenessLost();
                return;
            }
            pumpApc220Scheduler();
        } else {
            emit logMessage(QStringLiteral("Malformed Error payload"));
        }
        return;
    }
    emit logMessage(QStringLiteral("RX message 0x%1 ignored by Phase 1 Console")
                        .arg(static_cast<quint8>(packet.type), 2, 16, QLatin1Char('0')));
}

void RobotController::handleImuSnapshot(const Packet &packet)
{
    if (config_.linkProfile == LinkProfile::Apc220HalfDuplex && !heartbeatReady_) {
        emit logMessage(QStringLiteral("ImuSnapshot ignored while conservative host-link liveness is not ready"));
        imuMonitor_.handleLivenessLost();
        return;
    }

    imuMonitor_.handlePacket(packet, nowMs());
}

void RobotController::handleDepthSnapshot(const Packet &packet)
{
    if (config_.linkProfile == LinkProfile::Apc220HalfDuplex && !heartbeatReady_) {
        emit logMessage(QStringLiteral(
            "DepthSnapshot ignored while conservative host-link liveness is not ready"));
        depthMonitor_.handleLivenessLost();
        return;
    }

    depthMonitor_.handlePacket(packet, nowMs());
}

void RobotController::handleLeakStatus(const Packet &packet)
{
    if (config_.linkProfile == LinkProfile::Apc220HalfDuplex && !heartbeatReady_) {
        emit logMessage(QStringLiteral("LeakStatus ignored while conservative host-link liveness is not ready"));
        markApc220LivenessLost();
        return;
    }

    if (packet.payload.size() != 1) {
        emit logMessage(QStringLiteral("Invalid LeakStatus payload length=%1")
                            .arg(packet.payload.size()));
        setLeakState(LeakState::Unknown);
        return;
    }

    const quint8 rawState = static_cast<quint8>(packet.payload.front());
    if (!isValidLeakState(rawState)) {
        emit logMessage(QStringLiteral("Invalid LeakStatus value=%1")
                            .arg(rawState));
        setLeakState(LeakState::Unknown);
        return;
    }

    setLeakState(leakStateFromByte(rawState));
    lastLeakTelemetryAtMs_ = nowMs();
}

void RobotController::handleAck(const Packet &packet)
{
    if (packet.payload.size() != 4) {
        emit logMessage(QStringLiteral("Malformed ACK payload"));
        return;
    }
    const quint16 requestSequence = readLe16(packet.payload, 0);
    const auto requestType = static_cast<MessageType>(static_cast<quint8>(packet.payload[2]));
    const quint8 result = static_cast<quint8>(packet.payload[3]);
    std::optional<PendingRequest> request;
    const auto pendingIt = pending_.find(requestSequence);
    if (pendingIt != pending_.end()) {
        request = pendingIt.value();
        pending_.erase(pendingIt);
    } else if (deferredRetry_.has_value()
               && deferredRetry_->sequence == requestSequence) {
        request = *deferredRetry_;
        deferredRetry_.reset();
    }
    if (!request.has_value()) {
        emit logMessage(QStringLiteral("Unmatched ACK seq=%1").arg(requestSequence));
        return;
    }

    const bool isHeartbeat = request->type == MessageType::Heartbeat;
    const bool isApc220 = config_.linkProfile == LinkProfile::Apc220HalfDuplex;
    const qint64 ackRttMs = qMax<qint64>(0, nowMs() - request->sentAtMs);
    const auto clearDisablePending = [this, &request] {
        if (request->type == MessageType::ServoDisable) {
            setDisablePendingMask(
                static_cast<quint16>(disablePendingMask_ & ~request->servoMask));
        }
    };
    if (request->type != requestType) {
        clearDisablePending();
        if (isApc220 && isHeartbeat) {
            heartbeatReady_ = false;
            heartbeatDue_ = true;
            markApc220LivenessLost();
        }
        monitor_.ackStatus = QStringLiteral("ACK type mismatch seq=%1").arg(requestSequence);
        emit logMessage(monitor_.ackStatus);
        if (!isApc220 || !isHeartbeat) {
            pumpApc220Scheduler();
        }
        return;
    }
    monitor_.lastAckRttMs = ackRttMs;
    if (result != static_cast<quint8>(AckResult::Ok)) {
        clearDisablePending();
        if (isApc220 && isHeartbeat) {
            heartbeatReady_ = false;
            heartbeatDue_ = true;
            markApc220LivenessLost();
        }
        monitor_.ackStatus = QStringLiteral("ACK rejected seq=%1 result=%2 (%3)")
                                 .arg(requestSequence)
                                 .arg(result)
                                 .arg(ackResultText(result));
        if (config_.linkProfile == LinkProfile::Apc220HalfDuplex) {
            monitor_.ackStatus += QStringLiteral(" RTT=%1 ms").arg(monitor_.lastAckRttMs);
        }
        if (!isApc220 || !isHeartbeat) {
            pumpApc220Scheduler();
        }
        return;
    }

    if (isApc220 && isHeartbeat) {
        refreshApc220HeartbeatDue();
        heartbeatReady_ = true;
        // The heartbeat deadline is anchored at dispatch.  An ACK only
        // confirms liveness/RTT; preserve a deadline that elapsed while the
        // heartbeat exchange was in flight.
    } else if (request->type == MessageType::ServoEnable) {
        // A matching user Enable ACK is the explicit re-arm after any
        // liveness fail-closed transition.
        actuatorFailClosed_ = false;
        setEnabledMask(static_cast<quint16>(enabledMask_ | request->servoMask));
    } else if (request->type == MessageType::ServoDisable) {
        clearDisablePending();
        setEnabledMask(static_cast<quint16>(enabledMask_ & ~request->servoMask));
    } else if (request->type == MessageType::SetMotionMode
               && request->motionRequest.has_value()) {
        const MotionRequest motion = *request->motionRequest;
        if (motion.action == MotionAction::Start) {
            setMotionState(MotionState::Running, motion.mode);
        } else {
            setMotionState(MotionState::Stopping, MotionMode::Stop);
            motionStopTimer_.start(kMotionTransitionDurationMs);
        }
    }
    monitor_.ackStatus = QStringLiteral("ACK seq=%1").arg(requestSequence);
    if (config_.linkProfile == LinkProfile::Apc220HalfDuplex) {
        monitor_.ackStatus += QStringLiteral(" RTT=%1 ms").arg(monitor_.lastAckRttMs);
    }
    if (!isApc220 || !isHeartbeat || heartbeatReady_) {
        pumpApc220Scheduler();
    }
}

void RobotController::checkTimeouts()
{
    const qint64 now = nowMs();
    refreshLeakTelemetryStaleness(now);
    imuMonitor_.tick(now);
    depthMonitor_.tick(now);
    if (config_.linkProfile == LinkProfile::Apc220HalfDuplex) {
        refreshApc220HeartbeatDue();
        if (pending_.isEmpty()) {
            if (heartbeatReady_) {
                pumpApc220Scheduler();
            }
            updateMonitor();
            return;
        }

        auto it = pending_.begin();
        if (now - it->sentAtMs < config_.ackTimeoutMs) {
            return;
        }
        if (it->type == MessageType::Heartbeat) {
            heartbeatReady_ = false;
            // Fail closed at the first missed heartbeat.  Retry bookkeeping
            // remains active so link liveness may recover independently of
            // actuator state, but stale actuator work must not survive the
            // Firmware watchdog budget.
            failClosedApc220Actuators();
        }
        if (it->retries >= config_.maxRetries) {
            const MessageType timedOutType = it->type;
            const quint16 sequence = it->sequence;
            if (timedOutType == MessageType::ServoDisable) {
                setDisablePendingMask(
                    static_cast<quint16>(disablePendingMask_ & ~it->servoMask));
            }
            pending_.erase(it);
            ++monitor_.timeoutCount;
            monitor_.ackStatus = QStringLiteral("ACK timeout seq=%1").arg(sequence);
            emit logMessage(QStringLiteral("ACK timeout after %1 retries for message 0x%2 seq=%3")
                                .arg(config_.maxRetries)
                                .arg(static_cast<quint8>(timedOutType), 2, 16, QLatin1Char('0'))
                                .arg(sequence));
            if (timedOutType == MessageType::Heartbeat) {
                heartbeatReady_ = false;
                heartbeatDue_ = true;
                nextHeartbeatDueAtMs_ = now;
                nextHeartbeatSafetyDeadlineAtMs_ = now;
                pumpApc220Scheduler();
                updateMonitor();
                return;
            }
            pumpApc220Scheduler();
            updateMonitor();
            return;
        }

        if (it->type != MessageType::Heartbeat && !priorityCommandQueue_.isEmpty()) {
            const PendingRequest request = it.value();
            bool supersededByDisable = false;
            if (isActuatorCommand(request.type)) {
                for (const QueuedCommand &priority : priorityCommandQueue_) {
                    if (priority.type == MessageType::ServoDisable
                        && (priority.affectedMask & request.servoMask) != 0U) {
                        supersededByDisable = true;
                        break;
                    }
                }
            }
            pending_.erase(it);
            if (!supersededByDisable) {
                deferredRetry_ = request;
            }
            // A safety-priority Disable must not wait behind an ordinary
            // retry.  If it affects this request, discard the retry entirely;
            // otherwise preserve it for after the Disable exchange.
            pumpApc220Scheduler();
            updateMonitor();
            return;
        }

        if (it->type != MessageType::Heartbeat && heartbeatDue_) {
            deferredRetry_ = it.value();
            pending_.erase(it);
            pumpApc220Scheduler();
            updateMonitor();
            return;
        }

        if (it->type != MessageType::Heartbeat && !canStartApc220OrdinaryExchange()) {
            deferredRetry_ = it.value();
            pending_.erase(it);
            heartbeatDue_ = true;
            pumpApc220Scheduler();
            updateMonitor();
            return;
        }

        const quint16 sequence = it->sequence;
        dispatchApc220Retry(sequence);
        updateMonitor();
        return;
    }

    const QList<quint16> sequences = pending_.keys();
    for (quint16 sequence : sequences) {
        auto it = pending_.find(sequence);
        if (it == pending_.end() || now - it->sentAtMs < config_.ackTimeoutMs) {
            continue;
        }
        if (it->type == MessageType::Heartbeat) {
            failClosedMotionState();
        }
        if (it->retries >= config_.maxRetries) {
            const MessageType timedOutType = it->type;
            if (timedOutType == MessageType::ServoDisable) {
                setDisablePendingMask(
                    static_cast<quint16>(disablePendingMask_ & ~it->servoMask));
            }
            pending_.erase(it);
            ++monitor_.timeoutCount;
            monitor_.ackStatus = QStringLiteral("ACK timeout seq=%1").arg(sequence);
            emit logMessage(QStringLiteral("ACK timeout after %1 retries for message 0x%2 seq=%3")
                                .arg(config_.maxRetries)
                                .arg(static_cast<quint8>(timedOutType), 2, 16, QLatin1Char('0'))
                                .arg(sequence));
            continue;
        }
        if (transport_->write(it->frame)) {
            ++it->retries;
            it->sentAtMs = now;
            ++monitor_.txPacketCount;
            emit txHexChanged(QString::fromLatin1(it->frame.toHex(' ').toUpper()));
            emit logMessage(QStringLiteral("Retry %1/%2 seq=%3")
                                .arg(it->retries)
                                .arg(config_.maxRetries)
                                .arg(sequence));
        } else {
            noteWriteFailure(QStringLiteral("retry seq=%1").arg(sequence));
            it->sentAtMs = now;
        }
    }
    updateMonitor();
}

void RobotController::updateMonitor()
{
    emit protocolMonitorChanged(monitor_);
}

void RobotController::resetSchedulerState()
{
    motionStopTimer_.stop();
    pending_.clear();
    priorityCommandQueue_.clear();
    commandQueue_.clear();
    deferredRetry_.reset();
    heartbeatDue_ = false;
    heartbeatReady_ = false;
    actuatorFailClosed_ = false;
    nextHeartbeatDueAtMs_ = 0;
    nextHeartbeatSafetyDeadlineAtMs_ = 0;
    decoder_.reset();
    setEnabledMask(0);
    setDisablePendingMask(0);
    lastLeakTelemetryAtMs_ = -1;
    markApc220LivenessLost();
}

void RobotController::clearQueuedCommandsForDisable(quint16 affectedMask)
{
    const auto shouldDrop = [affectedMask](const QueuedCommand &command) {
        return (command.affectedMask & affectedMask) != 0U
            && isActuatorCommand(command.type);
    };

    QQueue<QueuedCommand> retainedPriority;
    while (!priorityCommandQueue_.isEmpty()) {
        const QueuedCommand command = priorityCommandQueue_.dequeue();
        if (!shouldDrop(command)) {
            retainedPriority.enqueue(command);
        }
    }
    priorityCommandQueue_ = std::move(retainedPriority);

    QQueue<QueuedCommand> retainedOrdinary;
    while (!commandQueue_.isEmpty()) {
        const QueuedCommand command = commandQueue_.dequeue();
        if (!shouldDrop(command)) {
            retainedOrdinary.enqueue(command);
        }
    }
    commandQueue_ = std::move(retainedOrdinary);

    if (deferredRetry_.has_value()
        && (deferredRetry_->servoMask & affectedMask) != 0U
        && isActuatorCommand(deferredRetry_->type)) {
        deferredRetry_.reset();
    }
}

void RobotController::failClosedApc220Actuators()
{
    const bool firstFailClosed = !actuatorFailClosed_;
    actuatorFailClosed_ = true;
    priorityCommandQueue_.clear();
    commandQueue_.clear();
    deferredRetry_.reset();
    setEnabledMask(0);
    setDisablePendingMask(0);
    markApc220LivenessLost();
    failClosedMotionState();
    if (firstFailClosed) {
        emit logMessage(QStringLiteral(
            "Conservative host-link liveness lost; logical Servo state and stale actuator commands were cleared"));
    }
}

void RobotController::refreshLeakTelemetryStaleness(qint64 now)
{
    if (!isConnected() || lastLeakTelemetryAtMs_ < 0
        || config_.leakTelemetryStaleTimeoutMs <= 0
        || leakState_ == LeakState::Unknown) {
        return;
    }

    if (now - lastLeakTelemetryAtMs_ >= config_.leakTelemetryStaleTimeoutMs) {
        emit logMessage(QStringLiteral("Leak telemetry stale; state set to Unknown"));
        setLeakState(LeakState::Unknown);
    }
}

void RobotController::setEnabledMask(quint16 mask)
{
    const quint16 changed = static_cast<quint16>(enabledMask_ ^ mask);
    enabledMask_ = mask;
    for (int index = 0; index < kServoCount; ++index) {
        const quint16 bit = static_cast<quint16>(1U << index);
        if ((changed & bit) != 0U) {
            emit servoStateChanged(index, (enabledMask_ & bit) != 0U);
        }
    }
}

void RobotController::setDisablePendingMask(quint16 mask)
{
    const quint16 changed = static_cast<quint16>(disablePendingMask_ ^ mask);
    disablePendingMask_ = mask;
    for (int index = 0; index < kServoCount; ++index) {
        const quint16 bit = static_cast<quint16>(1U << index);
        if ((changed & bit) != 0U) {
            emit servoDisablePendingChanged(index, (disablePendingMask_ & bit) != 0U);
        }
    }
}

void RobotController::setLeakState(LeakState state)
{
    if (leakState_ == state) {
        return;
    }
    leakState_ = state;
    emit leakStateChanged(leakState_);
}

void RobotController::failClosedMotionState()
{
    motionStopTimer_.stop();
    if (isMotionActive()) {
        setMotionState(MotionState::Faulted, MotionMode::Stop);
    }
}

void RobotController::setMotionState(MotionState state, MotionMode mode)
{
    if (motionState_ == state && motionMode_ == mode) {
        return;
    }
    motionState_ = state;
    motionMode_ = mode;
    emit motionStateChanged(motionState_, motionMode_);
}

void RobotController::markApc220LivenessLost()
{
    setLeakState(LeakState::Unknown);
    imuMonitor_.handleLivenessLost();
    depthMonitor_.handleLivenessLost();
}

void RobotController::noteWriteFailure(const QString &context)
{
    emit logMessage(QStringLiteral("TX failed: %1").arg(context));
}

bool RobotController::rejectUnsupportedServo(ServoId id, const QString &command)
{
    if (isServoSupported(id)) {
        return false;
    }
    const ServoDescriptor *descriptor = servoDescriptor(id);
    const QString name = descriptor == nullptr
        ? QStringLiteral("unknown servo")
        : QString::fromLatin1(descriptor->semanticName);
    emit logMessage(QStringLiteral("%1 rejected: %2 is unsupported")
                        .arg(command, name));
    return true;
}

QByteArray RobotController::maskPayload(quint16 mask)
{
    QByteArray payload;
    appendLe16(payload, mask);
    return payload;
}

qint64 RobotController::nowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

bool RobotController::isMotionCommand(MessageType type)
{
    return isMotionMessage(type);
}

bool RobotController::isActuatorCommand(MessageType type)
{
    return isServoActuatorCommand(type) || isMotionCommand(type);
}

} // namespace rb
