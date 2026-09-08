#include "robot/RobotController.h"

#include "protocol/PacketCodec.h"

#include <QDateTime>

#include <chrono>

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
        return false;
    }
    return false;
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
      portDiscovery_(std::move(portDiscovery))
{
    Q_ASSERT(transport_ != nullptr);
    heartbeatTimer_.setInterval(config_.heartbeatIntervalMs);
    retryTimer_.setInterval(qMax(10, config_.ackTimeoutMs / 4));

    connect(&heartbeatTimer_, &QTimer::timeout, this, &RobotController::sendHeartbeat);
    connect(&retryTimer_, &QTimer::timeout, this, &RobotController::checkTimeouts);
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
        if (state == TransportState::Connected) {
            resetSchedulerState();
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
    if (rejectUnsupportedServo(id, QStringLiteral("Neutral"))) {
        return false;
    }
    if (!isServoEnabled(id)) {
        emit logMessage(QStringLiteral("Neutral rejected: servo is not enabled and acknowledged"));
        return false;
    }
    return sendCommand(MessageType::Neutral, maskPayload(servoMask(id)), servoMask(id));
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
                                  bool expectAck)
{
    if (!isConnected()) {
        emit logMessage(QStringLiteral("Command rejected: transport is not connected"));
        return false;
    }

    if (config_.linkProfile == LinkProfile::Apc220HalfDuplex && expectAck) {
        if (type == MessageType::ServoEnable && actuatorFailClosed_ && !heartbeatReady_) {
            monitor_.ackStatus = QStringLiteral(
                "APC220 liveness recovering; Enable rejected");
            emit logMessage(monitor_.ackStatus);
            updateMonitor();
            return false;
        }

        refreshApc220HeartbeatDue();
        const QueuedCommand command{type, payload, affectedMask};
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
                    "Command rejected: APC220 command queue is full (%1)")
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
        pending_.insert(sequence, {sequence, frame, type, affectedMask, nowMs(), 0});
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
                    {sequence, frame, command.type, command.affectedMask, nowMs(), 0});
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
                "APC220 priority command dropped after write failure (message 0x%1)")
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
                "APC220 command dropped after write failure (message 0x%1)")
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
    // interval in addition to the configured ACK timeout.  The APC220
    // profile's 490 ms hard budget therefore leaves an explicit 10 ms below
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
            if (isServoActuatorCommand(request.type)) {
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
}

void RobotController::clearQueuedCommandsForDisable(quint16 affectedMask)
{
    const auto shouldDrop = [affectedMask](const QueuedCommand &command) {
        return (command.affectedMask & affectedMask) != 0U
            && isServoActuatorCommand(command.type);
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
        && isServoActuatorCommand(deferredRetry_->type)) {
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
    if (firstFailClosed) {
        emit logMessage(QStringLiteral(
            "APC220 liveness lost; logical Servo state and stale actuator commands were cleared"));
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

} // namespace rb
