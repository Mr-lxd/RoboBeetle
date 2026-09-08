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
    if (!sendCommand(MessageType::ServoDisable, maskPayload(mask), mask)) {
        return false;
    }
    setDisablePendingMask(static_cast<quint16>(disablePendingMask_ | mask));
    return true;
}

bool RobotController::disableAll()
{
    const quint16 mask = config_.supportedServoMask;
    if ((disablePendingMask_ & mask) != 0U) {
        emit logMessage(QStringLiteral("Disable All already awaiting ACK"));
        return false;
    }
    if (!sendCommand(MessageType::ServoDisable, maskPayload(mask), mask)) {
        return false;
    }
    setDisablePendingMask(static_cast<quint16>(disablePendingMask_ | mask));
    return true;
}

bool RobotController::setServoPwm(ServoId id, quint16 pulseUs)
{
    if (rejectUnsupportedServo(id, QStringLiteral("Set PWM"))) {
        return false;
    }
    if (!isServoEnabled(id)) {
        emit logMessage(QStringLiteral("Set PWM rejected: servo is not enabled and acknowledged"));
        return false;
    }
    if (pulseUs < config_.provisionalPwmMinUs || pulseUs > config_.provisionalPwmMaxUs) {
        emit logMessage(QStringLiteral("Set PWM rejected: %1 us is outside bring-up provisional range %2-%3 us")
                            .arg(pulseUs)
                            .arg(config_.provisionalPwmMinUs)
                            .arg(config_.provisionalPwmMaxUs));
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
    if (!isServoEnabled(id)) {
        emit logMessage(QStringLiteral("Set Angle rejected: servo is not enabled and acknowledged"));
        return false;
    }
    if (isServoDisablePending(id)) {
        emit logMessage(QStringLiteral("Set Angle rejected: servo disable is awaiting ACK"));
        return false;
    }
    if (angleCentidegrees < config_.provisionalAngleMinCdeg
        || angleCentidegrees > config_.provisionalAngleMaxCdeg) {
        emit logMessage(QStringLiteral("Set Angle rejected: %1 cdeg is outside provisional range %2-%3 cdeg")
                            .arg(angleCentidegrees)
                            .arg(config_.provisionalAngleMinCdeg)
                            .arg(config_.provisionalAngleMaxCdeg));
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
    return (enabledMask_ & servoMask(id)) != 0U;
}

bool RobotController::isServoDisablePending(ServoId id) const
{
    return (disablePendingMask_ & servoMask(id)) != 0U;
}

bool RobotController::isServoSupported(ServoId id) const
{
    return (config_.supportedServoMask & servoMask(id)) != 0U;
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
        const QueuedCommand command{type, payload, affectedMask};
        if (!heartbeatReady_ || !pending_.isEmpty() || heartbeatDue_ || deferredRetry_.has_value()
            || !commandQueue_.isEmpty()) {
            if (commandQueue_.size() >= kApc220CommandQueueCapacity) {
                emit logMessage(QStringLiteral(
                    "Command rejected: APC220 command queue is full (%1)")
                                    .arg(kApc220CommandQueueCapacity));
                return false;
            }
            commandQueue_.enqueue(command);
            monitor_.ackStatus = QStringLiteral("Queued message 0x%1 (%2/%3)")
                                     .arg(static_cast<quint8>(type), 2, 16, QLatin1Char('0'))
                                     .arg(commandQueue_.size())
                                     .arg(kApc220CommandQueueCapacity);
            updateMonitor();
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
    if (config_.linkProfile != LinkProfile::Apc220HalfDuplex || !isConnected()
        || !pending_.isEmpty()) {
        return;
    }

    if (!heartbeatReady_) {
        if (!heartbeatDue_) {
            return;
        }
        QByteArray payload;
        const quint32 uptime = static_cast<quint32>(QDateTime::currentMSecsSinceEpoch()
                                                     & 0xffffffffU);
        payload.append(static_cast<char>(uptime & 0xffU));
        payload.append(static_cast<char>((uptime >> 8U) & 0xffU));
        payload.append(static_cast<char>((uptime >> 16U) & 0xffU));
        payload.append(static_cast<char>((uptime >> 24U) & 0xffU));
        if (dispatchApc220Command({MessageType::Heartbeat, payload, 0})) {
            if (heartbeatReady_ || !pending_.isEmpty()) {
                heartbeatDue_ = false;
                nextHeartbeatDueAtMs_ = nowMs() + config_.heartbeatIntervalMs;
            } else {
                // A synchronous rejected/mismatched ACK consumed the request
                // during write(); keep the liveness heartbeat due.
                heartbeatDue_ = true;
                nextHeartbeatDueAtMs_ = nowMs();
            }
        }
        return;
    }

    if (heartbeatDue_) {
        QByteArray payload;
        const quint32 uptime = static_cast<quint32>(QDateTime::currentMSecsSinceEpoch()
                                                     & 0xffffffffU);
        payload.append(static_cast<char>(uptime & 0xffU));
        payload.append(static_cast<char>((uptime >> 8U) & 0xffU));
        payload.append(static_cast<char>((uptime >> 16U) & 0xffU));
        payload.append(static_cast<char>((uptime >> 24U) & 0xffU));
        if (dispatchApc220Command({MessageType::Heartbeat, payload, 0})) {
            if (heartbeatReady_ || !pending_.isEmpty()) {
                heartbeatDue_ = false;
                nextHeartbeatDueAtMs_ = nowMs() + config_.heartbeatIntervalMs;
            } else {
                // A synchronous rejected/mismatched ACK consumed the request
                // during write(); keep the liveness heartbeat due.
                heartbeatDue_ = true;
                nextHeartbeatDueAtMs_ = nowMs();
            }
        }
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
        heartbeatReady_ = true;
        heartbeatDue_ = false;
        nextHeartbeatDueAtMs_ = nowMs() + config_.heartbeatIntervalMs;
    } else if (request->type == MessageType::ServoEnable) {
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
        if (nextHeartbeatDueAtMs_ > 0 && now >= nextHeartbeatDueAtMs_) {
            heartbeatDue_ = true;
        }
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
                pumpApc220Scheduler();
                updateMonitor();
                return;
            }
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
    commandQueue_.clear();
    deferredRetry_.reset();
    heartbeatDue_ = false;
    heartbeatReady_ = false;
    nextHeartbeatDueAtMs_ = 0;
    decoder_.reset();
    setEnabledMask(0);
    setDisablePendingMask(0);
}

void RobotController::setEnabledMask(quint16 mask)
{
    const quint16 changed = static_cast<quint16>(enabledMask_ ^ mask);
    enabledMask_ = mask;
    for (int index = 0; index < 2; ++index) {
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
    for (int index = 0; index < 2; ++index) {
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
    emit logMessage(QStringLiteral("%1 rejected: servo %2 is unsupported in Phase 1")
                        .arg(command)
                        .arg(static_cast<quint8>(id)));
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
