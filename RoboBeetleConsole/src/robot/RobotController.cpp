#include "robot/RobotController.h"

#include "protocol/PacketCodec.h"

#include <QDateTime>

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
    });
    connect(transport_, &ITransport::stateChanged, this, [this](TransportState state) {
        const bool wasConnected = state_ == TransportState::Connected;
        state_ = state;
        if (state == TransportState::Connected) {
            heartbeatTimer_.start();
            retryTimer_.start();
            monitor_.ackStatus = QStringLiteral("Connected; awaiting commands");
        } else if (state == TransportState::Disconnected || state == TransportState::Error) {
            heartbeatTimer_.stop();
            retryTimer_.stop();
            pending_.clear();
            decoder_.reset();
            setEnabledMask(0);
            setDisablePendingMask(0);
            monitor_.ackStatus = state == TransportState::Error
                ? QStringLiteral("Transport error")
                : QStringLiteral("Disconnected");
            if (state == TransportState::Error && wasConnected) {
                emit logMessage(QStringLiteral(
                    "Safety notice: transport was lost; Disable All could not be delivered"));
            }
        }
        updateMonitor();
        emit connectionStateChanged(state);
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
    setEnabledMask(0);
    setDisablePendingMask(0);
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
        pending_.insert(sequence, {frame, type, affectedMask, nowMs(), 0});
        monitor_.ackStatus = QStringLiteral("Waiting for ACK seq=%1").arg(sequence);
    }
    updateMonitor();
    return true;
}

void RobotController::sendHeartbeat()
{
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
            const auto it = pending_.find(sequence);
            if (it != pending_.end()) {
                if (it->type == MessageType::ServoDisable) {
                    setDisablePendingMask(
                        static_cast<quint16>(disablePendingMask_ & ~it->servoMask));
                }
                pending_.erase(it);
            }
            const quint16 errorCode = readLe16(packet.payload, 3);
            monitor_.ackStatus = QStringLiteral("Error seq=%1 code=%2").arg(sequence).arg(errorCode);
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
    if (!pending_.contains(requestSequence)) {
        emit logMessage(QStringLiteral("Unmatched ACK seq=%1").arg(requestSequence));
        return;
    }

    const PendingRequest request = pending_.take(requestSequence);
    const auto clearDisablePending = [this, &request] {
        if (request.type == MessageType::ServoDisable) {
            setDisablePendingMask(
                static_cast<quint16>(disablePendingMask_ & ~request.servoMask));
        }
    };
    if (request.type != requestType) {
        clearDisablePending();
        monitor_.ackStatus = QStringLiteral("ACK type mismatch seq=%1").arg(requestSequence);
        emit logMessage(monitor_.ackStatus);
        return;
    }
    if (result != static_cast<quint8>(AckResult::Ok)) {
        clearDisablePending();
        monitor_.ackStatus = QStringLiteral("ACK rejected seq=%1 result=%2 (%3)")
                                 .arg(requestSequence)
                                 .arg(result)
                                 .arg(ackResultText(result));
        return;
    }

    if (request.type == MessageType::ServoEnable) {
        setEnabledMask(static_cast<quint16>(enabledMask_ | request.servoMask));
    } else if (request.type == MessageType::ServoDisable) {
        clearDisablePending();
        setEnabledMask(static_cast<quint16>(enabledMask_ & ~request.servoMask));
    }
    monitor_.ackStatus = QStringLiteral("ACK seq=%1").arg(requestSequence);
}

void RobotController::checkTimeouts()
{
    const qint64 now = nowMs();
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
    return QDateTime::currentMSecsSinceEpoch();
}

} // namespace rb
