#include "remote/RemoteRobotController.h"

#include "robot/DepthSnapshot.h"
#include "robot/ImuSnapshot.h"
#include "robot/ServoDescriptor.h"

#include <QByteArrayView>
#include <QDateTime>

#include <cstdint>

namespace rb {
namespace {

constexpr qint64 kRemoteCommandTimeoutMs = 2000;

quint16 readLe16(QByteArrayView payload, qsizetype offset)
{
    return static_cast<quint16>(
        static_cast<quint8>(payload[offset])
        | (static_cast<quint16>(static_cast<quint8>(payload[offset + 1])) << 8U));
}

quint32 readLe32(QByteArrayView payload, qsizetype offset)
{
    return static_cast<quint32>(static_cast<quint8>(payload[offset]))
        | (static_cast<quint32>(static_cast<quint8>(payload[offset + 1])) << 8U)
        | (static_cast<quint32>(static_cast<quint8>(payload[offset + 2])) << 16U)
        | (static_cast<quint32>(static_cast<quint8>(payload[offset + 3])) << 24U);
}

void appendLe16(QByteArray &payload, quint16 value)
{
    payload.append(static_cast<char>(value & 0xffU));
    payload.append(static_cast<char>((value >> 8U) & 0xffU));
}

QString commandKindText(robobeetle::gateway::RobotCommandKind kind)
{
    using Kind = robobeetle::gateway::RobotCommandKind;
    switch (kind) {
    case Kind::EnableServos: return QStringLiteral("EnableServos");
    case Kind::DisableServos: return QStringLiteral("DisableServos");
    case Kind::SetServoAngle: return QStringLiteral("SetServoAngle");
    case Kind::SetServoPwm: return QStringLiteral("SetServoPwm");
    case Kind::NeutralServos: return QStringLiteral("NeutralServos");
    case Kind::StartMotion: return QStringLiteral("StartMotion");
    case Kind::StopMotion: return QStringLiteral("StopMotion");
    case Kind::SetGaitBackend: return QStringLiteral("SetGaitBackend");
    case Kind::SetFrontRearCoordination:
        return QStringLiteral("SetFrontRearCoordination");
    }
    return QStringLiteral("UnknownCommand");
}

} // namespace

RemoteRobotController::RemoteRobotController(QObject *parent, TerminalNowMs terminalNow)
    : IConsoleController(parent),
      session_(this), terminalNow_(std::move(terminalNow))
{
    terminalClock_.start();
    connect(&motionCsvLogger_, &MotionStateCsvLogger::failed,
            this, &RemoteRobotController::motionRecordingFailed);
    if (!terminalNow_) terminalNow_ = [this] { return terminalClock_.elapsed(); };
    motionStopTimer_.setSingleShot(true);
    motionModeTransitionTimer_.setSingleShot(true);
    telemetryTimer_.setInterval(500);

    connect(&motionStopTimer_, &QTimer::timeout, this, [this] {
        if (motionState_ == MotionState::Stopping) {
            setMotionState(MotionState::Stopped, MotionMode::Stop);
        }
    });
    connect(&motionModeTransitionTimer_, &QTimer::timeout, this, [this] {
        if (motionState_ == MotionState::Running) {
            emit motionStateChanged(motionState_, motionMode_);
        }
    });
    connect(&telemetryTimer_, &QTimer::timeout,
            this, &RemoteRobotController::refreshTelemetryStaleness);
    telemetryTimer_.start();

    connect(&session_, &RbrpClientSession::connectionStateChanged,
            this, [this](TransportState state) {
        emit connectionStateChanged(state);
        if (state == TransportState::Closing
            || state == TransportState::Disconnected
            || state == TransportState::Error) {
            failClosedControlState(
                state == TransportState::Error
                    ? QStringLiteral("remote transport error")
                    : QStringLiteral("remote transport closed"));
            resetTelemetry();
        }
    });
    connect(&session_, &RbrpClientSession::sessionStateChanged,
            this, [this](RemoteSessionState) {
        emit controlAvailabilityChanged();
    });
    connect(&session_, &RbrpClientSession::authorityStateChanged,
            this, [this](ControlAuthorityState state, bool active) {
        const bool expectedUserRelease = userReleasePending_
            && state == ControlAuthorityState::Unowned && !active;
        if (expectedUserRelease) {
            failClosedControlState({});
            userReleasePending_ = false;
            emit logMessage(QStringLiteral("Remote control released"));
        } else if ((wasControlActive_ && !active)
                   || (state == ControlAuthorityState::Unowned
                       && !pending_.isEmpty())) {
            failClosedControlState(QStringLiteral("remote authority/link lost"));
        }
        wasControlActive_ = active;
        emit controlAvailabilityChanged();
        emit authorityStateChanged(state, active);
        updateMonitor(authorityText(state, active));
    });
    connect(&session_, &RbrpClientSession::frameSent,
            this, &RemoteRobotController::noteTxFrame);
    connect(&session_, &RbrpClientSession::frameReceived,
            this, [this](quint8 kind, quint32 requestId,
                         const QByteArray &payload) {
        noteRxFrame(kind, requestId, payload);
        handleFrame(kind, requestId, payload);
    });
    connect(&session_, &RbrpClientSession::protocolError,
            this, [this](const QString &message) {
        ++monitor_.timeoutCount;
        updateMonitor(QStringLiteral("RBRP error"));
        emit logMessage(message);
    });
    connect(&session_, &RbrpClientSession::logMessage,
            this, &RemoteRobotController::logMessage);

    updateMonitor(QStringLiteral("Disconnected"));
}

RemoteRobotController::~RemoteRobotController()
{
    // Socket closure can synchronously emit authority loss. Handle it while
    // all controller fields exist, then fence signals during member teardown.
    session_.disconnectFromHost();
    disconnect(&session_, nullptr, this, nullptr);
}

void RemoteRobotController::refreshSerialPorts()
{
    emit serialPortsChanged({});
}

void RemoteRobotController::connectController(
    const ConsoleConnectionConfiguration &configuration)
{
    session_.connectToHost(configuration.endpoint, configuration.tcpPort);
}

void RemoteRobotController::disconnectController()
{
    session_.disconnectFromHost();
}

void RemoteRobotController::shutdown()
{
    if (session_.authorityState() == ControlAuthorityState::Owned) {
        (void)session_.releaseControl();
    }
    session_.disconnectFromHost();
}

bool RemoteRobotController::acquireControl()
{
    return session_.acquireControl();
}

bool RemoteRobotController::releaseControl()
{
    userReleasePending_ = true;
    const bool released = session_.releaseControl();
    if (!released) {
        userReleasePending_ = false;
    }
    return released;
}

bool RemoteRobotController::enableServo(ServoId id)
{
    if (!isControlActive() || !isServoSupported(id) || isMotionActive()) {
        return false;
    }
    const quint16 mask = servoMask(id);
    if ((enabledMask_ & mask) != 0U) {
        return true;
    }

    PendingCommand pending;
    pending.kind = robobeetle::gateway::RobotCommandKind::EnableServos;
    pending.servoMask = mask;
    return submitCommand(pending.kind, maskPayload(mask), pending).has_value();
}

bool RemoteRobotController::disableServo(ServoId id)
{
    if (!isControlActive() || !isServoSupported(id)) {
        return false;
    }
    const quint16 mask = servoMask(id);
    PendingCommand pending;
    pending.kind = robobeetle::gateway::RobotCommandKind::DisableServos;
    pending.servoMask = mask;
    const auto requestId = submitCommand(pending.kind, maskPayload(mask), pending);
    if (!requestId.has_value()) {
        return false;
    }
    supersedePendingForDisable(mask);
    setDisablePendingMask(static_cast<quint16>(disablePendingMask_ | mask));
    return true;
}

bool RemoteRobotController::disableAll()
{
    if (!isControlActive()) {
        return false;
    }
    PendingCommand pending;
    pending.kind = robobeetle::gateway::RobotCommandKind::DisableServos;
    pending.servoMask = SupportedServoMask;
    const auto requestId =
        submitCommand(pending.kind, maskPayload(SupportedServoMask), pending);
    if (!requestId.has_value()) {
        return false;
    }
    supersedePendingForDisable(SupportedServoMask);
    setDisablePendingMask(SupportedServoMask);
    return true;
}

bool RemoteRobotController::setServoPwm(ServoId id, quint16 pulseUs)
{
    if (!isControlActive() || !isServoSupported(id) || !isServoEnabled(id)
        || isServoDisablePending(id) || isMotionActive()) {
        return false;
    }
    const ServoDescriptor *descriptor = servoDescriptor(id);
    if (descriptor == nullptr
        || pulseUs < descriptor->commandMinPwmUs
        || pulseUs > descriptor->commandMaxPwmUs) {
        emit logMessage(QStringLiteral(
            "Set PWM rejected: %1 us is outside the manual PWM envelope")
                            .arg(pulseUs));
        return false;
    }

    QByteArray payload;
    payload.append(static_cast<char>(id));
    appendLe16(payload, pulseUs);
    PendingCommand pending;
    pending.kind = robobeetle::gateway::RobotCommandKind::SetServoPwm;
    pending.servoMask = servoMask(id);
    return submitCommand(pending.kind, payload, pending).has_value();
}

bool RemoteRobotController::setServoAngle(ServoId id, qint16 angleCentidegrees)
{
    if (!isControlActive() || !isServoSupported(id) || !isServoEnabled(id)
        || isServoDisablePending(id) || isMotionActive()) {
        return false;
    }

    QByteArray payload;
    payload.append(static_cast<char>(id));
    appendLe16(payload, static_cast<quint16>(angleCentidegrees));
    PendingCommand pending;
    pending.kind = robobeetle::gateway::RobotCommandKind::SetServoAngle;
    pending.servoMask = servoMask(id);
    return submitCommand(pending.kind, payload, pending).has_value();
}

bool RemoteRobotController::neutralServo(ServoId id)
{
    if (!isControlActive() || !isServoSupported(id) || !isServoEnabled(id)
        || isServoDisablePending(id) || isMotionActive()) {
        return false;
    }
    PendingCommand pending;
    pending.kind = robobeetle::gateway::RobotCommandKind::NeutralServos;
    pending.servoMask = servoMask(id);
    return submitCommand(pending.kind, maskPayload(pending.servoMask), pending)
        .has_value();
}

bool RemoteRobotController::startMotion(MotionMode mode)
{
    if (!isMotionReady(mode) || mode == MotionMode::Backward
        || mode == MotionMode::Stop || mode == MotionMode::Count) {
        return false;
    }
    QByteArray payload(1, static_cast<char>(mode));
    PendingCommand pending;
    pending.kind = robobeetle::gateway::RobotCommandKind::StartMotion;
    pending.motionMode = mode;
    return submitCommand(pending.kind, payload, pending).has_value();
}

std::optional<quint32> RemoteRobotController::submitVisualMotion(MotionMode mode)
{
    if (!isControlActive() || (mode != MotionMode::Stop && !isMotionReady(mode))) {
        return std::nullopt;
    }
    if (mode != MotionMode::Stop && mode != MotionMode::Forward
        && mode != MotionMode::TurnLeft && mode != MotionMode::TurnRight
        && mode != MotionMode::Ascend && mode != MotionMode::Descend) {
        return std::nullopt;
    }
    PendingCommand pending;
    pending.kind = mode == MotionMode::Stop
        ? robobeetle::gateway::RobotCommandKind::StopMotion
        : robobeetle::gateway::RobotCommandKind::StartMotion;
    if (mode != MotionMode::Stop) pending.motionMode = mode;
    const auto id = submitCommand(pending.kind,
        mode == MotionMode::Stop ? QByteArray{} : QByteArray(1, static_cast<char>(mode)),
        pending);
    if (id && mode == MotionMode::Stop) {
        supersedePendingMotionStarts();
        motionModeTransitionTimer_.stop();
    }
    return id;
}

bool RemoteRobotController::stopMotion()
{
    if (!isControlActive()) {
        return false;
    }
    for (auto it = pending_.cbegin(); it != pending_.cend(); ++it) {
        if (!it->superseded
            && it->kind == robobeetle::gateway::RobotCommandKind::StopMotion) {
            return true;
        }
    }
    if (!isMotionActive()) {
        return false;
    }

    PendingCommand pending;
    pending.kind = robobeetle::gateway::RobotCommandKind::StopMotion;
    const auto requestId = submitCommand(pending.kind, {}, pending);
    if (!requestId.has_value()) {
        return false;
    }
    supersedePendingMotionStarts();
    motionModeTransitionTimer_.stop();
    return true;
}

bool RemoteRobotController::setGaitBackend(GaitBackend backend)
{
    if (!isControlActive() || !isValidGaitBackend(backend)
        || pendingGaitBackend_.has_value()
        || pendingFrontRearCoordination_.has_value()
        || isMotionActive()) {
        return false;
    }
    QByteArray payload(1, static_cast<char>(backend));
    PendingCommand pending;
    pending.kind = robobeetle::gateway::RobotCommandKind::SetGaitBackend;
    pending.gaitBackend = backend;
    const auto requestId = submitCommand(pending.kind, payload, pending);
    if (!requestId.has_value()) {
        return false;
    }
    pendingGaitBackend_ = backend;
    emit gaitBackendStateChanged();
    return true;
}

bool RemoteRobotController::setFrontRearCoordination(
    FrontRearCoordination coordination)
{
    if (!isControlActive() || !isValidFrontRearCoordination(coordination)
        || pendingGaitBackend_.has_value()
        || pendingFrontRearCoordination_.has_value()
        || isMotionActive()) {
        return false;
    }
    const QByteArray payload(1, static_cast<char>(coordination));
    PendingCommand pending;
    pending.kind = robobeetle::gateway::RobotCommandKind::SetFrontRearCoordination;
    pending.frontRearCoordination = coordination;
    const auto requestId = submitCommand(pending.kind, payload, pending);
    if (!requestId.has_value()) {
        return false;
    }
    pendingFrontRearCoordination_ = coordination;
    emit frontRearCoordinationStateChanged();
    return true;
}

bool RemoteRobotController::isServoSupported(ServoId id) const
{
    const quint8 raw = static_cast<quint8>(id);
    return raw < kServoCount && (SupportedServoMask & servoMask(id)) != 0U;
}

bool RemoteRobotController::isServoEnabled(ServoId id) const
{
    return isServoSupported(id) && (enabledMask_ & servoMask(id)) != 0U;
}

bool RemoteRobotController::isServoDisablePending(ServoId id) const
{
    return isServoSupported(id) && (disablePendingMask_ & servoMask(id)) != 0U;
}

bool RemoteRobotController::isMotionActive() const
{
    if (motionState_ == MotionState::Running
        || motionState_ == MotionState::Stopping) {
        return true;
    }
    for (auto it = pending_.cbegin(); it != pending_.cend(); ++it) {
        if (!it->superseded
            && (it->kind == robobeetle::gateway::RobotCommandKind::StartMotion
                || it->kind == robobeetle::gateway::RobotCommandKind::StopMotion)) {
            return true;
        }
    }
    return false;
}

bool RemoteRobotController::isMotionReady(MotionMode mode) const
{
    if (!isControlActive() || mode == MotionMode::Stop
        || mode == MotionMode::Backward || mode == MotionMode::Count
        || motionState_ == MotionState::Stopping
        || motionState_ == MotionState::Faulted
        || motionModeTransitionTimer_.isActive()) {
        return false;
    }
    for (auto it = pending_.cbegin(); it != pending_.cend(); ++it) {
        if (!it->superseded
            && (it->kind == robobeetle::gateway::RobotCommandKind::StartMotion
                || it->kind == robobeetle::gateway::RobotCommandKind::StopMotion)) {
            return false;
        }
    }
    const quint16 required = motionRequiredServoMask(mode);
    return (enabledMask_ & required) == required
        && (disablePendingMask_ & required) == 0U;
}

std::optional<quint32> RemoteRobotController::submitCommand(
    robobeetle::gateway::RobotCommandKind kind,
    const QByteArray &payload, PendingCommand pending)
{
    const auto requestId =
        session_.sendCommand(static_cast<quint8>(kind), payload);
    if (!requestId.has_value()) {
        emit logMessage(QStringLiteral("Command %1 rejected locally: remote control is not Active")
                            .arg(commandKindText(kind)));
        return std::nullopt;
    }
    if (pending.servoMask != 0) {
        for (auto it = pending_.begin(); it != pending_.end(); ++it)
            if ((it->servoMask & pending.servoMask) != 0) it->superseded = true;
        // No new pose is inferred before a correlated successful ACK.
        poseKnownMask_ &= ~pending.servoMask;
    }
    pending.sentAtMs = nowMs();
    pending.terminalSentMs = terminalNow_();
    pending_[*requestId] = pending;
    updateMonitor(QStringLiteral("%1 sent").arg(commandKindText(kind)));
    return requestId;
}

void RemoteRobotController::handleFrame(quint8 rawKind, quint32 requestId,
                                        const QByteArray &payload)
{
    using Kind = robobeetle::gateway::RbrpMessageKind;
    const auto kind = static_cast<Kind>(rawKind);
    switch (kind) {
    case Kind::HelloReply:
        updateMonitor(QStringLiteral("RBRP ready; control unowned"));
        break;
    case Kind::AcquireReply:
        updateMonitor(authorityText(authorityState(), isControlActive()));
        break;
    case Kind::ControlState:
        updateMonitor(authorityText(authorityState(), isControlActive()));
        break;
    case Kind::CommandSubmitted:
        handleCommandSubmitted(requestId, payload);
        break;
    case Kind::CommandOutcome:
        handleCommandOutcome(requestId, payload);
        break;
    case Kind::LeakTelemetry:
        handleLeakTelemetry(payload);
        break;
    case Kind::ImuTelemetry:
        handleImuTelemetry(payload);
        break;
    case Kind::DepthTelemetry:
        handleDepthTelemetry(payload);
        break;
    case Kind::MotionStateTelemetry: {
        const auto telemetry = decodeMotionStateTelemetry(payload);
        if (!telemetry) { emit logMessage(QStringLiteral("Invalid MotionStateTelemetry")); break; }
        if (motionMonitor_.linkEpoch() && *motionMonitor_.linkEpoch() != telemetry->link_epoch) {
            motionMonitor_.finishBatch();
            motionCsvLogger_.recordTotals(motionMonitor_);
        }
        for (const auto &record : motionMonitor_.accept(*telemetry)) motionCsvLogger_.record(record);
        emit motionTelemetryChanged();
        break;
    }
    case Kind::ServiceError:
        handleServiceError(requestId, payload);
        break;
    default:
        break;
    }
}

void RemoteRobotController::handleCommandSubmitted(
    quint32 requestId, const QByteArray &payload)
{
    auto it = pending_.find(requestId);
    if (it == pending_.end()) {
        emit logMessage(QStringLiteral(
            "Ignoring CommandSubmitted for non-current request %1").arg(requestId));
        return;
    }

    const quint8 status = static_cast<quint8>(payload[0]);
    const quint8 hasSequence = static_cast<quint8>(payload[1]);
    if (status > static_cast<quint8>(
                     robobeetle::gateway::CommandSubmittedStatus::TransportRejected)) {
        failClosedControlState(QStringLiteral(
            "invalid CommandSubmitted status from gateway"));
        session_.disconnectFromHost();
        return;
    }
    if (status == static_cast<quint8>(
                      robobeetle::gateway::CommandSubmittedStatus::Submitted)) {
        if (hasSequence != 1U) {
            failClosedControlState(QStringLiteral(
                "malformed CommandSubmitted from gateway"));
            session_.disconnectFromHost();
            return;
        }
        it->submittedSequence = readLe16(QByteArrayView(payload), 2);
        updateMonitor(QStringLiteral("%1 submitted")
                          .arg(commandKindText(it->kind)));
        return;
    }

    if (hasSequence != 0U) {
        failClosedControlState(QStringLiteral(
            "malformed rejected CommandSubmitted from gateway"));
        session_.disconnectFromHost();
        return;
    }

    terminalizePending(
        requestId,
        QStringLiteral("%1 not submitted (status %2)")
            .arg(commandKindText(it->kind)).arg(status),
        CommandTerminalResult::Rejected);
}

void RemoteRobotController::handleCommandOutcome(
    quint32 requestId, const QByteArray &payload)
{
    auto it = pending_.find(requestId);
    if (it == pending_.end()) {
        emit logMessage(QStringLiteral(
            "Ignoring stale CommandOutcome for request %1").arg(requestId));
        return;
    }

    const quint8 outcome = static_cast<quint8>(payload[0]);
    const auto kind = static_cast<robobeetle::gateway::RobotCommandKind>(
        static_cast<quint8>(payload[1]));
    const quint16 sequence = readLe16(QByteArrayView(payload), 2);
    if (kind != it->kind
        || !it->submittedSequence.has_value()
        || sequence != *it->submittedSequence
        || outcome > static_cast<quint8>(
                         robobeetle::gateway::GatewayCommandOutcome::Cancelled)) {
        failClosedControlState(QStringLiteral(
            "uncorrelated or malformed CommandOutcome"));
        session_.disconnectFromHost();
        return;
    }

    const PendingCommand pending = *it;
    const qint64 rtt = terminalNow_() - pending.terminalSentMs;
    monitor_.lastAckRttMs = rtt >= 0 ? rtt : -1;
    pending_.erase(it);

    const quint8 rawResult = static_cast<quint8>(payload[4]);
    CommandTerminalResult terminal = CommandTerminalResult::OutcomeUnknown;
    if (outcome == static_cast<quint8>(robobeetle::gateway::GatewayCommandOutcome::Accepted) && rawResult == 0)
        terminal = CommandTerminalResult::Ok;
    else if (outcome == static_cast<quint8>(robobeetle::gateway::GatewayCommandOutcome::Rejected))
        terminal = rawResult == 7 ? CommandTerminalResult::Busy : CommandTerminalResult::Rejected;
    if (!pending.superseded && terminal == CommandTerminalResult::Ok
        && pending.kind == robobeetle::gateway::RobotCommandKind::StopMotion) {
        // With STOP outstanding, readiness forbids a newer START. All existing
        // STOP requests therefore cover this same unresolved stop interval.
        // One successful STOP confirms it; unanswered duplicates must neither
        // block recovery nor later expire and release authority.
        QHash<quint32, PendingCommand> retiredStops;
        for (auto other = pending_.begin(); other != pending_.end();) {
            if (other->kind == robobeetle::gateway::RobotCommandKind::StopMotion) {
                retiredStops.insert(other.key(), other.value());
                other = pending_.erase(other);
            } else {
                ++other;
            }
        }
        // Erase the whole selected set before callbacks. A new operator STOP
        // submitted reentrantly belongs to a later interval and stays pending.
        for (auto retired = retiredStops.cbegin(); retired != retiredStops.cend(); ++retired) {
            // A confirmed STOP also covers these STOPs; 0xff means no individual ACK.
            emit commandTerminal(retired.key(), CommandTerminalResult::Ok,
                                 0xff, -1);
        }
    }
    emit commandTerminal(requestId, pending.superseded ? CommandTerminalResult::OutcomeUnknown : terminal, rawResult, monitor_.lastAckRttMs);
    if (pending.superseded) {
        updateMonitor(QStringLiteral("%1 superseded outcome ignored")
                          .arg(commandKindText(pending.kind)));
        emit logMessage(QStringLiteral(
            "Ignored late outcome for superseded %1 request %2")
                            .arg(commandKindText(pending.kind))
                            .arg(requestId));
        return;
    }

    if (outcome == static_cast<quint8>(
                       robobeetle::gateway::GatewayCommandOutcome::Accepted) && rawResult == 0) {
        applyAcceptedCommand(pending);
        updateMonitor(QStringLiteral("%1 accepted")
                          .arg(commandKindText(pending.kind)));
        return;
    }

    if (pending.servoMask != 0U
        && pending.kind == robobeetle::gateway::RobotCommandKind::DisableServos) {
        setDisablePendingMask(static_cast<quint16>(
            disablePendingMask_ & ~pending.servoMask));
    }
    if (pending.gaitBackend.has_value()) {
        pendingGaitBackend_.reset();
        emit gaitBackendStateChanged();
    }
    if (pending.frontRearCoordination.has_value()) {
        pendingFrontRearCoordination_.reset();
        emit frontRearCoordinationStateChanged();
    }

    const bool uncertain =
        outcome == static_cast<quint8>(
                       robobeetle::gateway::GatewayCommandOutcome::OutcomeUnknown)
        || outcome == static_cast<quint8>(
                         robobeetle::gateway::GatewayCommandOutcome::Cancelled);
    if (uncertain) {
        if (!session_.releaseControl()) {
            failClosedControlState(QStringLiteral(
                "command outcome became uncertain"));
        }
    } else if (pending.kind
                   == robobeetle::gateway::RobotCommandKind::StopMotion
               && isMotionActive()) {
        setMotionState(MotionState::Faulted, MotionMode::Stop);
    }
    updateMonitor(QStringLiteral("%1 outcome=%2 result=%3")
                      .arg(commandKindText(pending.kind))
                      .arg(outcome)
                      .arg(static_cast<quint8>(payload[4])));
}

void RemoteRobotController::handleLeakTelemetry(const QByteArray &payload)
{
    const quint8 raw = static_cast<quint8>(payload[2]);
    if (raw > static_cast<quint8>(LeakState::Wet)) {
        setLeakState(LeakState::Unknown);
        emit logMessage(QStringLiteral("Invalid LeakTelemetry state"));
        return;
    }
    lastLeakTelemetryAtMs_ = nowMs();
    setLeakState(static_cast<LeakState>(raw));
}

void RemoteRobotController::handleImuTelemetry(const QByteArray &payload)
{
    QString detail;
    const auto snapshot =
        ImuSnapshot::decodePayload(QByteArrayView(payload).sliced(2), &detail);
    if (!snapshot.has_value()) {
        imuState_.status = ImuStatus::Error;
        imuState_.snapshot.reset();
        imuState_.lastReceivedAtMs = -1;
        imuState_.error = detail;
        emit imuStateChanged();
        return;
    }
    imuState_.status = ImuStatus::Receiving;
    imuState_.snapshot = snapshot;
    imuState_.lastReceivedAtMs = nowMs();
    imuState_.error.clear();
    emit imuStateChanged();
}

void RemoteRobotController::handleDepthTelemetry(const QByteArray &payload)
{
    QString detail;
    const auto snapshot =
        DepthSnapshot::decodePayload(QByteArrayView(payload).sliced(2), &detail);
    if (!snapshot.has_value()) {
        depthState_.status = DepthStatus::Error;
        depthState_.snapshot.reset();
        depthState_.lastReceivedAtMs = -1;
        depthState_.error = detail;
        emit depthStateChanged();
        clearControlDepth();
        return;
    }

    const qint64 receivedAtMs = depthNowMs();
    const bool current = snapshot->depthValid()
        && snapshot->sampleAgeMs != DepthSnapshot::UnknownSampleAgeMs;
    depthState_.status = current ? DepthStatus::Receiving : DepthStatus::Stale;
    depthState_.snapshot = snapshot;
    depthState_.lastReceivedAtMs = receivedAtMs;
    depthState_.error = current
        ? QString{}
        : QStringLiteral("Depth sensor sample is stale or unavailable");
    emit depthStateChanged();

    ControlDepthLatest latest;
    latest.rawM = static_cast<double>(snapshot->depthMm) / 1000.0;
    latest.receivedAtMs = receivedAtMs;
    latest.sampleAgeMs = snapshot->sampleAgeMs == DepthSnapshot::UnknownSampleAgeMs
        ? -1
        : static_cast<qint64>(snapshot->sampleAgeMs);
    latest.depthValid = snapshot->depthValid();
    controlLatest_ = latest;

    if (current) {
        // Firmware repeats a frame (keepalive) without a new line: count a
        // sample only once, identified by the parser's valid-line counter.
        const quint32 line = snapshot->diagnostics.validLineCount;
        if (!lastZeroSampleLine_ || *lastZeroSampleLine_ != line) {
            lastZeroSampleLine_ = line;
            zeroSamples_.push_back({latest.rawM, receivedAtMs - latest.sampleAgeMs, line});
            const auto capacity = static_cast<std::size_t>(
                qMax(1, depthControlConfig_.zeroMinSamples));
            while (zeroSamples_.size() > capacity)
                zeroSamples_.pop_front();
        }
    } else {
        // A dropout breaks the run of consecutive samples used for zeroing.
        zeroSamples_.clear();
        lastZeroSampleLine_.reset();
    }
    emit controlDepthSampleChanged();
}

void RemoteRobotController::setDepthControlConfig(const DepthControlConfig &config)
{
    depthControlConfig_ = config;
    const auto capacity = static_cast<std::size_t>(qMax(1, config.zeroMinSamples));
    while (zeroSamples_.size() > capacity)
        zeroSamples_.pop_front();
    emit controlDepthSampleChanged();
}

std::optional<DepthControlSample> RemoteRobotController::controlDepthSample() const
{
    if (!controlLatest_)
        return std::nullopt;
    DepthControlSample sample;
    sample.rawDepthM = controlLatest_->depthValid ? controlLatest_->rawM : 0.0;
    if (controlLatest_->sampleAgeMs >= 0) {
        sample.ageMs = qMax<qint64>(0, depthNowMs() - controlLatest_->receivedAtMs)
            + controlLatest_->sampleAgeMs;
    }
    sample.fresh = controlLatest_->depthValid && sample.ageMs >= 0
        && sample.ageMs <= depthControlConfig_.controlFreshMs;
    if (zeroOffsetM_ && controlLatest_->depthValid)
        sample.calibratedDepthM = sample.rawDepthM - *zeroOffsetM_;
    return sample;
}

bool RemoteRobotController::zeroDepth(QString *error)
{
    const auto fail = [error](const QString &reason) {
        if (error) *error = reason;
        return false;
    };
    const DepthControlConfig &cfg = depthControlConfig_;
    if (!validDepthControlConfig(cfg))
        return fail(QStringLiteral("invalid depth control configuration"));
    if (isMotionActive())
        return fail(QStringLiteral("robot is moving; stop before zeroing depth"));
    const auto latest = controlDepthSample();
    if (!latest || !latest->fresh)
        return fail(QStringLiteral("no fresh depth sample"));
    if (zeroSamples_.size() < static_cast<std::size_t>(cfg.zeroMinSamples)) {
        return fail(QStringLiteral("not enough distinct depth samples (%1 of %2)")
                        .arg(zeroSamples_.size()).arg(cfg.zeroMinSamples));
    }

    qint64 oldest = zeroSamples_.front().sampledAtMs;
    qint64 newest = oldest;
    double lo = zeroSamples_.front().rawM;
    double hi = lo;
    double sum = 0.0;
    for (const auto &s : zeroSamples_) {
        oldest = qMin(oldest, s.sampledAtMs);
        newest = qMax(newest, s.sampledAtMs);
        lo = qMin(lo, s.rawM);
        hi = qMax(hi, s.rawM);
        sum += s.rawM;
    }
    const qint64 maxSpanMs = qMax<qint64>(
        1000, (cfg.zeroMinSamples + 1) * cfg.nominalSamplePeriodMs);
    if (newest - oldest > maxSpanMs) {
        return fail(QStringLiteral("depth samples span %1 ms (limit %2 ms)")
                        .arg(newest - oldest).arg(maxSpanMs));
    }
    if (hi - lo > cfg.zeroMaxRangeM) {
        return fail(QStringLiteral("depth is not steady: range %1 m (limit %2 m)")
                        .arg(hi - lo, 0, 'f', 4).arg(cfg.zeroMaxRangeM, 0, 'f', 4));
    }

    zeroOffsetM_ = sum / static_cast<double>(zeroSamples_.size());
    depthZeroedAtMs_ = depthNowMs();
    emit logMessage(QStringLiteral("Depth zeroed: offset %1 m").arg(*zeroOffsetM_, 0, 'f', 4));
    emit controlDepthSampleChanged();
    return true;
}

void RemoteRobotController::clearControlDepth()
{
    const bool had = controlLatest_.has_value() || !zeroSamples_.empty();
    controlLatest_.reset();
    zeroSamples_.clear();
    lastZeroSampleLine_.reset();
    if (had)
        emit controlDepthSampleChanged();
}

void RemoteRobotController::handleServiceError(
    quint32 requestId, const QByteArray &payload)
{
    const quint16 code = readLe16(payload, 0);
    const quint8 related = static_cast<quint8>(payload[2]);
    const quint32 detail = readLe32(payload, 4);
    emit logMessage(QStringLiteral(
        "Gateway ServiceError request=%1 code=%2 related=0x%3 detail=%4")
                        .arg(requestId).arg(code)
                        .arg(related, 2, 16, QLatin1Char('0')).arg(detail));
    terminalizePending(requestId,
                       QStringLiteral("service error %1").arg(code));
    updateMonitor(QStringLiteral("ServiceError %1").arg(code));
}

void RemoteRobotController::applyAcceptedCommand(
    const PendingCommand &pending)
{
    using Kind = robobeetle::gateway::RobotCommandKind;
    switch (pending.kind) {
    case Kind::EnableServos:
        poseKnownMask_ |= pending.servoMask & ~enabledMask_;
        setEnabledMask(static_cast<quint16>(
            enabledMask_ | pending.servoMask));
        break;
    case Kind::DisableServos:
        poseKnownMask_ &= ~pending.servoMask;
        setDisablePendingMask(static_cast<quint16>(
            disablePendingMask_ & ~pending.servoMask));
        setEnabledMask(static_cast<quint16>(
            enabledMask_ & ~pending.servoMask));
        if (isMotionActive()
            && (motionRequiredServoMask(motionMode_) & pending.servoMask) != 0U) {
            setMotionState(MotionState::Faulted, MotionMode::Stop);
        }
        break;
    case Kind::SetServoAngle:
    case Kind::NeutralServos:
        poseKnownMask_ |= pending.servoMask;
        break;
    case Kind::SetServoPwm:
        poseKnownMask_ &= ~pending.servoMask;
        break;
    case Kind::StartMotion:
        if (pending.motionMode.has_value()) {
            const bool isModeTransition =
                motionState_ == MotionState::Running
                && motionMode_ != *pending.motionMode;
            if (isModeTransition) {
                motionModeTransitionTimer_.start(kMotionTransitionDurationMs);
            } else {
                motionModeTransitionTimer_.stop();
            }
            setMotionState(MotionState::Running, *pending.motionMode);
        }
        break;
    case Kind::StopMotion:
        setMotionState(MotionState::Stopping, MotionMode::Stop);
        motionStopTimer_.start(kMotionTransitionDurationMs);
        break;
    case Kind::SetGaitBackend:
        if (pending.gaitBackend.has_value()) {
            confirmedGaitBackend_ = pending.gaitBackend;
        }
        pendingGaitBackend_.reset();
        emit gaitBackendStateChanged();
        break;
    case Kind::SetFrontRearCoordination:
        if (pending.frontRearCoordination.has_value()) {
            confirmedFrontRearCoordination_ = pending.frontRearCoordination;
        }
        pendingFrontRearCoordination_.reset();
        emit frontRearCoordinationStateChanged();
        break;
    }
}

void RemoteRobotController::terminalizePending(
    quint32 requestId, const QString &status, CommandTerminalResult result)
{
    auto it = pending_.find(requestId);
    if (it == pending_.end()) {
        return;
    }
    const PendingCommand pending = *it;
    pending_.erase(it);
    emit commandTerminal(requestId, result, 0xff, terminalNow_() - pending.terminalSentMs);
    if (pending.superseded) {
        emit logMessage(QStringLiteral("%1 (superseded)").arg(status));
        return;
    }
    if (pending.kind == robobeetle::gateway::RobotCommandKind::DisableServos) {
        setDisablePendingMask(static_cast<quint16>(
            disablePendingMask_ & ~pending.servoMask));
    }
    if (pending.gaitBackend.has_value()) {
        pendingGaitBackend_.reset();
        emit gaitBackendStateChanged();
    }
    if (pending.frontRearCoordination.has_value()) {
        pendingFrontRearCoordination_.reset();
        emit frontRearCoordinationStateChanged();
    }
    if (pending.kind == robobeetle::gateway::RobotCommandKind::StopMotion
        && isMotionActive()) {
        setMotionState(MotionState::Faulted, MotionMode::Stop);
    }
    emit logMessage(status);
}

void RemoteRobotController::supersedePendingMotionStarts()
{
    for (auto it = pending_.begin(); it != pending_.end(); ++it) {
        if (it->kind == robobeetle::gateway::RobotCommandKind::StartMotion) {
            it->superseded = true;
        }
    }
}

void RemoteRobotController::supersedePendingForDisable(quint16 affectedMask)
{
    for (auto it = pending_.begin(); it != pending_.end(); ++it) {
        bool affected = false;
        switch (it->kind) {
        case robobeetle::gateway::RobotCommandKind::EnableServos:
        case robobeetle::gateway::RobotCommandKind::SetServoAngle:
        case robobeetle::gateway::RobotCommandKind::SetServoPwm:
        case robobeetle::gateway::RobotCommandKind::NeutralServos:
            affected = (it->servoMask & affectedMask) != 0U;
            break;
        case robobeetle::gateway::RobotCommandKind::StartMotion:
            affected = it->motionMode.has_value()
                && (motionRequiredServoMask(*it->motionMode) & affectedMask) != 0U;
            break;
        case robobeetle::gateway::RobotCommandKind::StopMotion:
            affected = (affectedMask & SupportedServoMask) != 0U;
            break;
        default:
            break;
        }
        if (affected) {
            it->superseded = true;
        }
    }
}

void RemoteRobotController::failClosedControlState(const QString &reason)
{
    const bool hadMotion = isMotionActive();
    motionStopTimer_.stop();
    motionModeTransitionTimer_.stop();
    const auto abandoned = pending_;
    pending_.clear();
    for (auto it = abandoned.cbegin(); it != abandoned.cend(); ++it)
        emit commandTerminal(it.key(), CommandTerminalResult::OutcomeUnknown, 0xff, terminalNow_() - it->terminalSentMs);
    poseKnownMask_ = 0;
    clearControlDepth();
    setDisablePendingMask(0U);
    setEnabledMask(0U);
    if (pendingGaitBackend_.has_value() || confirmedGaitBackend_.has_value()) {
        pendingGaitBackend_.reset();
        confirmedGaitBackend_.reset();
        emit gaitBackendStateChanged();
    }
    if (pendingFrontRearCoordination_.has_value()
        || confirmedFrontRearCoordination_.has_value()) {
        pendingFrontRearCoordination_.reset();
        confirmedFrontRearCoordination_.reset();
        emit frontRearCoordinationStateChanged();
    }
    if (hadMotion) {
        setMotionState(MotionState::Faulted, MotionMode::Stop);
    } else if (motionState_ != MotionState::Stopped
               || motionMode_ != MotionMode::Stop) {
        setMotionState(MotionState::Stopped, MotionMode::Stop);
    }
    if (!reason.isEmpty()) {
        emit logMessage(QStringLiteral("Fail-closed: %1").arg(reason));
    }
}

void RemoteRobotController::resetTelemetry()
{
    motionMonitor_.finishBatch();
    motionCsvLogger_.recordTotals(motionMonitor_);
    motionMonitor_.reset();
    emit motionTelemetryChanged();
    lastLeakTelemetryAtMs_ = -1;
    setLeakState(LeakState::Unknown);

    const bool imuChanged = imuState_.status != ImuStatus::Unknown
        || imuState_.snapshot.has_value()
        || imuState_.lastReceivedAtMs >= 0
        || !imuState_.error.isEmpty();
    imuState_ = {};
    if (imuChanged) {
        emit imuStateChanged();
    }

    const bool depthChanged = depthState_.status != DepthStatus::Unknown
        || depthState_.snapshot.has_value()
        || depthState_.lastReceivedAtMs >= 0
        || !depthState_.error.isEmpty();
    depthState_ = {};
    if (depthChanged) {
        emit depthStateChanged();
    }
    clearControlDepth();
}

bool RemoteRobotController::startMotionRecording(const QString &directory, const QString &session) {
    return motionCsvLogger_.start(directory, session);
}
void RemoteRobotController::stopMotionRecording() {
    // Recording may stop mid-batch while the Remote stream continues. Only a
    // subsequent batch or actual stream/epoch end establishes missing tails.
    motionCsvLogger_.recordTotals(motionMonitor_);
    motionCsvLogger_.stop();
}

void RemoteRobotController::refreshTelemetryStaleness()
{
    const qint64 now = nowMs();
    for (auto it = pending_.cbegin(); it != pending_.cend(); ++it) {
        if (it->sentAtMs >= 0 && now - it->sentAtMs >= kRemoteCommandTimeoutMs) {
            ++monitor_.timeoutCount;
            updateMonitor(QStringLiteral("Remote command outcome timeout"));
            emit logMessage(QStringLiteral(
                "Command request %1 became uncertain after %2 ms; releasing authority")
                                .arg(it.key()).arg(kRemoteCommandTimeoutMs));
            terminalizePending(it.key(), QStringLiteral("Remote command outcome timeout"));
            if (!session_.releaseControl()) {
                session_.disconnectFromHost();
            }
            return;
        }
    }

    if (lastLeakTelemetryAtMs_ >= 0
        && now - lastLeakTelemetryAtMs_ >= 1500) {
        lastLeakTelemetryAtMs_ = -1;
        setLeakState(LeakState::Unknown);
    }
    if (imuState_.status == ImuStatus::Receiving
        && imuState_.lastReceivedAtMs >= 0
        && now - imuState_.lastReceivedAtMs >= ImuMonitor::StaleTimeoutMs) {
        imuState_.status = ImuStatus::Stale;
        imuState_.snapshot.reset();
        imuState_.error =
            QStringLiteral("No ImuSnapshot received within the stale window");
        emit imuStateChanged();
    }

    if (depthState_.status == DepthStatus::Receiving
        && depthState_.lastReceivedAtMs >= 0
        && depthNowMs() - depthState_.lastReceivedAtMs >= DepthMonitor::StaleTimeoutMs) {
        depthState_.status = DepthStatus::Stale;
        depthState_.snapshot.reset();
        depthState_.error =
            QStringLiteral("No DepthSnapshot received within the stale window");
        emit depthStateChanged();
    }
}

void RemoteRobotController::setEnabledMask(quint16 mask)
{
    mask = static_cast<quint16>(mask & SupportedServoMask);
    const quint16 changed = static_cast<quint16>(enabledMask_ ^ mask);
    enabledMask_ = mask;
    for (int index = 0; index < kServoCount; ++index) {
        const quint16 bit = static_cast<quint16>(1U << index);
        if ((changed & bit) != 0U) {
            emit servoStateChanged(index, (enabledMask_ & bit) != 0U);
        }
    }
}

void RemoteRobotController::setDisablePendingMask(quint16 mask)
{
    mask = static_cast<quint16>(mask & SupportedServoMask);
    const quint16 changed =
        static_cast<quint16>(disablePendingMask_ ^ mask);
    disablePendingMask_ = mask;
    for (int index = 0; index < kServoCount; ++index) {
        const quint16 bit = static_cast<quint16>(1U << index);
        if ((changed & bit) != 0U) {
            emit servoDisablePendingChanged(
                index, (disablePendingMask_ & bit) != 0U);
        }
    }
}

void RemoteRobotController::setLeakState(LeakState state)
{
    if (leakState_ == state) {
        return;
    }
    leakState_ = state;
    emit leakStateChanged(leakState_);
}

void RemoteRobotController::setMotionState(
    MotionState state, MotionMode mode)
{
    if (motionState_ == state && motionMode_ == mode) {
        return;
    }
    motionState_ = state;
    motionMode_ = mode;
    emit motionStateChanged(motionState_, motionMode_);
}

void RemoteRobotController::updateMonitor(const QString &status)
{
    if (!status.isEmpty()) {
        monitor_.ackStatus = status;
    }
    emit protocolMonitorChanged(monitor_);
}

void RemoteRobotController::noteTxFrame(const QByteArray &wire)
{
    ++monitor_.txPacketCount;
    emit txHexChanged(QString::fromLatin1(wire.toHex(' ').toUpper()));
    updateMonitor();
}

void RemoteRobotController::noteRxFrame(
    quint8 kind, quint32 requestId, const QByteArray &payload)
{
    ++monitor_.rxPacketCount;
    robobeetle::gateway::Bytes bytes;
    bytes.reserve(static_cast<std::size_t>(payload.size()));
    for (char value : payload) {
        bytes.push_back(static_cast<quint8>(value));
    }
    const auto encoded = robobeetle::gateway::encode_frame(
        static_cast<robobeetle::gateway::RbrpMessageKind>(kind),
        requestId, bytes);
    if (encoded.status == robobeetle::gateway::RbrpEncodeStatus::Ok) {
        const QByteArray wire(
            reinterpret_cast<const char *>(encoded.wire.data()),
            static_cast<qsizetype>(encoded.wire.size()));
        emit rxHexChanged(QString::fromLatin1(wire.toHex(' ').toUpper()));
    }
    updateMonitor();
}

QByteArray RemoteRobotController::maskPayload(quint16 mask)
{
    QByteArray payload;
    appendLe16(payload, mask);
    return payload;
}

qint64 RemoteRobotController::nowMs()
{
    return QDateTime::currentMSecsSinceEpoch();
}

QString RemoteRobotController::authorityText(
    ControlAuthorityState state, bool active)
{
    switch (state) {
    case ControlAuthorityState::Unowned:
        return QStringLiteral("Remote Unowned");
    case ControlAuthorityState::Acquiring:
        return QStringLiteral("Remote Acquiring");
    case ControlAuthorityState::Owned:
        return active ? QStringLiteral("Remote Owned / Active")
                      : QStringLiteral("Remote Owned / Link not Active");
    }
    return QStringLiteral("Remote Unknown");
}

} // namespace rb
