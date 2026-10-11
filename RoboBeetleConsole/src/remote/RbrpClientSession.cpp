#include "remote/RbrpClientSession.h"

#include <QAbstractSocket>
#include <QNetworkProxy>

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rb {
namespace {

quint16 readLe16(const robobeetle::gateway::Bytes &payload, std::size_t offset)
{
    return static_cast<quint16>(
        static_cast<quint16>(payload[offset])
        | (static_cast<quint16>(payload[offset + 1U]) << 8U));
}

quint32 readLe32(const robobeetle::gateway::Bytes &payload, std::size_t offset)
{
    return static_cast<quint32>(payload[offset])
        | (static_cast<quint32>(payload[offset + 1U]) << 8U)
        | (static_cast<quint32>(payload[offset + 2U]) << 16U)
        | (static_cast<quint32>(payload[offset + 3U]) << 24U);
}

robobeetle::gateway::Bytes toBytes(const QByteArray &bytes)
{
    const auto *begin =
        reinterpret_cast<const robobeetle::gateway::Byte *>(bytes.constData());
    return {begin, begin + bytes.size()};
}

bool gatewayKindIsServerToClient(robobeetle::gateway::RbrpMessageKind kind)
{
    using Kind = robobeetle::gateway::RbrpMessageKind;
    switch (kind) {
    case Kind::HelloReply:
    case Kind::AcquireReply:
    case Kind::ControlState:
    case Kind::CommandSubmitted:
    case Kind::CommandOutcome:
    case Kind::LeakTelemetry:
    case Kind::ImuTelemetry:
    case Kind::DepthTelemetry:
    case Kind::CpgParametersTelemetry:
    case Kind::MotionStateTelemetry:
    case Kind::ServiceError:
        return true;
    default:
        return false;
    }
}

} // namespace

RbrpClientSession::RbrpClientSession(QObject *parent)
    : QObject(parent)
{
    // The robot gateway is a direct LAN endpoint. Do not inherit desktop/system
    // proxy settings (for example an HTTP caching proxy), which are invalid for
    // a raw QTcpSocket connection and can prevent the TCP handshake entirely.
    socket_.setProxy(QNetworkProxy::NoProxy);

    heartbeatTimer_.setTimerType(Qt::PreciseTimer);
    heartbeatTimer_.setInterval(250);
    requestTimeoutTimer_.setSingleShot(true);
    requestTimeoutTimer_.setInterval(2000);

    connect(&heartbeatTimer_, &QTimer::timeout,
            this, &RbrpClientSession::sendHeartbeat);
    connect(&requestTimeoutTimer_, &QTimer::timeout, this, [this] {
        if (state_ == RemoteSessionState::HelloPending) {
            failProtocol(QStringLiteral("RBRP Hello timed out"));
        } else if (state_ == RemoteSessionState::AcquirePending) {
            failProtocol(QStringLiteral("RBRP AcquireControl timed out"));
        }
    });

    connect(&socket_, &QTcpSocket::connected, this, [this] {
        decoder_.reset();
        resetProtocolState();
        emit connectionStateChanged(TransportState::Connected);
        setSessionState(RemoteSessionState::HelloPending);
        sendHello();
    });

    connect(&socket_, &QTcpSocket::disconnected, this, [this] {
        heartbeatTimer_.stop();
        requestTimeoutTimer_.stop();
        decoder_.reset();
        resetProtocolState();
        setSessionState(RemoteSessionState::Disconnected);
        emit connectionStateChanged(TransportState::Disconnected);
    });

    connect(&socket_, &QTcpSocket::readyRead, this, [this] {
        const QByteArray bytes = socket_.readAll();
        std::vector<RbrpFrame> frames;
        const auto status = decoder_.feed(
            reinterpret_cast<const robobeetle::gateway::Byte *>(bytes.constData()),
            static_cast<std::size_t>(bytes.size()), frames);
        if (status == robobeetle::gateway::RbrpFeedStatus::Fatal) {
            failProtocol(QStringLiteral("Fatal RBRP framing error %1")
                             .arg(static_cast<int>(decoder_.fatal_error())));
            return;
        }
        for (const RbrpFrame &frame : frames) {
            handleFrame(frame);
            if (socket_.state() == QAbstractSocket::UnconnectedState) {
                break;
            }
        }
    });

    connect(&socket_, &QTcpSocket::errorOccurred, this,
            [this](QAbstractSocket::SocketError) {
        emit logMessage(QStringLiteral("TCP error: %1").arg(socket_.errorString()));
        heartbeatTimer_.stop();
        requestTimeoutTimer_.stop();
        resetProtocolState();
        setSessionState(RemoteSessionState::Error);
        emit connectionStateChanged(TransportState::Error);
        if (socket_.state() != QAbstractSocket::UnconnectedState) {
            socket_.abort();
        }
    });
}

void RbrpClientSession::connectToHost(const QString &host, quint16 port)
{
    const QString trimmed = host.trimmed();
    if (trimmed.isEmpty() || port == 0U) {
        heartbeatTimer_.stop();
        requestTimeoutTimer_.stop();
        if (socket_.state() != QAbstractSocket::UnconnectedState) {
            socket_.abort();
        }
        decoder_.reset();
        resetProtocolState();
        setSessionState(RemoteSessionState::Error);
        emit protocolError(QStringLiteral("Pi host and TCP port are required"));
        emit connectionStateChanged(TransportState::Error);
        return;
    }

    if (socket_.state() != QAbstractSocket::UnconnectedState) {
        socket_.abort();
    }
    decoder_.reset();
    resetProtocolState();
    setSessionState(RemoteSessionState::Connecting);
    emit connectionStateChanged(TransportState::Opening);
    emit logMessage(QStringLiteral("Connecting to Pi gateway %1:%2")
                        .arg(trimmed).arg(port));
    socket_.connectToHost(trimmed, port);
}

void RbrpClientSession::disconnectFromHost()
{
    if (socket_.state() == QAbstractSocket::UnconnectedState) {
        resetProtocolState();
        setSessionState(RemoteSessionState::Disconnected);
        emit connectionStateChanged(TransportState::Disconnected);
        return;
    }

    if (authorityState_ == ControlAuthorityState::Owned) {
        (void)releaseControl();
    }
    heartbeatTimer_.stop();
    requestTimeoutTimer_.stop();
    emit connectionStateChanged(TransportState::Closing);
    socket_.disconnectFromHost();
}

bool RbrpClientSession::acquireControl()
{
    if (!helloComplete_
        || state_ != RemoteSessionState::ReadyUnowned
        || authorityState_ != ControlAuthorityState::Unowned
        || !isConnected()) {
        return false;
    }

    const quint32 requestId = allocateRequestId();
    if (!sendFrame(RbrpMessageKind::AcquireControl, requestId, {})) {
        return false;
    }
    acquireRequestId_ = requestId;
    setSessionState(RemoteSessionState::AcquirePending);
    setAuthorityState(ControlAuthorityState::Acquiring, false);
    requestTimeoutTimer_.start();
    emit logMessage(QStringLiteral("AcquireControl submitted (%1)").arg(requestId));
    return true;
}

bool RbrpClientSession::releaseControl()
{
    if (authorityState_ != ControlAuthorityState::Owned || !isConnected()) {
        return false;
    }
    const quint32 requestId = allocateRequestId();
    if (!sendFrame(RbrpMessageKind::ReleaseControl, requestId, {})) {
        return false;
    }

    heartbeatTimer_.stop();
    requestTimeoutTimer_.stop();
    acquireRequestId_.reset();
    localReleaseFence_ = true;
    setAuthorityState(ControlAuthorityState::Unowned, false);
    setSessionState(helloComplete_ ? RemoteSessionState::ReadyUnowned
                                   : RemoteSessionState::HelloPending);
    emit logMessage(QStringLiteral("ReleaseControl submitted (%1)").arg(requestId));
    return true;
}

std::optional<quint32>
RbrpClientSession::sendCommand(quint8 commandKind,
                               const QByteArray &commandPayload)
{
    if (!isConnected() || !helloComplete_
        || authorityState_ != ControlAuthorityState::Owned || !controlActive_) {
        return std::nullopt;
    }

    QByteArray payload;
    payload.reserve(1 + commandPayload.size());
    payload.append(static_cast<char>(commandKind));
    payload.append(commandPayload);
    const quint32 requestId = allocateRequestId();
    if (!sendFrame(RbrpMessageKind::CommandRequest, requestId, payload)) {
        return std::nullopt;
    }
    return requestId;
}

bool RbrpClientSession::isConnected() const
{
    return socket_.state() == QAbstractSocket::ConnectedState;
}

bool RbrpClientSession::canAcquireControl() const
{
    return isConnected() && helloComplete_
        && state_ == RemoteSessionState::ReadyUnowned
        && authorityState_ == ControlAuthorityState::Unowned;
}

quint32 RbrpClientSession::allocateRequestId()
{
    const quint32 allocated = nextRequestId_++;
    if (nextRequestId_ == 0U) {
        nextRequestId_ = 1U;
    }
    return allocated == 0U ? allocateRequestId() : allocated;
}

bool RbrpClientSession::sendFrame(RbrpMessageKind kind, quint32 requestId,
                                  const QByteArray &payload)
{
    if (socket_.state() != QAbstractSocket::ConnectedState) {
        return false;
    }
    const auto encoded =
        robobeetle::gateway::encode_frame(kind, requestId, toBytes(payload));
    if (encoded.status != robobeetle::gateway::RbrpEncodeStatus::Ok) {
        failProtocol(QStringLiteral("Could not encode RBRP frame kind %1")
                         .arg(static_cast<int>(kind)));
        return false;
    }

    const QByteArray wire = toQByteArray(encoded.wire);
    const qint64 written = socket_.write(wire);
    if (written != wire.size()) {
        failProtocol(QStringLiteral("TCP write rejected RBRP frame"));
        return false;
    }
    emit frameSent(wire);
    return true;
}

void RbrpClientSession::sendHello()
{
    QByteArray payload(2, '\0');
    const quint32 requestId = allocateRequestId();
    if (!sendFrame(RbrpMessageKind::Hello, requestId, payload)) {
        return;
    }
    helloRequestId_ = requestId;
    requestTimeoutTimer_.start();
    emit logMessage(QStringLiteral("RBRP Hello submitted (%1)").arg(requestId));
}

void RbrpClientSession::sendHeartbeat()
{
    if (authorityState_ != ControlAuthorityState::Owned || !isConnected()) {
        heartbeatTimer_.stop();
        return;
    }
    const quint32 requestId = allocateRequestId();
    if (!sendFrame(RbrpMessageKind::ControlHeartbeat, requestId, {})) {
        failProtocol(QStringLiteral("Could not send ControlHeartbeat"));
    }
}

void RbrpClientSession::handleFrame(const RbrpFrame &frame)
{
    if (!gatewayKindIsServerToClient(frame.kind)) {
        failProtocol(QStringLiteral("Server sent wrong-direction RBRP kind %1")
                         .arg(static_cast<int>(frame.kind)));
        return;
    }
    if ((frame.kind == RbrpMessageKind::ControlState
         || frame.kind == RbrpMessageKind::LeakTelemetry
         || frame.kind == RbrpMessageKind::ImuTelemetry
         || frame.kind == RbrpMessageKind::DepthTelemetry
         || frame.kind == RbrpMessageKind::CpgParametersTelemetry
         || frame.kind == RbrpMessageKind::MotionStateTelemetry)
        && frame.request_id != 0U) {
        failProtocol(QStringLiteral(
            "Unsolicited gateway state/telemetry must use request_id 0"));
        return;
    }

    bool accepted = true;
    switch (frame.kind) {
    case RbrpMessageKind::HelloReply:
        accepted = handleHelloReply(frame);
        break;
    case RbrpMessageKind::AcquireReply:
        accepted = handleAcquireReply(frame);
        break;
    case RbrpMessageKind::ControlState:
        accepted = handleControlState(frame);
        break;
    case RbrpMessageKind::ServiceError:
        handleServiceError(frame);
        break;
    default:
        break;
    }
    if (!accepted) {
        return;
    }

    emit frameReceived(static_cast<quint8>(frame.kind), frame.request_id,
                       toQByteArray(frame.payload));
}

bool RbrpClientSession::handleHelloReply(const RbrpFrame &frame)
{
    if (state_ != RemoteSessionState::HelloPending
        || !helloRequestId_.has_value()
        || frame.request_id != *helloRequestId_) {
        failProtocol(QStringLiteral("Unexpected or uncorrelated HelloReply"));
        return false;
    }

    const quint16 capabilities = readLe16(frame.payload, 0U);
    const quint16 maxPayload = readLe16(frame.payload, 2U);
    const quint16 heartbeatMs = readLe16(frame.payload, 4U);
    const quint16 leaseMs = readLe16(frame.payload, 6U);
    if (capabilities != 0U || maxPayload != 512U
        || heartbeatMs != 250U || leaseMs != 1000U) {
        failProtocol(QStringLiteral(
            "RBRP v1 server contract mismatch (caps=%1 max=%2 heartbeat=%3 lease=%4)")
                         .arg(capabilities).arg(maxPayload)
                         .arg(heartbeatMs).arg(leaseMs));
        return false;
    }

    requestTimeoutTimer_.stop();
    helloRequestId_.reset();
    helloComplete_ = true;
    heartbeatIntervalMs_ = heartbeatMs;
    leaseTimeoutMs_ = leaseMs;
    heartbeatTimer_.setInterval(heartbeatIntervalMs_);
    setSessionState(RemoteSessionState::ReadyUnowned);
    emit logMessage(QStringLiteral("RBRP Hello complete"));
    return true;
}

bool RbrpClientSession::handleAcquireReply(const RbrpFrame &frame)
{
    if (state_ != RemoteSessionState::AcquirePending
        || !acquireRequestId_.has_value()
        || frame.request_id != *acquireRequestId_) {
        failProtocol(QStringLiteral("Unexpected or uncorrelated AcquireReply"));
        return false;
    }

    requestTimeoutTimer_.stop();
    acquireRequestId_.reset();
    const quint8 result = frame.payload[0];
    const quint8 authority = frame.payload[1];
    const quint8 session = frame.payload[2];
    const quint8 link = frame.payload[3];
    if (result > static_cast<quint8>(robobeetle::gateway::AcquireResult::InvalidState)
        || authority > static_cast<quint8>(robobeetle::gateway::AuthorityState::Owned)
        || session > static_cast<quint8>(
                         robobeetle::gateway::GatewayApplicationSessionState::Online)
        || link > static_cast<quint8>(
                      robobeetle::gateway::GatewayApplicationLinkState::Lost)) {
        failProtocol(QStringLiteral("AcquireReply contains invalid enum value"));
        return false;
    }

    const bool granted =
        (result == static_cast<quint8>(robobeetle::gateway::AcquireResult::Granted)
         || result == static_cast<quint8>(
                          robobeetle::gateway::AcquireResult::AlreadyOwnedBySource))
        && authority == static_cast<quint8>(
                            robobeetle::gateway::AuthorityState::Owned);
    if (!granted) {
        heartbeatTimer_.stop();
        setAuthorityState(ControlAuthorityState::Unowned, false);
        setSessionState(RemoteSessionState::ReadyUnowned);
        emit logMessage(QStringLiteral("AcquireControl not granted (result %1)")
                            .arg(result));
        return true;
    }

    localReleaseFence_ = false;
    const bool active =
        session == static_cast<quint8>(
                       robobeetle::gateway::GatewayApplicationSessionState::Online)
        && link == static_cast<quint8>(
                      robobeetle::gateway::GatewayApplicationLinkState::Active);
    setAuthorityState(ControlAuthorityState::Owned, active);
    setSessionState(RemoteSessionState::Owned);
    heartbeatTimer_.start();
    emit logMessage(QStringLiteral("Remote authority acquired"));
    return true;
}

bool RbrpClientSession::handleControlState(const RbrpFrame &frame)
{
    if (frame.request_id != 0U) {
        failProtocol(QStringLiteral("ControlState must use request_id 0"));
        return false;
    }
    const quint8 authority = frame.payload[0];
    const quint8 session = frame.payload[1];
    const quint8 link = frame.payload[2];
    if (authority > static_cast<quint8>(robobeetle::gateway::AuthorityState::Owned)
        || session > static_cast<quint8>(
                         robobeetle::gateway::GatewayApplicationSessionState::Online)
        || link > static_cast<quint8>(
                      robobeetle::gateway::GatewayApplicationLinkState::Lost)) {
        failProtocol(QStringLiteral("ControlState contains invalid enum value"));
        return false;
    }

    if (localReleaseFence_
        && authority == static_cast<quint8>(
                            robobeetle::gateway::AuthorityState::Owned)) {
        emit logMessage(QStringLiteral(
            "Ignoring stale Owned ControlState after local Release"));
        return true;
    }

    if (authority != static_cast<quint8>(
                         robobeetle::gateway::AuthorityState::Owned)) {
        heartbeatTimer_.stop();
        setAuthorityState(ControlAuthorityState::Unowned, false);
        if (helloComplete_) {
            setSessionState(RemoteSessionState::ReadyUnowned);
        }
        return true;
    }

    const bool active =
        session == static_cast<quint8>(
                       robobeetle::gateway::GatewayApplicationSessionState::Online)
        && link == static_cast<quint8>(
                      robobeetle::gateway::GatewayApplicationLinkState::Active);
    setAuthorityState(ControlAuthorityState::Owned, active);
    setSessionState(RemoteSessionState::Owned);
    if (!heartbeatTimer_.isActive()) {
        heartbeatTimer_.start();
    }
    return true;
}

void RbrpClientSession::handleServiceError(const RbrpFrame &frame)
{
    const quint16 code = readLe16(frame.payload, 0U);
    const quint8 relatedKind = frame.payload[2];
    emit logMessage(QStringLiteral("RBRP ServiceError code=%1 related=0x%2 detail=%3")
                        .arg(code)
                        .arg(relatedKind, 2, 16, QLatin1Char('0'))
                        .arg(readLe32(frame.payload, 4U)));

    if (code == static_cast<quint16>(
                    robobeetle::gateway::ServiceErrorCode::NotAuthority)) {
        heartbeatTimer_.stop();
        setAuthorityState(ControlAuthorityState::Unowned, false);
        if (helloComplete_) {
            setSessionState(RemoteSessionState::ReadyUnowned);
        }
    }
    if (relatedKind == static_cast<quint8>(RbrpMessageKind::Hello)
        && state_ == RemoteSessionState::HelloPending) {
        failProtocol(QStringLiteral("Gateway rejected RBRP Hello"));
    } else if (relatedKind == static_cast<quint8>(
                                 RbrpMessageKind::AcquireControl)
               && state_ == RemoteSessionState::AcquirePending) {
        requestTimeoutTimer_.stop();
        acquireRequestId_.reset();
        setAuthorityState(ControlAuthorityState::Unowned, false);
        setSessionState(RemoteSessionState::ReadyUnowned);
    }
}

void RbrpClientSession::setSessionState(RemoteSessionState state)
{
    if (state_ == state) {
        return;
    }
    state_ = state;
    emit sessionStateChanged(state_);
}

void RbrpClientSession::setAuthorityState(ControlAuthorityState state,
                                          bool active)
{
    if (authorityState_ == state && controlActive_ == active) {
        return;
    }
    authorityState_ = state;
    controlActive_ = active;
    emit authorityStateChanged(authorityState_, controlActive_);
}

void RbrpClientSession::resetProtocolState()
{
    helloComplete_ = false;
    helloRequestId_.reset();
    acquireRequestId_.reset();
    localReleaseFence_ = false;
    heartbeatIntervalMs_ = 250U;
    leaseTimeoutMs_ = 1000U;
    setAuthorityState(ControlAuthorityState::Unowned, false);
}

void RbrpClientSession::failProtocol(const QString &message)
{
    heartbeatTimer_.stop();
    requestTimeoutTimer_.stop();
    setAuthorityState(ControlAuthorityState::Unowned, false);
    setSessionState(RemoteSessionState::Error);
    emit protocolError(message);
    emit logMessage(QStringLiteral("RBRP protocol error: %1").arg(message));
    emit connectionStateChanged(TransportState::Error);
    socket_.abort();
}

QByteArray
RbrpClientSession::toQByteArray(const robobeetle::gateway::Bytes &bytes)
{
    return QByteArray(reinterpret_cast<const char *>(bytes.data()),
                      static_cast<qsizetype>(bytes.size()));
}

} // namespace rb
