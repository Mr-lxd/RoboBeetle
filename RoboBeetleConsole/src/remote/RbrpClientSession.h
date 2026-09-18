#pragma once

#include "controller/IConsoleController.h"
#include "robobeetle/gateway/rbrp_codec.hpp"

#include <QByteArray>
#include <QObject>
#include <QString>
#include <QTimer>
#include <QTcpSocket>

#include <optional>

namespace rb {

enum class RemoteSessionState {
    Disconnected,
    Connecting,
    HelloPending,
    ReadyUnowned,
    AcquirePending,
    Owned,
    Error,
};

class RbrpClientSession final : public QObject {
    Q_OBJECT

public:
    explicit RbrpClientSession(QObject *parent = nullptr);

    void connectToHost(const QString &host, quint16 port);
    void disconnectFromHost();

    bool acquireControl();
    bool releaseControl();
    std::optional<quint32> sendCommand(quint8 commandKind,
                                       const QByteArray &commandPayload);

    [[nodiscard]] bool isConnected() const;
    [[nodiscard]] bool canAcquireControl() const;
    [[nodiscard]] bool helloComplete() const { return helloComplete_; }
    [[nodiscard]] ControlAuthorityState authorityState() const { return authorityState_; }
    [[nodiscard]] bool isControlActive() const { return controlActive_; }
    [[nodiscard]] RemoteSessionState sessionState() const { return state_; }
    [[nodiscard]] quint16 heartbeatIntervalMs() const { return heartbeatIntervalMs_; }
    [[nodiscard]] quint16 leaseTimeoutMs() const { return leaseTimeoutMs_; }

signals:
    void connectionStateChanged(rb::TransportState state);
    void authorityStateChanged(rb::ControlAuthorityState state, bool active);
    void sessionStateChanged(rb::RemoteSessionState state);
    void frameReceived(quint8 kind, quint32 requestId, const QByteArray &payload);
    void frameSent(const QByteArray &wire);
    void protocolError(const QString &message);
    void logMessage(const QString &message);

private:
    using RbrpFrame = robobeetle::gateway::RbrpFrame;
    using RbrpMessageKind = robobeetle::gateway::RbrpMessageKind;

    quint32 allocateRequestId();
    bool sendFrame(RbrpMessageKind kind, quint32 requestId,
                   const QByteArray &payload);
    void sendHello();
    void sendHeartbeat();
    void handleFrame(const RbrpFrame &frame);
    bool handleHelloReply(const RbrpFrame &frame);
    bool handleAcquireReply(const RbrpFrame &frame);
    bool handleControlState(const RbrpFrame &frame);
    void handleServiceError(const RbrpFrame &frame);

    void setSessionState(RemoteSessionState state);
    void setAuthorityState(ControlAuthorityState state, bool active);
    void resetProtocolState();
    void failProtocol(const QString &message);
    static QByteArray toQByteArray(const robobeetle::gateway::Bytes &bytes);

    QTcpSocket socket_;
    robobeetle::gateway::RbrpDecoder decoder_;
    QTimer heartbeatTimer_;
    QTimer requestTimeoutTimer_;

    RemoteSessionState state_{RemoteSessionState::Disconnected};
    ControlAuthorityState authorityState_{ControlAuthorityState::Unowned};
    bool controlActive_{false};
    bool helloComplete_{false};
    quint16 heartbeatIntervalMs_{250};
    quint16 leaseTimeoutMs_{1000};
    quint32 nextRequestId_{1};
    std::optional<quint32> helloRequestId_;
    std::optional<quint32> acquireRequestId_;
};

} // namespace rb

Q_DECLARE_METATYPE(rb::RemoteSessionState)
