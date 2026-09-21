#include "vision/DetectionClient.h"

#include <QAbstractSocket>
#include <QHostAddress>
#include <QNetworkProxy>
#include <QTcpSocket>

namespace rb::vision {

DetectionClient::DetectionClient(QObject *parent)
    : QObject(parent)
{
}

bool DetectionClient::endpointBusy() const noexcept
{
    return socket_ != nullptr
        && socket_->state() != QAbstractSocket::UnconnectedState;
}

void DetectionClient::connectToHost(const QString &host, quint16 port)
{
    QString normalizedHost = host.trimmed().toLower();
    QHostAddress address;
    if (address.setAddress(normalizedHost)) {
        normalizedHost = address.toString();
    }
    if (normalizedHost.isEmpty() || port == 0U || shuttingDown_) {
        const QString message =
            QStringLiteral("Detection metadata host and port must be valid");
        emit protocolError(message);
        emit logMessage(message);
        setState(DetectionConnectionState::Error);
        return;
    }

    if (normalizedHost == host_ && port == port_
        && (state_ == DetectionConnectionState::Connecting
            || state_ == DetectionConnectionState::Connected)) {
        return;
    }

    ++generation_;
    userDisconnect_ = false;
    host_ = normalizedHost;
    port_ = port;
    decoder_.reset();
    haveFrame_ = false;
    lastFrameId_ = 0U;
    replaceSocket();

    QTcpSocket *socket = socket_;
    const quint64 generation = generation_;
    setState(DetectionConnectionState::Connecting);
    emit logMessage(
        QStringLiteral("Connecting detection metadata to %1:%2")
            .arg(host_)
            .arg(port_));

    connect(socket, &QTcpSocket::connected, this,
            [this, socket, generation] {
        if (socket != socket_ || generation != generation_ || shuttingDown_) {
            return;
        }
        decoder_.reset();
        haveFrame_ = false;
        lastFrameId_ = 0U;
        setState(DetectionConnectionState::Connected);
        emit logMessage(QStringLiteral("Detection metadata connected"));
    });
    connect(socket, &QTcpSocket::readyRead, this,
            [this, socket, generation] {
        handleReadyRead(socket, generation);
    });
    connect(socket, &QTcpSocket::stateChanged, this,
            [this, socket, generation](QAbstractSocket::SocketState) {
        if (socket != socket_ || generation != generation_) {
            return;
        }
        emitBusyIfChanged();
    });
    connect(socket, &QTcpSocket::disconnected, this,
            [this, socket, generation] {
        if (socket != socket_ || generation != generation_) {
            return;
        }
        if (state_ != DetectionConnectionState::Error) {
            setState(DetectionConnectionState::Disconnected);
        }
        emitBusyIfChanged();
        emit logMessage(QStringLiteral("Detection metadata disconnected"));
    });
    connect(socket, &QTcpSocket::errorOccurred, this,
            [this, socket, generation](QAbstractSocket::SocketError error) {
        if (socket != socket_ || generation != generation_
            || userDisconnect_ || shuttingDown_
            || error == QAbstractSocket::RemoteHostClosedError) {
            return;
        }
        const QString message = QStringLiteral(
            "Detection metadata socket error: %1").arg(socket->errorString());
        emit logMessage(message);
        setState(DetectionConnectionState::Error);
    });

    socket->connectToHost(host_, port_);
    emitBusyIfChanged();
}

void DetectionClient::disconnectFromHost()
{
    ++generation_;
    userDisconnect_ = true;
    decoder_.reset();
    haveFrame_ = false;
    lastFrameId_ = 0U;

    QTcpSocket *old = socket_;
    socket_ = nullptr;
    if (old != nullptr) {
        old->disconnect(this);
        old->abort();
        old->deleteLater();
    }
    endpointBusy_ = false;
    setState(DetectionConnectionState::Disconnected);
    emit endpointActivityChanged(false);
}

void DetectionClient::shutdown()
{
    shuttingDown_ = true;
    disconnectFromHost();
}

void DetectionClient::replaceSocket()
{
    QTcpSocket *old = socket_;
    socket_ = new QTcpSocket(this);
    socket_->setProxy(QNetworkProxy::NoProxy);
    socket_->setReadBufferSize(128 * 1024);
    if (old != nullptr) {
        old->disconnect(this);
        old->abort();
        old->deleteLater();
    }
    endpointBusy_ = false;
}

void DetectionClient::handleReadyRead(
    QTcpSocket *socket,
    quint64 generation)
{
    if (socket != socket_ || generation != generation_ || shuttingDown_) {
        return;
    }
    const QByteArray bytes = socket->readAll();
    if (bytes.isEmpty()) {
        return;
    }

    DetectionDecodeResult decoded = decoder_.feed(bytes);
    if (decoded.fatal) {
        const QString message = QStringLiteral(
            "Detection metadata protocol error: %1").arg(decoded.error);
        emit protocolError(message);
        emit logMessage(message);
        setState(DetectionConnectionState::Error);
        socket->abort();
        return;
    }

    for (const DetectionFrame &frame : decoded.frames) {
        if (haveFrame_ && frame.frameId <= lastFrameId_) {
            const QString message = QStringLiteral(
                "Detection metadata frame_id must increase");
            emit protocolError(message);
            emit logMessage(message);
            setState(DetectionConnectionState::Error);
            socket->abort();
            return;
        }
        haveFrame_ = true;
        lastFrameId_ = frame.frameId;
        emit metadataReady(frame);
    }
}

void DetectionClient::setState(DetectionConnectionState state)
{
    if (state_ == state) {
        return;
    }
    state_ = state;
    emit connectionStateChanged(state_);
}

void DetectionClient::emitBusyIfChanged()
{
    const bool busy = endpointBusy();
    if (busy == endpointBusy_) {
        return;
    }
    endpointBusy_ = busy;
    emit endpointActivityChanged(busy);
}

} // namespace rb::vision
