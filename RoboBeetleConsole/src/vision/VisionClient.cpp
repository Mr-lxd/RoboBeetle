#include "vision/VisionClient.h"

#include "vision/VisionProtocol.h"

#include <QAbstractSocket>
#include <QByteArray>
#include <QNetworkProxy>

#include <cstdint>
#include <span>

namespace rb::vision {

VisionClient::VisionClient(QObject *parent)
    : QObject(parent)
{
    // Vision is a direct engineering-LAN TCP endpoint, just like RBRP.
    // Never inherit desktop/system HTTP proxy settings for this raw socket.
    socket_.setProxy(QNetworkProxy::NoProxy);
    socket_.setReadBufferSize(kVisionSocketReadBufferSize);

    partialFrameTimer_.setSingleShot(true);
    partialFrameTimer_.setInterval(kVisionPartialFrameInactivityTimeoutMs);
    connect(&partialFrameTimer_, &QTimer::timeout, this, [this] {
        if (!decoder_.hasPartialFrame() ||
            socket_.state() == QAbstractSocket::UnconnectedState) {
            return;
        }
        const QString message =
            QStringLiteral("Vision partial frame timed out waiting for more bytes");
        emit protocolError(message);
        emit logMessage(message);
        setState(VisionConnectionState::Error);
        socket_.abort();
    });

    connect(&socket_, &QTcpSocket::connected, this, [this] {
        decoder_.reset();
        fpsTimer_.restart();
        setState(VisionConnectionState::Connected);
        emit logMessage(QStringLiteral("Vision stream connected"));
    });
    connect(&socket_, &QTcpSocket::readyRead,
            this, &VisionClient::handleReadyRead);
    connect(&socket_, &QTcpSocket::disconnected,
            this, &VisionClient::handleDisconnected);
    connect(&socket_, &QTcpSocket::errorOccurred,
            this, &VisionClient::handleSocketError);
}

void VisionClient::connectToHost(const QString &host, quint16 port)
{
    const QString trimmed = host.trimmed();
    if (trimmed.isEmpty() || port == 0U) {
        emit protocolError(QStringLiteral("Vision host and port must be valid"));
        setState(VisionConnectionState::Error);
        return;
    }

    if (socket_.state() != QAbstractSocket::UnconnectedState) {
        socket_.abort();
    }
    userDisconnect_ = false;
    resetSessionState();
    setState(VisionConnectionState::Connecting);
    emit logMessage(
        QStringLiteral("Connecting Vision stream to %1:%2")
            .arg(trimmed)
            .arg(port));
    socket_.connectToHost(trimmed, port);
}

void VisionClient::disconnectFromHost()
{
    userDisconnect_ = true;
    partialFrameTimer_.stop();
    if (socket_.state() == QAbstractSocket::UnconnectedState) {
        decoder_.reset();
        setState(VisionConnectionState::Disconnected);
        return;
    }
    socket_.disconnectFromHost();
}

void VisionClient::shutdown()
{
    userDisconnect_ = true;
    partialFrameTimer_.stop();
    socket_.abort();
    decoder_.reset();
    setState(VisionConnectionState::Disconnected);
}

void VisionClient::handleReadyRead()
{
    const QByteArray bytes = socket_.readAll();
    if (bytes.isEmpty()) {
        return;
    }

    const auto *data = reinterpret_cast<const std::uint8_t *>(bytes.constData());
    auto result = decoder_.feed(
        std::span<const std::uint8_t>(data, static_cast<std::size_t>(bytes.size())));

    if (result.fatal) {
        const QString message = QStringLiteral("Fatal RBVS framing error");
        emit protocolError(message);
        emit logMessage(message);
        setState(VisionConnectionState::Error);
        socket_.abort();
        return;
    }

    totalWireFrames_ += result.completedWireFrames;
    replacedWireFrames_ += result.replacedWireFrames;
    updateFps(result.completedWireFrames);

    if (result.latestFrame) {
        haveFrame_ = true;
        lastFrameId_ = result.latestFrame->header.frameId;
        emit frameReady(
            result.latestFrame->image,
            result.latestFrame->header.frameId,
            result.latestFrame->header.captureTimestampNs);
    }
    updatePartialFrameWatchdog();
    emitDiagnostics();
}

void VisionClient::handleDisconnected()
{
    partialFrameTimer_.stop();
    if (!userDisconnect_) {
        const VisionStreamError finishError = decoder_.finish();
        if (finishError == VisionStreamError::TruncatedFrame) {
            emit logMessage(
                QStringLiteral("Vision peer disconnected mid-frame; partial frame discarded"));
        }
    }
    decoder_.reset();
    if (state_ != VisionConnectionState::Error) {
        setState(VisionConnectionState::Disconnected);
    }
    emit logMessage(QStringLiteral("Vision stream disconnected"));
}

void VisionClient::handleSocketError(QAbstractSocket::SocketError error)
{
    if (userDisconnect_ ||
        error == QAbstractSocket::RemoteHostClosedError) {
        return;
    }
    emit logMessage(
        QStringLiteral("Vision socket error: %1").arg(socket_.errorString()));
    setState(VisionConnectionState::Error);
}

void VisionClient::setState(VisionConnectionState state)
{
    if (state_ == state) {
        return;
    }
    state_ = state;
    emit connectionStateChanged(state_);
}

void VisionClient::resetSessionState()
{
    partialFrameTimer_.stop();
    decoder_.reset();
    fpsTimer_.invalidate();
    fpsWindowFrames_ = 0;
    receivedFps_ = 0.0;
    lastFrameId_ = 0;
    totalWireFrames_ = 0;
    replacedWireFrames_ = 0;
    haveFrame_ = false;
}

void VisionClient::updateFps(quint64 completedWireFrames)
{
    if (!fpsTimer_.isValid()) {
        fpsTimer_.start();
    }
    fpsWindowFrames_ += completedWireFrames;
    const qint64 elapsed = fpsTimer_.elapsed();
    if (elapsed < 1000) {
        return;
    }
    receivedFps_ =
        static_cast<double>(fpsWindowFrames_) * 1000.0 / static_cast<double>(elapsed);
    fpsWindowFrames_ = 0;
    fpsTimer_.restart();
}

void VisionClient::updatePartialFrameWatchdog()
{
    if (decoder_.hasPartialFrame()) {
        partialFrameTimer_.start();
    } else {
        partialFrameTimer_.stop();
    }
}

void VisionClient::emitDiagnostics()
{
    emit diagnosticsChanged(
        receivedFps_,
        haveFrame_ ? lastFrameId_ : 0U,
        totalWireFrames_,
        replacedWireFrames_,
        decoder_.jpegDecodeErrors());
}

} // namespace rb::vision
