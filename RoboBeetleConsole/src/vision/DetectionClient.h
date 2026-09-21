#pragma once

#include "vision/DetectionMetadata.h"
#include "vision/DetectionStreamDecoder.h"

#include <QObject>
#include <QString>

class QTcpSocket;

namespace rb::vision {

enum class DetectionConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Error,
};

class DetectionClient final : public QObject {
    Q_OBJECT

public:
    explicit DetectionClient(QObject *parent = nullptr);

    void connectToHost(
        const QString &host,
        quint16 port = kDetectionStreamDefaultPort);
    void disconnectFromHost();
    void shutdown();

    [[nodiscard]] DetectionConnectionState state() const noexcept
    {
        return state_;
    }
    [[nodiscard]] bool isConnected() const noexcept
    {
        return state_ == DetectionConnectionState::Connected;
    }
    [[nodiscard]] bool endpointBusy() const noexcept;
    [[nodiscard]] QString host() const { return host_; }
    [[nodiscard]] quint16 port() const noexcept { return port_; }
    [[nodiscard]] quint64 lastFrameId() const noexcept { return lastFrameId_; }

signals:
    void connectionStateChanged(rb::vision::DetectionConnectionState state);
    void endpointActivityChanged(bool busy);
    void metadataReady(rb::vision::DetectionFrame frame);
    void protocolError(const QString &message);
    void logMessage(const QString &message);

private:
    void replaceSocket();
    void handleReadyRead(QTcpSocket *socket, quint64 generation);
    void setState(DetectionConnectionState state);
    void emitBusyIfChanged();

    QTcpSocket *socket_{nullptr};
    DetectionStreamDecoder decoder_;
    DetectionConnectionState state_{DetectionConnectionState::Disconnected};
    QString host_;
    quint16 port_{kDetectionStreamDefaultPort};
    quint64 generation_{0};
    quint64 lastFrameId_{0};
    bool haveFrame_{false};
    bool userDisconnect_{false};
    bool endpointBusy_{false};
    bool shuttingDown_{false};
};

} // namespace rb::vision

Q_DECLARE_METATYPE(rb::vision::DetectionConnectionState)
