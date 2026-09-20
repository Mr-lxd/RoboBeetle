#pragma once

#include "vision/VisionFrameDecoder.h"

#include <QElapsedTimer>
#include <QObject>
#include <QString>
#include <QTcpSocket>
#include <QTimer>

namespace rb::vision {

inline constexpr qint64 kVisionSocketReadBufferSize = 256 * 1024;
inline constexpr int kVisionPartialFrameInactivityTimeoutMs = 2000;

enum class VisionConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Error,
};

class VisionClient final : public QObject {
    Q_OBJECT

public:
    explicit VisionClient(QObject *parent = nullptr);

    void connectToHost(const QString &host, quint16 port);
    void disconnectFromHost();
    void shutdown();

    [[nodiscard]] VisionConnectionState state() const noexcept { return state_; }
    [[nodiscard]] bool isConnected() const noexcept
    {
        return state_ == VisionConnectionState::Connected;
    }
    [[nodiscard]] bool endpointBusy() const noexcept
    {
        return socket_.state() != QAbstractSocket::UnconnectedState;
    }
    [[nodiscard]] double receivedFps() const noexcept { return receivedFps_; }
    [[nodiscard]] quint64 lastFrameId() const noexcept { return lastFrameId_; }
    [[nodiscard]] quint64 totalWireFrames() const noexcept { return totalWireFrames_; }
    [[nodiscard]] quint64 replacedWireFrames() const noexcept { return replacedWireFrames_; }

signals:
    void connectionStateChanged(rb::vision::VisionConnectionState state);
    void endpointActivityChanged(bool busy);
    void frameReady(const QImage &image,
                    quint64 frameId,
                    quint64 captureTimestampNs);
    void diagnosticsChanged(double receivedFps,
                            quint64 lastFrameId,
                            quint64 totalWireFrames,
                            quint64 replacedWireFrames,
                            quint64 jpegDecodeErrors);
    void protocolError(const QString &message);
    void logMessage(const QString &message);

private:
    void handleReadyRead();
    void handleDisconnected();
    void handleSocketError(QAbstractSocket::SocketError error);
    void setState(VisionConnectionState state);
    void resetSessionState();
    void updateFps(quint64 completedWireFrames);
    void updatePartialFrameWatchdog();
    void emitDiagnostics();

    QTcpSocket socket_;
    QTimer partialFrameTimer_;
    VisionFrameDecoder decoder_;
    VisionConnectionState state_{VisionConnectionState::Disconnected};
    QElapsedTimer fpsTimer_;
    quint64 fpsWindowFrames_{0};
    double receivedFps_{0.0};
    quint64 lastFrameId_{0};
    quint64 totalWireFrames_{0};
    quint64 replacedWireFrames_{0};
    bool haveFrame_{false};
    bool userDisconnect_{false};
    bool endpointBusy_{false};
};

} // namespace rb::vision

Q_DECLARE_METATYPE(rb::vision::VisionConnectionState)
