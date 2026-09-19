#pragma once

#include <QNetworkAccessManager>
#include <QObject>
#include <QString>
#include <QTimer>
#include <QUrl>

#include <optional>

namespace rb::vision {

inline constexpr quint16 kVisionControlDefaultPort = 47011;

struct VisionCaptureStatus {
    bool cameraRunning{false};
    bool haveLatestFrame{false};
    quint64 latestFrameId{0};
    QString state{QStringLiteral("idle")};
    bool recording{false};
    QString sessionId;
    QString segment;
    quint64 recordedFrames{0};
    quint64 snapshotCount{0};
    quint64 queueBytes{0};
    quint64 maxQueueBytes{0};
    bool haveFreeDisk{false};
    quint64 freeDiskBytes{0};
    QString lastError;
};

class VisionControlClient final : public QObject {
    Q_OBJECT

public:
    explicit VisionControlClient(QObject *parent = nullptr);
    bool setEndpoint(
        const QString &host,
        quint16 port = kVisionControlDefaultPort);
    void refreshStatus();
    void requestSnapshot();
    void startRecording();
    void stopRecording();
    void startPolling(int intervalMs = 1000);
    void stopPolling();
    void shutdown();

    [[nodiscard]] QString host() const { return host_; }
    [[nodiscard]] quint16 port() const noexcept { return port_; }
    [[nodiscard]] bool requestInFlight() const noexcept
    {
        return requestInFlight_;
    }
    [[nodiscard]] bool actionInFlight() const noexcept;
    [[nodiscard]] VisionCaptureStatus status() const { return status_; }

signals:
    void statusChanged(rb::vision::VisionCaptureStatus status);
    void actionSucceeded(const QString &action);
    void errorOccurred(const QString &message);
    void logMessage(const QString &message);

private:
    enum class RequestKind {
        Status,
        Snapshot,
        StartRecording,
        StopRecording,
    };

    void issue(RequestKind kind, const QString &path, bool post);
    void handleFinished(
        QNetworkReply *reply,
        RequestKind kind,
        quint64 endpointGeneration);
    void dispatchPendingAction();
    void applyPayload(const QJsonObject &object);
    [[nodiscard]] QUrl endpointUrl(const QString &path) const;
    [[nodiscard]] static QString actionName(RequestKind kind);

    QNetworkAccessManager manager_;
    QTimer pollTimer_;
    QString host_;
    quint16 port_{kVisionControlDefaultPort};
    bool requestInFlight_{false};
    RequestKind activeRequestKind_{RequestKind::Status};
    std::optional<RequestKind> pendingAction_;
    QString pendingPath_;
    bool pendingPost_{false};
    QNetworkReply *activeReply_{nullptr};
    quint64 endpointGeneration_{0};
    VisionCaptureStatus status_;
};

} // namespace rb::vision

Q_DECLARE_METATYPE(rb::vision::VisionCaptureStatus)
