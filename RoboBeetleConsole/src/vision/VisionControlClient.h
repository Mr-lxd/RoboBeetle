#pragma once

#include <QNetworkAccessManager>
#include <QElapsedTimer>
#include <QObject>
#include <QString>
#include <QTimer>
#include <QUrl>

#include <optional>

namespace rb::vision {

inline constexpr quint16 kVisionControlDefaultPort = 47011;

struct VisionCaptureStatus {
    bool haveCameraStatus{false};
    bool haveCaptureStatus{false};
    bool haveInferenceStatus{false};
    bool haveInferenceState{false};
    std::optional<bool> inferenceConfigured;
    std::optional<bool> inferenceControlSupported;
    std::optional<bool> detectionStreamSupported;
    std::optional<quint16> detectionStreamPort;
    std::optional<int> detectionStreamVersion;
    bool inferenceOperationValid{false};
    QString inferenceOperation;
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
    QString inferenceState{QStringLiteral("disabled")};
    QString inferenceArtifactName;
    QString inferenceModelSha256;
    bool haveInferenceConfidenceThreshold{false};
    double inferenceConfidenceThreshold{0.0};
    bool haveInferenceLatestFrame{false};
    quint64 inferenceLatestFrameId{0};
    bool haveInferenceCaptureTimestampNs{false};
    quint64 inferenceCaptureTimestampNs{0};
    bool haveInferenceProcessedFrames{false};
    quint64 inferenceProcessedFrames{0};
    bool haveInferenceSkippedFrames{false};
    quint64 inferenceSkippedFrames{0};
    bool haveInferenceFps{false};
    double inferenceFps{0.0};
    bool haveInferenceLatencyMs{false};
    double inferenceLatencyMs{0.0};
    bool haveInferenceDetectionCount{false};
    quint64 inferenceDetectionCount{0};
    bool haveVisionProcessRss{false};
    quint64 visionProcessRssBytes{0};
    bool haveSystemTotalMemory{false};
    quint64 systemTotalMemoryBytes{0};
    QString inferenceLastError;
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
    void startInference();
    void stopInference();
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
    [[nodiscard]] bool actionBusy() const noexcept;
    [[nodiscard]] bool hasFreshStatus() const;
    [[nodiscard]] bool inferenceReconcilePending() const noexcept
    {
        return inferenceReconcilePending_;
    }
    [[nodiscard]] bool isPolling() const noexcept
    {
        return pollTimer_.isActive();
    }
    [[nodiscard]] VisionCaptureStatus status() const { return status_; }

signals:
    void statusChanged(rb::vision::VisionCaptureStatus status);
    void actionSucceeded(const QString &action);
    void errorOccurred(const QString &message);
    void logMessage(const QString &message);
    void requestStateChanged();
    void authoritativeStatusRefreshed();
    void inferenceActionAcknowledged(
        const QString &action,
        const QString &outcome);
    void requestFailed(
        const QString &action,
        const QString &code,
        const QString &message,
        bool outcomeUncertain);

private:
    enum class RequestKind {
        Status,
        Snapshot,
        StartRecording,
        StopRecording,
        StartInference,
        StopInference,
    };

    void issue(RequestKind kind, const QString &path, bool post);
    void handleFinished(
        QNetworkReply *reply,
        RequestKind kind,
        quint64 endpointGeneration);
    void dispatchPendingAction();
    void applyPayload(const QJsonObject &object, RequestKind kind);
    void applyStatusPayload(const QJsonObject &object);
    void applyCapturePayload(const QJsonObject &object);
    void resetInferenceDiagnostics();
    void resetInferenceMetadata();
    void invalidateFreshness();
    void expireFreshness();
    void requestInference(RequestKind kind);
    [[nodiscard]] bool inferenceActionAdmissible(
        RequestKind kind,
        QString *code = nullptr,
        QString *message = nullptr) const;
    [[nodiscard]] bool isInferenceRequest(RequestKind kind) const noexcept;
    [[nodiscard]] bool isKnownInferenceState() const noexcept;
    [[nodiscard]] bool isKnownInferenceOperation() const noexcept;
    void reportFailure(
        const QString &action,
        const QString &code,
        const QString &message,
        bool outcomeUncertain);
    void scheduleInferenceReconciliation();
    void resetEndpointState();
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
    quint64 activeRequestGeneration_{0};
    bool inferenceReconcilePending_{false};
    bool statusFresh_{false};
    QElapsedTimer lastStatusElapsed_;
    QTimer freshnessTimer_;
    bool replyFinalizing_{false};
    bool shuttingDown_{false};
    VisionCaptureStatus status_;
};

} // namespace rb::vision

Q_DECLARE_METATYPE(rb::vision::VisionCaptureStatus)
