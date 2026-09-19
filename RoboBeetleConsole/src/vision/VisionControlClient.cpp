#include "vision/VisionControlClient.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QNetworkProxy>
#include <QNetworkReply>
#include <QNetworkRequest>

namespace rb::vision {

VisionControlClient::VisionControlClient(QObject *parent)
    : QObject(parent)
{
    manager_.setProxy(QNetworkProxy::NoProxy);
    pollTimer_.setSingleShot(false);
    pollTimer_.setInterval(1000);
    connect(&pollTimer_, &QTimer::timeout, this, [this] {
        if (!requestInFlight_) {
            refreshStatus();
        }
    });
}

bool VisionControlClient::setEndpoint(const QString &host, quint16 port)
{
    const QString normalizedHost = host.trimmed();
    if (host_ == normalizedHost && port_ == port) {
        return true;
    }

    // Once a mutating request has been sent, aborting the local reply cannot
    // undo a POST that the old server may already have executed. Keep the
    // endpoint stable until that action has a definite response or timeout.
    if (actionInFlight()) {
        const QString message = QStringLiteral(
            "Vision capture endpoint cannot change while an action is in flight");
        emit errorOccurred(message);
        emit logMessage(message);
        return false;
    }

    ++endpointGeneration_;
    pendingAction_.reset();
    pendingPath_.clear();
    pendingPost_ = false;

    QNetworkReply *staleReply = activeReply_;
    activeReply_ = nullptr;
    requestInFlight_ = false;

    host_ = normalizedHost;
    port_ = port;
    status_ = VisionCaptureStatus{};
    emit statusChanged(status_);

    if (staleReply != nullptr) {
        staleReply->abort();
    }
    return true;
}

bool VisionControlClient::actionInFlight() const noexcept
{
    return requestInFlight_ && activeRequestKind_ != RequestKind::Status;
}

void VisionControlClient::refreshStatus()
{
    issue(
        RequestKind::Status,
        QStringLiteral("/api/v1/vision/status"),
        false);
}

void VisionControlClient::requestSnapshot()
{
    issue(
        RequestKind::Snapshot,
        QStringLiteral("/api/v1/vision/snapshot"),
        true);
}

void VisionControlClient::startRecording()
{
    issue(
        RequestKind::StartRecording,
        QStringLiteral("/api/v1/vision/recording/start"),
        true);
}

void VisionControlClient::stopRecording()
{
    issue(
        RequestKind::StopRecording,
        QStringLiteral("/api/v1/vision/recording/stop"),
        true);
}

void VisionControlClient::startPolling(int intervalMs)
{
    if (intervalMs > 0) {
        pollTimer_.setInterval(intervalMs);
    }
    pollTimer_.start();
    if (!requestInFlight_) {
        refreshStatus();
    }
}
void VisionControlClient::stopPolling()
{
    pollTimer_.stop();
}

void VisionControlClient::shutdown()
{
    pollTimer_.stop();
    ++endpointGeneration_;
    pendingAction_.reset();
    pendingPath_.clear();
    pendingPost_ = false;
    activeReply_ = nullptr;
    requestInFlight_ = false;

    const auto replies = manager_.findChildren<QNetworkReply *>();
    for (QNetworkReply *reply : replies) {
        reply->abort();
    }
}

void VisionControlClient::issue(
    RequestKind kind,
    const QString &path,
    bool post)
{
    if (host_.isEmpty() || port_ == 0U) {
        emit errorOccurred(QStringLiteral("Vision capture host and port must be valid"));
        return;
    }
    if (requestInFlight_) {
        if (kind == RequestKind::Status) {
            return;
        }
        if (activeRequestKind_ == RequestKind::Status
            && !pendingAction_.has_value()) {
            pendingAction_ = kind;
            pendingPath_ = path;
            pendingPost_ = post;
            return;
        }
        emit errorOccurred(
            QStringLiteral("Vision capture action already in flight"));
        return;
    }

    QNetworkRequest request(endpointUrl(path));
    request.setHeader(
        QNetworkRequest::ContentTypeHeader,
        QStringLiteral("application/json"));
    request.setTransferTimeout(
        kind == RequestKind::StopRecording ? 15000 : 2000);
    const quint64 requestGeneration = endpointGeneration_;
    QNetworkReply *reply = post
        ? manager_.post(request, QByteArrayLiteral("{}"))
        : manager_.get(request);
    requestInFlight_ = true;
    activeRequestKind_ = kind;
    activeReply_ = reply;
    connect(
        reply,
        &QNetworkReply::finished,
        this,
        [this, reply, kind, requestGeneration] {
            handleFinished(reply, kind, requestGeneration);
        });
}

void VisionControlClient::handleFinished(
    QNetworkReply *reply,
    RequestKind kind,
    quint64 endpointGeneration)
{
    const QByteArray body = reply->readAll();
    const auto networkError = reply->error();
    const QString networkErrorText = reply->errorString();

    if (reply == activeReply_) {
        activeReply_ = nullptr;
        requestInFlight_ = false;
    }
    reply->deleteLater();

    // A reply from an older endpoint must not update current status or release
    // a pending action onto the new endpoint.
    if (endpointGeneration != endpointGeneration_) {
        return;
    }

    const auto finishRequest = [this] {
        dispatchPendingAction();
    };

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(body, &parseError);
    if (parseError.error != QJsonParseError::NoError
        || !document.isObject()) {
        const QString message = networkError != QNetworkReply::NoError
            ? QStringLiteral("Vision capture network error: %1")
                  .arg(networkErrorText)
            : QStringLiteral("Vision capture returned invalid JSON");
        emit errorOccurred(message);
        emit logMessage(message);
        finishRequest();
        return;
    }
    const QJsonObject object = document.object();
    if (!object.value(QStringLiteral("ok")).toBool(false)) {
        const QString message = object.value(QStringLiteral("error"))
            .toString(QStringLiteral("Vision capture request failed"));
        emit errorOccurred(message);
        emit logMessage(QStringLiteral("Vision capture: %1").arg(message));
        finishRequest();
        return;
    }
    if (networkError != QNetworkReply::NoError) {
        const QString message =
            QStringLiteral("Vision capture network error: %1")
                .arg(networkErrorText);
        emit errorOccurred(message);
        emit logMessage(message);
        finishRequest();
        return;
    }

    applyPayload(object);
    if (kind != RequestKind::Status) {
        const QString action = actionName(kind);
        emit actionSucceeded(action);
        emit logMessage(
            QStringLiteral("Vision capture action succeeded: %1")
                .arg(action));
    }
    finishRequest();
}

void VisionControlClient::dispatchPendingAction()
{
    if (!pendingAction_.has_value() || requestInFlight_) {
        return;
    }
    const RequestKind kind = *pendingAction_;
    const QString path = pendingPath_;
    const bool post = pendingPost_;
    pendingAction_.reset();
    pendingPath_.clear();
    pendingPost_ = false;
    issue(kind, path, post);
}

void VisionControlClient::applyPayload(const QJsonObject &object)
{
    const QJsonObject camera =
        object.value(QStringLiteral("camera")).toObject();
    if (!camera.isEmpty()) {
        status_.cameraRunning =
            camera.value(QStringLiteral("running")).toBool(false);
        const QJsonValue latest =
            camera.value(QStringLiteral("latest_frame_id"));
        status_.haveLatestFrame = !latest.isNull() && !latest.isUndefined();
        if (status_.haveLatestFrame) {
            status_.latestFrameId =
                static_cast<quint64>(latest.toInteger());
        }
    }

    const QJsonObject capture =
        object.value(QStringLiteral("capture")).toObject();
    if (!capture.isEmpty()) {
        status_.state =
            capture.value(QStringLiteral("state")).toString(
                QStringLiteral("idle"));
        status_.recording =
            capture.value(QStringLiteral("recording")).toBool(false);
        status_.sessionId =
            capture.value(QStringLiteral("session_id")).toString();
        status_.segment =
            capture.value(QStringLiteral("segment")).toString();
        status_.recordedFrames = static_cast<quint64>(
            capture.value(QStringLiteral("recorded_frames")).toInteger());
        status_.snapshotCount = static_cast<quint64>(
            capture.value(QStringLiteral("snapshot_count")).toInteger());
        status_.queueBytes = static_cast<quint64>(
            capture.value(QStringLiteral("queue_bytes")).toInteger());
        status_.maxQueueBytes = static_cast<quint64>(
            capture.value(QStringLiteral("max_queue_bytes")).toInteger());
        const QJsonValue freeDisk =
            capture.value(QStringLiteral("free_disk_bytes"));
        status_.haveFreeDisk =
            !freeDisk.isNull() && !freeDisk.isUndefined();
        if (status_.haveFreeDisk) {
            status_.freeDiskBytes =
                static_cast<quint64>(freeDisk.toInteger());
        }
        status_.lastError =
            capture.value(QStringLiteral("last_error")).toString();
    }
    emit statusChanged(status_);
}

QUrl VisionControlClient::endpointUrl(const QString &path) const
{
    QUrl url;
    url.setScheme(QStringLiteral("http"));
    url.setHost(host_);
    url.setPort(port_);
    url.setPath(path);
    return url;
}

QString VisionControlClient::actionName(RequestKind kind)
{
    switch (kind) {
    case RequestKind::Snapshot:
        return QStringLiteral("snapshot");
    case RequestKind::StartRecording:
        return QStringLiteral("recording/start");
    case RequestKind::StopRecording:
        return QStringLiteral("recording/stop");
    case RequestKind::Status:
        return QStringLiteral("status");
    }
    return QStringLiteral("unknown");
}

} // namespace rb::vision
