#include "vision/VisionControlClient.h"

#include <QHostAddress>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QNetworkProxy>
#include <QNetworkReply>
#include <QNetworkRequest>

#include <cmath>
#include <limits>

namespace rb::vision {

namespace {

constexpr int kFreshnessWindowMs = 3500;

bool strictBool(const QJsonValue &value, bool *out)
{
    if (!value.isBool()) {
        return false;
    }
    *out = value.toBool();
    return true;
}

bool strictString(const QJsonValue &value, QString *out)
{
    if (!value.isString()) {
        return false;
    }
    *out = value.toString();
    return true;
}

bool validNumber(const QJsonValue &value)
{
    return value.isDouble();
}

std::optional<quint16> strictPort(const QJsonValue &value)
{
    if (!value.isDouble()) {
        return std::nullopt;
    }
    const double number = value.toDouble();
    if (!std::isfinite(number)
        || number < 1.0
        || number > 65535.0
        || std::floor(number) != number) {
        return std::nullopt;
    }
    return static_cast<quint16>(value.toInteger());
}

std::optional<int> strictPositiveInt(const QJsonValue &value)
{
    if (!value.isDouble()) {
        return std::nullopt;
    }
    const double number = value.toDouble();
    if (!std::isfinite(number)
        || number < 1.0
        || number > static_cast<double>(std::numeric_limits<int>::max())
        || std::floor(number) != number) {
        return std::nullopt;
    }
    return static_cast<int>(value.toInteger());
}

QString responseMessage(const QJsonObject &object, const QString &fallback)
{
    const QJsonValue message = object.value(QStringLiteral("message"));
    if (message.isString()) {
        return message.toString();
    }
    const QJsonValue error = object.value(QStringLiteral("error"));
    return error.isString() ? error.toString() : fallback;
}

} // namespace

VisionControlClient::VisionControlClient(QObject *parent)
    : QObject(parent)
{
    manager_.setProxy(QNetworkProxy::NoProxy);
    pollTimer_.setSingleShot(false);
    pollTimer_.setInterval(1000);
    freshnessTimer_.setSingleShot(true);
    freshnessTimer_.setInterval(kFreshnessWindowMs);
    connect(&pollTimer_, &QTimer::timeout, this, [this] {
        if (!requestInFlight_ && !shuttingDown_) {
            refreshStatus();
        }
    });
    connect(&freshnessTimer_, &QTimer::timeout, this, [this] {
        expireFreshness();
    });
}

bool VisionControlClient::setEndpoint(const QString &host, quint16 port)
{
    QString normalizedHost = host.trimmed().toLower();
    QHostAddress address;
    if (address.setAddress(normalizedHost)) {
        normalizedHost = address.toString();
    }
    if (host_ == normalizedHost && port_ == port) {
        return true;
    }

    if (actionInFlight() || replyFinalizing_) {
        const QString message = QStringLiteral(
            "Vision capture endpoint cannot change while an action is in flight");
        emit errorOccurred(message);
        emit logMessage(message);
        return false;
    }

    const QString queuedAction = pendingAction_.has_value()
        ? actionName(*pendingAction_)
        : QString();
    ++endpointGeneration_;
    pendingAction_.reset();
    pendingPath_.clear();
    pendingPost_ = false;

    QNetworkReply *staleReply = activeReply_;
    activeReply_ = nullptr;
    requestInFlight_ = false;
    replyFinalizing_ = false;
    host_ = normalizedHost;
    port_ = port;
    resetEndpointState();

    if (staleReply != nullptr) {
        staleReply->abort();
    }
    if (!queuedAction.isEmpty()) {
        reportFailure(
            queuedAction,
            QStringLiteral("endpoint_changed"),
            QStringLiteral("Vision endpoint changed before the queued action was sent"),
            false);
    }
    emit requestStateChanged();
    return true;
}

bool VisionControlClient::actionInFlight() const noexcept
{
    return requestInFlight_ && activeRequestKind_ != RequestKind::Status;
}

bool VisionControlClient::actionBusy() const noexcept
{
    return actionInFlight() || pendingAction_.has_value() || replyFinalizing_;
}

bool VisionControlClient::hasFreshStatus() const
{
    return statusFresh_ && lastStatusElapsed_.isValid()
        && lastStatusElapsed_.elapsed() < kFreshnessWindowMs;
}

void VisionControlClient::refreshStatus()
{
    issue(RequestKind::Status, QStringLiteral("/api/v1/vision/status"), false);
}

void VisionControlClient::requestSnapshot()
{
    issue(RequestKind::Snapshot, QStringLiteral("/api/v1/vision/snapshot"), true);
}

void VisionControlClient::startRecording()
{
    issue(RequestKind::StartRecording,
          QStringLiteral("/api/v1/vision/recording/start"), true);
}

void VisionControlClient::stopRecording()
{
    issue(RequestKind::StopRecording,
          QStringLiteral("/api/v1/vision/recording/stop"), true);
}

void VisionControlClient::startInference()
{
    requestInference(RequestKind::StartInference);
}

void VisionControlClient::stopInference()
{
    requestInference(RequestKind::StopInference);
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
    freshnessTimer_.stop();
    ++endpointGeneration_;
    pendingAction_.reset();
    pendingPath_.clear();
    pendingPost_ = false;
    activeReply_ = nullptr;
    requestInFlight_ = false;
    replyFinalizing_ = false;
    inferenceReconcilePending_ = false;
    shuttingDown_ = true;
    invalidateFreshness();
    resetInferenceMetadata();

    const auto replies = manager_.findChildren<QNetworkReply *>();
    for (QNetworkReply *reply : replies) {
        reply->abort();
    }
    emit requestStateChanged();
}

void VisionControlClient::issue(RequestKind kind, const QString &path, bool post)
{
    if (shuttingDown_) {
        reportFailure(actionName(kind), QStringLiteral("service_shutting_down"),
                      QStringLiteral("Vision control client is shutting down"), false);
        return;
    }
    if (host_.isEmpty() || port_ == 0U) {
        const QString message = QStringLiteral("Vision capture host and port must be valid");
        if (isInferenceRequest(kind)) {
            reportFailure(actionName(kind), QStringLiteral("endpoint_invalid"), message, false);
        } else {
            emit errorOccurred(message);
            emit logMessage(message);
        }
        return;
    }

    if (requestInFlight_) {
        if (kind == RequestKind::Status) {
            return;
        }
        if (activeRequestKind_ == RequestKind::Status && !pendingAction_.has_value()) {
            pendingAction_ = kind;
            pendingPath_ = path;
            pendingPost_ = post;
            emit requestStateChanged();
            return;
        }
        if (isInferenceRequest(kind)) {
            reportFailure(actionName(kind), QStringLiteral("action_busy"),
                          QStringLiteral("Another Vision action is already in flight"), false);
        } else {
            emit errorOccurred(QStringLiteral("Vision capture action already in flight"));
        }
        return;
    }

    QNetworkRequest request(endpointUrl(path));
    request.setHeader(QNetworkRequest::ContentTypeHeader, QStringLiteral("application/json"));
    request.setAttribute(QNetworkRequest::RedirectPolicyAttribute,
                         QNetworkRequest::ManualRedirectPolicy);
    request.setTransferTimeout(kind == RequestKind::StopRecording ? 15000 : 2000);
    const quint64 requestGeneration = endpointGeneration_;
    QNetworkReply *reply = post ? manager_.post(request, QByteArrayLiteral("{}"))
                                : manager_.get(request);
    activeRequestGeneration_ = requestGeneration;
    requestInFlight_ = true;
    activeRequestKind_ = kind;
    activeReply_ = reply;
    emit requestStateChanged();
    connect(reply, &QNetworkReply::finished, this,
            [this, reply, kind, requestGeneration] {
                handleFinished(reply, kind, requestGeneration);
            });
}

void VisionControlClient::handleFinished(QNetworkReply *reply,
                                         RequestKind kind,
                                         quint64 endpointGeneration)
{
    const QByteArray body = reply->readAll();
    const auto networkError = reply->error();
    const QString networkErrorText = reply->errorString();
    const int httpStatus = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
    const bool isCurrent = reply == activeReply_ && endpointGeneration == endpointGeneration_
        && !shuttingDown_;
    if (!isCurrent) {
        reply->deleteLater();
        return;
    }

    activeReply_ = nullptr;
    requestInFlight_ = false;
    replyFinalizing_ = kind != RequestKind::Status;
    const QString action = actionName(kind);

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(body, &parseError);
    const bool jsonObject = parseError.error == QJsonParseError::NoError && document.isObject();
    const QJsonObject object = jsonObject ? document.object() : QJsonObject{};
    const bool redirect = httpStatus >= 300 && httpStatus < 400;
    const bool appError = jsonObject && object.value(QStringLiteral("ok")).isBool()
        && !object.value(QStringLiteral("ok")).toBool();
    const bool ok = jsonObject && object.value(QStringLiteral("ok")).isBool()
        && object.value(QStringLiteral("ok")).toBool();
    const bool transportFailure = networkError != QNetworkReply::NoError;

    auto finishStatusFailure = [&](const QString &code, const QString &message) {
        invalidateFreshness();
        if (pendingAction_.has_value()) {
            const RequestKind pending = *pendingAction_;
            pendingAction_.reset();
            pendingPath_.clear();
            pendingPost_ = false;
            reportFailure(actionName(pending), code,
                          QStringLiteral("Queued action canceled: %1").arg(message), false);
        }
        inferenceReconcilePending_ = false;
        emit errorOccurred(message);
        emit logMessage(QStringLiteral("Vision capture [%1] %2: %3")
                            .arg(action, code, message));
    };

    if (kind == RequestKind::Status) {
        if (redirect) {
            finishStatusFailure(QStringLiteral("redirect_rejected"),
                                QStringLiteral("Vision status redirect was rejected"));
        } else if (!jsonObject) {
            const QString message = transportFailure
                ? QStringLiteral("Vision capture network error: %1").arg(networkErrorText)
                : QStringLiteral("Vision capture returned invalid JSON");
            finishStatusFailure(transportFailure ? QStringLiteral("network_error")
                                                 : QStringLiteral("invalid_response"), message);
        } else if (appError || !ok) {
            finishStatusFailure(object.value(QStringLiteral("error"))
                                    .toString(QStringLiteral("request_failed")),
                                responseMessage(object,
                                                QStringLiteral("Vision status request failed")));
        } else if (transportFailure || httpStatus < 200 || httpStatus >= 300) {
            finishStatusFailure(QStringLiteral("network_error"),
                                QStringLiteral("Vision capture network error: %1")
                                    .arg(networkErrorText));
        } else {
            applyPayload(object, RequestKind::Status);
            statusFresh_ = true;
            lastStatusElapsed_.start();
            freshnessTimer_.start(kFreshnessWindowMs);
            inferenceReconcilePending_ = false;
            emit authoritativeStatusRefreshed();
            dispatchPendingAction();
            emit requestStateChanged();
        }
        replyFinalizing_ = false;
        emit requestStateChanged();
        reply->deleteLater();
        return;
    }

    const bool inference = isInferenceRequest(kind);
    bool successfulCapture = false;
    bool successfulInference = false;
    QString failureCode;
    QString failureMessage;
    bool uncertain = false;

    if (redirect) {
        failureCode = QStringLiteral("redirect_rejected");
        failureMessage = QStringLiteral("Vision control redirect was rejected");
        uncertain = true;
    } else if (!jsonObject) {
        failureCode = transportFailure ? QStringLiteral("network_error")
                                       : QStringLiteral("invalid_response");
        failureMessage = transportFailure
            ? QStringLiteral("Vision capture network error: %1").arg(networkErrorText)
            : QStringLiteral("Vision capture returned invalid JSON");
        uncertain = true;
    } else if (appError || !ok) {
        failureCode = object.value(QStringLiteral("error"))
            .toString(QStringLiteral("request_failed"));
        failureMessage = responseMessage(object,
                                          QStringLiteral("Vision capture request failed"));
        uncertain = false;
    } else if (transportFailure || httpStatus < 200 || httpStatus >= 300) {
        failureCode = QStringLiteral("network_error");
        failureMessage = QStringLiteral("Vision capture network error: %1")
            .arg(networkErrorText);
        uncertain = true;
    } else if (inference) {
        const QString responseAction = object.value(QStringLiteral("action")).toString();
        const QString outcome = object.value(QStringLiteral("outcome")).toString();
        const bool validStatus = (httpStatus == 202 && outcome == QStringLiteral("accepted"))
            || (httpStatus == 200 &&
                ((kind == RequestKind::StartInference &&
                  (outcome == QStringLiteral("already_starting") ||
                   outcome == QStringLiteral("already_running") ||
                   outcome == QStringLiteral("already_retrying"))) ||
                 (kind == RequestKind::StopInference &&
                  (outcome == QStringLiteral("already_disabled") ||
                   outcome == QStringLiteral("already_stopping")))));
        if (responseAction != action || !validStatus) {
            failureCode = QStringLiteral("invalid_response");
            failureMessage = QStringLiteral("Vision inference acknowledgement did not match the request");
            uncertain = true;
        } else {
            successfulInference = true;
            scheduleInferenceReconciliation();
            emit inferenceActionAcknowledged(action, outcome);
        }
    } else {
        successfulCapture = true;
        applyPayload(object, kind);
        emit actionSucceeded(action);
        emit logMessage(QStringLiteral("Vision capture action succeeded: %1").arg(action));
    }

    if (inference && !successfulInference) {
        scheduleInferenceReconciliation();
        reportFailure(action, failureCode, failureMessage, uncertain);
    } else if (!inference && !successfulCapture) {
        if (!requestInFlight_) {
            issue(RequestKind::Status, QStringLiteral("/api/v1/vision/status"), false);
        }
        reportFailure(action, failureCode, failureMessage, uncertain);
    }
    replyFinalizing_ = false;
    emit requestStateChanged();
    reply->deleteLater();
}

void VisionControlClient::dispatchPendingAction()
{
    if (!pendingAction_.has_value() || requestInFlight_ || shuttingDown_) {
        return;
    }
    const RequestKind kind = *pendingAction_;
    const QString path = pendingPath_;
    const bool post = pendingPost_;
    pendingAction_.reset();
    pendingPath_.clear();
    pendingPost_ = false;
    if (isInferenceRequest(kind)) {
        QString code;
        QString message;
        if (!inferenceActionAdmissible(kind, &code, &message)) {
            reportFailure(actionName(kind), code, message, false);
            emit requestStateChanged();
            return;
        }
    }
    issue(kind, path, post);
}

void VisionControlClient::applyPayload(const QJsonObject &object, RequestKind kind)
{
    if (kind == RequestKind::Status) {
        applyStatusPayload(object);
    } else if (!isInferenceRequest(kind)) {
        applyCapturePayload(object);
    }
    if (kind != RequestKind::Status && !isInferenceRequest(kind)) {
        emit statusChanged(status_);
    } else if (kind == RequestKind::Status) {
        emit statusChanged(status_);
    }
}

void VisionControlClient::applyStatusPayload(const QJsonObject &object)
{
    status_.haveCameraStatus = false;
    status_.haveCaptureStatus = false;
    status_.haveInferenceStatus = false;
    status_.haveInferenceState = false;
    status_.inferenceConfigured.reset();
    status_.inferenceControlSupported.reset();
    status_.detectionStreamSupported.reset();
    status_.detectionStreamPort.reset();
    status_.detectionStreamVersion.reset();
    status_.inferenceOperationValid = false;
    status_.inferenceOperation.clear();

    const QJsonObject camera = object.value(QStringLiteral("camera")).toObject();
    bool cameraRunning = false;
    if (!camera.isEmpty() && strictBool(camera.value(QStringLiteral("running")), &cameraRunning)) {
        status_.haveCameraStatus = true;
        status_.cameraRunning = cameraRunning;
        const QJsonValue latest = camera.value(QStringLiteral("latest_frame_id"));
        status_.haveLatestFrame = validNumber(latest);
        status_.latestFrameId = status_.haveLatestFrame ? static_cast<quint64>(latest.toInteger()) : 0U;
    } else {
        status_.cameraRunning = false;
        status_.haveLatestFrame = false;
        status_.latestFrameId = 0U;
    }

    const QJsonObject capture = object.value(QStringLiteral("capture")).toObject();
    bool recording = false;
    QString captureState;
    const bool haveRecording = strictBool(capture.value(QStringLiteral("recording")), &recording);
    const bool haveCaptureState = strictString(capture.value(QStringLiteral("state")), &captureState);
    if (!capture.isEmpty()) {
        if (haveRecording && haveCaptureState) {
            status_.haveCaptureStatus = true;
        }
        if (haveRecording) {
            status_.recording = recording;
        }
        if (haveCaptureState) {
            status_.state = captureState;
        }
        status_.sessionId = capture.value(QStringLiteral("session_id")).toString();
        status_.segment = capture.value(QStringLiteral("segment")).toString();
        status_.recordedFrames = static_cast<quint64>(capture.value(QStringLiteral("recorded_frames")).toInteger());
        status_.snapshotCount = static_cast<quint64>(capture.value(QStringLiteral("snapshot_count")).toInteger());
        status_.queueBytes = static_cast<quint64>(capture.value(QStringLiteral("queue_bytes")).toInteger());
        status_.maxQueueBytes = static_cast<quint64>(capture.value(QStringLiteral("max_queue_bytes")).toInteger());
        const QJsonValue freeDisk = capture.value(QStringLiteral("free_disk_bytes"));
        status_.haveFreeDisk = validNumber(freeDisk);
        status_.freeDiskBytes = status_.haveFreeDisk ? static_cast<quint64>(freeDisk.toInteger()) : 0U;
        status_.lastError = capture.value(QStringLiteral("last_error")).toString();
    } else {
        status_.recording = false;
        status_.state = QStringLiteral("idle");
    }

    const QJsonValue inferenceValue = object.value(QStringLiteral("inference"));
    if (!inferenceValue.isObject()) {
        resetInferenceDiagnostics();
        return;
    }
    status_.haveInferenceStatus = true;
    resetInferenceDiagnostics();
    const QJsonObject inference = inferenceValue.toObject();
    bool boolValue = false;
    if (strictBool(inference.value(QStringLiteral("configured")), &boolValue)) {
        status_.inferenceConfigured = boolValue;
    }
    if (strictBool(inference.value(QStringLiteral("control_supported")), &boolValue)) {
        status_.inferenceControlSupported = boolValue;
    }
    if (strictBool(
            inference.value(QStringLiteral("detection_stream_supported")),
            &boolValue)) {
        status_.detectionStreamSupported = boolValue;
    }
    status_.detectionStreamPort = strictPort(
        inference.value(QStringLiteral("detection_stream_port")));
    status_.detectionStreamVersion = strictPositiveInt(
        inference.value(QStringLiteral("detection_stream_version")));

    QString state;
    if (strictString(inference.value(QStringLiteral("state")), &state) && !state.isEmpty()) {
        status_.haveInferenceState = true;
        status_.inferenceState = state;
    }
    const QJsonValue operation = inference.value(QStringLiteral("operation"));
    if (operation.isNull()) {
        status_.inferenceOperationValid = true;
        status_.inferenceOperation.clear();
    } else if (operation.isString()
               && (operation.toString() == QStringLiteral("stopping")
                   || operation.toString() == QStringLiteral("retrying"))) {
        status_.inferenceOperationValid = true;
        status_.inferenceOperation = operation.toString();
    }
    status_.inferenceArtifactName = inference.value(QStringLiteral("artifact_name")).toString();
    status_.inferenceModelSha256 = inference.value(QStringLiteral("model_sha256")).toString();
    const auto readOptionalDouble = [&inference](const QString &name, bool &have, double &value) {
        const QJsonValue field = inference.value(name);
        have = validNumber(field);
        value = have ? field.toDouble() : 0.0;
    };
    const auto readOptionalUInt64 = [&inference](const QString &name, bool &have, quint64 &value) {
        const QJsonValue field = inference.value(name);
        have = validNumber(field);
        value = have ? static_cast<quint64>(field.toInteger()) : 0U;
    };
    readOptionalDouble(QStringLiteral("confidence_threshold"), status_.haveInferenceConfidenceThreshold,
                       status_.inferenceConfidenceThreshold);
    readOptionalUInt64(QStringLiteral("latest_frame_id"), status_.haveInferenceLatestFrame,
                       status_.inferenceLatestFrameId);
    readOptionalUInt64(QStringLiteral("capture_timestamp_ns"), status_.haveInferenceCaptureTimestampNs,
                       status_.inferenceCaptureTimestampNs);
    readOptionalUInt64(QStringLiteral("processed_frames"), status_.haveInferenceProcessedFrames,
                       status_.inferenceProcessedFrames);
    readOptionalUInt64(QStringLiteral("skipped_frames"), status_.haveInferenceSkippedFrames,
                       status_.inferenceSkippedFrames);
    readOptionalDouble(QStringLiteral("inference_fps"), status_.haveInferenceFps, status_.inferenceFps);
    readOptionalDouble(QStringLiteral("latency_ms"), status_.haveInferenceLatencyMs, status_.inferenceLatencyMs);
    readOptionalUInt64(QStringLiteral("detection_count"), status_.haveInferenceDetectionCount,
                       status_.inferenceDetectionCount);
    readOptionalUInt64(QStringLiteral("vision_process_rss_bytes"),
                       status_.haveVisionProcessRss,
                       status_.visionProcessRssBytes);
    readOptionalUInt64(QStringLiteral("system_total_memory_bytes"),
                       status_.haveSystemTotalMemory,
                       status_.systemTotalMemoryBytes);
    status_.inferenceLastError = inference.value(QStringLiteral("last_error")).toString();
}

void VisionControlClient::applyCapturePayload(const QJsonObject &object)
{
    const QJsonObject camera = object.value(QStringLiteral("camera")).toObject();
    bool cameraRunning = false;
    if (!camera.isEmpty() && strictBool(camera.value(QStringLiteral("running")), &cameraRunning)) {
        status_.haveCameraStatus = true;
        status_.cameraRunning = cameraRunning;
        const QJsonValue latest = camera.value(QStringLiteral("latest_frame_id"));
        status_.haveLatestFrame = validNumber(latest);
        status_.latestFrameId = status_.haveLatestFrame ? static_cast<quint64>(latest.toInteger()) : 0U;
    }
    const QJsonObject capture = object.value(QStringLiteral("capture")).toObject();
    bool recording = false;
    QString captureState;
    const bool haveRecording = strictBool(capture.value(QStringLiteral("recording")), &recording);
    const bool haveCaptureState = strictString(capture.value(QStringLiteral("state")), &captureState);
    if (!capture.isEmpty()) {
        if (haveRecording && haveCaptureState) {
            status_.haveCaptureStatus = true;
        }
        if (haveRecording) {
            status_.recording = recording;
        }
        if (haveCaptureState) {
            status_.state = captureState;
        }
        status_.sessionId = capture.value(QStringLiteral("session_id")).toString();
        status_.segment = capture.value(QStringLiteral("segment")).toString();
        status_.recordedFrames = static_cast<quint64>(capture.value(QStringLiteral("recorded_frames")).toInteger());
        status_.snapshotCount = static_cast<quint64>(capture.value(QStringLiteral("snapshot_count")).toInteger());
        status_.queueBytes = static_cast<quint64>(capture.value(QStringLiteral("queue_bytes")).toInteger());
        status_.maxQueueBytes = static_cast<quint64>(capture.value(QStringLiteral("max_queue_bytes")).toInteger());
        const QJsonValue freeDisk = capture.value(QStringLiteral("free_disk_bytes"));
        status_.haveFreeDisk = validNumber(freeDisk);
        status_.freeDiskBytes = status_.haveFreeDisk ? static_cast<quint64>(freeDisk.toInteger()) : 0U;
        status_.lastError = capture.value(QStringLiteral("last_error")).toString();
    }
}

void VisionControlClient::resetInferenceDiagnostics()
{
    status_.inferenceState = QStringLiteral("disabled");
    status_.inferenceArtifactName.clear();
    status_.inferenceModelSha256.clear();
    status_.haveInferenceConfidenceThreshold = false;
    status_.inferenceConfidenceThreshold = 0.0;
    status_.haveInferenceLatestFrame = false;
    status_.inferenceLatestFrameId = 0;
    status_.haveInferenceCaptureTimestampNs = false;
    status_.inferenceCaptureTimestampNs = 0;
    status_.haveInferenceProcessedFrames = false;
    status_.inferenceProcessedFrames = 0;
    status_.haveInferenceSkippedFrames = false;
    status_.inferenceSkippedFrames = 0;
    status_.haveInferenceFps = false;
    status_.inferenceFps = 0.0;
    status_.haveInferenceLatencyMs = false;
    status_.inferenceLatencyMs = 0.0;
    status_.haveInferenceDetectionCount = false;
    status_.inferenceDetectionCount = 0;
    status_.haveVisionProcessRss = false;
    status_.visionProcessRssBytes = 0;
    status_.haveSystemTotalMemory = false;
    status_.systemTotalMemoryBytes = 0;
    status_.inferenceLastError.clear();
}

void VisionControlClient::resetInferenceMetadata()
{
    status_.haveInferenceStatus = false;
    status_.haveInferenceState = false;
    status_.inferenceConfigured.reset();
    status_.inferenceControlSupported.reset();
    status_.detectionStreamSupported.reset();
    status_.detectionStreamPort.reset();
    status_.detectionStreamVersion.reset();
    status_.inferenceOperationValid = false;
    status_.inferenceOperation.clear();
    resetInferenceDiagnostics();
}

void VisionControlClient::resetEndpointState()
{
    invalidateFreshness();
    inferenceReconcilePending_ = false;
    status_ = VisionCaptureStatus{};
    resetInferenceMetadata();
    emit statusChanged(status_);
}

void VisionControlClient::invalidateFreshness()
{
    statusFresh_ = false;
    freshnessTimer_.stop();
}

void VisionControlClient::expireFreshness()
{
    if (!statusFresh_) {
        return;
    }
    statusFresh_ = false;
    emit requestStateChanged();
}

void VisionControlClient::requestInference(RequestKind kind)
{
    const QString action = actionName(kind);
    if (shuttingDown_) {
        reportFailure(action, QStringLiteral("service_shutting_down"),
                      QStringLiteral("Vision control client is shutting down"), false);
        return;
    }
    if (inferenceReconcilePending_) {
        reportFailure(action, QStringLiteral("inference_reconcile_pending"),
                      QStringLiteral("Wait for the authoritative inference status refresh"), false);
        return;
    }
    if (actionBusy()) {
        reportFailure(action, QStringLiteral("action_busy"),
                      QStringLiteral("Another Vision action is already in flight"), false);
        return;
    }
    QString code;
    QString message;
    if (!inferenceActionAdmissible(kind, &code, &message)) {
        reportFailure(action, code, message, false);
        return;
    }
    const QString path = kind == RequestKind::StartInference
        ? QStringLiteral("/api/v1/vision/inference/start")
        : QStringLiteral("/api/v1/vision/inference/stop");
    if (requestInFlight_) {
        if (activeRequestKind_ == RequestKind::Status && !pendingAction_.has_value()) {
            pendingAction_ = kind;
            pendingPath_ = path;
            pendingPost_ = true;
            emit requestStateChanged();
            return;
        }
        reportFailure(action, QStringLiteral("action_busy"),
                      QStringLiteral("Another Vision action is already in flight"), false);
        return;
    }
    issue(kind, path, true);
}

bool VisionControlClient::inferenceActionAdmissible(RequestKind kind,
                                                     QString *code,
                                                     QString *message) const
{
    const auto fail = [&](const QString &value, const QString &detail) {
        if (code != nullptr) {
            *code = value;
        }
        if (message != nullptr) {
            *message = detail;
        }
        return false;
    };
    if (!hasFreshStatus() || !status_.haveInferenceStatus || !status_.haveInferenceState
        || !status_.inferenceConfigured.has_value() || !*status_.inferenceConfigured
        || !status_.inferenceControlSupported.has_value() || !*status_.inferenceControlSupported
        || !status_.inferenceOperationValid || !status_.inferenceOperation.isEmpty()
        || !isKnownInferenceState()) {
        return fail(QStringLiteral("inference_unavailable"),
                    QStringLiteral("Inference control is unavailable until a valid status is confirmed"));
    }
    if (kind == RequestKind::StartInference
        && (!status_.haveCameraStatus || !status_.cameraRunning)) {
        return fail(QStringLiteral("camera_not_running"),
                    QStringLiteral("Start inference requires a confirmed running camera"));
    }
    return true;
}

bool VisionControlClient::isInferenceRequest(RequestKind kind) const noexcept
{
    return kind == RequestKind::StartInference || kind == RequestKind::StopInference;
}

bool VisionControlClient::isKnownInferenceState() const noexcept
{
    return status_.inferenceState == QStringLiteral("disabled")
        || status_.inferenceState == QStringLiteral("starting")
        || status_.inferenceState == QStringLiteral("running")
        || status_.inferenceState == QStringLiteral("failed");
}

bool VisionControlClient::isKnownInferenceOperation() const noexcept
{
    return status_.inferenceOperationValid && status_.inferenceOperation.isEmpty();
}

void VisionControlClient::reportFailure(const QString &action,
                                        const QString &code,
                                        const QString &message,
                                        bool outcomeUncertain)
{
    emit requestFailed(action, code, message, outcomeUncertain);
    emit errorOccurred(message);
    emit logMessage(QStringLiteral("Vision capture [%1] %2: %3").arg(action, code, message));
}

void VisionControlClient::scheduleInferenceReconciliation()
{
    inferenceReconcilePending_ = true;
    emit requestStateChanged();
    if (!requestInFlight_ && !shuttingDown_) {
        issue(RequestKind::Status, QStringLiteral("/api/v1/vision/status"), false);
    }
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
    case RequestKind::StartInference:
        return QStringLiteral("inference/start");
    case RequestKind::StopInference:
        return QStringLiteral("inference/stop");
    case RequestKind::Status:
        return QStringLiteral("status");
    }
    return QStringLiteral("unknown");
}

} // namespace rb::vision
