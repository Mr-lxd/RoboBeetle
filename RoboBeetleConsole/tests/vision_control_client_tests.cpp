#include "vision/VisionControlClient.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QHostAddress>
#include <QNetworkProxy>
#include <QTcpServer>
#include <QTcpSocket>
#include <QThread>

#include <cstdio>
#include <cstdlib>
#include <functional>

namespace {

using namespace rb::vision;

int failures = 0;

void expect(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

bool waitUntil(const std::function<bool()> &predicate, int timeoutMs = 2000)
{
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeoutMs) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        if (predicate()) {
            return true;
        }
        QThread::msleep(2);
    }
    QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    return predicate();
}

QTcpSocket *acceptClient(QTcpServer &server)
{
    if (!waitUntil([&server] { return server.hasPendingConnections(); })) {
        return nullptr;
    }
    return server.nextPendingConnection();
}

QByteArray readRequest(QTcpSocket *socket)
{
    QByteArray request;
    waitUntil([&] {
        if (socket->bytesAvailable() > 0) {
            request += socket->readAll();
        }
        return request.contains("\r\n\r\n");
    });
    return request;
}

void waitForMs(int timeoutMs)
{
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeoutMs) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        QThread::msleep(2);
    }
}

QByteArray readRequestWithBody(QTcpSocket *socket, int bodyBytes)
{
    QByteArray request = readRequest(socket);
    const qsizetype headerEnd = request.indexOf("\r\n\r\n");
    if (headerEnd < 0) {
        return request;
    }
    waitUntil([&] {
        if (request.size() < headerEnd + 4 + bodyBytes
            && socket->bytesAvailable() > 0) {
            request += socket->readAll();
        }
        return request.size() >= headerEnd + 4 + bodyBytes;
    });
    return request;
}

QByteArray statusReason(int status)
{
    switch (status) {
    case 200: return QByteArrayLiteral("OK");
    case 202: return QByteArrayLiteral("Accepted");
    case 301: return QByteArrayLiteral("Moved Permanently");
    case 302: return QByteArrayLiteral("Found");
    case 400: return QByteArrayLiteral("Bad Request");
    case 409: return QByteArrayLiteral("Conflict");
    case 500: return QByteArrayLiteral("Internal Server Error");
    default: return QByteArrayLiteral("Response");
    }
}

void sendResponse(
    QTcpSocket *socket,
    int status,
    const QByteArray &body,
    const QByteArray &extraHeaders = {})
{
    QByteArray response =
        "HTTP/1.1 " + QByteArray::number(status) + " " + statusReason(status) + "\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: " + QByteArray::number(body.size()) + "\r\n"
        + extraHeaders
        + "Connection: close\r\n\r\n" + body;
    socket->write(response);
    socket->flush();
    socket->waitForBytesWritten(100);
    socket->disconnectFromHost();
}

void sendJson(QTcpSocket *socket, int status, const QByteArray &body)
{
    sendResponse(socket, status, body);
}

QByteArray statusBody(
    const QByteArray &inference,
    bool cameraRunning = true,
    bool recording = false)
{
    QByteArray body = QByteArrayLiteral(
        "{\"ok\":true,\"camera\":{\"running\":");
    body += cameraRunning ? QByteArrayLiteral("true") : QByteArrayLiteral("false");
    body += QByteArrayLiteral(
        ",\"latest_frame_id\":0},\"capture\":{\"state\":\"");
    body += recording ? QByteArrayLiteral("recording") : QByteArrayLiteral("idle");
    body += recording
        ? QByteArrayLiteral("\",\"recording\":true,\"recorded_frames\":0}")
        : QByteArrayLiteral("\",\"recording\":false,\"recorded_frames\":0}");
    body += QByteArrayLiteral(",\"inference\":");
    body += inference;
    body += QByteArrayLiteral("}");
    return body;
}

QByteArray configuredInference(
    const QByteArray &state,
    const QByteArray &operation = QByteArrayLiteral("null"))
{
    return QByteArrayLiteral(
        "{\"configured\":true,\"control_supported\":true,\"operation\":")
        + operation
        + QByteArrayLiteral(",\"state\":")
        + state
        + QByteArrayLiteral(
              ",\"artifact_name\":null,\"model_sha256\":null,"
              "\"last_error\":null}");
}

void statusBypassesProxyAndParsesJson()
{
    const QNetworkProxy originalProxy = QNetworkProxy::applicationProxy();
    QNetworkProxy::setApplicationProxy(
        QNetworkProxy(
            QNetworkProxy::HttpProxy,
            QStringLiteral("127.0.0.1"),
            9));

    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "Vision control fake server must listen");

    VisionControlClient client;
    client.setEndpoint(
        QStringLiteral("127.0.0.1"),
        server.serverPort());

    bool statusSeen = false;
    VisionCaptureStatus seen;
    QObject::connect(
        &client,
        &VisionControlClient::statusChanged,
        [&statusSeen, &seen](VisionCaptureStatus status) {
            statusSeen = true;
            seen = status;
        });

    client.refreshStatus();
    QTcpSocket *peer = acceptClient(server);
    expect(peer != nullptr,
           "VisionControlClient bypasses application proxy");
    if (peer != nullptr) {
        const QByteArray request = readRequest(peer);
        expect(
            request.startsWith("GET /api/v1/vision/status HTTP/1.1"),
            "status request uses frozen capture-control path");
        sendJson(
            peer,
            200,
            QByteArrayLiteral(
                "{\"ok\":true,"
                "\"camera\":{\"running\":true,\"latest_frame_id\":42},"
                "\"capture\":{\"state\":\"recording\","
                "\"recording\":true,\"session_id\":\"capture-001\","
                "\"segment\":\"raw.avi\",\"recorded_frames\":17,"
                "\"snapshot_count\":2,\"queue_bytes\":4096,"
                "\"max_queue_bytes\":67108864,"
                "\"free_disk_bytes\":2147483648,\"last_error\":null}}"));
        expect(waitUntil([&statusSeen] { return statusSeen; }),
               "status JSON produces statusChanged");
        expect(seen.cameraRunning && seen.haveLatestFrame,
               "camera status is parsed");
        expect(seen.latestFrameId == 42U,
               "latest frame id is parsed");
        expect(seen.recording && seen.state == QStringLiteral("recording"),
               "recording state is parsed");
        expect(seen.recordedFrames == 17U && seen.snapshotCount == 2U,
               "capture counters are parsed");
        expect(seen.haveFreeDisk && seen.freeDiskBytes == 2147483648ULL,
               "free disk diagnostics are parsed");
        peer->deleteLater();
    }

    client.shutdown();
    QNetworkProxy::setApplicationProxy(originalProxy);
}

void actionQueuesBehindStatusPollInsteadOfFailing()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "queued-action fake server must listen");

    VisionControlClient client;
    client.setEndpoint(
        QStringLiteral("127.0.0.1"),
        server.serverPort());

    QString error;
    QString succeeded;
    QObject::connect(
        &client,
        &VisionControlClient::errorOccurred,
        [&error](const QString &message) { error = message; });
    QObject::connect(
        &client,
        &VisionControlClient::actionSucceeded,
        [&succeeded](const QString &action) { succeeded = action; });

    client.refreshStatus();
    QTcpSocket *statusPeer = acceptClient(server);
    expect(statusPeer != nullptr, "status poll connects");
    if (statusPeer == nullptr) {
        return;
    }
    const QByteArray statusRequest = readRequest(statusPeer);
    expect(
        statusRequest.startsWith("GET /api/v1/vision/status HTTP/1.1"),
        "first request is status");

    client.requestSnapshot();
    expect(error.isEmpty(),
           "user action queues while status request is in flight");

    sendJson(
        statusPeer,
        200,
        QByteArrayLiteral(
            "{\"ok\":true,"
            "\"camera\":{\"running\":true,\"latest_frame_id\":1},"
            "\"capture\":{\"state\":\"idle\",\"recording\":false,"
            "\"recorded_frames\":0,\"snapshot_count\":0,"
            "\"queue_bytes\":0,\"max_queue_bytes\":67108864,"
            "\"free_disk_bytes\":1073741824,\"last_error\":null}}"));

    QTcpSocket *actionPeer = acceptClient(server);
    expect(actionPeer != nullptr,
           "queued user action dispatches immediately after status");
    if (actionPeer != nullptr) {
        const QByteArray actionRequest = readRequest(actionPeer);
        expect(
            actionRequest.startsWith(
                "POST /api/v1/vision/snapshot HTTP/1.1"),
            "queued request preserves snapshot action");
        sendJson(
            actionPeer,
            200,
            QByteArrayLiteral(
                "{\"ok\":true,\"capture\":{\"state\":\"idle\","
                "\"recording\":false,\"recorded_frames\":0,"
                "\"snapshot_count\":1,\"queue_bytes\":0,"
                "\"max_queue_bytes\":67108864,"
                "\"free_disk_bytes\":1073741824,"
                "\"last_error\":null}}"));
        expect(waitUntil([&succeeded] { return succeeded == QStringLiteral("snapshot"); }),
               "queued snapshot eventually succeeds");
        actionPeer->deleteLater();
    }

    statusPeer->deleteLater();
    client.shutdown();
}

void statusParsesRunningInferenceDiagnostics()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "inference status fake server must listen");

    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());

    client.refreshStatus();
    QTcpSocket *peer = acceptClient(server);
    expect(peer != nullptr, "inference status request connects");
    if (peer == nullptr) {
        return;
    }
    readRequest(peer);
    sendJson(
        peer,
        200,
        QByteArrayLiteral(
            "{\"ok\":true,\"inference\":{"
            "\"state\":\"running\","
            "\"artifact_name\":\"beetle.onnx\","
            "\"model_sha256\":\"abc123\","
            "\"confidence_threshold\":0.75,"
            "\"latest_frame_id\":0,"
            "\"capture_timestamp_ns\":123456789,"
            "\"processed_frames\":12,"
            "\"skipped_frames\":3,"
            "\"inference_fps\":8.5,"
            "\"latency_ms\":4.25,"
            "\"detection_count\":2,"
            "\"last_error\":null}}"));

    expect(
        waitUntil([&client] {
            return client.status().inferenceState == QStringLiteral("running");
        }),
        "running inference status is applied");
    const VisionCaptureStatus seen = client.status();
    expect(seen.inferenceArtifactName == QStringLiteral("beetle.onnx"),
           "inference artifact name is parsed");
    expect(seen.inferenceModelSha256 == QStringLiteral("abc123"),
           "inference model hash is parsed");
    expect(seen.haveInferenceConfidenceThreshold
               && seen.inferenceConfidenceThreshold == 0.75,
           "inference confidence threshold is parsed");
    expect(seen.haveInferenceLatestFrame && seen.inferenceLatestFrameId == 0U,
           "inference frame id zero remains valid");
    expect(seen.haveInferenceCaptureTimestampNs
               && seen.inferenceCaptureTimestampNs == 123456789ULL,
           "inference capture timestamp is parsed");
    expect(seen.haveInferenceProcessedFrames
               && seen.inferenceProcessedFrames == 12U
               && seen.haveInferenceSkippedFrames
               && seen.inferenceSkippedFrames == 3U,
           "inference frame counters are parsed");
    expect(seen.haveInferenceFps && seen.inferenceFps == 8.5
               && seen.haveInferenceLatencyMs && seen.inferenceLatencyMs == 4.25,
           "inference rate and latency are parsed");
    expect(seen.haveInferenceDetectionCount
               && seen.inferenceDetectionCount == 2U,
           "inference detection count is parsed");
    expect(seen.inferenceLastError.isEmpty(),
           "null inference last error stays absent");

    peer->deleteLater();
    client.shutdown();
}

void failedInferencePreservesNullFlagsAndNumericZero()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "failed inference fake server must listen");

    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());

    client.refreshStatus();
    QTcpSocket *peer = acceptClient(server);
    expect(peer != nullptr, "failed inference status request connects");
    if (peer == nullptr) {
        return;
    }
    readRequest(peer);
    sendJson(
        peer,
        200,
        QByteArrayLiteral(
            "{\"ok\":true,\"inference\":{"
            "\"state\":\"failed\","
            "\"artifact_name\":null,\"model_sha256\":null,"
            "\"confidence_threshold\":null,"
            "\"latest_frame_id\":0,"
            "\"capture_timestamp_ns\":null,"
            "\"processed_frames\":null,\"skipped_frames\":null,"
            "\"inference_fps\":null,\"latency_ms\":null,"
            "\"detection_count\":null,"
            "\"last_error\":\"model load failed\"}}"));

    expect(
        waitUntil([&client] {
            return client.status().inferenceState == QStringLiteral("failed");
        }),
        "failed inference state is applied");
    const VisionCaptureStatus seen = client.status();
    expect(!seen.haveInferenceConfidenceThreshold
               && !seen.haveInferenceCaptureTimestampNs
               && !seen.haveInferenceProcessedFrames
               && !seen.haveInferenceSkippedFrames
               && !seen.haveInferenceFps
               && !seen.haveInferenceLatencyMs
               && !seen.haveInferenceDetectionCount,
           "null inference numeric fields remain absent");
    expect(seen.haveInferenceLatestFrame && seen.inferenceLatestFrameId == 0U,
           "failed inference frame id zero remains present");
    expect(seen.inferenceLastError == QStringLiteral("model load failed"),
           "failed inference error is parsed");

    peer->deleteLater();
    client.shutdown();
}

void legacyStatusResetsInferenceAfterActionPreservesIt()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "legacy inference fake server must listen");

    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());

    client.refreshStatus();
    QTcpSocket *statusPeer = acceptClient(server);
    expect(statusPeer != nullptr, "initial inference status request connects");
    if (statusPeer == nullptr) {
        return;
    }
    readRequest(statusPeer);
    sendJson(
        statusPeer,
        200,
        QByteArrayLiteral(
            "{\"ok\":true,\"inference\":{\"state\":\"running\","
            "\"latest_frame_id\":7,\"processed_frames\":11}}"));
    expect(
        waitUntil([&client] {
            return client.status().inferenceState == QStringLiteral("running");
        }),
        "initial inference status becomes running");
    statusPeer->deleteLater();

    client.requestSnapshot();
    QTcpSocket *actionPeer = acceptClient(server);
    expect(actionPeer != nullptr, "snapshot action request connects");
    if (actionPeer == nullptr) {
        client.shutdown();
        return;
    }
    readRequest(actionPeer);
    sendJson(
        actionPeer,
        200,
        QByteArrayLiteral(
            "{\"ok\":true,\"capture\":{\"state\":\"idle\","
            "\"snapshot_count\":1}}"));
    expect(
        waitUntil([&client] {
            return client.status().snapshotCount == 1U;
        }),
        "snapshot action succeeds without inference payload");
    expect(client.status().inferenceState == QStringLiteral("running")
               && client.status().haveInferenceLatestFrame
               && client.status().inferenceLatestFrameId == 7U,
           "action response without inference preserves existing diagnostics");
    actionPeer->deleteLater();

    client.startRecording();
    QTcpSocket *startPeer = acceptClient(server);
    expect(startPeer != nullptr, "recording start action request connects");
    if (startPeer == nullptr) {
        client.shutdown();
        return;
    }
    const QByteArray startRequest = readRequest(startPeer);
    expect(
        startRequest.startsWith(
            "POST /api/v1/vision/recording/start HTTP/1.1"),
        "recording start action uses the expected control path");
    sendJson(
        startPeer,
        200,
        QByteArrayLiteral(
            "{\"ok\":true,\"capture\":{\"state\":\"recording\","
            "\"recording\":true}}"));
    expect(
        waitUntil([&client] {
            return client.status().state == QStringLiteral("recording");
        }),
        "recording start action succeeds without inference payload");
    expect(client.status().inferenceState == QStringLiteral("running")
               && client.status().haveInferenceLatestFrame
               && client.status().inferenceLatestFrameId == 7U,
           "recording start without inference preserves existing diagnostics");
    startPeer->deleteLater();

    client.stopRecording();
    QTcpSocket *stopPeer = acceptClient(server);
    expect(stopPeer != nullptr, "recording stop action request connects");
    if (stopPeer == nullptr) {
        client.shutdown();
        return;
    }
    const QByteArray stopRequest = readRequest(stopPeer);
    expect(
        stopRequest.startsWith(
            "POST /api/v1/vision/recording/stop HTTP/1.1"),
        "recording stop action uses the expected control path");
    sendJson(
        stopPeer,
        200,
        QByteArrayLiteral(
            "{\"ok\":true,\"capture\":{\"state\":\"idle\","
            "\"recording\":false}}"));
    expect(
        waitUntil([&client] {
            return client.status().state == QStringLiteral("idle");
        }),
        "recording stop action succeeds without inference payload");
    expect(client.status().inferenceState == QStringLiteral("running")
               && client.status().haveInferenceLatestFrame
               && client.status().inferenceLatestFrameId == 7U,
           "recording stop without inference preserves existing diagnostics");
    stopPeer->deleteLater();

    client.refreshStatus();
    QTcpSocket *legacyPeer = acceptClient(server);
    expect(legacyPeer != nullptr, "legacy status request connects");
    if (legacyPeer != nullptr) {
        readRequest(legacyPeer);
        sendJson(
            legacyPeer,
            200,
            QByteArrayLiteral(
                "{\"ok\":true,\"capture\":{\"state\":\"idle\","
                "\"snapshot_count\":2}}"));
        expect(
            waitUntil([&client] {
                return client.status().snapshotCount == 2U;
            }),
            "legacy status response is applied");
        const VisionCaptureStatus seen = client.status();
        expect(seen.inferenceState == QStringLiteral("disabled"),
               "legacy status disables inference");
        expect(!seen.haveInferenceLatestFrame
                   && !seen.haveInferenceProcessedFrames
                   && seen.inferenceLatestFrameId == 0U
                   && seen.inferenceProcessedFrames == 0U,
               "legacy status restores disabled inference defaults");
        legacyPeer->deleteLater();
    }

    client.shutdown();
}

void actionPathsAndErrorsAreSurfaced()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "action fake server must listen");

    VisionControlClient client;
    client.setEndpoint(
        QStringLiteral("127.0.0.1"),
        server.serverPort());

    QString succeeded;
    QString error;
    QObject::connect(
        &client,
        &VisionControlClient::actionSucceeded,
        [&succeeded](const QString &action) { succeeded = action; });
    QObject::connect(
        &client,
        &VisionControlClient::errorOccurred,
        [&error](const QString &message) { error = message; });

    client.requestSnapshot();
    QTcpSocket *snapshotPeer = acceptClient(server);
    expect(snapshotPeer != nullptr, "snapshot request connects");
    if (snapshotPeer != nullptr) {
        const QByteArray request = readRequest(snapshotPeer);
        expect(
            request.startsWith("POST /api/v1/vision/snapshot HTTP/1.1"),
            "snapshot uses dedicated control path");
        sendJson(
            snapshotPeer,
            200,
            QByteArrayLiteral(
                "{\"ok\":true,\"capture\":{\"state\":\"idle\","
                "\"recording\":false,\"snapshot_count\":3}}"));
        expect(waitUntil([&succeeded] { return !succeeded.isEmpty(); }),
               "successful action is signalled");
        expect(succeeded == QStringLiteral("snapshot"),
               "snapshot action name is stable");
        expect(client.status().snapshotCount == 3U,
               "action response updates capture status");
        snapshotPeer->deleteLater();
    }

    succeeded.clear();
    client.startRecording();
    QTcpSocket *startPeer = acceptClient(server);
    expect(startPeer != nullptr, "recording start request connects");
    if (startPeer != nullptr) {
        const QByteArray request = readRequest(startPeer);
        expect(
            request.startsWith(
                "POST /api/v1/vision/recording/start HTTP/1.1"),
            "recording start uses dedicated control path");
        sendJson(
            startPeer,
            409,
            QByteArrayLiteral(
                "{\"ok\":false,\"error\":\"recording is already active\"}"));
        expect(waitUntil([&error] { return !error.isEmpty(); }),
               "HTTP JSON failure is surfaced");
        expect(error.contains(QStringLiteral("already active")),
               "server capture error text is preserved");
        startPeer->deleteLater();
    }

    client.shutdown();
}

void endpointChangeCancelsOldPendingAction()
{
    QTcpServer serverA;
    QTcpServer serverB;
    expect(serverA.listen(QHostAddress::LocalHost, 0),
           "endpoint-A fake server must listen");
    expect(serverB.listen(QHostAddress::LocalHost, 0),
           "endpoint-B fake server must listen");

    VisionControlClient client;
    client.setEndpoint(
        QStringLiteral("127.0.0.1"),
        serverA.serverPort());

    bool staleStatusApplied = false;
    QObject::connect(
        &client,
        &VisionControlClient::statusChanged,
        [&staleStatusApplied](VisionCaptureStatus status) {
            if (status.haveLatestFrame && status.latestFrameId == 111U) {
                staleStatusApplied = true;
            }
        });

    client.refreshStatus();
    QTcpSocket *peerA = acceptClient(serverA);
    expect(peerA != nullptr, "old endpoint status request connects");
    if (peerA == nullptr) {
        return;
    }
    readRequest(peerA);

    client.stopRecording();
    client.setEndpoint(
        QStringLiteral("127.0.0.1"),
        serverB.serverPort());

    QElapsedTimer quiet;
    quiet.start();
    while (quiet.elapsed() < 150) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        QThread::msleep(2);
    }

    expect(!serverB.hasPendingConnections(),
           "old pending action is not dispatched to new endpoint");
    expect(!client.requestInFlight(),
           "endpoint change cancels old in-flight request");
    expect(!staleStatusApplied,
           "old endpoint status is not applied after endpoint change");

    client.refreshStatus();
    QTcpSocket *peerB = acceptClient(serverB);
    expect(peerB != nullptr, "new endpoint receives fresh status request");
    if (peerB != nullptr) {
        const QByteArray request = readRequest(peerB);
        expect(
            request.startsWith("GET /api/v1/vision/status HTTP/1.1"),
            "new endpoint receives status rather than old queued action");
        sendJson(
            peerB,
            200,
            QByteArrayLiteral(
                "{\"ok\":true,"
                "\"camera\":{\"running\":true,\"latest_frame_id\":222},"
                "\"capture\":{\"state\":\"idle\",\"recording\":false,"
                "\"recorded_frames\":0,\"snapshot_count\":0,"
                "\"queue_bytes\":0,\"max_queue_bytes\":67108864,"
                "\"free_disk_bytes\":1073741824,\"last_error\":null}}"));
        expect(
            waitUntil([&client] {
                const auto status = client.status();
                return status.haveLatestFrame && status.latestFrameId == 222U;
            }),
            "new endpoint status becomes authoritative");
        peerB->deleteLater();
    }

    peerA->deleteLater();
    client.shutdown();
}

void activeActionBlocksEndpointChangeUntilResult()
{
    QTcpServer serverA;
    QTcpServer serverB;
    expect(serverA.listen(QHostAddress::LocalHost, 0),
           "active-action endpoint-A server must listen");
    expect(serverB.listen(QHostAddress::LocalHost, 0),
           "active-action endpoint-B server must listen");

    VisionControlClient client;
    client.setEndpoint(
        QStringLiteral("127.0.0.1"),
        serverA.serverPort());

    QString error;
    QString succeeded;
    QObject::connect(
        &client,
        &VisionControlClient::errorOccurred,
        [&error](const QString &message) { error = message; });
    QObject::connect(
        &client,
        &VisionControlClient::actionSucceeded,
        [&succeeded](const QString &action) { succeeded = action; });

    client.startRecording();
    QTcpSocket *peerA = acceptClient(serverA);
    expect(peerA != nullptr, "Start Recording reaches endpoint A");
    if (peerA == nullptr) {
        return;
    }
    const QByteArray request = readRequest(peerA);
    expect(
        request.startsWith(
            "POST /api/v1/vision/recording/start HTTP/1.1"),
        "active request is recording/start");
    expect(client.actionInFlight(),
           "mutating request is reported as action in flight");

    const QString oldHost = client.host();
    const quint16 oldPort = client.port();
    expect(
        !client.setEndpoint(
            QStringLiteral("127.0.0.1"),
            serverB.serverPort()),
        "endpoint change is rejected while action is in flight");
    expect(client.host() == oldHost && client.port() == oldPort,
           "rejected endpoint change preserves endpoint A");
    expect(client.actionInFlight() && client.requestInFlight(),
           "rejected endpoint change keeps old action tracked");
    expect(error.contains(QStringLiteral("cannot change")),
           "blocked endpoint change is surfaced");
    expect(!serverB.hasPendingConnections(),
           "blocked endpoint change sends nothing to endpoint B");

    sendJson(
        peerA,
        200,
        QByteArrayLiteral(
            "{\"ok\":true,\"capture\":{\"state\":\"recording\","
            "\"recording\":true,\"recorded_frames\":0,"
            "\"snapshot_count\":0,\"queue_bytes\":0,"
            "\"max_queue_bytes\":67108864,"
            "\"free_disk_bytes\":1073741824,\"last_error\":null}}"));

    expect(
        waitUntil([&] {
            return !client.actionInFlight()
                && succeeded == QStringLiteral("recording/start");
        }),
        "endpoint A action reaches a definite result");

    expect(
        client.setEndpoint(
            QStringLiteral("127.0.0.1"),
            serverB.serverPort()),
        "endpoint may change after action completion");
    expect(client.port() == serverB.serverPort(),
           "completed action permits endpoint B");

    peerA->deleteLater();
    client.shutdown();
}

void stopRecordingAllowsLongerServerFinalization()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "slow-stop fake server must listen");

    VisionControlClient client;
    client.setEndpoint(
        QStringLiteral("127.0.0.1"),
        server.serverPort());

    QString succeeded;
    QString error;
    QObject::connect(
        &client,
        &VisionControlClient::actionSucceeded,
        [&succeeded](const QString &action) { succeeded = action; });
    QObject::connect(
        &client,
        &VisionControlClient::errorOccurred,
        [&error](const QString &message) { error = message; });

    client.stopRecording();
    QTcpSocket *peer = acceptClient(server);
    expect(peer != nullptr, "slow Stop Recording request connects");
    if (peer == nullptr) {
        return;
    }
    const QByteArray request = readRequest(peer);
    expect(
        request.startsWith(
            "POST /api/v1/vision/recording/stop HTTP/1.1"),
        "slow action is Stop Recording");

    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < 2200) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        QThread::msleep(2);
    }

    expect(error.isEmpty(),
           "Stop Recording is not aborted by the normal 2 second timeout");
    expect(client.requestInFlight(),
           "Stop Recording remains in flight while server finalizes");

    sendJson(
        peer,
        200,
        QByteArrayLiteral(
            "{\"ok\":true,\"capture\":{\"state\":\"idle\","
            "\"recording\":false,\"recorded_frames\":10,"
            "\"snapshot_count\":0,\"queue_bytes\":0,"
            "\"max_queue_bytes\":67108864,"
            "\"free_disk_bytes\":1073741824,\"last_error\":null}}"));

    expect(
        waitUntil([&succeeded] {
            return succeeded == QStringLiteral("recording/stop");
        }),
        "slow Stop Recording eventually succeeds");

    peer->deleteLater();
    client.shutdown();
}

void configured_disabled_status_parses_capabilities_and_null_operation()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "configured status server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());

    client.refreshStatus();
    QTcpSocket *peer = acceptClient(server);
    expect(peer != nullptr, "configured status request connects");
    if (peer == nullptr) {
        return;
    }
    readRequest(peer);
    sendJson(
        peer,
        200,
        statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));

    expect(waitUntil([&client] {
        return client.status().haveInferenceStatus;
    }), "configured inference object is received");
    const VisionCaptureStatus seen = client.status();
    expect(seen.haveCameraStatus && seen.haveCaptureStatus,
           "camera and capture presence flags are strict");
    expect(seen.haveInferenceState
               && seen.inferenceState == QStringLiteral("disabled"),
           "disabled worker state is parsed");
    expect(seen.inferenceConfigured.has_value()
               && *seen.inferenceConfigured,
           "configured capability is parsed as a bool");
    expect(seen.inferenceControlSupported.has_value()
               && *seen.inferenceControlSupported,
           "control_supported capability is parsed as a bool");
    expect(seen.inferenceOperationValid && seen.inferenceOperation.isEmpty(),
           "JSON null operation is valid idle metadata");
    expect(client.hasFreshStatus(), "successful GET is fresh");

    peer->deleteLater();
    client.shutdown();
}

void strict_capability_and_operation_impostors_fail_closed()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "strict metadata server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    QString failureCode;
    QObject::connect(
        &client,
        &VisionControlClient::requestFailed,
        [&failureCode](const QString &, const QString &code,
                       const QString &, bool) { failureCode = code; });

    client.refreshStatus();
    QTcpSocket *peer = acceptClient(server);
    expect(peer != nullptr, "strict metadata request connects");
    if (peer == nullptr) {
        return;
    }
    readRequest(peer);
    sendJson(
        peer,
        200,
        statusBody(QByteArrayLiteral(
            "{\"configured\":1,\"control_supported\":\"true\","
            "\"state\":\"running\",\"last_error\":null}")));
    expect(waitUntil([&client] {
        return client.status().haveInferenceStatus;
    }), "impostor status is received");
    const VisionCaptureStatus first = client.status();
    expect(!first.inferenceConfigured.has_value()
               && !first.inferenceControlSupported.has_value()
               && !first.inferenceOperationValid,
           "numeric/string capability impostors remain absent");
    client.startInference();
    expect(failureCode == QStringLiteral("inference_unavailable"),
           "impostor capabilities fail closed without a POST");
    expect(!server.hasPendingConnections(),
           "impostor capabilities do not create an action request");
    peer->deleteLater();

    client.refreshStatus();
    peer = acceptClient(server);
    expect(peer != nullptr, "unknown operation request connects");
    if (peer != nullptr) {
        readRequest(peer);
        sendJson(
            peer,
            200,
            statusBody(QByteArrayLiteral(
                "{\"configured\":true,\"control_supported\":true,"
                "\"operation\":\"idle\",\"state\":\"running\","
                "\"last_error\":null}")));
        expect(waitUntil([&client] {
            return client.status().inferenceOperationValid == false;
        }), "unknown operation stays invalid");
        peer->deleteLater();
    }
    client.shutdown();
}

void legacy_status_is_readable_but_manual_actions_are_unavailable()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "legacy status server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    QString failureCode;
    QObject::connect(
        &client,
        &VisionControlClient::requestFailed,
        [&failureCode](const QString &, const QString &code,
                       const QString &, bool) { failureCode = code; });

    client.refreshStatus();
    QTcpSocket *peer = acceptClient(server);
    expect(peer != nullptr, "legacy status request connects");
    if (peer == nullptr) {
        return;
    }
    readRequest(peer);
    sendJson(
        peer,
        200,
        QByteArrayLiteral(
            "{\"ok\":true,\"camera\":{\"running\":true},"
            "\"capture\":{\"state\":\"idle\",\"recording\":false}}"));
    expect(waitUntil([&client] {
        return client.hasFreshStatus();
    }), "legacy status remains readable");
    expect(!client.status().haveInferenceStatus
               && client.status().inferenceState == QStringLiteral("disabled"),
           "legacy status resets inference presence without proving Disabled");

    client.startInference();
    expect(failureCode == QStringLiteral("inference_unavailable"),
           "legacy status cannot enable manual inference");
    expect(!server.hasPendingConnections(),
           "legacy start sends no unsupported inference POST");
    peer->deleteLater();
    client.shutdown();
}

void capture_post_preserves_inference_metadata_and_freshness()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "capture compatibility server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());

    client.refreshStatus();
    QTcpSocket *statusPeer = acceptClient(server);
    expect(statusPeer != nullptr, "initial inference status connects");
    if (statusPeer == nullptr) {
        return;
    }
    readRequest(statusPeer);
    sendJson(
        statusPeer,
        200,
        statusBody(QByteArrayLiteral(
            "{\"configured\":true,\"control_supported\":true,"
            "\"operation\":null,\"state\":\"running\","
            "\"artifact_name\":\"beetle.onnx\","
            "\"model_sha256\":\"abc\",\"last_error\":null}")));
    expect(waitUntil([&client] {
        return client.status().inferenceState == QStringLiteral("running");
    }), "initial running inference status arrives");
    statusPeer->deleteLater();

    client.requestSnapshot();
    QTcpSocket *actionPeer = acceptClient(server);
    expect(actionPeer != nullptr, "capture action connects");
    if (actionPeer == nullptr) {
        client.shutdown();
        return;
    }
    readRequest(actionPeer);
    sendJson(
        actionPeer,
        200,
        QByteArrayLiteral(
            "{\"ok\":true,\"capture\":{\"state\":\"idle\","
            "\"recording\":false,\"snapshot_count\":1},"
            "\"inference\":{\"state\":\"disabled\"}}"));
    expect(waitUntil([&client] {
        return client.status().snapshotCount == 1U;
    }), "capture action response is applied");
    expect(client.status().inferenceState == QStringLiteral("running")
               && client.status().inferenceArtifactName == QStringLiteral("beetle.onnx")
               && client.hasFreshStatus(),
           "capture POST cannot replace or renew the last inference GET");
    actionPeer->deleteLater();
    client.shutdown();
}

void zero_metrics_and_full_sha_are_preserved()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "zero-metric server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    client.refreshStatus();
    QTcpSocket *peer = acceptClient(server);
    expect(peer != nullptr, "zero-metric status connects");
    if (peer == nullptr) {
        return;
    }
    readRequest(peer);
    sendJson(
        peer,
        200,
        statusBody(QByteArrayLiteral(
            "{\"configured\":true,\"control_supported\":true,"
            "\"operation\":null,\"state\":\"running\","
            "\"artifact_name\":\"beetle.onnx\","
            "\"model_sha256\":\"012345678901234567890123456789012345678901234567890123456789abcd\","
            "\"confidence_threshold\":0,\"latest_frame_id\":0,"
            "\"capture_timestamp_ns\":0,\"processed_frames\":0,"
            "\"skipped_frames\":0,\"inference_fps\":0,\"latency_ms\":0,"
            "\"detection_count\":0,\"last_error\":null}")));
    expect(waitUntil([&client] {
        return client.status().haveInferenceStatus;
    }), "zero-metric status arrives");
    const auto seen = client.status();
    expect(seen.inferenceModelSha256.size() == 64,
           "complete model SHA is retained");
    expect(seen.haveInferenceConfidenceThreshold
               && seen.inferenceConfidenceThreshold == 0.0
               && seen.haveInferenceLatestFrame
               && seen.inferenceLatestFrameId == 0U
               && seen.haveInferenceProcessedFrames
               && seen.inferenceProcessedFrames == 0U
               && seen.haveInferenceDetectionCount
               && seen.inferenceDetectionCount == 0U,
           "zero metrics remain present rather than becoming absent");
    peer->deleteLater();
    client.shutdown();
}

void manual_start_and_stop_use_exact_paths_body_and_injected_port()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "manual action server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());

    client.refreshStatus();
    QTcpSocket *statusPeer = acceptClient(server);
    expect(statusPeer != nullptr, "manual action initial status connects");
    if (statusPeer == nullptr) {
        return;
    }
    readRequest(statusPeer);
    sendJson(
        statusPeer,
        200,
        statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
    expect(waitUntil([&client] { return client.hasFreshStatus(); }),
           "manual action status is fresh");
    statusPeer->deleteLater();

    client.startInference();
    QTcpSocket *startPeer = acceptClient(server);
    expect(startPeer != nullptr, "manual Start reaches injected HTTP port");
    if (startPeer == nullptr) {
        client.shutdown();
        return;
    }
    const QByteArray startRequest = readRequestWithBody(startPeer, 2);
    expect(startRequest.startsWith(
               "POST /api/v1/vision/inference/start HTTP/1.1"),
           "Start uses the exact inference/start path");
    expect(startRequest.contains("Content-Type: application/json")
               && startRequest.endsWith("{}"),
           "Start sends JSON empty object body and content type");
    sendJson(
        startPeer,
        202,
        QByteArrayLiteral(
            "{\"ok\":true,\"action\":\"inference/start\","
            "\"outcome\":\"accepted\"}"));
    startPeer->deleteLater();

    QTcpSocket *startRefreshPeer = acceptClient(server);
    expect(startRefreshPeer != nullptr,
           "accepted Start schedules one authoritative GET");
    if (startRefreshPeer != nullptr) {
        expect(readRequest(startRefreshPeer).startsWith(
                   "GET /api/v1/vision/status HTTP/1.1"),
               "Start reconciliation uses GET status");
        sendJson(
            startRefreshPeer,
            200,
            statusBody(configuredInference(QByteArrayLiteral("\"running\""))));
        startRefreshPeer->deleteLater();
    }
    expect(waitUntil([&client] {
        return client.status().inferenceState == QStringLiteral("running")
            && !client.inferenceReconcilePending();
    }), "Start does not claim running until GET confirms it");

    client.stopInference();
    QTcpSocket *stopPeer = acceptClient(server);
    expect(stopPeer != nullptr, "manual Stop reaches injected HTTP port");
    if (stopPeer != nullptr) {
        const QByteArray stopRequest = readRequestWithBody(stopPeer, 2);
        expect(stopRequest.startsWith(
                   "POST /api/v1/vision/inference/stop HTTP/1.1"),
               "Stop uses the exact inference/stop path");
        expect(stopRequest.contains("Content-Type: application/json")
                   && stopRequest.endsWith("{}"),
               "Stop sends JSON empty object body and content type");
        sendJson(
            stopPeer,
            202,
            QByteArrayLiteral(
                "{\"ok\":true,\"action\":\"inference/stop\","
                "\"outcome\":\"accepted\"}"));
        stopPeer->deleteLater();
    }
    QTcpSocket *stopRefreshPeer = acceptClient(server);
    expect(stopRefreshPeer != nullptr,
           "accepted Stop schedules one authoritative GET");
    if (stopRefreshPeer != nullptr) {
        readRequest(stopRefreshPeer);
        sendJson(
            stopRefreshPeer,
            200,
            statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
        stopRefreshPeer->deleteLater();
    }
    expect(waitUntil([&client] {
        return client.status().inferenceState == QStringLiteral("disabled")
            && !client.inferenceReconcilePending();
    }), "Stop is confirmed only by a disabled idle GET");
    client.shutdown();
}

void retry_and_clear_error_send_one_post_each()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "retry server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());

    client.refreshStatus();
    QTcpSocket *initial = acceptClient(server);
    expect(initial != nullptr, "failed status connects");
    if (initial == nullptr) {
        return;
    }
    readRequest(initial);
    sendJson(
        initial,
        200,
        statusBody(QByteArrayLiteral(
            "{\"configured\":true,\"control_supported\":true,"
            "\"operation\":null,\"state\":\"failed\","
            "\"last_error\":\"model load failed\"}")));
    expect(waitUntil([&client] {
        return client.status().inferenceState == QStringLiteral("failed");
    }), "failed status enables retry path");
    initial->deleteLater();

    client.startInference();
    QTcpSocket *retry = acceptClient(server);
    expect(retry != nullptr, "Retry uses a Start request");
    if (retry != nullptr) {
        expect(readRequestWithBody(retry, 2).startsWith(
                   "POST /api/v1/vision/inference/start HTTP/1.1"),
               "Retry sends exactly the Start path");
        sendJson(
            retry,
            202,
            QByteArrayLiteral(
                "{\"ok\":true,\"action\":\"inference/start\","
                "\"outcome\":\"accepted\"}"));
        retry->deleteLater();
    }
    QTcpSocket *retryRefresh = acceptClient(server);
    if (retryRefresh != nullptr) {
        readRequest(retryRefresh);
        sendJson(
            retryRefresh,
            200,
            statusBody(QByteArrayLiteral(
                "{\"configured\":true,\"control_supported\":true,"
                "\"operation\":null,\"state\":\"failed\","
                "\"last_error\":\"retry failed\"}")));
        retryRefresh->deleteLater();
    }
    expect(waitUntil([&client] { return !client.inferenceReconcilePending(); }),
           "retry reconciliation completes before Clear Error");
    expect(!server.hasPendingConnections(),
           "Retry does not create an automatic second Start or Stop");

    client.stopInference();
    QTcpSocket *clear = acceptClient(server);
    expect(clear != nullptr, "Clear Error uses a Stop request");
    if (clear != nullptr) {
        expect(readRequestWithBody(clear, 2).startsWith(
                   "POST /api/v1/vision/inference/stop HTTP/1.1"),
               "Clear Error sends exactly the Stop path");
        sendJson(
            clear,
            202,
            QByteArrayLiteral(
                "{\"ok\":true,\"action\":\"inference/stop\","
                "\"outcome\":\"accepted\"}"));
        clear->deleteLater();
    }
    QTcpSocket *clearRefresh = acceptClient(server);
    if (clearRefresh != nullptr) {
        readRequest(clearRefresh);
        sendJson(
            clearRefresh,
            200,
            statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
        clearRefresh->deleteLater();
    }
    expect(!server.hasPendingConnections(),
           "Clear Error does not create an automatic second Stop");
    client.shutdown();
}

void inference_ack_preserves_state_and_reconciles_once()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "ACK reconciliation server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    client.refreshStatus();
    QTcpSocket *initial = acceptClient(server);
    expect(initial != nullptr, "ACK initial status connects");
    if (initial == nullptr) {
        return;
    }
    readRequest(initial);
    sendJson(
        initial,
        200,
        statusBody(configuredInference(QByteArrayLiteral("\"running\""))));
    expect(waitUntil([&client] {
        return client.status().inferenceState == QStringLiteral("running");
    }), "ACK test starts from running state");
    initial->deleteLater();

    QString acknowledgedOutcome;
    QObject::connect(
        &client,
        &VisionControlClient::inferenceActionAcknowledged,
        [&acknowledgedOutcome](const QString &, const QString &outcome) {
            acknowledgedOutcome = outcome;
        });
    client.startInference();
    QTcpSocket *action = acceptClient(server);
    expect(action != nullptr, "duplicate Start connects");
    if (action == nullptr) {
        client.shutdown();
        return;
    }
    readRequestWithBody(action, 2);
    sendJson(
        action,
        200,
        QByteArrayLiteral(
            "{\"ok\":true,\"action\":\"inference/start\","
            "\"outcome\":\"already_running\","
            "\"inference\":{\"state\":\"disabled\"}}"));
    expect(waitUntil([&acknowledgedOutcome] {
        return !acknowledgedOutcome.isEmpty();
    }), "duplicate ACK is processed");
    expect(client.status().inferenceState == QStringLiteral("running"),
           "ACK fake status fields cannot replace the last GET state");
    expect(client.inferenceReconcilePending(),
           "terminal ACK marks reconciliation before signal delivery");
    expect(acknowledgedOutcome == QStringLiteral("already_running"),
           "duplicate ACK emits typed outcome");
    action->deleteLater();

    QTcpSocket *refresh = acceptClient(server);
    expect(refresh != nullptr, "duplicate ACK schedules one GET");
    if (refresh != nullptr) {
        readRequest(refresh);
        sendJson(
            refresh,
            200,
            statusBody(configuredInference(QByteArrayLiteral("\"running\""))));
        refresh->deleteLater();
    }
    expect(waitUntil([&client] {
        return !client.inferenceReconcilePending();
    }), "one reconciliation GET resolves the ACK");
    expect(!server.hasPendingConnections(),
           "one inference ACK does not create a GET loop");
    client.shutdown();
}

void malformed_or_fake_inference_ack_cannot_change_state()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "malformed ACK server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    client.refreshStatus();
    QTcpSocket *initial = acceptClient(server);
    expect(initial != nullptr, "malformed ACK initial status connects");
    if (initial == nullptr) {
        return;
    }
    readRequest(initial);
    sendJson(
        initial,
        200,
        statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
    expect(waitUntil([&client] { return client.hasFreshStatus(); }),
           "malformed ACK starts from fresh disabled status");
    initial->deleteLater();

    QString failureCode;
    bool uncertain = false;
    QObject::connect(
        &client,
        &VisionControlClient::requestFailed,
        [&failureCode, &uncertain](const QString &, const QString &code,
                                   const QString &, bool outcomeUncertain) {
            failureCode = code;
            uncertain = outcomeUncertain;
        });
    client.startInference();
    QTcpSocket *action = acceptClient(server);
    expect(action != nullptr, "malformed ACK action connects");
    if (action != nullptr) {
        readRequestWithBody(action, 2);
        sendJson(
            action,
            202,
            QByteArrayLiteral(
                "{\"ok\":true,\"action\":\"inference/stop\","
                "\"outcome\":\"accepted\","
                "\"inference\":{\"state\":\"running\"}}"));
        action->deleteLater();
    }
    expect(waitUntil([&failureCode] { return !failureCode.isEmpty(); }),
           "malformed ACK is processed");
    expect(failureCode == QStringLiteral("invalid_response") && uncertain,
           "wrong ACK action is an uncertain invalid response");
    expect(client.status().inferenceState == QStringLiteral("disabled"),
           "malformed ACK cannot claim Running");

    QTcpSocket *refresh = acceptClient(server);
    expect(refresh != nullptr, "malformed ACK still reconciles once");
    if (refresh != nullptr) {
        readRequest(refresh);
        sendJson(
            refresh,
            200,
            statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
        refresh->deleteLater();
    }
    client.shutdown();
}

void inference_error_preserves_code_message_and_capture_state()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "inference error server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    client.refreshStatus();
    QTcpSocket *initial = acceptClient(server);
    expect(initial != nullptr, "inference error initial status connects");
    if (initial == nullptr) {
        return;
    }
    readRequest(initial);
    sendJson(
        initial,
        200,
        statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
    expect(waitUntil([&client] { return client.hasFreshStatus(); }),
           "inference error starts from fresh status");
    initial->deleteLater();

    QString errorCode;
    QString errorMessage;
    bool uncertain = true;
    QObject::connect(
        &client,
        &VisionControlClient::requestFailed,
        [&errorCode, &errorMessage, &uncertain](const QString &, const QString &code,
                                                const QString &message, bool value) {
            errorCode = code;
            errorMessage = message;
            uncertain = value;
        });
    client.startInference();
    QTcpSocket *action = acceptClient(server);
    expect(action != nullptr, "inference error action connects");
    if (action != nullptr) {
        readRequestWithBody(action, 2);
        sendJson(
            action,
            409,
            QByteArrayLiteral(
                "{\"ok\":false,\"error\":\"inference_busy\","
                "\"message\":\"inference is stopping\"}"));
        action->deleteLater();
    }
    expect(waitUntil([&errorCode] { return !errorCode.isEmpty(); }),
           "inference application error is processed");
    expect(errorCode == QStringLiteral("inference_busy")
               && errorMessage == QStringLiteral("inference is stopping")
               && !uncertain,
           "valid application error preserves code/message and certainty");
    expect(client.status().state == QStringLiteral("idle")
               && client.status().inferenceState == QStringLiteral("disabled"),
           "inference error does not overwrite capture state");

    QTcpSocket *refresh = acceptClient(server);
    expect(refresh != nullptr, "valid inference error still reconciles once");
    if (refresh != nullptr) {
        readRequest(refresh);
        sendJson(
            refresh,
            200,
            statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
        refresh->deleteLater();
    }
    client.shutdown();
}

void second_mutation_is_rejected_while_one_is_active_or_queued()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "mutation admission server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    client.refreshStatus();
    QTcpSocket *initial = acceptClient(server);
    expect(initial != nullptr, "mutation admission initial status connects");
    if (initial == nullptr) {
        return;
    }
    readRequest(initial);
    sendJson(
        initial,
        200,
        statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
    expect(waitUntil([&client] { return client.hasFreshStatus(); }),
           "mutation admission starts from fresh status");
    initial->deleteLater();

    QString failureCode;
    QObject::connect(
        &client,
        &VisionControlClient::requestFailed,
        [&failureCode](const QString &, const QString &code,
                       const QString &, bool) { failureCode = code; });

    client.startInference();
    QTcpSocket *active = acceptClient(server);
    expect(active != nullptr, "first mutation becomes active");
    client.stopInference();
    expect(failureCode == QStringLiteral("action_busy"),
           "second mutation is rejected while POST is active");
    expect(!server.hasPendingConnections(),
           "active mutation rejection does not create a second POST");
    if (active != nullptr) {
        readRequestWithBody(active, 2);
        sendJson(
            active,
            202,
            QByteArrayLiteral(
                "{\"ok\":true,\"action\":\"inference/start\","
                "\"outcome\":\"accepted\"}"));
        active->deleteLater();
    }
    QTcpSocket *activeRefresh = acceptClient(server);
    if (activeRefresh != nullptr) {
        readRequest(activeRefresh);
        sendJson(
            activeRefresh,
            200,
            statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
        activeRefresh->deleteLater();
    }
    waitUntil([&client] { return !client.inferenceReconcilePending(); });

    client.refreshStatus();
    QTcpSocket *status = acceptClient(server);
    expect(status != nullptr, "queued mutation status connects");
    if (status != nullptr) {
        readRequest(status);
        client.startInference();
        expect(client.actionBusy(), "one mutation is queued behind GET");
        client.stopInference();
        expect(failureCode == QStringLiteral("action_busy"),
               "second mutation is rejected while one is queued");
        sendJson(
            status,
            200,
            statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
        status->deleteLater();
    }
    QTcpSocket *queued = acceptClient(server);
    expect(queued != nullptr, "the single queued mutation dispatches once");
    if (queued != nullptr) {
        expect(readRequestWithBody(queued, 2).startsWith(
                   "POST /api/v1/vision/inference/start HTTP/1.1"),
               "queued mutation preserves its original kind");
        sendJson(
            queued,
            202,
            QByteArrayLiteral(
                "{\"ok\":true,\"action\":\"inference/start\","
                "\"outcome\":\"accepted\"}"));
        queued->deleteLater();
    }
    QTcpSocket *queuedRefresh = acceptClient(server);
    if (queuedRefresh != nullptr) {
        readRequest(queuedRefresh);
        sendJson(
            queuedRefresh,
            200,
            statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
        queuedRefresh->deleteLater();
    }
    client.shutdown();
}

void failed_get_cancels_queued_action_and_revalidates_capability()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "failed GET admission server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    client.refreshStatus();
    QTcpSocket *initial = acceptClient(server);
    expect(initial != nullptr, "failed GET initial status connects");
    if (initial == nullptr) {
        return;
    }
    readRequest(initial);
    sendJson(
        initial,
        200,
        statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
    expect(waitUntil([&client] { return client.hasFreshStatus(); }),
           "failed GET test starts from fresh status");
    initial->deleteLater();

    QString failedAction;
    QObject::connect(
        &client,
        &VisionControlClient::requestFailed,
        [&failedAction](const QString &action, const QString &,
                        const QString &, bool) { failedAction = action; });
    client.refreshStatus();
    QTcpSocket *failedStatus = acceptClient(server);
    expect(failedStatus != nullptr, "queued action GET connects");
    if (failedStatus != nullptr) {
        readRequest(failedStatus);
        client.startInference();
        expect(client.actionBusy(), "Start queues behind normal GET");
        sendJson(
            failedStatus,
            500,
            QByteArrayLiteral(
                "{\"ok\":false,\"error\":\"internal_error\","
                "\"message\":\"status unavailable\"}"));
        failedStatus->deleteLater();
    }
    expect(waitUntil([&failedAction] {
        return !failedAction.isEmpty();
    }), "failed GET explicitly cancels queued action");
    expect(failedAction == QStringLiteral("inference/start"),
           "failed GET reports the canceled queued action");
    expect(!server.hasPendingConnections(),
           "failed GET does not execute canceled mutation");

    client.refreshStatus();
    QTcpSocket *unavailable = acceptClient(server);
    expect(unavailable != nullptr, "capability revalidation GET connects");
    if (unavailable != nullptr) {
        readRequest(unavailable);
        client.startInference();
        sendJson(
            unavailable,
            200,
            statusBody(QByteArrayLiteral(
                "{\"configured\":false,\"control_supported\":true,"
                "\"operation\":null,\"state\":\"disabled\"}")));
        unavailable->deleteLater();
    }
    expect(waitUntil([&client] {
        return !client.actionBusy();
    }), "queued action is revalidated after successful GET");
    expect(!server.hasPendingConnections(),
           "unconfigured capability prevents queued POST dispatch");
    client.shutdown();
}

void reply_finalization_has_no_reentrant_idle_gap()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "reentrancy server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    client.refreshStatus();
    QTcpSocket *initial = acceptClient(server);
    expect(initial != nullptr, "reentrancy initial status connects");
    if (initial == nullptr) {
        return;
    }
    readRequest(initial);
    sendJson(
        initial,
        200,
        statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
    expect(waitUntil([&client] { return client.hasFreshStatus(); }),
           "reentrancy starts from fresh status");
    initial->deleteLater();

    QString failureCode;
    QObject::connect(
        &client,
        &VisionControlClient::requestFailed,
        [&failureCode](const QString &, const QString &code,
                       const QString &, bool) { failureCode = code; });
    QObject::connect(
        &client,
        &VisionControlClient::inferenceActionAcknowledged,
        [&client](const QString &, const QString &) { client.stopInference(); });

    client.startInference();
    QTcpSocket *action = acceptClient(server);
    expect(action != nullptr, "reentrancy action connects");
    if (action != nullptr) {
        readRequestWithBody(action, 2);
        sendJson(
            action,
            202,
            QByteArrayLiteral(
                "{\"ok\":true,\"action\":\"inference/start\","
                "\"outcome\":\"accepted\"}"));
        action->deleteLater();
    }
    expect(waitUntil([&failureCode] { return !failureCode.isEmpty(); }),
           "reentrant ACK is processed");
    expect(failureCode == QStringLiteral("inference_reconcile_pending"),
           "reentrant slot cannot create a POST during finalization");
    expect(!server.hasPendingConnections(),
           "reentrant ACK handling has no transient idle request gap");
    QTcpSocket *refresh = acceptClient(server);
    if (refresh != nullptr) {
        readRequest(refresh);
        sendJson(
            refresh,
            200,
            statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
        refresh->deleteLater();
    }
    client.shutdown();
}

void ambiguous_inference_post_is_uncertain_without_auto_retry()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "ambiguous POST server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    client.refreshStatus();
    QTcpSocket *initial = acceptClient(server);
    expect(initial != nullptr, "ambiguous POST initial status connects");
    if (initial == nullptr) {
        return;
    }
    readRequest(initial);
    sendJson(
        initial,
        200,
        statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
    expect(waitUntil([&client] { return client.hasFreshStatus(); }),
           "ambiguous POST starts from fresh status");
    initial->deleteLater();

    bool uncertain = false;
    QString code;
    QObject::connect(
        &client,
        &VisionControlClient::requestFailed,
        [&uncertain, &code](const QString &, const QString &value,
                            const QString &, bool outcomeUncertain) {
            code = value;
            uncertain = outcomeUncertain;
        });
    client.startInference();
    QTcpSocket *action = acceptClient(server);
    expect(action != nullptr, "ambiguous POST reaches server");
    if (action != nullptr) {
        readRequestWithBody(action, 2);
        action->abort();
        action->deleteLater();
    }
    expect(waitUntil([&code] { return !code.isEmpty(); }),
           "transport failure retires the sent POST");
    expect(uncertain, "transport failure is outcome-uncertain");
    QTcpSocket *refresh = acceptClient(server);
    expect(refresh != nullptr, "ambiguous POST schedules one status refresh");
    if (refresh != nullptr) {
        readRequest(refresh);
        sendJson(
            refresh,
            200,
            statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
        refresh->deleteLater();
    }
    expect(!server.hasPendingConnections(),
           "ambiguous POST is never automatically retried");
    client.shutdown();
}

void freshness_expires_and_capture_post_does_not_renew_it()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "freshness server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    client.refreshStatus();
    QTcpSocket *status = acceptClient(server);
    expect(status != nullptr, "freshness status connects");
    if (status == nullptr) {
        return;
    }
    readRequest(status);
    sendJson(
        status,
        200,
        statusBody(configuredInference(QByteArrayLiteral("\"running\""))));
    expect(waitUntil([&client] { return client.hasFreshStatus(); }),
           "freshness starts after successful GET");
    status->deleteLater();

    client.requestSnapshot();
    QTcpSocket *capture = acceptClient(server);
    expect(capture != nullptr, "freshness capture POST connects");
    if (capture != nullptr) {
        readRequestWithBody(capture, 2);
        sendJson(
            capture,
            200,
            QByteArrayLiteral(
                "{\"ok\":true,\"capture\":{\"state\":\"idle\","
                "\"recording\":false}}"));
        capture->deleteLater();
    }
    expect(waitUntil([&client] { return !client.hasFreshStatus(); }, 4200),
           "capture POST does not renew inference freshness");
    client.shutdown();
}

void failed_get_disables_actions_until_refresh_recovers()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "failed status server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    client.refreshStatus();
    QTcpSocket *initial = acceptClient(server);
    expect(initial != nullptr, "failed status initial GET connects");
    if (initial == nullptr) {
        return;
    }
    readRequest(initial);
    sendJson(
        initial,
        200,
        statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
    expect(waitUntil([&client] { return client.hasFreshStatus(); }),
           "failed status starts from fresh sample");
    initial->deleteLater();

    client.refreshStatus();
    QTcpSocket *failed = acceptClient(server);
    expect(failed != nullptr, "failed refresh connects");
    if (failed != nullptr) {
        readRequest(failed);
        sendJson(
            failed,
            500,
            QByteArrayLiteral(
                "{\"ok\":false,\"error\":\"internal_error\","
                "\"message\":\"status unavailable\"}"));
        failed->deleteLater();
    }
    expect(waitUntil([&client] { return !client.hasFreshStatus(); }),
           "failed GET invalidates freshness");
    client.startInference();
    expect(!server.hasPendingConnections(),
           "stale status disables Start without a POST");

    client.refreshStatus();
    QTcpSocket *recovered = acceptClient(server);
    expect(recovered != nullptr, "manual Refresh Status reaches endpoint");
    if (recovered != nullptr) {
        readRequest(recovered);
        sendJson(
            recovered,
            200,
            statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
    }
    expect(waitUntil([&client] { return client.hasFreshStatus(); }),
           "manual Refresh recovers fresh status");
    if (recovered != nullptr) {
        recovered->deleteLater();
    }
    client.shutdown();
}

void inference_actions_wait_for_reconciliation_get()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "reconciliation admission server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    client.refreshStatus();
    QTcpSocket *initial = acceptClient(server);
    expect(initial != nullptr, "reconciliation initial GET connects");
    if (initial == nullptr) {
        return;
    }
    readRequest(initial);
    sendJson(
        initial,
        200,
        statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
    expect(waitUntil([&client] { return client.hasFreshStatus(); }),
           "reconciliation starts from fresh status");
    initial->deleteLater();

    QString failureCode;
    QObject::connect(
        &client,
        &VisionControlClient::requestFailed,
        [&failureCode](const QString &, const QString &code,
                       const QString &, bool) { failureCode = code; });
    client.startInference();
    QTcpSocket *start = acceptClient(server);
    expect(start != nullptr, "reconciliation Start connects");
    if (start != nullptr) {
        readRequestWithBody(start, 2);
        sendJson(
            start,
            202,
            QByteArrayLiteral(
                "{\"ok\":true,\"action\":\"inference/start\","
                "\"outcome\":\"accepted\"}"));
        start->deleteLater();
    }
    expect(waitUntil([&client] { return client.inferenceReconcilePending(); }),
           "reconciliation flag is set before the next action");
    client.stopInference();
    expect(failureCode == QStringLiteral("inference_reconcile_pending"),
           "new inference action is blocked until reconciliation GET");
    expect(!server.hasPendingConnections(),
           "blocked action does not create a second POST");

    QTcpSocket *refresh = acceptClient(server);
    expect(refresh != nullptr, "reconciliation GET connects");
    if (refresh != nullptr) {
        readRequest(refresh);
        sendJson(
            refresh,
            200,
            statusBody(configuredInference(QByteArrayLiteral("\"running\""))));
        refresh->deleteLater();
    }
    expect(waitUntil([&client] { return !client.inferenceReconcilePending(); }),
           "reconciliation GET completes before next mutation");
    client.stopInference();
    QTcpSocket *stop = acceptClient(server);
    expect(stop != nullptr, "Stop becomes available after reconciliation");
    if (stop != nullptr) {
        readRequestWithBody(stop, 2);
        sendJson(
            stop,
            202,
            QByteArrayLiteral(
                "{\"ok\":true,\"action\":\"inference/stop\","
                "\"outcome\":\"accepted\"}"));
        stop->deleteLater();
    }
    QTcpSocket *stopRefresh = acceptClient(server);
    if (stopRefresh != nullptr) {
        readRequest(stopRefresh);
        sendJson(
            stopRefresh,
            200,
            statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
        stopRefresh->deleteLater();
    }
    client.shutdown();
}

void sent_post_blocks_low_level_endpoint_change()
{
    QTcpServer serverA;
    QTcpServer serverB;
    expect(serverA.listen(QHostAddress::LocalHost, 0),
           "active inference endpoint A must listen");
    expect(serverB.listen(QHostAddress::LocalHost, 0),
           "active inference endpoint B must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), serverA.serverPort());
    client.refreshStatus();
    QTcpSocket *initial = acceptClient(serverA);
    expect(initial != nullptr, "active endpoint initial GET connects");
    if (initial == nullptr) {
        return;
    }
    readRequest(initial);
    sendJson(
        initial,
        200,
        statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
    expect(waitUntil([&client] { return client.hasFreshStatus(); }),
           "active endpoint starts from fresh status");
    initial->deleteLater();

    client.startInference();
    QTcpSocket *action = acceptClient(serverA);
    expect(action != nullptr, "active inference POST reaches endpoint A");
    const quint16 oldPort = client.port();
    expect(!client.setEndpoint(QStringLiteral("127.0.0.1"), serverB.serverPort()),
           "sent inference POST blocks low-level endpoint change");
    expect(client.port() == oldPort,
           "blocked endpoint change keeps the old committed endpoint");
    expect(!serverB.hasPendingConnections(),
           "blocked endpoint change sends no request to endpoint B");
    if (action != nullptr) {
        readRequestWithBody(action, 2);
        sendJson(
            action,
            202,
            QByteArrayLiteral(
                "{\"ok\":true,\"action\":\"inference/start\","
                "\"outcome\":\"accepted\"}"));
        action->deleteLater();
    }
    QTcpSocket *refresh = acceptClient(serverA);
    if (refresh != nullptr) {
        readRequest(refresh);
        sendJson(
            refresh,
            200,
            statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
        refresh->deleteLater();
    }
    client.shutdown();
}

void endpoint_switch_cancels_get_and_stale_reply_cannot_cross_generation()
{
    QTcpServer serverA;
    QTcpServer serverB;
    expect(serverA.listen(QHostAddress::LocalHost, 0),
           "generation endpoint A must listen");
    expect(serverB.listen(QHostAddress::LocalHost, 0),
           "generation endpoint B must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), serverA.serverPort());
    bool staleApplied = false;
    QObject::connect(
        &client,
        &VisionControlClient::statusChanged,
        [&staleApplied](VisionCaptureStatus status) {
            if (status.haveLatestFrame && status.latestFrameId == 111U) {
                staleApplied = true;
            }
        });
    client.refreshStatus();
    QTcpSocket *peerA = acceptClient(serverA);
    expect(peerA != nullptr, "old generation GET connects");
    if (peerA == nullptr) {
        return;
    }
    readRequest(peerA);
    client.startInference();
    expect(client.setEndpoint(QStringLiteral("127.0.0.1"), serverB.serverPort()),
           "GET plus queued action permits low-level endpoint switch");
    expect(!client.actionBusy(),
           "endpoint switch cancels the queued action explicitly");
    sendJson(
        peerA,
        200,
        QByteArrayLiteral(
            "{\"ok\":true,\"camera\":{\"running\":true,"
            "\"latest_frame_id\":111},\"capture\":{"
            "\"state\":\"idle\",\"recording\":false}}"));
    waitForMs(100);
    expect(!staleApplied, "stale old-generation reply cannot mutate status");
    expect(!serverB.hasPendingConnections(),
           "queued action is never moved to the new endpoint");
    peerA->deleteLater();
    client.shutdown();
}

void redirect_is_rejected_without_following_or_resending()
{
    QTcpServer server;
    QTcpServer redirected;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "redirect source server must listen");
    expect(redirected.listen(QHostAddress::LocalHost, 0),
           "redirect destination server must listen");
    VisionControlClient client;
    client.setEndpoint(QStringLiteral("127.0.0.1"), server.serverPort());
    client.refreshStatus();
    QTcpSocket *initial = acceptClient(server);
    expect(initial != nullptr, "redirect initial status connects");
    if (initial == nullptr) {
        return;
    }
    readRequest(initial);
    sendJson(
        initial,
        200,
        statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
    expect(waitUntil([&client] { return client.hasFreshStatus(); }),
           "redirect starts from fresh status");
    initial->deleteLater();

    QString failureCode;
    QObject::connect(
        &client,
        &VisionControlClient::requestFailed,
        [&failureCode](const QString &, const QString &code,
                       const QString &, bool) { failureCode = code; });
    client.startInference();
    QTcpSocket *action = acceptClient(server);
    expect(action != nullptr, "redirect action reaches source endpoint");
    if (action != nullptr) {
        readRequestWithBody(action, 2);
        const QByteArray location =
            "Location: http://127.0.0.1:" + QByteArray::number(redirected.serverPort()) + "/other\r\n";
        sendResponse(
            action,
            302,
            QByteArrayLiteral("{}"),
            location);
        action->deleteLater();
    }
    expect(waitUntil([&failureCode] { return !failureCode.isEmpty(); }),
           "redirect response is surfaced as a request failure");
    expect(failureCode == QStringLiteral("redirect_rejected"),
           "redirect is classified explicitly");
    expect(!redirected.hasPendingConnections(),
           "redirected inference POST is never sent");
    QTcpSocket *refresh = acceptClient(server);
    if (refresh != nullptr) {
        readRequest(refresh);
        sendJson(
            refresh,
            200,
            statusBody(configuredInference(QByteArrayLiteral("\"disabled\""))));
        refresh->deleteLater();
    }
    client.shutdown();
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    statusBypassesProxyAndParsesJson();
    actionQueuesBehindStatusPollInsteadOfFailing();
    statusParsesRunningInferenceDiagnostics();
    failedInferencePreservesNullFlagsAndNumericZero();
    legacyStatusResetsInferenceAfterActionPreservesIt();
    actionPathsAndErrorsAreSurfaced();
    endpointChangeCancelsOldPendingAction();
    activeActionBlocksEndpointChangeUntilResult();
    stopRecordingAllowsLongerServerFinalization();
    configured_disabled_status_parses_capabilities_and_null_operation();
    strict_capability_and_operation_impostors_fail_closed();
    legacy_status_is_readable_but_manual_actions_are_unavailable();
    capture_post_preserves_inference_metadata_and_freshness();
    zero_metrics_and_full_sha_are_preserved();
    manual_start_and_stop_use_exact_paths_body_and_injected_port();
    retry_and_clear_error_send_one_post_each();
    inference_ack_preserves_state_and_reconciles_once();
    malformed_or_fake_inference_ack_cannot_change_state();
    inference_error_preserves_code_message_and_capture_state();
    second_mutation_is_rejected_while_one_is_active_or_queued();
    failed_get_cancels_queued_action_and_revalidates_capability();
    reply_finalization_has_no_reentrant_idle_gap();
    ambiguous_inference_post_is_uncertain_without_auto_retry();
    freshness_expires_and_capture_post_does_not_renew_it();
    failed_get_disables_actions_until_refresh_recovers();
    inference_actions_wait_for_reconciliation_get();
    sent_post_blocks_low_level_endpoint_change();
    endpoint_switch_cancels_get_and_stale_reply_cannot_cross_generation();
    redirect_is_rejected_without_following_or_resending();
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
