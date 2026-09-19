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

void sendJson(QTcpSocket *socket, int status, const QByteArray &body)
{
    const QByteArray statusText =
        status == 200 ? QByteArrayLiteral("OK") : QByteArrayLiteral("Conflict");
    QByteArray response =
        "HTTP/1.1 " + QByteArray::number(status) + " " + statusText + "\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: " + QByteArray::number(body.size()) + "\r\n"
        "Connection: close\r\n\r\n" + body;
    socket->write(response);
    socket->flush();
    socket->disconnectFromHost();
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

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    statusBypassesProxyAndParsesJson();
    actionQueuesBehindStatusPollInsteadOfFailing();
    actionPathsAndErrorsAreSurfaced();
    endpointChangeCancelsOldPendingAction();
    activeActionBlocksEndpointChangeUntilResult();
    stopRecordingAllowsLongerServerFinalization();
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
