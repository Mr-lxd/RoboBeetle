#include "vision/DetectionClient.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QHostAddress>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
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

QByteArray line(quint64 frameId, quint64 timestampNs)
{
    const QJsonObject detection{
        {QStringLiteral("class_id"), 0},
        {QStringLiteral("class_name"), QStringLiteral("fish")},
        {QStringLiteral("confidence"), 0.88},
        {QStringLiteral("original_x"), 12.0},
        {QStringLiteral("original_y"), 20.0},
    };
    const QJsonObject object{
        {QStringLiteral("type"), QStringLiteral("detections")},
        {QStringLiteral("version"), kDetectionStreamVersion},
        {QStringLiteral("frame_id"), static_cast<qint64>(frameId)},
        {QStringLiteral("capture_timestamp_ns"), static_cast<qint64>(timestampNs)},
        {QStringLiteral("width"), 640},
        {QStringLiteral("height"), 480},
        {QStringLiteral("coordinate_space"), QStringLiteral("original_frame_pixels")},
        {QStringLiteral("detections"), QJsonArray{detection}},
    };
    return QJsonDocument(object).toJson(QJsonDocument::Compact) + '\n';
}

QTcpSocket *acceptClient(QTcpServer &server)
{
    if (!waitUntil([&] { return server.hasPendingConnections(); })) {
        return nullptr;
    }
    return server.nextPendingConnection();
}

void clientBypassesApplicationProxyAndReceivesMetadata()
{
    const QNetworkProxy originalProxy = QNetworkProxy::applicationProxy();
    QNetworkProxy::setApplicationProxy(
        QNetworkProxy(
            QNetworkProxy::HttpProxy,
            QStringLiteral("127.0.0.1"),
            9));

    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "detection fake server must listen");

    DetectionClient client;
    QVector<DetectionFrame> received;
    QObject::connect(
        &client, &DetectionClient::metadataReady,
        [&received](DetectionFrame frame) {
            received.push_back(std::move(frame));
        });

    client.connectToHost(
        QStringLiteral("127.0.0.1"), server.serverPort());
    QTcpSocket *peer = acceptClient(server);
    expect(peer != nullptr,
           "DetectionClient bypasses application proxy");
    expect(waitUntil([&] { return client.isConnected(); }),
           "DetectionClient reports connected state");

    if (peer != nullptr) {
        const QByteArray payload = line(7, 7'000);
        peer->write(payload.left(payload.size() / 2));
        peer->flush();
        QCoreApplication::processEvents();
        expect(received.isEmpty(),
               "partial metadata does not emit a frame");
        peer->write(payload.mid(payload.size() / 2));
        peer->flush();
        expect(waitUntil([&] { return received.size() == 1; }),
               "complete metadata emits one frame");
        if (received.size() == 1) {
            expect(received.front().frameId == 7U,
                   "metadata frame id reaches client");
        }
        peer->disconnectFromHost();
        peer->deleteLater();
    }

    client.shutdown();
    QNetworkProxy::setApplicationProxy(originalProxy);
}

void duplicateOrRegressingFrameIdFailsClosed()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "duplicate-frame fake server must listen");

    DetectionClient client;
    QString protocolError;
    QObject::connect(
        &client, &DetectionClient::protocolError,
        [&protocolError](const QString &message) {
            protocolError = message;
        });

    client.connectToHost(
        QStringLiteral("127.0.0.1"), server.serverPort());
    QTcpSocket *peer = acceptClient(server);
    expect(peer != nullptr, "duplicate-frame client connects");
    if (peer != nullptr) {
        peer->write(line(10, 1000) + line(10, 1100));
        peer->flush();
        expect(waitUntil([&] {
            return client.state() == DetectionConnectionState::Error;
        }), "duplicate metadata frame id moves client to Error");
        expect(protocolError.contains(QStringLiteral("frame_id")),
               "duplicate frame failure identifies frame_id");
        peer->deleteLater();
    }
    client.shutdown();
}

void endpointSwitchRejectsOldSocketData()
{
    QTcpServer first;
    QTcpServer second;
    expect(first.listen(QHostAddress::LocalHost, 0)
               && second.listen(QHostAddress::LocalHost, 0),
           "both endpoint-generation fake servers listen");

    DetectionClient client;
    QVector<quint64> frameIds;
    QObject::connect(
        &client, &DetectionClient::metadataReady,
        [&frameIds](DetectionFrame frame) {
            frameIds.push_back(frame.frameId);
        });

    client.connectToHost(
        QStringLiteral("127.0.0.1"), first.serverPort());
    QTcpSocket *oldPeer = acceptClient(first);
    expect(oldPeer != nullptr, "first detection endpoint connects");

    client.connectToHost(
        QStringLiteral("127.0.0.1"), second.serverPort());
    QTcpSocket *newPeer = acceptClient(second);
    expect(newPeer != nullptr, "second detection endpoint connects");

    if (oldPeer != nullptr) {
        oldPeer->write(line(1, 100));
        oldPeer->flush();
    }
    if (newPeer != nullptr) {
        newPeer->write(line(2, 200));
        newPeer->flush();
    }

    expect(waitUntil([&] { return frameIds.size() == 1; }),
           "only current endpoint metadata is emitted");
    if (frameIds.size() == 1) {
        expect(frameIds.front() == 2U,
               "stale endpoint callback cannot publish metadata");
    }

    if (oldPeer != nullptr) oldPeer->deleteLater();
    if (newPeer != nullptr) newPeer->deleteLater();
    client.shutdown();
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    clientBypassesApplicationProxyAndReceivesMetadata();
    duplicateOrRegressingFrameIdFailsClosed();
    endpointSwitchRejectsOldSocketData();
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
