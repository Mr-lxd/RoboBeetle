#include "vision/VisionClient.h"
#include "vision/VisionProtocol.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QHostAddress>
#include <QNetworkProxy>
#include <QTcpServer>
#include <QTcpSocket>
#include <QThread>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <vector>

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

bool waitUntil(const std::function<bool()> &predicate, int timeoutMs = 1500)
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

std::vector<std::uint8_t> fixturePayload()
{
    const auto path = std::filesystem::path(__FILE__).parent_path()
        / "fixtures" / "rbvs_v1_2x2.jpg";
    std::ifstream input(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

QByteArray makeFrame(std::uint64_t frameId)
{
    const auto payload = fixturePayload();
    VisionFrameHeader header;
    header.frameId = frameId;
    header.captureTimestampNs = frameId + 1U;
    header.width = 2U;
    header.height = 2U;
    header.payloadSize = static_cast<std::uint32_t>(payload.size());
    const auto encoded = encodeVisionHeader(header);
    expect(encoded.wire.has_value(), "VisionClient test header must encode");

    QByteArray wire;
    if (!encoded.wire) {
        return wire;
    }
    wire.append(reinterpret_cast<const char *>(encoded.wire->data()),
                static_cast<qsizetype>(encoded.wire->size()));
    wire.append(reinterpret_cast<const char *>(payload.data()),
                static_cast<qsizetype>(payload.size()));
    return wire;
}

QTcpSocket *acceptClient(QTcpServer &server)
{
    if (!waitUntil([&server] { return server.hasPendingConnections(); })) {
        return nullptr;
    }
    return server.nextPendingConnection();
}

void directLanSocketIgnoresApplicationProxy()
{
    const QNetworkProxy originalProxy = QNetworkProxy::applicationProxy();
    QNetworkProxy::setApplicationProxy(
        QNetworkProxy(QNetworkProxy::HttpProxy,
                      QStringLiteral("127.0.0.1"),
                      9));

    QTcpServer server;
    const bool listening = server.listen(QHostAddress::LocalHost, 0);
    expect(listening, "proxy regression fake server must listen");

    VisionClient client;
    if (listening) {
        client.connectToHost(QStringLiteral("127.0.0.1"), server.serverPort());
        QTcpSocket *peer = acceptClient(server);
        expect(peer != nullptr,
               "VisionClient must bypass application HTTP proxy for direct LAN TCP");
        expect(peer != nullptr && waitUntil([&client] { return client.isConnected(); }),
               "VisionClient direct connection succeeds despite application proxy");
        client.shutdown();
        if (peer != nullptr) {
            peer->deleteLater();
        }
    }

    QNetworkProxy::setApplicationProxy(originalProxy);
}

void connectReceiveDisconnect()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "VisionClient fake server must listen");

    VisionClient client;
    bool frameSeen = false;
    quint64 seenId = 999U;
    QObject::connect(&client, &VisionClient::frameReady,
                     [&frameSeen, &seenId](const QImage &image,
                                           quint64 frameId,
                                           quint64 timestampNs) {
        frameSeen = !image.isNull() && image.size() == QSize(2, 2)
            && timestampNs == frameId + 1U;
        seenId = frameId;
    });

    client.connectToHost(QStringLiteral("127.0.0.1"), server.serverPort());
    QTcpSocket *peer = acceptClient(server);
    expect(peer != nullptr, "VisionClient connects independently to Vision TCP");
    if (peer == nullptr) {
        return;
    }
    expect(waitUntil([&client] { return client.isConnected(); }),
           "VisionClient reports Connected without robot authority");

    const QByteArray frame = makeFrame(0U);
    peer->write(frame);
    peer->flush();

    expect(waitUntil([&frameSeen] { return frameSeen; }),
           "VisionClient emits a decoded JPEG frame");
    expect(seenId == 0U && client.lastFrameId() == 0U,
           "VisionClient accepts frame id zero at a fresh connection");
    expect(client.totalWireFrames() == 1U,
           "VisionClient diagnostics count completed wire frames");

    peer->disconnectFromHost();
    expect(waitUntil([&client] {
        return client.state() == VisionConnectionState::Disconnected;
    }), "remote Vision disconnect does not require control-plane action");
    peer->deleteLater();
}

void reconnectResetsSequenceSession()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "reconnect fake server must listen");

    VisionClient client;
    std::vector<quint64> ids;
    QObject::connect(&client, &VisionClient::frameReady,
                     [&ids](const QImage &, quint64 frameId, quint64) {
        ids.push_back(frameId);
    });

    client.connectToHost(QStringLiteral("127.0.0.1"), server.serverPort());
    QTcpSocket *firstPeer = acceptClient(server);
    expect(firstPeer != nullptr, "first Vision connection accepted");
    if (firstPeer == nullptr) {
        return;
    }
    firstPeer->write(makeFrame(9U));
    firstPeer->flush();
    expect(waitUntil([&ids] { return ids.size() == 1U; }),
           "first session frame received");
    firstPeer->disconnectFromHost();
    expect(waitUntil([&client] {
        return client.state() == VisionConnectionState::Disconnected;
    }), "first session disconnect completes");
    firstPeer->deleteLater();

    client.connectToHost(QStringLiteral("127.0.0.1"), server.serverPort());
    QTcpSocket *secondPeer = acceptClient(server);
    expect(secondPeer != nullptr, "second Vision connection accepted");
    if (secondPeer == nullptr) {
        return;
    }
    secondPeer->write(makeFrame(0U));
    secondPeer->flush();
    expect(waitUntil([&ids] { return ids.size() == 2U; }),
           "new TCP session permits sequence restart at zero");
    expect(ids.size() == 2U && ids[0] == 9U && ids[1] == 0U,
           "sequence validation is connection-scoped");

    client.shutdown();
    secondPeer->deleteLater();
}

void partialFrameInactivityMovesClientToError()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "partial-frame fake server must listen");

    VisionClient client;
    bool timeoutErrorSeen = false;
    QObject::connect(&client, &VisionClient::protocolError,
                     [&timeoutErrorSeen](const QString &message) {
        if (message.contains(QStringLiteral("partial frame timed out"))) {
            timeoutErrorSeen = true;
        }
    });

    client.connectToHost(QStringLiteral("127.0.0.1"), server.serverPort());
    QTcpSocket *peer = acceptClient(server);
    expect(peer != nullptr, "partial-frame client connects");
    if (peer == nullptr) {
        return;
    }
    expect(waitUntil([&client] { return client.isConnected(); }),
           "partial-frame client reaches Connected");

    const QByteArray frame = makeFrame(5U);
    peer->write(frame.left(40));
    peer->flush();

    expect(waitUntil([&client, &timeoutErrorSeen] {
        return timeoutErrorSeen &&
               client.state() == VisionConnectionState::Error;
    }, kVisionPartialFrameInactivityTimeoutMs + 1200),
           "stalled partial RBVS frame is aborted after inactivity timeout");

    peer->deleteLater();
}

void malformedHeaderMovesClientToError()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "malformed-header fake server must listen");

    VisionClient client;
    bool protocolErrorSeen = false;
    QObject::connect(&client, &VisionClient::protocolError,
                     [&protocolErrorSeen](const QString &) {
        protocolErrorSeen = true;
    });

    client.connectToHost(QStringLiteral("127.0.0.1"), server.serverPort());
    QTcpSocket *peer = acceptClient(server);
    expect(peer != nullptr, "malformed-header client connects");
    if (peer == nullptr) {
        return;
    }

    QByteArray bad = makeFrame(0U);
    bad[0] = 'X';
    peer->write(bad);
    peer->flush();

    expect(waitUntil([&client, &protocolErrorSeen] {
        return protocolErrorSeen &&
               client.state() == VisionConnectionState::Error;
    }), "fatal RBVS framing error is surfaced and connection is aborted");
    peer->deleteLater();
}

void endpointBusyTracksRawSocketActivity()
{
    QTcpServer server;
    expect(server.listen(QHostAddress::LocalHost, 0),
           "endpoint activity server must listen");
    VisionClient client;
    bool busySeen = false;
    bool idleSeen = false;
    QObject::connect(&client, &VisionClient::endpointActivityChanged,
                     [&busySeen, &idleSeen](bool busy) {
        if (busy) busySeen = true;
        else idleSeen = true;
    });
    expect(!client.endpointBusy(), "unconnected Vision endpoint is idle");
    client.connectToHost(QStringLiteral("127.0.0.1"), server.serverPort());
    QTcpSocket *peer = acceptClient(server);
    expect(peer != nullptr, "endpoint activity connection reaches fake server");
    expect(waitUntil([&client, &busySeen] {
        return busySeen && client.endpointBusy();
    }), "endpoint activity reports Connecting/Connected as busy");
    client.disconnectFromHost();
    expect(waitUntil([&client, &idleSeen] {
        return idleSeen && !client.endpointBusy();
    }), "endpoint activity reports disconnected as idle");
    if (peer != nullptr) peer->deleteLater();
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    directLanSocketIgnoresApplicationProxy();
    connectReceiveDisconnect();
    reconnectResetsSequenceSession();
    partialFrameInactivityMovesClientToError();
    malformedHeaderMovesClientToError();
    endpointBusyTracksRawSocketActivity();
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
