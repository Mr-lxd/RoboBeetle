#include "remote/RbrpClientSession.h"

#include "robobeetle/gateway/rbrp_codec.hpp"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QHostAddress>
#include <QNetworkProxy>
#include <QTcpServer>
#include <QTcpSocket>
#include <QThread>

#include <cstdio>
#include <functional>
#include <optional>
#include <vector>

namespace {

using namespace robobeetle::gateway;

int failures = 0;

void expect(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

bool waitUntil(const std::function<bool()> &predicate, int timeoutMs = 1000)
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

class FakeGateway {
public:
    FakeGateway()
    {
        const bool ok = server_.listen(QHostAddress::LocalHost, 0);
        expect(ok, "fake gateway must listen on localhost");
    }
    quint16 port() const { return server_.serverPort(); }

    bool acceptClient()
    {
        if (!waitUntil([this] { return server_.hasPendingConnections(); })) {
            return false;
        }
        peer_ = server_.nextPendingConnection();
        return peer_ != nullptr;
    }

    std::optional<RbrpFrame> nextFrame(
        std::optional<RbrpMessageKind> wanted = std::nullopt,
        int timeoutMs = 1000)
    {
        QElapsedTimer timer;
        timer.start();
        while (timer.elapsed() < timeoutMs) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
            pump();
            for (auto it = frames_.begin(); it != frames_.end(); ++it) {
                if (!wanted.has_value() || it->kind == *wanted) {
                    RbrpFrame frame = *it;
                    frames_.erase(it);
                    return frame;
                }
            }
            QThread::msleep(2);
        }
        pump();
        return std::nullopt;
    }

    void send(const GatewayMessage &message)
    {
        const auto encoded = encode_gateway_message(message);
        expect(encoded.status == RbrpEncodeStatus::Ok,
               "gateway reply must encode");
        if (peer_ == nullptr || encoded.status != RbrpEncodeStatus::Ok) {
            return;
        }
        peer_->write(reinterpret_cast<const char *>(encoded.wire.data()),
                     static_cast<qint64>(encoded.wire.size()));
        peer_->flush();
    }

    void sendWire(const Bytes &wire)
    {
        if (peer_ == nullptr) {
            return;
        }
        peer_->write(reinterpret_cast<const char *>(wire.data()),
                     static_cast<qint64>(wire.size()));
        peer_->flush();
    }

    void dropClient()
    {
        if (peer_ == nullptr) {
            return;
        }
        peer_->abort();
        peer_->deleteLater();
        peer_ = nullptr;
        decoder_.reset();
        frames_.clear();
        QCoreApplication::processEvents();
    }

private:
    void pump()
    {
        if (peer_ == nullptr || peer_->bytesAvailable() == 0) {
            return;
        }
        const QByteArray bytes = peer_->readAll();
        const auto status = decoder_.feed(
            reinterpret_cast<const Byte *>(bytes.constData()),
            static_cast<std::size_t>(bytes.size()), frames_);
        expect(status == RbrpFeedStatus::Ok,
               "client traffic must remain valid RBRP");
    }

    QTcpServer server_;
    QTcpSocket *peer_{nullptr};
    RbrpDecoder decoder_;
    std::vector<RbrpFrame> frames_;
};

void testHandshakeHeartbeatCommandAndReconnect()
{
    const QNetworkProxy previousProxy = QNetworkProxy::applicationProxy();
    QNetworkProxy::setApplicationProxy(QNetworkProxy(
        QNetworkProxy::HttpCachingProxy,
        QStringLiteral("127.0.0.1"), 9));

    FakeGateway gateway;
    rb::RbrpClientSession session;

    session.connectToHost(QStringLiteral("127.0.0.1"), gateway.port());
    expect(gateway.acceptClient(), "client must connect to fake gateway");

    const auto hello = gateway.nextFrame(RbrpMessageKind::Hello);
    expect(hello.has_value(), "client must send Hello after TCP connect");
    if (!hello.has_value()) {
        return;
    }
    expect(hello->request_id != 0U, "Hello request_id must be nonzero");
    expect(hello->payload == Bytes({0U, 0U}),
           "Hello capabilities must be zero");
    expect(!session.canAcquireControl(),
           "Acquire must stay unavailable before HelloReply");

    gateway.send(GatewayMessage{
        hello->request_id, HelloReply{0U, 512U, 250U, 1000U}});
    expect(waitUntil([&] { return session.canAcquireControl(); }),
           "matching HelloReply must make Acquire available");
    expect(session.helloComplete(), "Hello must be complete");
    expect(session.heartbeatIntervalMs() == 250U
               && session.leaseTimeoutMs() == 1000U,
           "HelloReply must preserve frozen heartbeat/lease contract");

    expect(session.acquireControl(), "AcquireControl must submit after Hello");
    const auto acquire = gateway.nextFrame(RbrpMessageKind::AcquireControl);
    expect(acquire.has_value(), "gateway must receive AcquireControl");
    if (!acquire.has_value()) {
        return;
    }
    expect(acquire->request_id != hello->request_id,
           "request_id must advance between Hello and Acquire");

    gateway.send(GatewayMessage{
        acquire->request_id,
        AcquireReply{AcquireResult::Granted, AuthorityState::Owned,
                     GatewayApplicationSessionState::SafetyQuiet,
                     GatewayApplicationLinkState::Unconfirmed, 1000U, 0U}});
    expect(waitUntil([&] {
        return session.authorityState() == rb::ControlAuthorityState::Owned;
    }), "AcquireReply Granted must enter Owned");
    expect(!session.isControlActive(),
           "Owned must not imply Active before gateway link is Online");

    gateway.send(GatewayMessage{
        0U, ControlStateMessage{
                AuthorityState::Owned,
                GatewayApplicationSessionState::Online,
                GatewayApplicationLinkState::Active,
                GatewayStateReason::Acquired, 900U}});
    expect(waitUntil([&] { return session.isControlActive(); }),
           "ControlState Online+Active must enable remote control");

    const auto heartbeat =
        gateway.nextFrame(RbrpMessageKind::ControlHeartbeat, 700);
    expect(heartbeat.has_value(),
           "Owned session must send the frozen 250-ms ControlHeartbeat");

    expect(session.releaseControl(),
           "explicit ReleaseControl must immediately fail closed locally");
    const auto release = gateway.nextFrame(RbrpMessageKind::ReleaseControl);
    expect(release.has_value(), "gateway must receive ReleaseControl");
    expect(session.authorityState() == rb::ControlAuthorityState::Unowned
               && !session.isControlActive(),
           "local Release must immediately clear Owned/Active state");

    gateway.send(GatewayMessage{
        0U, ControlStateMessage{
                AuthorityState::Owned,
                GatewayApplicationSessionState::Online,
                GatewayApplicationLinkState::Active,
                GatewayStateReason::Acquired, 900U}});
    QCoreApplication::processEvents();
    expect(session.authorityState() == rb::ControlAuthorityState::Unowned
               && !session.isControlActive(),
           "stale Owned ControlState after local Release must not resurrect authority");
    expect(!gateway.nextFrame(RbrpMessageKind::ControlHeartbeat, 350).has_value(),
           "stale Owned ControlState after local Release must not restart heartbeat");

    expect(session.acquireControl(),
           "explicit Acquire must be able to start a new authority generation");
    const auto reacquire = gateway.nextFrame(RbrpMessageKind::AcquireControl);
    expect(reacquire.has_value(), "gateway must receive explicit re-Acquire");
    if (!reacquire.has_value()) {
        return;
    }
    gateway.send(GatewayMessage{
        reacquire->request_id,
        AcquireReply{AcquireResult::Granted, AuthorityState::Owned,
                     GatewayApplicationSessionState::Online,
                     GatewayApplicationLinkState::Active, 1000U, 0U}});
    expect(waitUntil([&] {
        return session.authorityState() == rb::ControlAuthorityState::Owned
            && session.isControlActive();
    }), "matching AcquireReply must end the local Release fence");

    QByteArray enablePayload;
    enablePayload.append(char(0x01));
    enablePayload.append(char(0x00));
    const auto commandId =
        session.sendCommand(static_cast<quint8>(RobotCommandKind::EnableServos),
                            enablePayload);
    expect(commandId.has_value(), "Active session must accept typed command");
    const auto command = gateway.nextFrame(RbrpMessageKind::CommandRequest);
    expect(command.has_value(), "gateway must receive typed CommandRequest");
    if (command.has_value()) {
        expect(command->request_id == *commandId,
               "CommandRequest must preserve allocated request_id");
        expect(command->request_id != acquire->request_id,
               "command request_id must not reuse Acquire request_id");
        expect(command->payload == Bytes({0x01U, 0x01U, 0x00U}),
               "EnableServos wire payload must be typed mask command");
    }

    const auto wrongDirection =
        encode_frame(RbrpMessageKind::Hello, 999U, Bytes{0U, 0U});
    expect(wrongDirection.status == RbrpEncodeStatus::Ok,
           "wrong-direction test frame must encode");
    gateway.sendWire(wrongDirection.wire);
    expect(waitUntil([&] { return !session.isConnected(); }),
           "wrong-direction server frame must fail closed and disconnect");
    expect(session.authorityState() == rb::ControlAuthorityState::Unowned
               && !session.isControlActive(),
           "protocol failure must clear authority and Active state");

    gateway.dropClient();
    session.connectToHost(QStringLiteral("127.0.0.1"), gateway.port());
    expect(gateway.acceptClient(), "client must reconnect to fake gateway");
    const auto hello2 = gateway.nextFrame(RbrpMessageKind::Hello);
    expect(hello2.has_value(), "reconnect must start with a new Hello");
    if (!hello2.has_value()) {
        return;
    }
    gateway.send(GatewayMessage{
        hello2->request_id, HelloReply{0U, 512U, 250U, 1000U}});
    expect(waitUntil([&] { return session.canAcquireControl(); }),
           "reconnect Hello must return to ReadyUnowned");
    expect(session.authorityState() == rb::ControlAuthorityState::Unowned,
           "reconnect must not restore old authority");

    const auto unexpectedAcquire =
        gateway.nextFrame(RbrpMessageKind::AcquireControl, 350);
    const auto unexpectedCommand =
        gateway.nextFrame(RbrpMessageKind::CommandRequest, 100);
    expect(!unexpectedAcquire.has_value() && !unexpectedCommand.has_value(),
           "reconnect must not auto-Acquire or replay an old command");

    session.disconnectFromHost();
    waitUntil([&] { return !session.isConnected(); });
    QNetworkProxy::setApplicationProxy(previousProxy);
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    testHandshakeHeartbeatCommandAndReconnect();
    if (failures == 0) {
        std::fprintf(stdout, "All RBRP client session tests passed\n");
    }
    return failures == 0 ? 0 : 1;
}
