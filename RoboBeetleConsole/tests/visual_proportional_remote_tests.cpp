#include "remote/RemoteRobotController.h"


#include "robobeetle/gateway/gateway_types.hpp"
#include "robobeetle/gateway/rbrp_codec.hpp"

#include <QApplication>
#include <QElapsedTimer>
#include <QHostAddress>
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

bool pumpUntil(const std::function<bool()> &predicate, int timeoutMs = 1500)
{
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeoutMs) {
        QApplication::processEvents(QEventLoop::AllEvents, 10);
        if (predicate()) {
            return true;
        }
        QThread::msleep(1);
    }
    QApplication::processEvents(QEventLoop::AllEvents, 10);
    return predicate();
}

QByteArray wireBytes(const Bytes &bytes)
{
    return QByteArray(reinterpret_cast<const char *>(bytes.data()),
                      static_cast<qsizetype>(bytes.size()));
}

class FakeGatewayPeer {
public:
    FakeGatewayPeer()
    {
        expect(server_.listen(QHostAddress::LocalHost, 0),
               "fake gateway must listen");
    }

    quint16 port() const { return server_.serverPort(); }

    bool accept()
    {
        if (!pumpUntil([this] { return server_.hasPendingConnections(); })) {
            return false;
        }
        peer_ = server_.nextPendingConnection();
        decoder_.reset();
        queued_.clear();
        if (peer_ != nullptr)
        {
            QObject::connect(peer_, &QTcpSocket::readyRead, peer_, [this] { receive(); });
            receive();
        }
        return peer_ != nullptr;
    }

    std::optional<RbrpFrame> nextFrame(RbrpMessageKind kind,
                                       int timeoutMs = 1500)
    {
        std::optional<RbrpFrame> result;
        const bool found = pumpUntil([&] {
            for (auto it = queued_.begin(); it != queued_.end(); ++it) {
                if (it->kind == kind) {
                    result = *it;
                    queued_.erase(it);
                    return true;
                }
            }
            receive();
            return false;
        }, timeoutMs);
        return found ? result : std::nullopt;
    }

    void send(const GatewayMessage &message)
    {
        const auto encoded = encode_gateway_message(message);
        expect(encoded.status == RbrpEncodeStatus::Ok,
               "fake gateway response must encode");
        if (peer_ == nullptr || encoded.status != RbrpEncodeStatus::Ok) {
            return;
        }
        peer_->write(wireBytes(encoded.wire));
        peer_->flush();
        QApplication::processEvents(QEventLoop::AllEvents, 10);
    }

    void disconnectPeer()
    {
        if (peer_ == nullptr) {
            return;
        }
        peer_->disconnectFromHost();
        (void)pumpUntil(
            [this] { return peer_->state() == QAbstractSocket::UnconnectedState; });
        peer_->deleteLater();
        peer_ = nullptr;
        decoder_.reset();
        queued_.clear();
    }

private:
  void receive()
  {
      if (peer_ == nullptr || peer_->bytesAvailable() == 0)
          return;
      const QByteArray bytes = peer_->readAll();
      std::vector<RbrpFrame> decoded;
      const auto status = decoder_.feed(reinterpret_cast<const Byte *>(bytes.constData()),
                                        static_cast<std::size_t>(bytes.size()), decoded);
      expect(status == RbrpFeedStatus::Ok, "fake gateway must decode controller traffic");
      for (const auto &frame : decoded)
      {
          if (frame.kind != RbrpMessageKind::CommandRequest ||
              frame.payload != Bytes{static_cast<Byte>(RobotCommandKind::QueryCpgParameters)})
          {
              queued_.push_back(frame);
              continue;
          }
          // Acquire performs one read-only query. Complete it independently of
          // the actuator traffic observed by all existing safety assertions.
          constexpr quint16 querySequence = 0xff00;
          CommandSubmittedMessage submitted;
          submitted.status = CommandSubmittedStatus::Submitted;
          submitted.sequence = querySequence;
          send({frame.request_id, submitted});
          GatewayCommandOutcomeMessage outcome;
          outcome.command_kind = RobotCommandKind::QueryCpgParameters;
          outcome.event.outcome = GatewayCommandOutcome::Accepted;
          outcome.event.sequence = querySequence;
          outcome.event.result = 0;
          send({frame.request_id, outcome});
          robobeetle::protocol::CpgParametersSnapshot snapshot;
          snapshot.request_sequence = querySequence;
          snapshot.feature_level = 2;
          const auto payload = robobeetle::protocol::encode_cpg_snapshot(snapshot);
          expect(payload.has_value(), "fake parameter snapshot must encode");
          if (payload)
              send({0, GatewayCpgParametersTelemetry{1, 1000, *payload}});
      }
  }

    QTcpServer server_;
    QTcpSocket *peer_{nullptr};
    RbrpDecoder decoder_;
    std::vector<RbrpFrame> queued_;
};

GatewayMessage helloReply(RequestId requestId)
{
    HelloReply reply;
    reply.server_capabilities = 0;
    reply.max_payload = 512;
    reply.control_heartbeat_interval_ms = 250;
    reply.authority_lease_timeout_ms = 1000;
    return {requestId, reply};
}

GatewayMessage acquireReply(RequestId requestId)
{
    AcquireReply reply;
    reply.result = AcquireResult::Granted;
    reply.authority_state = AuthorityState::Owned;
    reply.session_state = GatewayApplicationSessionState::SafetyQuiet;
    reply.link_state = GatewayApplicationLinkState::Unconfirmed;
    reply.lease_timeout_ms = 1000;
    return {requestId, reply};
}

GatewayMessage activeState()
{
    ControlStateMessage state;
    state.authority_state = AuthorityState::Owned;
    state.session_state = GatewayApplicationSessionState::Online;
    state.link_state = GatewayApplicationLinkState::Active;
    state.reason = GatewayStateReason::Acquired;
    state.lease_remaining_ms = 900;
    return {0, state};
}

void sendSubmitted(FakeGatewayPeer &gateway,
                   RequestId requestId, quint16 sequence)
{
    CommandSubmittedMessage submitted;
    submitted.status = CommandSubmittedStatus::Submitted;
    submitted.sequence = sequence;
    gateway.send({requestId, submitted});
}

void sendOutcome(FakeGatewayPeer &gateway, RequestId requestId,
                 RobotCommandKind kind, quint16 sequence,
                 GatewayCommandOutcome outcome)
{
    GatewayCommandOutcomeMessage message;
    message.command_kind = kind;
    message.event.outcome = outcome;
    message.event.sequence = sequence;
    message.event.result = 0;
    gateway.send({requestId, message});
}
void ready(FakeGatewayPeer &g, rb::RemoteRobotController &c) {
    rb::ConsoleConnectionConfiguration cfg; cfg.endpoint=QStringLiteral("127.0.0.1"); cfg.tcpPort=g.port();
    c.connectController(cfg); expect(g.accept(), "loopback connects");
    auto hello=g.nextFrame(RbrpMessageKind::Hello); if(hello) g.send(helloReply(hello->request_id));
    expect(pumpUntil([&]{return c.canAcquireControl();}), "hello ready"); c.acquireControl();
    auto acquire=g.nextFrame(RbrpMessageKind::AcquireControl); if(acquire) {g.send(acquireReply(acquire->request_id));g.send(activeState());}
    expect(pumpUntil([&]{return c.isControlActive();}), "authority ready");
    robobeetle::protocol::CpgParametersSnapshot snapshot; snapshot.feature_level=2;
    g.send({0,GatewayCpgParametersTelemetry{1,1000,*robobeetle::protocol::encode_cpg_snapshot(snapshot)}});
    for (int i=0;i<5;++i) {
        const auto servo=static_cast<rb::ServoId>(i);
        expect(c.enableServo(servo), "enable each servo"); auto f=g.nextFrame(RbrpMessageKind::CommandRequest);
        if(f){sendSubmitted(g,f->request_id,100+i);sendOutcome(g,f->request_id,RobotCommandKind::EnableServos,100+i,GatewayCommandOutcome::Accepted);}
        expect(pumpUntil([&]{return c.isServoEnabled(servo);}), "enable confirmed");
        expect(c.neutralServo(servo), "neutral each servo"); f=g.nextFrame(RbrpMessageKind::CommandRequest);
        if(f){sendSubmitted(g,f->request_id,110+i);sendOutcome(g,f->request_id,RobotCommandKind::NeutralServos,110+i,GatewayCommandOutcome::Accepted);}
        expect(pumpUntil([&]{return (c.inferredPoseKnownMask() & (1U<<i)) != 0;}), "pose confirmed");
    }
}
void sample(FakeGatewayPeer &g, quint8 reason, quint8 state=0, quint8 mode=0, quint8 schema=2) {
    rb::MotionStateBatch batch; batch.schema_version=schema; batch.sample_count=1;
    batch.samples[0].stop_reason=reason; batch.samples[0].state=state;
    batch.samples[0].control_mode=mode; batch.samples[0].effective_pitch=-17;
    g.send({0,GatewayMotionStateTelemetry{1,1000,0,*robobeetle::protocol::encode_motion_state_batch(batch)}});
}
void testPorts() {
    FakeGatewayPeer g; rb::RemoteRobotController c;
    expect(!c.submitVisualProportionalStart({}), "unowned proportional start rejected");
    expect(!c.submitVisualProportionalStop(), "unowned proportional stop rejected");
    ready(g,c); int telemetry=0; QObject::connect(&c,&rb::IConsoleController::proportionalTelemetryChanged,[&]{++telemetry;});
    sample(g,0); expect(pumpUntil([&]{return telemetry==1;}), "schema2 sample emits observable telemetry");
    sample(g,3); expect(pumpUntil([&]{return c.proportionalStopReason()==3;}), "InputTimeout latched");
    expect(c.actualProportionalPitch()==-17, "existing actual pitch exposed");
    sample(g,5); expect(pumpUntil([&]{return c.proportionalStopReason()==5;}), "heartbeat reason latched");
    const int before=telemetry; sample(g,0,0,0,1); pumpUntil([]{return false;},20);
    expect(telemetry==before && c.proportionalStopReason()==5, "schema1 cannot replace schema2 stop reason");
    sample(g,0); pumpUntil([&]{return c.hasCpgSchema2();});
    rb::IConsoleController::ProportionalConfig invalid; invalid.max_scale=0;
    expect(!c.submitVisualProportionalStart(invalid), "invalid config rejects locally");
    expect(!c.stopMotion(), "legacy idle STOP preserves no-op behavior");
    const auto idleStop = c.submitVisualProportionalStop();
    const auto idleFrame = g.nextFrame(RbrpMessageKind::CommandRequest);
    expect(idleStop && idleFrame && idleFrame->request_id == *idleStop
           && idleFrame->payload == Bytes{static_cast<Byte>(RobotCommandKind::StopMotion)},
           "idle visual STOP returns wire ID for initial ARRIVED");
    std::optional<rb::CommandTerminalResult> idleResult;
    QObject::connect(&c, &rb::IConsoleController::commandTerminal,
                     [&](quint32 id, rb::CommandTerminalResult result, quint8, qint64) {
        if (idleStop && id == *idleStop) idleResult = result;
    });
    if (idleStop) {
        sendSubmitted(g, *idleStop, 190);
        sendOutcome(g, *idleStop, RobotCommandKind::StopMotion, 190, GatewayCommandOutcome::Accepted);
        expect(pumpUntil([&] { return idleResult.has_value(); }), "idle visual STOP receives terminal");
        expect(idleResult == rb::CommandTerminalResult::Ok, "idle visual STOP ACK permits recovery");
    }
    expect(pumpUntil([&] { return c.motionState() == rb::MotionState::Stopped && !c.isMotionActive(); }),
           "idle STOP settles before visual restart");
    auto start=c.submitVisualProportionalStart({}); auto f=g.nextFrame(RbrpMessageKind::CommandRequest);
    expect(start && f && *start==f->request_id && f->payload[0]==static_cast<Byte>(RobotCommandKind::StartProportional), "visual start returns wire request ID");
    expect(c.proportionalPending() && !c.setProportionalInput(600,0,0), "pending start rejects stream");
    if(!start)return;
    sendSubmitted(g,*start,200);
    auto stop=c.submitVisualProportionalStop(); f=g.nextFrame(RbrpMessageKind::CommandRequest);
    expect(stop && f && *stop==f->request_id && f->payload==Bytes{static_cast<Byte>(RobotCommandKind::StopMotion)}, "visual stop returns ordinary STOP wire ID");
    expect(!c.proportionalPending() && !c.proportionalActive(), "stop clears proportional state immediately");
    expect(c.submitVisualProportionalStop()==stop && c.stopMotion(), "duplicate and legacy stop reuse outstanding ID");
    expect(!g.nextFrame(RbrpMessageKind::CommandRequest,20), "duplicate stop emits no second wire command");
    std::optional<rb::CommandTerminalResult> terminal;
    QObject::connect(&c,&rb::IConsoleController::commandTerminal,[&](quint32 id, rb::CommandTerminalResult result, quint8, qint64){if(id==*start)terminal=result;});
    sendOutcome(g,*start,RobotCommandKind::StartProportional,200,GatewayCommandOutcome::Accepted);
    expect(pumpUntil([&]{return terminal.has_value();}), "late start terminal correlated");
    expect(terminal==rb::CommandTerminalResult::OutcomeUnknown && !c.proportionalActive(), "late start OK cannot revive stopped session");
    if(stop){sendSubmitted(g,*stop,201);sendOutcome(g,*stop,RobotCommandKind::StopMotion,201,GatewayCommandOutcome::Accepted);}
    sample(g,1); expect(pumpUntil([&]{return c.motionState()==rb::MotionState::Stopped && !c.isMotionActive();}), "STOP telemetry settles wait");
    start=c.submitVisualProportionalStart({}); f=g.nextFrame(RbrpMessageKind::CommandRequest);
    expect(start.has_value(), "restart available after STOPPED");
    if(start){sendSubmitted(g,*start,202);sendOutcome(g,*start,RobotCommandKind::StartProportional,202,GatewayCommandOutcome::Accepted);}
    expect(pumpUntil([&]{return c.proportionalActive();}), "accepted visual start enables stream");
    expect(c.setProportionalInput(600,-1000,500), "active visual stream writes latest slot");
    stop=c.submitVisualProportionalStop(); expect(stop && !c.setProportionalInput(600,0,0), "stop rejects subsequent stream");
    expect(!g.nextFrame(RbrpMessageKind::ProportionalInput,160), "cleared latest slot never reaches wire after stop");
    g.disconnectPeer();
}
void testFirmwareStopReasons() {
    for (const quint8 reason : {quint8(3), quint8(5)}) {
        FakeGatewayPeer gateway;
        rb::RemoteRobotController controller;
        ready(gateway, controller);
        sample(gateway, 0);
        expect(pumpUntil([&] { return controller.hasCpgSchema2(); }), "schema2 ready for timeout test");
        const auto start = controller.submitVisualProportionalStart({});
        const auto frame = gateway.nextFrame(RbrpMessageKind::CommandRequest);
        expect(start && frame && frame->request_id == *start, "firmware reason test starts proportional");
        if (!start) continue;
        sendSubmitted(gateway, *start, 300);
        sendOutcome(gateway, *start, RobotCommandKind::StartProportional, 300, GatewayCommandOutcome::Accepted);
        expect(pumpUntil([&] { return controller.proportionalActive(); }), "start ACK activates proportional");
        sample(gateway, 0, 1, 1);
        pumpUntil([] { return false; }, 20);
        bool reasonAvailableAtStop = false;
        QObject::connect(&controller, &rb::IConsoleController::proportionalStopped, [&] {
            reasonAvailableAtStop = controller.proportionalStopReason() == reason;
        });
        sample(gateway, reason, 2, 0);
        expect(pumpUntil([&] { return !controller.proportionalActive(); }), "firmware timeout clears active stream");
        expect(reasonAvailableAtStop && !controller.setProportionalInput(600, 0, 0),
               "stopped signal exposes timeout reason and stream is unavailable");
        gateway.disconnectPeer();
    }
}
} // namespace
int main(int argc,char **argv){QApplication app(argc,argv);testPorts();testFirmwareStopReasons();if (!failures) std::puts("visual_proportional_remote_tests passed");return failures?1:0;}
