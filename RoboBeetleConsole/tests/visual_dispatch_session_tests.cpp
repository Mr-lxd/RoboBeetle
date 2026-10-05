#include "vision/VisualDispatchSession.h"
#include "helpers/VisualControllerFixture.h"
#include "remote/RemoteRobotController.h"
#include "ui/MainWindow.h"

#include "robobeetle/gateway/gateway_types.hpp"
#include "robobeetle/gateway/rbrp_codec.hpp"

#include <QApplication>
#include <QElapsedTimer>
#include <QGroupBox>
#include <QHostAddress>
#include <QLabel>
#include <QPushButton>
#include <QStringList>
#include <QTcpServer>
#include <QTcpSocket>
#include <QThread>

#include <algorithm>
#include <cstdio>
#include <functional>
#include <memory>
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
            if (peer_ == nullptr || peer_->bytesAvailable() == 0) {
                return false;
            }
            const QByteArray bytes = peer_->readAll();
            std::vector<RbrpFrame> decoded;
            const auto status = decoder_.feed(
                reinterpret_cast<const Byte *>(bytes.constData()),
                static_cast<std::size_t>(bytes.size()), decoded);
            expect(status == RbrpFeedStatus::Ok,
                   "fake gateway must decode controller traffic");
            queued_.insert(queued_.end(), decoded.begin(), decoded.end());
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

QPushButton *buttonWithText(const QWidget *root, const QString &text)
{
    for (QPushButton *button : root->findChildren<QPushButton *>()) {
        if (button->text() == text) {
            return button;
        }
    }
    return nullptr;
}

QGroupBox *groupWithTitle(const QWidget *root, const QString &title)
{
    for (QGroupBox *box : root->findChildren<QGroupBox *>()) {
        if (box->title() == title) {
            return box;
        }
    }
    return nullptr;
}

void completeHello(FakeGatewayPeer &gateway)
{
    const auto hello = gateway.nextFrame(RbrpMessageKind::Hello);
    expect(hello.has_value(), "remote controller must send Hello");
    if (hello.has_value()) {
        gateway.send(helloReply(hello->request_id));
    }
}

void completeAcquire(FakeGatewayPeer &gateway)
{
    const auto acquire = gateway.nextFrame(RbrpMessageKind::AcquireControl);
    expect(acquire.has_value(), "UI Acquire must emit AcquireControl");
    if (acquire.has_value()) {
        gateway.send(acquireReply(acquire->request_id));
        gateway.send(activeState());
    }
}

void testDeterministicRuntimeCases()
{
    using namespace rb::vision;
    auto snapshot=[] {VisualDiagnosticSnapshot s;s.state=VisualState::Tracking;s.command.proposed=ProposedCommand::Forward;s.command.ex_f=0.5;return s;};
    {
        rb::test::VisualControllerFixture controller;
        const auto snap = snapshot();
        VisualDispatchSession session(&controller, [&] { return std::optional{snap}; },
                                      [] { return 0; });
        expect(session.confirmedTurnSign() == VisualPolicyConfig::configuredTurnSign
                   && VisualPolicyConfig::configuredTurnSign == 1,
               "session starts with the configured +1 turn sign already confirmed");
        expect(!session.featureEnabled() && !session.armed() && controller.sends.empty(),
               "configured turn sign neither enables the feature, arms, nor submits motion");
    }
    for (double error : {-0.5, 0.5}) {
        rb::test::VisualControllerFixture c;qint64 now=0;auto snap=snapshot();
        snap.command.ex_f=error;
        snap.command.proposed=error>0?ProposedCommand::TurnLeft:ProposedCommand::TurnRight;
        snap.target=TargetState{};snap.target->ex=-error;
        VisualDispatchSession s(&c,[&]{return std::optional{snap};},[&]{return now;});
        s.timerTick();expect(c.sends.empty(),"off default sends nothing");
        s.setFeatureEnabled(true);expect(s.arm()==ArmReason::Ready,"configured turn sign arms without operator confirmation");s.timerTick();
        expect(c.sends.size()==1&&c.sends.back().second==(error>0?rb::MotionMode::TurnRight:rb::MotionMode::TurnLeft),"filtered error and configured +1 sign map command from same snapshot");
        c.ack(c.sends.back().first,rb::CommandTerminalResult::Ok);expect(s.currentMode().has_value(),"ACK establishes visible mode");
        s.disarm();expect(!s.armed()&&!s.currentMode()&&c.sends.size()==2&&c.sends.back().second==rb::MotionMode::Stop,"disarm sends exactly one operator STOP");
        c.ack(c.sends.back().first,rb::CommandTerminalResult::Busy,7);expect(s.operatorStopResult()==rb::CommandTerminalResult::Busy,"operator busy remains visible");
        now=3000;s.timerTick();expect(c.sends.size()==2,"operator stop has no retries");
    }
    {
        rb::test::VisualControllerFixture c;auto snap=snapshot();qint64 now=0;VisualDispatchSession s(&c,[&]{return std::optional{snap};},[&]{return now;});
        std::vector<VisualDispatchRecord> records;QObject::connect(&s,&VisualDispatchSession::dispatchRecorded,[&](auto record){records.push_back(record);});
        s.setFeatureEnabled(true);s.arm();s.timerTick();auto old=c.sends.back().first;
        c.disconnectController();
        expect(!s.armed()&&records.size()>=3&&records[1].command==ProposedCommand::Stop,"loss attempts immediate policy safety STOP before clearing associations");
        now=5000;s.timerTick();c.ack(old,rb::CommandTerminalResult::Ok);expect(c.sends.size()==1&&!s.currentMode(),"loss ends retries and stale ACK associations");
    }
    {
        rb::test::VisualControllerFixture c;auto snap=snapshot();VisualDispatchSession s(&c,[&]{return std::optional{snap};},[]{return 0;});
        s.setFeatureEnabled(true);s.arm();s.disarm();auto stop=c.sends.back().first;
        s.setFeatureEnabled(false);c.ack(stop,rb::CommandTerminalResult::Ok);
        expect(s.operatorStopResult()==rb::CommandTerminalResult::Ok&&c.sends.size()==1,"operator STOP correlation survives later feature off without extra STOP");
    }
    for (auto result : {rb::CommandTerminalResult::Ok,
                        rb::CommandTerminalResult::Busy,
                        rb::CommandTerminalResult::Rejected,
                        rb::CommandTerminalResult::OutcomeUnknown}) {
        rb::test::VisualControllerFixture controller;
        auto snap = snapshot();
        qint64 now = 0;
        VisualDispatchSession session(&controller, [&] { return std::optional{snap}; },
                                      [&] { return now; });
        session.setFeatureEnabled(true);
                session.arm();
        session.timerTick();
        const auto oldStart = controller.sends.back().first;
        session.setFeatureEnabled(false);
        expect(controller.sends.size() == 2 && !session.armed(),
               "off during pending START sends exactly one operator STOP");
        controller.ack(oldStart, rb::CommandTerminalResult::Ok);
        expect(!session.armed() && !session.currentMode(),
               "direct off pending START late OK cannot revive confirmed mode");
        controller.ack(controller.sends.back().first, result);
        expect(session.operatorStopResult() == result,
               "off retains every operator STOP terminal result");
        now = 5000;
        session.timerTick();
        expect(controller.sends.size() == 2,
               "operator STOP terminal matrix never triggers automatic retries");
    }
    for(bool enabledOnly:{false,true}) {
        rb::test::VisualControllerFixture c;auto snap=snapshot();VisualDispatchSession s(&c,[&]{return std::optional{snap};},[]{return 0;});
        s.setFeatureEnabled(enabledOnly);s.setFeatureEnabled(false);expect(c.sends.empty(),"off never armed sends zero");
    }
    {
        rb::test::VisualControllerFixture c;auto snap=snapshot();qint64 now=0;VisualDispatchSession s(&c,[&]{return std::optional{snap};},[&]{return now;});
        s.setFeatureEnabled(true);s.arm();s.timerTick();auto old=c.sends.back().first;
        s.manualInput(ManualInputKind::Motion);s.setFeatureEnabled(false);c.ack(old,rb::CommandTerminalResult::Ok);expect(c.sends.size()==1&&!s.armed()&&!s.currentMode(),"manual pending takeover and late OK cannot revive mode or STOP on off");
    }
    {
        rb::test::VisualControllerFixture c;auto snap=snapshot();qint64 now=0;VisualDispatchSession s(&c,[&]{return std::optional{snap};},[&]{return now;});
        s.setFeatureEnabled(true);s.arm();s.timerTick();c.ack(c.sends.back().first,rb::CommandTerminalResult::Rejected,6);
        expect(s.poseMismatch()==QStringLiteral("可能是姿态未知（Qt 推断与固件不一致）"),"HardwareFailure diagnostic is inference only");expect(c.sends.size()==2&&c.sends.back().second==rb::MotionMode::Stop,"rejected START immediately safety stops");
        c.ack(c.sends.back().first,rb::CommandTerminalResult::Ok);expect(!s.poseMismatch().isEmpty(),"STOP preserves hardware diagnostic");
    }
    {
        rb::test::VisualControllerFixture c;c.synchronous=true;auto snap=snapshot();VisualDispatchSession s(&c,[&]{return std::optional{snap};},[]{return 0;});
        s.setFeatureEnabled(true);s.arm();s.timerTick();QApplication::processEvents();expect(s.currentMode()==ProposedCommand::Forward,"synchronous callback deferred until mapping installed");
        s.disarm();QApplication::processEvents();expect(s.operatorStopResult()==rb::CommandTerminalResult::Ok,"synchronous operator stop correlation");
    }
    {
        rb::test::VisualControllerFixture c;c.backend=rb::ConsoleBackendKind::DirectSerial;auto snap=snapshot();VisualDispatchSession s(&c,[&]{return std::optional{snap};},[]{return 0;});
        s.setFeatureEnabled(true);expect(s.arm()==ArmReason::LinkDisconnected,"direct maintenance cannot arm despite mock active flags");s.timerTick();expect(c.sends.empty(),"direct maintenance always dry run");
    }
}

void testTimerAndRealDiagnosticGrace()
{
    using namespace rb::vision;
    {
        rb::test::VisualControllerFixture c;qint64 now=0;VisualDiagnosticSnapshot snap;snap.state=VisualState::Tracking;snap.command.proposed=ProposedCommand::Forward;
        VisualDispatchSession s(&c,[&]{return std::optional{snap};},[&]{return now;});s.setFeatureEnabled(true);s.arm();
        expect(pumpUntil([&]{return c.sends.size()==1;},200),"50ms timer starts motion without manual tick");
        now=1000;expect(pumpUntil([&]{return c.sends.size()==2;},200),"timer detects outstanding nonSTOP deadline without frames");
        for(qint64 deadline:{2000,3000,4000}){now=deadline;auto count=c.sends.size();expect(pumpUntil([&]{return c.sends.size()>count;},200),"timer retries STOP without frames");}
        expect(s.stopTimeoutAlert(),"timer alone latches three-timeout STOP alert");
        s.setFeatureEnabled(false);auto n=c.sends.size();now=10000;QEventLoop loop;QTimer::singleShot(80,&loop,&QEventLoop::quit);loop.exec();expect(c.sends.size()==n,"off cancels timer automatic retries");
    }
    {
        rb::test::VisualControllerFixture c;qint64 now=0;VisualDiagnosticSession diagnostic({},[&]{return now;});
        DetectionFrame frame{1,1001,{640,480},{{0,"fish",.9,{320,240}}}};
        diagnostic.onDetectionArrival(frame,{DetectionDisplayState::Target,frame});
        VisualDispatchSession s(&c,[&]{return std::optional{diagnostic.snapshot()};},[&]{return now;});s.setFeatureEnabled(true);expect(s.arm()==ArmReason::Ready,"real diagnostic tracking arms");s.timerTick();c.ack(c.sends.back().first,rb::CommandTerminalResult::Ok);
        now=20;frame.frameId=2;diagnostic.onDetectionArrival(frame,{DetectionDisplayState::AwaitingVideo,frame});
        now=420;diagnostic.refresh({DetectionDisplayState::AwaitingVideo,frame});s.timerTick();
        expect(diagnostic.snapshot().awaitingVideo&&diagnostic.snapshot().state==VisualState::Tracking&&s.armed()&&c.sends.size()==1,"real diagnostic 400ms grace preserves ACKconfirmed Forward");
        now=520;diagnostic.refresh({DetectionDisplayState::AwaitingVideo,frame});s.timerTick();
        expect(diagnostic.snapshot().state==VisualState::Stale&&!s.armed()&&c.sends.size()==2&&c.sends.back().second==rb::MotionMode::Stop,"real diagnostic 500ms missing video propagates STALE STOP");
    }
    {
        rb::test::VisualControllerFixture c;qint64 now=0;std::optional<VisualDiagnosticSnapshot> snapshot{VisualDiagnosticSnapshot{}};snapshot->state=VisualState::Tracking;snapshot->command.proposed=ProposedCommand::Forward;
        VisualDispatchSession s(&c,[&]{return snapshot;},[&]{return now;});s.setFeatureEnabled(true);s.arm();s.timerTick();snapshot.reset();now=50;s.timerTick();expect(!s.armed()&&c.sends.size()==2&&c.sends.back().second==rb::MotionMode::Stop,"only missing snapshot produces adapter STALE");
    }
}

void testWireFailureSafetyStop()
{
    using namespace rb::vision;
    FakeGatewayPeer gateway;rb::RemoteRobotController c;rb::ConsoleConnectionConfiguration cfg;
    cfg.endpoint=QStringLiteral("127.0.0.1");cfg.tcpPort=gateway.port();c.connectController(cfg);gateway.accept();completeHello(gateway);
    pumpUntil([&]{return c.canAcquireControl();});c.acquireControl();completeAcquire(gateway);pumpUntil([&]{return c.isControlActive();});
    int seq=10;
    for(int i:{0,1,3,4}){c.enableServo(static_cast<rb::ServoId>(i));auto f=gateway.nextFrame(RbrpMessageKind::CommandRequest);if(f){sendSubmitted(gateway,f->request_id,seq);sendOutcome(gateway,f->request_id,RobotCommandKind::EnableServos,seq++,GatewayCommandOutcome::Accepted);pumpUntil([&]{return c.isServoEnabled(static_cast<rb::ServoId>(i));});}}
    qint64 now=0;VisualDiagnosticSnapshot snap;snap.state=VisualState::Tracking;snap.command.proposed=ProposedCommand::Forward;
    VisualDispatchSession s(&c,[&]{return std::optional{snap};},[&]{return now;});s.setFeatureEnabled(true);expect(s.arm()==ArmReason::Ready,"wire failure session arm");s.timerTick();
    auto start=gateway.nextFrame(RbrpMessageKind::CommandRequest);expect(start&&start->payload==Bytes{static_cast<Byte>(RobotCommandKind::StartMotion),1},"visual START maps modebyte on existing RBRP socket");
    if(!start){gateway.disconnectPeer();return;}
    sendSubmitted(gateway,start->request_id,50);GatewayCommandOutcomeMessage rejected;rejected.command_kind=RobotCommandKind::StartMotion;rejected.event={GatewayCommandOutcome::Rejected,50,6};gateway.send({start->request_id,rejected});
    expect(pumpUntil([&]{return !s.poseMismatch().isEmpty();}),"wire raw HardwareFailure triggers Qt inferred mismatch diagnostic");
    auto stop=gateway.nextFrame(RbrpMessageKind::CommandRequest);expect(stop&&stop->payload==Bytes{static_cast<Byte>(RobotCommandKind::StopMotion)},"wire rejected START sends empty safety STOP");
    if(stop){sendSubmitted(gateway,stop->request_id,51);sendOutcome(gateway,stop->request_id,RobotCommandKind::StopMotion,51,GatewayCommandOutcome::Accepted);}
    expect(pumpUntil([&]{return c.motionState()==rb::MotionState::Stopped && !c.isMotionActive();}),"wire safety STOP completes");
    expect(s.poseMismatch()==QStringLiteral("可能是姿态未知（Qt 推断与固件不一致）"),"wire STOP preserves diagnostic exactly");
    now=1500;expect(s.arm()==ArmReason::Ready,"explicit rearm after failed command");s.timerTick();auto uncertain=gateway.nextFrame(RbrpMessageKind::CommandRequest);
    if(uncertain){sendSubmitted(gateway,uncertain->request_id,52);sendOutcome(gateway,uncertain->request_id,RobotCommandKind::StartMotion,52,GatewayCommandOutcome::OutcomeUnknown);}
    auto safety=gateway.nextFrame(RbrpMessageKind::CommandRequest);auto release=gateway.nextFrame(RbrpMessageKind::ReleaseControl);
    expect(safety&&release&&safety->request_id<release->request_id&&safety->payload==Bytes{static_cast<Byte>(RobotCommandKind::StopMotion)},"wire unknown START submits immediate STOP before authority release");
    expect(!s.armed()&&!s.currentMode(),"unknown and authority loss cannot retain confirmed mode");gateway.disconnectPeer();
}

void testAcceptedStopRetiresRetryEpisode()
{
    using namespace rb::vision;
    for (bool acceptFirst : {false, true}) {
        FakeGatewayPeer gateway;
        rb::RemoteRobotController controller;
        rb::ConsoleConnectionConfiguration config;
        config.endpoint = QStringLiteral("127.0.0.1");
        config.tcpPort = gateway.port();
        controller.connectController(config);
        expect(gateway.accept(), "STOP retry regression connects");
        completeHello(gateway);
        pumpUntil([&] { return controller.canAcquireControl(); });
        controller.acquireControl();
        completeAcquire(gateway);
        pumpUntil([&] { return controller.isControlActive(); });
        quint16 sequence = 10;
        for (int i : {0, 1, 3, 4}) {
            controller.enableServo(static_cast<rb::ServoId>(i));
            auto frame = gateway.nextFrame(RbrpMessageKind::CommandRequest);
            if (frame) {
                sendSubmitted(gateway, frame->request_id, sequence);
                sendOutcome(gateway, frame->request_id, RobotCommandKind::EnableServos,
                            sequence++, GatewayCommandOutcome::Accepted);
                pumpUntil([&] { return controller.isServoEnabled(static_cast<rb::ServoId>(i)); });
            }
        }
        qint64 now = 0;
        VisualDiagnosticSnapshot snapshot;
        snapshot.state = VisualState::Tracking;
        snapshot.command.proposed = ProposedCommand::Forward;
        VisualDispatchSession session(&controller, [&] { return std::optional{snapshot}; },
                                      [&] { return now; });
        session.setFeatureEnabled(true);
                expect(session.arm() == ArmReason::Ready, "STOP retry regression arms");
        session.timerTick();
        auto start = gateway.nextFrame(RbrpMessageKind::CommandRequest);
        if (!start) { gateway.disconnectPeer(); continue; }
        sendSubmitted(gateway, start->request_id, 50);
        sendOutcome(gateway, start->request_id, RobotCommandKind::StartMotion, 50,
                    GatewayCommandOutcome::Accepted);
        expect(pumpUntil([&] { return session.currentMode() == ProposedCommand::Forward; }),
               "initial forward ACK confirmed");
        now = 50;
        snapshot.state = VisualState::Lost;
        session.timerTick();
        auto first = gateway.nextFrame(RbrpMessageKind::CommandRequest);
        if (!first) { gateway.disconnectPeer(); continue; }
        sendSubmitted(gateway, first->request_id, 51);
        now = 1050;
        session.timerTick();
        auto retry = gateway.nextFrame(RbrpMessageKind::CommandRequest);
        if (!retry) { gateway.disconnectPeer(); continue; }
        sendSubmitted(gateway, retry->request_id, 52);
        const auto confirmedId = acceptFirst ? first->request_id : retry->request_id;
        const auto retiredId = acceptFirst ? retry->request_id : first->request_id;
        sendOutcome(gateway, confirmedId, RobotCommandKind::StopMotion,
                    acceptFirst ? 51 : 52, GatewayCommandOutcome::Accepted);
        expect(pumpUntil([&] { return session.currentMode() == ProposedCommand::Stop; }),
               "any STOP in episode can confirm STOP");
        expect(pumpUntil([&] { return controller.motionState() == rb::MotionState::Stopped; }),
               "accepted STOP transition finishes");
        expect(controller.isMotionReady(rb::MotionMode::Forward),
               "accepted STOP retires unanswered retry readiness blockers");
        now = 2050;
        snapshot.state = VisualState::Tracking;
        session.timerTick();
        auto resumed = gateway.nextFrame(RbrpMessageKind::CommandRequest);
        expect(resumed && resumed->payload == Bytes{static_cast<Byte>(RobotCommandKind::StartMotion), 1},
               "Tracking resumes START after accepted STOP and dwell");
        expect(session.armed(), "accepted STOP retry preserves arming for Tracking recovery");
        if (resumed && resumed->payload[0] == static_cast<Byte>(RobotCommandKind::StartMotion)) {
            sendSubmitted(gateway, resumed->request_id, 53);
            sendOutcome(gateway, resumed->request_id, RobotCommandKind::StartMotion, 53,
                        GatewayCommandOutcome::Accepted);
            expect(pumpUntil([&] { return session.currentMode() == ProposedCommand::Forward; }),
                   "resumed START ACK confirms forward");
        }
        // Leave the other STOP without an outcome beyond the full controller
        // timeout window, then deliver its old ACK after Forward resumed.
        expect(!gateway.nextFrame(RbrpMessageKind::ReleaseControl, 3000),
               "retired STOP never causes later command timeout authority loss");
        sendOutcome(gateway, retiredId, RobotCommandKind::StopMotion,
                    acceptFirst ? 52 : 51, GatewayCommandOutcome::Accepted);
        QApplication::processEvents();
        expect(controller.isControlActive()
               && controller.motionState() == rb::MotionState::Running
               && controller.motionMode() == rb::MotionMode::Forward
               && session.armed() && session.currentMode() == ProposedCommand::Forward,
               "late retired STOP ACK cannot reset resumed motion or arming");
        gateway.disconnectPeer();
    }
}

void testSessionRuntime()
{
    using namespace rb::vision;
    FakeGatewayPeer gateway; rb::RemoteRobotController c;
    rb::ConsoleConnectionConfiguration cfg; cfg.endpoint=QStringLiteral("127.0.0.1");cfg.tcpPort=gateway.port();
    c.connectController(cfg); expect(gateway.accept(),"runtime connects");completeHello(gateway);
    pumpUntil([&]{return c.canAcquireControl();});c.acquireControl();completeAcquire(gateway);pumpUntil([&]{return c.isControlActive();});
    int seq=10;
    for (int i : {0,1,3,4}) {
        c.enableServo(static_cast<rb::ServoId>(i)); auto f=gateway.nextFrame(RbrpMessageKind::CommandRequest);
        if(f){sendSubmitted(gateway,f->request_id,seq);sendOutcome(gateway,f->request_id,RobotCommandKind::EnableServos,seq++,GatewayCommandOutcome::Accepted);pumpUntil([&]{return c.isServoEnabled(static_cast<rb::ServoId>(i));});}
    }
    qint64 now=0; VisualDiagnosticSnapshot snap;snap.state=VisualState::Tracking;snap.command.proposed=ProposedCommand::Forward;
    VisualDispatchSession session(&c,[&]{return std::optional{snap};},[&]{return now;});
    std::vector<VisualDispatchRecord> records;QObject::connect(&session,&VisualDispatchSession::dispatchRecorded,[&](auto r){records.push_back(r);});
    expect(!session.featureEnabled()&&!session.armed(),"runtime starts feature off and disarmed");
    session.setFeatureEnabled(true);expect(session.arm()==ArmReason::Ready,"configured turn sign needs no operator confirmation");
    expect(session.armed(),"operator action arms runtime");
    if(!session.armed()){gateway.disconnectPeer();return;}
    session.timerTick();auto forward=gateway.nextFrame(RbrpMessageKind::CommandRequest);expect(forward.has_value(),"runtime forwards snapshot command");
    if(!forward){gateway.disconnectPeer();return;}
    sendSubmitted(gateway,forward->request_id,200);sendOutcome(gateway,forward->request_id,RobotCommandKind::StartMotion,200,GatewayCommandOutcome::Accepted);
    expect(pumpUntil([&]{return session.currentMode()==ProposedCommand::Forward;}),"mode requires correlated OK");
    now=400;snap.awaitingVideo=true;session.timerTick();expect(session.armed(),"AwaitingVideo 400ms preserves Tracking arming");
    expect(!gateway.nextFrame(RbrpMessageKind::CommandRequest,20),"video grace sends no STOP");
    now=500;snap.state=VisualState::Stale;session.timerTick();expect(!session.armed(),"upstream STALE disarms");
    expect(gateway.nextFrame(RbrpMessageKind::CommandRequest).has_value(),"STALE sends safety STOP");
    now=1500;session.timerTick();expect(gateway.nextFrame(RbrpMessageKind::CommandRequest).has_value(),"STOP timeout retries without frames");
    now=2500;session.timerTick();gateway.nextFrame(RbrpMessageKind::CommandRequest);
    now=3500;session.timerTick();gateway.nextFrame(RbrpMessageKind::CommandRequest);expect(session.stopTimeoutAlert(),"three expiries latch alert");
    session.setFeatureEnabled(false);auto op=gateway.nextFrame(RbrpMessageKind::CommandRequest);expect(op.has_value(),"off while automatic STOP awaits sends one operator STOP");
    if(op){sendSubmitted(gateway,op->request_id,201);sendOutcome(gateway,op->request_id,RobotCommandKind::StopMotion,201,GatewayCommandOutcome::Accepted);expect(pumpUntil([&]{return session.operatorStopResult()==rb::CommandTerminalResult::Ok;}),"off retains operator STOP result");}
    now=6000;session.timerTick();expect(!gateway.nextFrame(RbrpMessageKind::CommandRequest,20),"off never retries STOP");
    session.setFeatureEnabled(true);session.manualInput(ManualInputKind::Motion);session.setFeatureEnabled(false);expect(!gateway.nextFrame(RbrpMessageKind::CommandRequest,20),"off after manual takeover sends zero commands");
    expect(records.size()>=8,"runtime records sends and terminal outcomes");
    gateway.disconnectPeer();
}
void testReentrantSafetySnapshotIsRetained()
{
    using namespace rb::vision;
    class ReentrantController : public rb::test::VisualControllerFixture {
    public:
        std::function<void()> duringSubmit;
        int depth{0}, maximumDepth{0};
        std::optional<quint32> submitVisualMotion(rb::MotionMode mode) override {
            ++depth; maximumDepth = std::max(maximumDepth, depth);
            const auto id = VisualControllerFixture::submitVisualMotion(mode);
            if (duringSubmit) { auto once = std::move(duringSubmit); duringSubmit = {}; once(); }
            --depth;
            return id;
        }
    };
    for (bool insideSubmit : {true, false})
    for (auto state : {VisualState::Stale, VisualState::InferenceOff, VisualState::Lost}) {
        ReentrantController controller;
        qint64 now = 0;
        VisualDiagnosticSession diagnostic({}, [&] { return now; });
        DetectionFrame frame{1, 1001, {640, 480}, {{0, "fish", .9, {320, 240}}}};
        diagnostic.onDetectionArrival(frame, {DetectionDisplayState::Target, frame});
        VisualDispatchSession session(&controller, &diagnostic);
        session.setFeatureEnabled(true);
        expect(session.arm() == ArmReason::Ready, "reentrant diagnostic fixture arms");
        const auto emitTransient = [&] {
            auto event = diagnostic.snapshot();
            event.state = state; event.command.proposed = ProposedCommand::Stop;
            emit diagnostic.diagnosticChanged(event);
            // The provider remains TRACKING; recovery arrives before queued safety evaluation.
            now = 10;
            emit diagnostic.diagnosticChanged(diagnostic.snapshot());
        };
        bool emitted = false;
        if (insideSubmit) controller.duringSubmit = emitTransient;
        else QObject::connect(&session, &VisualDispatchSession::dispatchRecorded,
            [&](const VisualDispatchRecord &record) {
                if (!emitted && record.command == ProposedCommand::Forward && record.result == "SENT") {
                    emitted = true;
                    emitTransient(); // submitting_ is false but evaluate is still on the stack.
                }
            });
        session.timerTick();
        expect(controller.sends.size() == 1 && controller.maximumDepth == 1,
               "diagnostic during submission never recursively submits STOP");
        QApplication::processEvents();
        expect(controller.sends.size() == 2 && controller.sends.back().second == rb::MotionMode::Stop,
               "queued safety event uses captured snapshot despite TRACKING recovery");
        expect(session.armed() == (state == VisualState::Lost),
               "queued STALE/INFERENCE_OFF disarm; queued LOST retains arming");
        if (controller.sends.size() == 2) controller.ack(controller.sends.back().first, rb::CommandTerminalResult::Ok);
        controller.ack(controller.sends.front().first, rb::CommandTerminalResult::Ok);
        now = 1010; session.timerTick();
        if (state != VisualState::Lost)
            expect(!session.armed() && controller.sends.size() == 2,
                   "late START OK and next dwell cannot revive a transient safety disarm");
        else
            expect(session.armed() && controller.sends.size() == 3
                   && controller.sends.back().second == rb::MotionMode::Forward,
                   "transient LOST resumes only after STOP acceptance dwell");
        expect(controller.maximumDepth == 1, "deferred STOP submission is never nested");
    }
}

void testAuthorityLossInSameEvent()
{
    using namespace rb::vision;
    rb::test::VisualControllerFixture controller;
    qint64 now = 0;
    VisualDiagnosticSnapshot snapshot;
    snapshot.state = VisualState::Tracking;
    snapshot.command.proposed = ProposedCommand::Forward;
    VisualDispatchSession session(&controller, [&] { return std::optional{snapshot}; },
                                  [&] { return now; });
    session.setFeatureEnabled(true);
    expect(session.arm() == ArmReason::Ready, "authority-loss fixture is armed");
    session.timerTick();
    controller.ack(controller.sends.back().first, rb::CommandTerminalResult::Ok);
    controller.active = false;
    emit controller.authorityStateChanged(rb::ControlAuthorityState::Unowned, false);
    // No event pumping or timer tick: disarm must happen inside this signal.
    expect(controller.isConnected() && !session.armed() && !session.currentMode(),
           "authority loss disarms in the same event with TCP still connected");
    const auto count = controller.sends.size();
    controller.active = true;
    emit controller.authorityStateChanged(rb::ControlAuthorityState::Owned, true);
    now = 2000;
    session.timerTick();
    expect(!session.armed() && controller.sends.size() == count,
           "authority restoration cannot rearm or revive old retries");
}

void testPoseMismatchRequiresCompleteMask()
{
    using namespace rb::vision;
    const quint16 required = 0x0003; // Also prove this follows configured requirements.
    for (const quint16 known : {quint16(0), quint16(1), quint16(2), quint16(4), required}) {
        rb::test::VisualControllerFixture controller;
        controller.known = required;
        VisualDiagnosticSnapshot snapshot;
        snapshot.state = VisualState::Tracking;
        snapshot.command.proposed = ProposedCommand::Forward;
        VisualDispatchConfig config;
        config.requiredServoMask = required;
        VisualDispatchSession session(&controller, [&] { return std::optional{snapshot}; },
                                      [] { return 0; }, config);
        session.setFeatureEnabled(true);
                expect(session.arm() == ArmReason::Ready, "pose-mask rejection fixture arms");
        session.timerTick();
        controller.known = known;
        controller.ack(controller.sends.back().first, rb::CommandTerminalResult::Rejected, 6);
        expect(session.poseMismatch().isEmpty() == ((known & required) != required),
               "pose mismatch explanation requires every configured required bit known");
    }
}

void testAutomaticStopCoversOperatorStop()
{
    using namespace rb::vision;
    FakeGatewayPeer gateway;
    rb::RemoteRobotController controller;
    rb::ConsoleConnectionConfiguration config;
    config.endpoint = QStringLiteral("127.0.0.1"); config.tcpPort = gateway.port();
    controller.connectController(config);
    expect(gateway.accept(), "covered STOP loopback connects");
    completeHello(gateway);
    pumpUntil([&] { return controller.canAcquireControl(); });
    controller.acquireControl(); completeAcquire(gateway);
    pumpUntil([&] { return controller.isControlActive(); });
    quint16 seq = 10;
    for (int i : {0, 1, 3, 4}) {
        controller.enableServo(static_cast<rb::ServoId>(i));
        const auto request = gateway.nextFrame(RbrpMessageKind::CommandRequest);
        if (!request) { expect(false, "covered STOP pose request exists"); return; }
        sendSubmitted(gateway, request->request_id, seq);
        sendOutcome(gateway, request->request_id, RobotCommandKind::EnableServos,
                    seq++, GatewayCommandOutcome::Accepted);
        expect(pumpUntil([&] { return controller.isServoEnabled(static_cast<rb::ServoId>(i)); }),
               "covered STOP enable ACK establishes evidence");
    }
    VisualDiagnosticSnapshot snapshot;
    snapshot.state = VisualState::Tracking;
    snapshot.command.proposed = ProposedCommand::Hold;
    VisualDispatchSession session(&controller, [&] { return std::optional{snapshot}; },
                                  [] { return 0; });
    std::vector<VisualDispatchRecord> records;
    QObject::connect(&session, &VisualDispatchSession::dispatchRecorded,
                     [&](const auto &record) { records.push_back(record); });
    session.setFeatureEnabled(true);
    expect(session.arm() == ArmReason::Ready, "covered STOP session arms");
    session.timerTick(); // Unconfirmed HOLD requests an automatic STOP.
    const auto automatic = gateway.nextFrame(RbrpMessageKind::CommandRequest);
    if (!automatic) { expect(false, "automatic STOP exists before feature off"); return; }
    sendSubmitted(gateway, automatic->request_id, 50);
    session.setFeatureEnabled(false);
    const auto operatorStop = gateway.nextFrame(RbrpMessageKind::CommandRequest);
    if (!operatorStop) { expect(false, "feature off sends operator STOP"); return; }
    sendSubmitted(gateway, operatorStop->request_id, 51);
    sendOutcome(gateway, automatic->request_id, RobotCommandKind::StopMotion, 50,
                GatewayCommandOutcome::Accepted);
    expect(pumpUntil([&] { return session.operatorStopResult().has_value(); }),
           "automatic STOP produces covered operator result");
    expect(session.operatorStopResult() == rb::CommandTerminalResult::Ok,
           "automatic STOP OK covers the operator STOP instead of unknown");
    bool covered = false;
    for (const auto &record : records)
        if (record.wireId == operatorStop->request_id && record.result == "OK (covered)")
            covered = true;
    expect(covered, "covered operator STOP has explicit audit/display result");
    expect(pumpUntil([&] { return controller.motionState() == rb::MotionState::Stopped
                                  && !controller.isMotionActive(); }),
           "covered operator STOP leaves no pending readiness blocker");
    sendOutcome(gateway, operatorStop->request_id, RobotCommandKind::StopMotion, 51,
                GatewayCommandOutcome::Accepted);
    expect(session.operatorStopResult() == rb::CommandTerminalResult::Ok,
           "late operator STOP ACK cannot undo covered success");
    gateway.disconnectPeer();
}

// Task 06: readiness() reports every condition independently and is a pure
// query. Each item is falsified on its own while the other two stay satisfied.
void testAutoFollowReadinessEachItemIndependently()
{
    using namespace rb::vision;
    VisualDiagnosticSnapshot snap;
    snap.state = VisualState::Tracking;
    snap.command.proposed = ProposedCommand::Forward;
    const auto build = [&snap](rb::test::VisualControllerFixture &c, qint64 &now) {
        return std::make_unique<VisualDispatchSession>(
            &c, [&snap] { return std::optional{snap}; }, [&now] { return now; });
    };

    // Baseline: everything satisfied.
    {
        rb::test::VisualControllerFixture c; qint64 now = 0;
        auto session = build(c, now);
        const auto ready = session->readiness();
        expect(ready.linkAndControl && ready.servosReady && ready.tracking
                   && ready.allReady() && ready.missingCount() == 0,
               "fully provisioned session reports every readiness item satisfied");
    }
    // 1. Link + control alone is false.
    {
        rb::test::VisualControllerFixture c; qint64 now = 0;
        c.active = false;
        auto session = build(c, now);
        const auto ready = session->readiness();
        expect(!ready.linkAndControl && ready.servosReady && ready.tracking
                   && !ready.allReady() && ready.missingCount() == 1,
               "losing control authority falsifies only the link/control item");
        c.active = true; c.connected = false;
        const auto disconnected = session->readiness();
        expect(!disconnected.linkAndControl && disconnected.servosReady
                   && disconnected.tracking && disconnected.missingCount() == 1,
               "a closed link falsifies only the link/control item");
        c.backend = rb::ConsoleBackendKind::DirectSerial; c.connected = true;
        expect(!session->readiness().linkAndControl,
               "direct maintenance link can never satisfy the readiness item");
    }
    // 2. Servos alone is false: disabled mask and unknown pose are both fatal.
    {
        rb::test::VisualControllerFixture c; qint64 now = 0;
        c.enabled = 0;
        auto session = build(c, now);
        const auto ready = session->readiness();
        expect(ready.linkAndControl && !ready.servosReady && ready.tracking
                   && ready.missingCount() == 1,
               "disabled required servos falsify only the servo/pose item");
        c.enabled = 0x1b; c.known = 0;
        const auto unknownPose = session->readiness();
        expect(unknownPose.linkAndControl && !unknownPose.servosReady
                   && unknownPose.tracking && unknownPose.missingCount() == 1,
               "enabled servos with unknown pose still falsify the servo/pose item");
        c.known = 0x1b;
        expect(session->readiness().servosReady,
               "complete enabled and known mask satisfies the servo/pose item");
    }
    // 3. Tracking alone is false.
    {
        rb::test::VisualControllerFixture c; qint64 now = 0;
        auto session = build(c, now);
        expect(session->readiness().tracking, "TRACKING snapshot satisfies the tracking item");
        snap.state = VisualState::Stale;
        const auto stale = session->readiness();
        expect(stale.linkAndControl && stale.servosReady && !stale.tracking
                   && stale.missingCount() == 1,
               "STALE falsifies only the tracking item");
        snap.state = VisualState::NoTarget;
        expect(!session->readiness().tracking,
               "NO_TARGET falsifies only the tracking item");
        snap.state = VisualState::Tracking;
        expect(session->readiness().tracking, "restored TRACKING satisfies the item again");
    }
    // All three false at once, and readiness() must not disturb any state.
    {
        rb::test::VisualControllerFixture c; qint64 now = 0;
        c.active = false; c.enabled = 0; c.known = 0;
        snap.state = VisualState::Stale;
        auto session = build(c, now);
        const auto ready = session->readiness();
        expect(!ready.linkAndControl && !ready.servosReady && !ready.tracking
                   && ready.missingCount() == 3 && !ready.allReady(),
               "a fully unprovisioned session reports three missing items");
        expect(!session->armed() && !session->featureEnabled() && c.sends.empty(),
               "readiness() is a pure query: no arming, no feature change, no submission");
        snap.state = VisualState::Tracking;
    }
}

}
// Task 06: eligibility() and stopAwaiting() back the READY and STOPPING labels.
void testAutoFollowEligibilityAndStopAwaiting()
{
    using namespace rb::vision;
    VisualDiagnosticSnapshot snap;
    snap.state = VisualState::Tracking;
    snap.command.proposed = ProposedCommand::Forward;
    rb::test::VisualControllerFixture c;
    qint64 now = 100;
    VisualDispatchSession session(
        &c, [&snap] { return std::optional{snap}; }, [&now] { return now; });
    session.setTimerEnabled(false);
    session.setFeatureEnabled(true);

    expect(session.readiness().allReady() && session.eligibility() == ArmReason::Ready,
           "a provisioned session is eligible");
    expect(!session.stopAwaiting(), "nothing awaits a STOP initially");

    // The checklist can be complete while arm() would still refuse: a clock that
    // went backwards is the reachable example (InvalidTime).
    session.timerTick();
    now = 50;
    expect(session.readiness().allReady(),
           "the readiness checklist does not look at the clock");
    expect(session.eligibility() == ArmReason::InvalidTime,
           "eligibility reports InvalidTime although the checklist is complete");
    expect(session.arm() == ArmReason::InvalidTime && !session.armed(),
           "arm() agrees with eligibility()");
    now = 200;
    expect(session.eligibility() == ArmReason::Ready, "eligibility recovers with the clock");

    // eligibility() is a pure query.
    const bool armedBefore = session.armed();
    (void)session.eligibility();
    expect(session.armed() == armedBefore && !session.stopAwaiting(),
           "eligibility() changes no state");

    // Armed, then STALE: disarmed with the automatic STOP unconfirmed.
    expect(session.arm() == ArmReason::Ready && session.armed(), "arm succeeds when eligible");
    snap.state = VisualState::Stale;
    now = 300;
    session.timerTick();
    expect(!session.armed() && session.stopAwaiting(),
           "STALE leaves the session disarmed and awaiting the STOP outcome");
    expect(!c.sends.empty() && c.sends.back().second == rb::MotionMode::Stop,
           "the awaited STOP was sent");
    c.ack(c.sends.back().first, rb::CommandTerminalResult::Ok);
    now = 400;
    session.timerTick();
    expect(!session.stopAwaiting(), "an accepted STOP ends the awaiting state");
}

// ---- Pitch axis / depth at the session level (Task 06 PR-C) ----------------------
rb::DepthControlSample sampleAt(double metres, bool fresh = true)
{
    rb::DepthControlSample sample;
    sample.rawDepthM = metres + 0.115;
    sample.calibratedDepthM = metres;
    sample.ageMs = fresh ? 20 : 5000;
    sample.fresh = fresh;
    return sample;
}

struct PitchSession {
    rb::test::VisualControllerFixture controller;
    rb::vision::VisualDiagnosticSnapshot snap;
    qint64 now{100};
    std::unique_ptr<rb::vision::VisualDispatchSession> session;
    explicit PitchSession(rb::vision::VisualAxisMode axis = rb::vision::VisualAxisMode::Pitch)
    {
        using namespace rb::vision;
        snap.state = VisualState::Tracking;
        snap.command.proposed = ProposedCommand::Forward;
        snap.command.ex_f = 0.0;
        controller.enabled = 0x1f; controller.known = 0x1f;
        controller.depthSample = sampleAt(0.2);
        session = std::make_unique<VisualDispatchSession>(
            &controller, [this] { return std::optional{snap}; }, [this] { return now; });
        session->setFeatureEnabled(true);
       
        session->setAxisMode(axis);
    }
    // Arm, send `command` and have it confirmed.
    void armAndConfirm(rb::vision::ProposedCommand command)
    {
        using namespace rb::vision;
        snap.command.proposed = command;
        expect(session->arm() == ArmReason::Ready, "pitch session arms");
        session->timerTick();
        expect(!controller.sends.empty(), "pitch session sent a command");
        controller.ack(controller.sends.back().first, rb::CommandTerminalResult::Ok);
    }
};

void testAxisSwitchStopsAndDisarms()
{
    using namespace rb::vision;
    {
        PitchSession s(VisualAxisMode::Yaw);
        expect(s.session->axisMode() == VisualAxisMode::Yaw, "default axis is Yaw");
        s.session->setAxisMode(VisualAxisMode::Pitch);
        expect(s.controller.sends.empty(), "switching axis while disarmed sends nothing");
        expect(s.session->axisMode() == VisualAxisMode::Pitch, "axis switched");
    }
    {
        PitchSession s(VisualAxisMode::Yaw);
        s.snap.command.proposed = ProposedCommand::Forward;
        s.controller.enabled = 0x1b; s.controller.known = 0x1b;
        expect(s.session->arm() == ArmReason::Ready && s.session->armed(), "yaw session arms");
        s.session->setAxisMode(VisualAxisMode::Both);
        expect(!s.session->armed() && s.controller.sends.size() == 1
                   && s.controller.sends.back().second == rb::MotionMode::Stop,
               "switching axis while armed sends exactly one operator STOP and disarms");
        s.session->setAxisMode(VisualAxisMode::Both);
        expect(s.controller.sends.size() == 1, "re-selecting the same axis does nothing");
        s.controller.enabled = 0x1f; s.controller.known = 0x1f;
        expect(s.session->arm() == ArmReason::Ready, "re-arming after the switch is explicit and works");
    }
    {   // Unconfirmed automatic STOP counts like an armed session.
        PitchSession s;
        s.armAndConfirm(ProposedCommand::Forward);
        s.snap.state = VisualState::Stale;
        s.session->timerTick();
        const auto before = s.controller.sends.size();
        expect(!s.session->armed() && s.controller.sends.back().second == rb::MotionMode::Stop,
               "STALE sent the automatic STOP");
        s.session->setAxisMode(VisualAxisMode::Both);
        expect(s.controller.sends.size() == before + 1, "axis switch while a STOP is unconfirmed sends the operator STOP");
    }
}

void testDepthSampleIsEvaluatedImmediately()
{
    using namespace rb::vision;
    {   // Hard limit between two timer ticks: STOP and disarm without waiting for the timer.
        PitchSession s;
        s.armAndConfirm(ProposedCommand::Descend);
        expect(s.controller.sends.back().second == rb::MotionMode::Descend, "DESCEND went out as Descend");
        const auto before = s.controller.sends.size();
        s.controller.publishDepth(sampleAt(0.51));
        expect(!s.session->armed() && s.controller.sends.size() == before + 1
                   && s.controller.sends.back().second == rb::MotionMode::Stop,
               "a hard-limit sample stops and disarms immediately (no timer tick)");
    }
    {   // Soft floor: STOP but stay armed.
        PitchSession s;
        s.armAndConfirm(ProposedCommand::Descend);
        const auto before = s.controller.sends.size();
        s.controller.publishDepth(sampleAt(0.30));
        expect(s.session->armed() && s.controller.sends.size() == before + 1
                   && s.controller.sends.back().second == rb::MotionMode::Stop,
               "a soft-floor sample stops the dive and keeps the session armed");
    }
    {   // Yaw never reacts to depth events.
        PitchSession s(VisualAxisMode::Yaw);
        s.controller.enabled = 0x1b; s.controller.known = 0x1b;
        s.snap.command.proposed = ProposedCommand::Forward;
        expect(s.session->arm() == ArmReason::Ready, "yaw arms");
        s.session->timerTick();
        const auto before = s.controller.sends.size();
        s.controller.publishDepth(sampleAt(0.9));
        expect(s.session->armed() && s.controller.sends.size() == before, "Yaw ignores depth events");
    }
    {   // Disarmed with nothing pending: no evaluation, no output.
        PitchSession s;
        s.controller.publishDepth(sampleAt(0.9));
        expect(s.controller.sends.empty() && !s.session->armed(), "an idle session does nothing on depth events");
    }
    {   // Staleness has no event: the timer notices it.
        PitchSession s;
        s.armAndConfirm(ProposedCommand::Forward);
        const auto before = s.controller.sends.size();
        s.controller.depthSample = sampleAt(0.2, false);
        s.now += 60;
        s.session->timerTick();
        expect(!s.session->armed() && s.controller.sends.size() == before + 1
                   && s.controller.sends.back().second == rb::MotionMode::Stop,
               "a stale depth sample is caught by the next timer tick");
    }
}

void testDepthEventsKeepTheirValueWhenDeferred()
{
    using namespace rb::vision;
    class Hooked : public rb::test::VisualControllerFixture {
    public:
        std::function<void()> duringSubmit;
        std::optional<quint32> submitVisualMotion(rb::MotionMode mode) override {
            const auto id = VisualControllerFixture::submitVisualMotion(mode);
            if (duringSubmit) { auto once = std::move(duringSubmit); duringSubmit = {}; once(); }
            return id;
        }
    };
    Hooked controller;
    VisualDiagnosticSnapshot snap;
    snap.state = VisualState::Tracking;
    snap.command.proposed = ProposedCommand::Forward;
    snap.command.ex_f = 0.0;
    qint64 now = 100;
    controller.enabled = 0x1f; controller.known = 0x1f;
    controller.depthSample = sampleAt(0.2);
    VisualDispatchSession session(&controller, [&] { return std::optional{snap}; }, [&] { return now; });
    session.setFeatureEnabled(true);
    session.setAxisMode(VisualAxisMode::Pitch);
    expect(session.arm() == ArmReason::Ready, "deferral fixture arms");
    session.timerTick();
    controller.ack(controller.sends.back().first, rb::CommandTerminalResult::Ok);
    // A new command is submitted; while that submission is on the stack the depth
    // reading crosses the hard limit and is back in range before it unwinds.
    snap.command.proposed = ProposedCommand::Descend;
    now += 1500;
    controller.duringSubmit = [&] {
        controller.publishDepth(sampleAt(0.51));
        controller.publishDepth(sampleAt(0.2));
    };
    session.timerTick();
    QApplication::processEvents();
    bool stopped = false;
    for (const auto &send : controller.sends) stopped = stopped || send.second == rb::MotionMode::Stop;
    expect(stopped && !session.armed(),
           "a transient hard-limit sample is evaluated with its own value even if a newer in-range sample follows");
}

void testSessionSurvivesControllerLifetime()
{
    using namespace rb::vision;
    class Emitting : public rb::test::VisualControllerFixture {
    public:
        ~Emitting() override { emit controlDepthSampleChanged(); }
    };
    // Session destroyed first: a later controller signal must not reach it.
    {
        rb::test::VisualControllerFixture controller;
        {
            VisualDispatchSession session(&controller, [] { return std::optional<VisualDiagnosticSnapshot>{}; });
            session.setFeatureEnabled(true);
        }
        controller.publishDepth(sampleAt(0.9));
        expect(true, "no crash when the controller signals after the session is gone");
    }
    // Controller destroyed first: it signals from its destructor while the session is armed.
    {
        auto controller = std::make_unique<Emitting>();
        VisualDiagnosticSnapshot snap;
        snap.state = VisualState::Tracking; snap.command.proposed = ProposedCommand::Forward; snap.command.ex_f = 0.0;
        controller->enabled = 0x1f; controller->known = 0x1f;
        controller->depthSample = sampleAt(0.2);
        qint64 now = 10;
        VisualDispatchSession session(controller.get(), [&] { return std::optional{snap}; }, [&] { return now; });
        session.setFeatureEnabled(true);
        session.setAxisMode(VisualAxisMode::Pitch);
        expect(session.arm() == ArmReason::Ready, "lifetime fixture arms");
        controller.reset();   // emits controlDepthSampleChanged() during destruction
        expect(true, "no crash when the controller signals while being destroyed");
    }
}

void testFrontAxisPoseMismatchUsesAxisMask()
{
    using namespace rb::vision;
    PitchSession s;
    s.controller.known = 0x1b;          // front axis pose unknown
    expect(s.session->arm() == ArmReason::PoseUnknown, "Pitch refuses to arm without the front axis pose");
    s.controller.known = 0x1f;
    expect(s.session->arm() == ArmReason::Ready, "and arms with it");
}


// Task 06 PR-C: Pitch/Both add the front axis and depth to the readiness checklist.
void testReadinessIncludesFrontAxisAndDepthInPitch()
{
    using namespace rb::vision;
    {   // Yaw: the two extra items are inert, whatever the depth or front axis say.
        PitchSession s(VisualAxisMode::Yaw);
        s.controller.enabled = 0x1b; s.controller.known = 0x1b; s.controller.depthSample.reset();
        const auto r = s.session->readiness();
        expect(!r.pitchActive && r.allReady() && r.totalCount() == 3 && r.missingCount() == 0,
               "Yaw readiness ignores the front axis and depth");
    }
    {   // Pitch, everything ready.
        PitchSession s;
        const auto r = s.session->readiness();
        expect(r.pitchActive && r.frontAxisReady && r.depthReady && r.allReady() && r.totalCount() == 5,
               "Pitch readiness is complete with front axis and a zeroed fresh depth");
    }
    {   // Front axis only.
        PitchSession s;
        s.controller.enabled = 0x1b;
        auto r = s.session->readiness();
        expect(!r.frontAxisReady && r.depthReady && r.missingCount() == 1, "disabled front axis falsifies only that item");
        s.controller.enabled = 0x1f; s.controller.known = 0x1b;
        r = s.session->readiness();
        expect(!r.frontAxisReady && r.missingCount() == 1, "unknown front axis pose falsifies only that item");
    }
    {   // Depth items.
        PitchSession s;
        s.controller.depthSample.reset();
        auto r = s.session->readiness();
        expect(!r.depthReady && r.frontAxisReady && r.missingCount() == 1
                   && r.depthNote && *r.depthNote == QStringLiteral("Depth unavailable"), "no depth sample");
        auto notZeroed = sampleAt(0.2); notZeroed.calibratedDepthM.reset();
        s.controller.depthSample = notZeroed;
        r = s.session->readiness();
        expect(!r.depthReady && r.depthNote && *r.depthNote == QStringLiteral("Depth not zeroed"),
               "fresh but not zeroed says Depth not zeroed");
        s.controller.depthSample = sampleAt(0.2, false);
        r = s.session->readiness();
        expect(!r.depthReady && r.depthNote && *r.depthNote == QStringLiteral("Depth unavailable"), "stale depth");
        s.controller.depthSample = sampleAt(0.55);
        r = s.session->readiness();
        expect(!r.depthReady && r.depthNote && *r.depthNote == QStringLiteral("Depth at hard limit"), "hard limit");
        s.controller.depthSample = sampleAt(0.30);
        expect(s.session->readiness().depthReady, "the soft floor and surface bands are ready (gated, not refused)");
        s.controller.depthSample = sampleAt(0.0);
        expect(s.session->readiness().depthReady, "surface is ready");
    }
}

int main(int argc,char **argv){QApplication app(argc,argv);testAutoFollowEligibilityAndStopAwaiting();testAxisSwitchStopsAndDisarms();testDepthSampleIsEvaluatedImmediately();testDepthEventsKeepTheirValueWhenDeferred();testSessionSurvivesControllerLifetime();testFrontAxisPoseMismatchUsesAxisMask();testReadinessIncludesFrontAxisAndDepthInPitch();testAutoFollowReadinessEachItemIndependently();testReentrantSafetySnapshotIsRetained();testAuthorityLossInSameEvent();testPoseMismatchRequiresCompleteMask();testAutomaticStopCoversOperatorStop();testDeterministicRuntimeCases();testTimerAndRealDiagnosticGrace();testWireFailureSafetyStop();testAcceptedStopRetiresRetryEpisode();testSessionRuntime();return failures?1:0;}
