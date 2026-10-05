#include "ui/MainWindow.h"
#include "ui/AutoFollowState.h"
#include <string>
#include "vision/VisualDispatchSession.h"
#include "vision/VideoView.h"
#include "vision/VisualCsvLogger.h"
#include "remote/RemoteRobotController.h"
#include "robobeetle/gateway/gateway_types.hpp"
#include "robobeetle/gateway/rbrp_codec.hpp"
#include <QTcpSocket>
#include <QTcpServer>
#include <QHostAddress>
#include <QTemporaryDir>
#include <QFile>
#include "helpers/VisualControllerFixture.h"
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QKeyEvent>
#include <QLabel>
#include <QProgressBar>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QSlider>
#include <QSet>
#include <QSpinBox>
#include <QThread>
#include <cstdio>

namespace {
using namespace rb;
using namespace rb::vision;
using namespace robobeetle::gateway;
int failures=0;
void check(bool ok,const char *why) { if(!ok){std::fprintf(stderr,"FAIL: %s\n",why);++failures;} }
void wait(int ms) { QElapsedTimer t;t.start();while(t.elapsed()<ms){QApplication::processEvents();QThread::msleep(1);} }
template<class T> T *child(MainWindow &w,const char *name){return w.findChild<T*>(QString::fromLatin1(name));}
void click(MainWindow &w,const char *name) {auto *b=child<QPushButton>(w,name);check(b!=nullptr,name);if(b)b->click();}
void frame(MainWindow &w,quint64 id=1,double u=320) {
    auto *d=w.findChild<VisualDiagnosticSession*>();check(d!=nullptr,"diagnostic child exists");if(!d)return;
    DetectionFrame f{id,1000000000+id,{640,480},{{1,"fish",.9,{u,240}}}};
    d->onDetectionArrival(f,{DetectionDisplayState::Target,f});
}
VisualDispatchSession *arm(MainWindow &w) {
    frame(w);auto *s=w.findChild<VisualDispatchSession*>();check(s!=nullptr,"dispatch child exists");if(!s)return nullptr;
    auto *gate=child<QCheckBox>(w,"visualDispatchEnabled");
    check(gate!=nullptr,"session gate exists");if(!gate)return nullptr;
    gate->setChecked(true);click(w,"visualArmButton");
    check(s->armed()==(w.findChild<QCheckBox*>("visualDispatchEnabled")->isChecked() && s->armReason()==ArmReason::Ready),"arm follows eligibility");return s;
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
        check(server_.listen(QHostAddress::LocalHost, 0),
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
            check(status == RbrpFeedStatus::Ok,
                   "fake gateway must decode controller traffic");
            queued_.insert(queued_.end(), decoded.begin(), decoded.end());
            return false;
        }, timeoutMs);
        return found ? result : std::nullopt;
    }

    void send(const GatewayMessage &message)
    {
        const auto encoded = encode_gateway_message(message);
        check(encoded.status == RbrpEncodeStatus::Ok,
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

void controls() {
    test::VisualControllerFixture c;MainWindow w(&c);frame(w);
    auto *gate=child<QCheckBox>(w,"visualDispatchEnabled");
    check(gate&&!gate->isChecked(),"feature defaults OFF");
    check(!child<QComboBox>(w,"visualTurnSign")&&!child<QPushButton>(w,"visualTurnSignConfirm"),"turn-direction selection workflow is removed");
    auto *s=w.findChild<VisualDispatchSession*>();if(!gate||!s)return;
    check(s->confirmedTurnSign()==VisualPolicyConfig::configuredTurnSign,"session direction comes from the config constant");
    gate->setChecked(true);check(!s->armed(),"gate ON does not arm");click(w,"visualArmButton");
    check(s->armed(),"configured direction plus readiness enables explicit Arm");
    auto *status=child<QLabel>(w,"visualDispatchStatus");
    s->timerTick();check(!c.sends.empty(),"real snapshot drives dispatch");
    if(!c.sends.empty())c.ack(c.sends.back().first,CommandTerminalResult::Ok);
    check(status->text().contains("ACK-confirmed: FORWARD"),"UI displays ACK confirmed actual mode");
    auto *v=w.findChild<VideoView*>();check(v&&!v->visualDiagnosticText().contains("no motion output")&&!v->visualDiagnosticText().contains("not sent"),"enabled video text is truthful");
    check(v&&v->visualDiagnosticText().contains("turn sign: 1 confirmed")
          &&!v->visualDiagnosticText().contains("turn_sign="),"active sign shows the configured +1 without conflicting preview default");
    check(!child<QCheckBox>(w,"visualCsvEnabled")->text().contains("no motion output"),"enabled CSV label truthful");
    const auto n=c.sends.size();click(w,"visualArmButton");check(!s->armed()&&c.sends.size()==n+1,"toggle Disarm sends one operator STOP");
    check(status->text().contains("operator STOP: SENT"),"operator STOP submission shown before terminal ACK");
    c.ack(c.sends.back().first,CommandTerminalResult::Busy,7);
    check(status->text().contains("operator STOP: BUSY"),"operator STOP terminal result shown");
    gate->setChecked(false);check(c.sends.size()==n+1,"OFF after completed disarm emits no extra STOP");
    check(v->visualDiagnosticText().contains(QStringLiteral("turn_sign=1 (\u5b9e\u673a\u7b26\u53f7\u672a\u9a8c\u8bc1)")),"OFF preserves exact legacy diagnostic sign line");
    MainWindow second(&c);check(!child<QCheckBox>(second,"visualDispatchEnabled")->isChecked(),"second instance never persists gate");
}
void manualPaths() {
    QStringList names{"motionModeButton1","motionModeButton3","motionModeButton4","motionModeButton5","motionModeButton6","motionStopButton","enableAllButton","disableAllButton","gaitBackendCombo","frontRearCoordinationCombo"};
    for(int i=0;i<5;++i)for(const auto *base:{"servoEnableButton","servoNeutralButton","servoApplyButton","servoAngleApplyButton","servoPwmSpin","servoPwmSlider","servoAngleSpin"})names<<QString::fromLatin1(base)+QString::number(i);
    for(bool dwell : {false, true}) for(const auto &name:names) {
        test::VisualControllerFixture c;MainWindow w(&c);auto *s=arm(w);if(!s)continue;
        s->timerTick(); // Pending automatic ACK; takeover must cancel it and retries.
        if (dwell && !c.sends.empty()) {
            c.ack(c.sends.back().first,CommandTerminalResult::Ok);
            frame(w,2,620); // changed proposal is blocked by dwell/EMA
            const auto count=c.sends.size();s->timerTick();
            check(c.sends.size()==count,"confirmed forward holds a changed proposal during dwell");
        }
        c.beforeManual=[&]{check(!s->armed(),"takeover precedes controller call");};
        auto *control=w.findChild<QWidget*>(name);check(control!=nullptr,qPrintable(name));if(!control)continue;
        control->setEnabled(true); // Handler coverage even when evidence makes a button temporarily unavailable.
        const auto before=c.sends.size();
        if(auto *b=qobject_cast<QPushButton*>(control))b->click();
        else if(auto *b=qobject_cast<QComboBox*>(control))b->setCurrentIndex(1);
        else if(auto *b=qobject_cast<QSpinBox*>(control))b->setValue(b->value()+1);
        else if(auto *b=qobject_cast<QSlider*>(control))b->setValue(b->value()+1);
        else if(auto *b=qobject_cast<QDoubleSpinBox*>(control))b->setValue(b->value()+1);
        check(!s->armed(),qPrintable("manual takeover: "+name));
        if(before)c.ack(c.sends.front().first,CommandTerminalResult::Ok);
        s->timerTick();check(c.sends.size()==before,"manual handler emits no automatic STOP or late ACK revival");
        child<QCheckBox>(w,"visualDispatchEnabled")->setChecked(false);
        check(c.sends.size()==before,"OFF after manual takeover does not stop manual motion");
    }
}
void key(QWidget *widget,int code) {
    QKeyEvent down(QEvent::KeyPress,code,Qt::NoModifier);
    QKeyEvent up(QEvent::KeyRelease,code,Qt::NoModifier);
    QApplication::sendEvent(widget,&down);QApplication::sendEvent(widget,&up);
}
void keyboardPaths() {
    // No explicit QShortcut/QAction or custom movement key handlers exist.
    // Ordinary QPushButton Space/Enter and editor arrow keys share origin handlers.
    QStringList names{"motionModeButton1","motionModeButton3","motionModeButton4","motionModeButton5","motionModeButton6","motionStopButton","enableAllButton","disableAllButton"};
    for(int i=0;i<5;++i) for(const auto *base:{"servoEnableButton","servoNeutralButton","servoApplyButton","servoAngleApplyButton"})names<<QString::fromLatin1(base)+QString::number(i);
    for(const auto &name:names) {
        test::VisualControllerFixture c;MainWindow w(&c);auto *s=arm(w);if(!s)continue;
        auto *b=w.findChild<QPushButton*>(name);if(!b)continue;
        b->setEnabled(true);c.beforeManual=[&]{check(!s->armed(),"keyboard takeover precedes manual submission");};
        key(b,Qt::Key_Space);check(!s->armed(),qPrintable("Space takeover: "+name));
        const auto n=c.sends.size();s->timerTick();check(c.sends.size()==n,"keyboard handler cancels automatic dispatch");
    }
    for(int i=0;i<5;++i)for(const auto *base:{"servoPwmSpin","servoPwmSlider","servoAngleSpin"}) {
        test::VisualControllerFixture c;MainWindow w(&c);auto *s=arm(w);if(!s)continue;
        auto *b=w.findChild<QWidget*>(QString::fromLatin1(base)+QString::number(i));
        check(b!=nullptr,"keyboard editor exists");if(!b)continue;b->setEnabled(true);
        key(b,Qt::Key_Up);check(!s->armed(),"arrow editing triggers manual takeover");
    }
    for(const auto *name:{"gaitBackendCombo","frontRearCoordinationCombo"}) {
        test::VisualControllerFixture c;MainWindow w(&c);auto *s=arm(w);if(!s)continue;
        auto *b=child<QComboBox>(w,name);b->setEnabled(true);key(b,Qt::Key_Down);
        check(!s->armed(),"combo arrow selection triggers takeover");
    }
    // QPushButton Enter activation is provided for focused auto-default buttons
    // in dialogs, not these QMainWindow controls. Record their actual behavior.
    test::VisualControllerFixture c;MainWindow w(&c);auto *s=arm(w);if(!s)return;
    auto *b=child<QPushButton>(w,"motionModeButton1");key(b,Qt::Key_Return);key(b,Qt::Key_Enter);
    check(s->armed()&&c.manualRecords.empty(),"Return on ordinary main-window button is not a motion accelerator");
}
void rawRejection() {
    test::VisualControllerFixture c;MainWindow w(&c);auto *s=arm(w);if(!s)return;
    s->timerTick();c.ack(c.sends.back().first,CommandTerminalResult::Rejected,6);
    auto *status=child<QLabel>(w,"visualDispatchStatus");
    check(status->text().contains("raw result: 6"),"START raw HARDWARE_FAILURE result remains visible");
    check(status->text().contains(QString::fromUtf8("\xe5\x8f\xaf\xe8\x83\xbd\xe6\x98\xaf\xe5\xa7\xbf\xe6\x80\x81\xe6\x9c\xaa\xe7\x9f\xa5")),"possible pose mismatch explanation shown");
    c.ack(c.sends.back().first,CommandTerminalResult::Ok);
    check(status->text().contains("raw result: 6"),"safety STOP does not overwrite START rejection");
}
void signAndOffMatrix() {
    {
        test::VisualControllerFixture c;MainWindow w(&c);auto *s=arm(w);if(!s)return;
        s->timerTick();
        check(s->confirmedTurnSign()==VisualPolicyConfig::configuredTurnSign,"configured direction survives arming");
        check(!child<QComboBox>(w,"visualTurnSign")&&!child<QPushButton>(w,"visualTurnSignConfirm"),"operator confirmation workflow is absent");
        const auto n=c.sends.size();
        click(w,"visualArmButton");check(!s->armed()&&c.sends.size()==n+1&&c.sends.back().second==rb::MotionMode::Stop,"combined toggle disarms and sends one operator STOP");
        click(w,"visualArmButton");check(s->armed(),"the same combined action re-arms without any sign reconfirmation");
    }
    for(auto result:{CommandTerminalResult::Ok,CommandTerminalResult::Busy,CommandTerminalResult::Rejected,CommandTerminalResult::OutcomeUnknown}) {
        test::VisualControllerFixture c;MainWindow w(&c);auto *s=arm(w);if(!s)continue;
        s->timerTick();const auto old=c.sends.back().first;
        child<QCheckBox>(w,"visualDispatchEnabled")->setChecked(false);
        check(c.sends.size()==2&&!s->armed(),"UI OFF pending START sends exactly one operator STOP");
        c.ack(old,CommandTerminalResult::Ok);check(!s->currentMode(),"late START cannot revive OFF UI mode");
        c.ack(c.sends.back().first,result);
        QString expected;
        switch(result) {
        case CommandTerminalResult::Ok:expected="OK";break;
        case CommandTerminalResult::Busy:expected="BUSY";break;
        case CommandTerminalResult::Rejected:expected="REJECTED";break;
        case CommandTerminalResult::OutcomeUnknown:expected="OUTCOME_UNKNOWN";break;
        }
        check(child<QLabel>(w,"visualDispatchStatus")->text().contains("operator STOP: "+expected),"every operator STOP result remains visible after OFF");
    }
    test::VisualControllerFixture c;MainWindow w(&c);
    child<QCheckBox>(w,"visualDispatchEnabled")->setChecked(true);
    child<QCheckBox>(w,"visualDispatchEnabled")->setChecked(false);
    check(c.sends.empty(),"UI ON never armed then OFF emits zero STOP");
}
void enableReleaseBranchesAndAlert() {
    for(int i=0;i<5;++i)for(bool enabled:{false,true}) {
        test::VisualControllerFixture c;MainWindow w(&c);auto *s=arm(w);if(!s)continue;
        if(enabled)c.enabled|=1<<i;else c.enabled&=~(1<<i);
        emit c.servoStateChanged(i,enabled);
        c.beforeManual=[&]{check(!s->armed(),"enable/release clears automation before controller call");};
        auto *b=w.findChild<QPushButton*>(QStringLiteral("servoEnableButton%1").arg(i));
        check(b->isEnabled(),"enable/release real fixture control is actionable");b->click();
        check(!s->armed()&&!c.manualRecords.empty()&&c.manualRecords.back()==QStringLiteral("%1:%2").arg(enabled?"disable":"enable").arg(i),"each servo enable and release branch tested");
    }
    test::VisualControllerFixture c;MainWindow w(&c);auto *s=arm(w);if(!s)return;
    QTemporaryDir dir;auto *logger=w.findChild<VisualCsvLogger*>();check(logger->start(dir.path()),"retry CSV starts");
    s->timerTick();c.ack(c.sends.back().first,CommandTerminalResult::Ok);
    auto *d=w.findChild<VisualDiagnosticSession*>();d->refresh({DetectionDisplayState::Stale,{}});
    s->timerTick();check(!s->armed()&&c.sends.back().second==rb::MotionMode::Stop,"STALE starts STOP episode");
    wait(3150);
    check(s->stopTimeoutAlert()&&child<QLabel>(w,"visualDispatchStatus")->text().contains("STOP TIMEOUT ALERT"),"three STOP timeouts latch visible alert without frames");
    c.ack(c.sends.back().first,CommandTerminalResult::Ok);
    check(child<QLabel>(w,"visualDispatchStatus")->text().contains("STOP TIMEOUT ALERT"),"late STOP confirmation preserves alert");
    logger->stop();QFile file(logger->filePath());check(file.open(QIODevice::ReadOnly),"retry CSV readable");
    const auto lines=file.readAll().split('\n');const auto cols=lines.front().split(',');
    QSet<QByteArray> policyIds,wireIds;int retries=0;
    for(const auto &line:lines) {
        const auto row=line.split(',');
        if(row.value(cols.indexOf("dispatch_command"))=="STOP"&&row.value(cols.indexOf("dispatch_result"))=="SENT") {
            ++retries;policyIds.insert(row.value(cols.indexOf("policy_request_id")));wireIds.insert(row.value(cols.indexOf("request_id")));
        }
    }
    check(retries>=4&&policyIds.size()==retries&&wireIds.size()==retries,"each real timer STOP retry records a separate policy/wire ID send row");
}
void loopbackWindowAndCsv() {
    FakeGatewayPeer gateway;RemoteRobotController c;MainWindow w(&c);
    ConsoleConnectionConfiguration cfg;cfg.endpoint=QStringLiteral("127.0.0.1");cfg.tcpPort=gateway.port();
    c.connectController(cfg);check(gateway.accept(),"MainWindow real remote loopback connects");
    auto hello=gateway.nextFrame(RbrpMessageKind::Hello);if(!hello)return;gateway.send(helloReply(hello->request_id));
    check(pumpUntil([&]{return c.canAcquireControl();}),"remote handshake ready");c.acquireControl();
    auto acquire=gateway.nextFrame(RbrpMessageKind::AcquireControl);if(!acquire)return;
    gateway.send(acquireReply(acquire->request_id));gateway.send(activeState());
    check(pumpUntil([&]{return c.isControlActive();}),"real controller authority established");
    quint16 seq=10;
    for(int i:{0,1,3,4}) {
        c.enableServo(static_cast<ServoId>(i));auto f=gateway.nextFrame(RbrpMessageKind::CommandRequest);if(!f)return;
        sendSubmitted(gateway,f->request_id,seq);sendOutcome(gateway,f->request_id,RobotCommandKind::EnableServos,seq++,GatewayCommandOutcome::Accepted);
        check(pumpUntil([&]{return c.isServoEnabled(static_cast<ServoId>(i));}),"real ACK enables required paddle");
    }
    check((c.inferredPoseKnownMask()&0x1b)==0x1b,"real ACK evidence establishes required poses");
    // Fresh diagnostics alone, with default OFF, never generate robot commands.
    frame(w);wait(80);check(!gateway.nextFrame(RbrpMessageKind::CommandRequest,20),"default OFF loopback has zero automatic writes");
    QTemporaryDir dir;auto *logger=w.findChild<VisualCsvLogger*>();check(logger->start(dir.path()),"loopback CSV recording starts");
    auto *s=arm(w);if(!s||!s->armed())return;
    auto start=gateway.nextFrame(RbrpMessageKind::CommandRequest);check(start&&start->payload==Bytes{static_cast<Byte>(RobotCommandKind::StartMotion),1},"MainWindow dispatch sends START via sole controller socket");if(!start)return;
    sendSubmitted(gateway,start->request_id,seq);sendOutcome(gateway,start->request_id,RobotCommandKind::StartMotion,seq++,GatewayCommandOutcome::Accepted);
    check(pumpUntil([&]{return s->currentMode()==ProposedCommand::Forward;}),"wait for correlated real START ACK");
    check(child<QLabel>(w,"visualDispatchStatus")->text().contains("ACK-confirmed: FORWARD"),"real ACK appears in MainWindow");
    auto stop=gateway.nextFrame(RbrpMessageKind::CommandRequest,1000);
    check(stop&&stop->payload==Bytes{static_cast<Byte>(RobotCommandKind::StopMotion)},"no frames: upstream STALE triggers timer STOP");if(!stop)return;
    check(!s->armed()&&w.findChild<VisualDiagnosticSession*>()->snapshot().state==VisualState::Stale,"MainWindow preserves upstream snapshot state");
    sendSubmitted(gateway,stop->request_id,seq);sendOutcome(gateway,stop->request_id,RobotCommandKind::StopMotion,seq++,GatewayCommandOutcome::Accepted);
    check(pumpUntil([&]{return c.motionState()==MotionState::Stopped && !c.isMotionActive();}),"real STOP settles before requalification");
    frame(w,2);click(w,"visualArmButton");check(s->armed(),"fresh source and explicit rearm qualify again");
    auto resumed=gateway.nextFrame(RbrpMessageKind::CommandRequest);if(!resumed)return;
    sendSubmitted(gateway,resumed->request_id,seq);sendOutcome(gateway,resumed->request_id,RobotCommandKind::StartMotion,seq++,GatewayCommandOutcome::Accepted);
    check(pumpUntil([&]{return s->currentMode()==ProposedCommand::Forward;}),"resumed real START ACK");
    auto *manualStop=child<QPushButton>(w,"motionStopButton");check(manualStop->isEnabled(),"ordinary manual STOP is actionable");manualStop->click();
    check(!s->armed(),"real manual input takes over before controller command");
    auto manual=gateway.nextFrame(RbrpMessageKind::CommandRequest);check(manual&&manual->payload==Bytes{static_cast<Byte>(RobotCommandKind::StopMotion)},"manual STOP reaches existing controller");if(!manual)return;
    sendSubmitted(gateway,manual->request_id,seq);sendOutcome(gateway,manual->request_id,RobotCommandKind::StopMotion,seq++,GatewayCommandOutcome::Accepted);
    check(pumpUntil([&]{return c.motionState()==MotionState::Stopped && !c.isMotionActive();}),"manual STOP ACK settles");
    child<QCheckBox>(w,"visualDispatchEnabled")->setChecked(false);
    check(!gateway.nextFrame(RbrpMessageKind::CommandRequest,1200),"manual takeover cancels automation and OFF emits no STOP");
    logger->stop();QFile file(logger->filePath());check(file.open(QIODevice::ReadOnly),"loopback CSV readable");
    const auto lines=file.readAll().split('\n');const auto columns=lines.front().split(',');
    const auto named=[&](const QList<QByteArray>&row,const char *name){return row.value(columns.indexOf(name));};
    bool send=false,outcome=false;
    for(const auto &line:lines){const auto row=line.split(',');
        if(named(row,"request_id")==QByteArray::number(start->request_id)) {
            check(named(row,"dispatch_command")=="FORWARD"&&!named(row,"policy_request_id").isEmpty(),"loopback CSV ties policy and RBRP IDs");
            if(named(row,"dispatch_result")=="SENT")send=true;
            if(named(row,"dispatch_result")=="OK"){outcome=true;check(!named(row,"ack_rtt_ms").isEmpty(),"loopback CSV has measured terminal RTT");}
        }
    }
    check(send&&outcome,"loopback CSV records actual send and matching ACK outcome");gateway.disconnectPeer();
}

void negativeAndTimer() {
    test::VisualControllerFixture c;MainWindow w(&c);auto *s=arm(w);if(!s)return;
    auto *back=child<QPushButton>(w,"motionModeButton2");auto *emergency=child<QPushButton>(w,"emergencyStopButton");
    check(back&&!back->isEnabled()&&emergency&&!emergency->isEnabled(),"Backward and emergency remain disabled");
    if(back)back->click();if(emergency)emergency->click();check(s->armed(),"disabled controls do not take over");
    // Programmatic controller refresh must not be confused with user editor input.
    emit c.servoStateChanged(0,true);check(s->armed(),"programmatic refresh does not disarm");
    wait(80);check(!c.sends.empty(),"50ms dispatch timer runs without UI frame arrivals");
    if(!c.sends.empty())c.ack(c.sends.back().first,CommandTerminalResult::Ok);
    wait(520);check(!s->armed()&&c.sends.back().second==rb::MotionMode::Stop,"upstream STALE causes timer STOP and disarm");
    test::VisualControllerFixture direct;direct.backend=ConsoleBackendKind::DirectSerial;MainWindow dw(&direct);arm(dw);
    check(direct.sends.empty()&&!dw.findChild<VisualDispatchSession*>()->armed(),"direct maintenance cannot arm or send");
    check(dw.findChild<VideoView*>()->visualDiagnosticText().contains("DRY_RUN"),"direct maintenance video always DRY_RUN");
}
void transientSafetyEvents()
{
    for (const auto state : {VisualState::Stale, VisualState::InferenceOff, VisualState::Lost}) {
        test::VisualControllerFixture controller;
        MainWindow window(&controller);
        auto *session = arm(window);
        auto *diagnostic = window.findChild<VisualDiagnosticSession*>();
        if (!session || !session->armed() || !diagnostic) return;
        session->timerTick();
        check(controller.sends.size() == 1, "transient fixture sends initial Forward");
        if (controller.sends.empty()) return;
        controller.ack(controller.sends.back().first, CommandTerminalResult::Ok);
        check(session->currentMode() == ProposedCommand::Forward, "transient fixture confirms Forward");
        auto snapshot = diagnostic->snapshot();
        snapshot.state = state;
        snapshot.command.proposed = ProposedCommand::Stop;
        emit diagnostic->diagnosticChanged(snapshot);
        // No processEvents/timer tick: the hazardous snapshot must be consumed now.
        check(controller.sends.size() == 2 && controller.sends.back().second == rb::MotionMode::Stop,
              "safety diagnostic immediately submits STOP between timer ticks");
        check(session->armed() == (state == VisualState::Lost),
              "STALE/INFERENCE_OFF disarm immediately; LOST retains arming");
        if (controller.sends.size() == 2)
            controller.ack(controller.sends.back().first, CommandTerminalResult::Ok);
        QThread::msleep(10); // No event pumping; no polling opportunity inside the transient.
        snapshot.state = VisualState::Tracking;
        snapshot.command.proposed = ProposedCommand::Forward;
        emit diagnostic->diagnosticChanged(snapshot);
        session->timerTick();
        check(session->armed() == (state == VisualState::Lost), "10ms recovery cannot undo safety disarm");
        if (state != VisualState::Lost)
            check(controller.sends.size() == 2, "recovery does not automatically resend Forward");
    }
}

void replayDesktopTransientStale()
{
    // Exact diagnostic inputs extracted from the operator CSV; times relative to first row.
    const auto fixture = QString::fromUtf8(__FILE__).replace("main_window_visual_dispatch_tests.cpp",
        "fixtures/visual-dispatch-transient-stale.csv");
    QFile file(fixture);
    check(file.open(QIODevice::ReadOnly), "desktop CSV replay fixture opens");
    if (!file.isOpen()) return;
    test::VisualControllerFixture controller;
    MainWindow window(&controller);
    auto *session = arm(window);
    auto *diagnostic = window.findChild<VisualDiagnosticSession*>();
    if (!session || !session->armed() || !diagnostic) return;
    session->timerTick();
    if (controller.sends.empty()) { check(false, "replay initial Forward exists"); return; }
    controller.ack(controller.sends.back().first, CommandTerminalResult::Ok);
    QElapsedTimer clock; clock.start();
    qint64 nextTick = 50;
    file.readLine(); // header
    bool sawStale = false, sawRecovery = false;
    while (!file.atEnd()) {
        const auto fields = file.readLine().trimmed().split(',');
        if (fields.size() != 6) { check(false, "replay diagnostic row has six fields"); continue; }
        const auto relativeMs = fields[0].toLongLong();
        const auto elapsed = relativeMs - 111400;
        while (nextTick <= elapsed) {
            if (nextTick > clock.elapsed()) QThread::msleep(nextTick - clock.elapsed());
            session->timerTick();
            if (!controller.sends.empty()) controller.ack(controller.sends.back().first, CommandTerminalResult::Ok);
            nextTick += 50;
        }
        if (elapsed > clock.elapsed()) QThread::msleep(elapsed - clock.elapsed());
        auto snapshot = diagnostic->snapshot();
        snapshot.localMonoMs = relativeMs + 158472;
        snapshot.state = fields[2] == "STALE" ? VisualState::Stale
            : fields[2] == "NO_TARGET" ? VisualState::NoTarget : VisualState::Tracking;
        snapshot.command.proposed = fields[3] == "STOP" ? ProposedCommand::Stop
            : fields[3] == "HOLD" ? ProposedCommand::Hold : ProposedCommand::Forward;
        snapshot.command.ex_f = fields[4].isEmpty() ? std::nullopt : std::optional{fields[4].toDouble()};
        snapshot.awaitingVideo = fields[5] == "1";
        emit diagnostic->diagnosticChanged(snapshot);
        if (snapshot.state == VisualState::Stale) {
            sawStale = true;
            check(relativeMs == 111988 && !session->armed(), "CSV 111.988s STALE disarms synchronously");
            check(controller.sends.size() == 2 && controller.sends.back().second == rb::MotionMode::Stop,
                  "CSV 111.988s STALE sends immediate STOP");
            if (controller.sends.size() == 2) controller.ack(controller.sends.back().first, CommandTerminalResult::Ok);
        }
        if (sawStale && snapshot.state == VisualState::Tracking) sawRecovery = true;
    }
    session->timerTick();
    check(sawStale && sawRecovery, "actual CSV replay includes STALE and subsequent TRACKING");
    check(!session->armed() && controller.sends.size() == 2,
          "CSV 111.4-113.5s replay stops/disarms and never auto-resumes Forward");
}

// Task 06: the green region's state pill, checklist and combined Arm/Disarm.
void autoFollowPanelContract()
{
    test::VisualControllerFixture c;MainWindow w(&c);
    // The green region condenses against its real height, so the window must be
    // realized at the minimum supported size before visibility is meaningful.
    w.resize(1100,720);w.show();QApplication::processEvents();
    auto *session=w.findChild<VisualDispatchSession*>();auto *gate=child<QCheckBox>(w,"visualDispatchEnabled");
    auto *pill=child<QLabel>(w,"autoFollowStatePill");auto *action=child<QPushButton>(w,"visualArmButton");
    auto *checklist=child<QLabel>(w,"autoFollowChecklist");auto *alert=child<QLabel>(w,"autoFollowAlert");
    check(session&&gate&&pill&&action&&checklist&&alert,"Task 06 green region widgets exist");
    if(!session||!gate||!pill||!action||!checklist||!alert) return;
    // Drive evaluation explicitly: the live 50 ms timer would otherwise
    // overwrite the injected fixture state between assertions.
    session->setTimerEnabled(false);

    // Structure: two independent sub-regions inside the Vision Status panel.
    auto *summary=child<QWidget>(w,"visionSummaryPanel");
    auto *autoGroup=child<QWidget>(w,"autoFollowCard");auto *controlsGroup=child<QWidget>(w,"visionControlsGroup");
    check(summary&&autoGroup&&controlsGroup&&summary->isAncestorOf(autoGroup)
          &&summary->isAncestorOf(controlsGroup)&&autoGroup!=controlsGroup,
          "Vision Controls and Auto Follow are two independent sub-regions of the panel");
    check(autoGroup->isAncestorOf(gate)&&autoGroup->isAncestorOf(action)&&autoGroup->isAncestorOf(pill),
          "the dispatch switch, Arm action and state pill live in Auto Follow");
    for(const auto *name:{"visionConnectButton","startInferenceButton","snapshotButton","startRecordingButton"})
        check(controlsGroup->isAncestorOf(child<QPushButton>(w,name)),"vision actions live in Vision Controls");
    check(!child<QPushButton>(w,"visualDisarmButton"),"the separate Disarm button is gone");
    check(child<QWidget>(w,"autoFollowAxisSelector")&&child<QWidget>(w,"autoFollowAxisSelector")->isVisible()
          &&autoGroup->isAncestorOf(child<QWidget>(w,"autoFollowAxisSelector")),
          "the axis selector is shown inside Auto Follow");
    check(!child<QWidget>(w,"autoFollowDepthRow"),"the depth row is not part of the green region");
    {   // Depth Sensor card: bar and Zero action are always shown; wiring comes later.
        auto *card=child<QWidget>(w,"depthCard");auto *bar=child<QProgressBar>(w,"autoFollowDepthBar");
        auto *zero=child<QPushButton>(w,"autoFollowZeroDepthButton");
        check(card&&bar&&zero&&card->isAncestorOf(bar)&&card->isAncestorOf(zero)&&!autoGroup->isAncestorOf(zero),
              "the depth bar and Zero button live in the Depth Sensor card, not in Auto Follow");
        check(bar&&bar->isVisible()&&bar->minimum()==0&&bar->maximum()==500&&bar->value()==0&&bar->format()=="Not zeroed",
              "the bar covers 0-0.50 m and shows Not zeroed");
        check(zero&&zero->isVisible()&&!zero->isEnabled(),"Zero is shown but disabled in this preview");
    }

    // 1. DRY RUN: feature off, ready underneath, nothing armed.
    check(pill->text()=="DRY RUN"&&!action->isEnabled()&&action->text().contains("not ready"),
          "feature off shows DRY RUN and refuses Arm while the link is down");

    // 2. NOT READY: all three conditions independently visible.
    gate->setChecked(true);
    c.active=false;c.enabled=0;c.known=0;
    session->timerTick();
    check(pill->text()=="NOT READY","feature on without prerequisites is NOT READY");
    // One line: the first missing item plus a count; the tooltip lists everything.
    check(!checklist->isHidden()&&checklist->text().contains("Link + control")&&checklist->text().contains("+2 more"),
          "NOT READY names the first missing condition and counts the rest");
    check(checklist->toolTip().contains("Link + control")&&checklist->toolTip().contains("Servos enabled")
          &&checklist->toolTip().contains("Tracking"),"the checklist tooltip lists every readiness condition");
    check(checklist->text().contains(QStringLiteral("\u2717"))&&checklist->toolTip().contains(QStringLiteral("\u2717")),
          "unsatisfied conditions are marked as unmet");
    check(!action->isEnabled()&&action->text().contains("3"),"Arm is disabled and counts three unmet conditions");
    check(alert->isHidden()&&alert->text().isEmpty(),"no alert banner without a fault");

    // 3. READY: requires an explicit operator Arm.
    c.active=true;c.enabled=0x1b;c.known=0x1b;
    frame(w);
    session->timerTick();
    check(pill->text()=="READY"&&action->isEnabled()&&action->text()=="Arm","all conditions met shows READY and enables Arm");
    check(!session->armed(),"READY never arms by itself");

    // 4. ARMED: the combined action becomes Disarm with the danger role.
    action->click();
    check(pill->text()=="ARMED"&&action->text()=="Disarm","the combined action becomes Disarm once armed");
    check(action->property("consoleActionRole").toString()=="danger","armed action uses the danger role");
    check(action->isEnabled(),"armed Disarm is always available");
    check(checklist->isHidden(),"the checklist is hidden while armed");

    // The same action disarms and sends exactly one operator STOP.
    const auto n=c.sends.size();action->click();
    check(!session->armed()&&c.sends.size()==n+1&&c.sends.back().second==rb::MotionMode::Stop,
          "the combined action disarms and sends one operator STOP");
    check(action->text().contains("not ready")||action->text()=="Arm","disarmed action returns to Arm");

    // 5. FAULT has the highest priority over ARMED and DRY RUN.
    {
        test::VisualControllerFixture fault;MainWindow fw(&fault);
        fw.resize(1100,720);fw.show();QApplication::processEvents();
        auto *fs=fw.findChild<VisualDispatchSession*>();auto *fgate=child<QCheckBox>(fw,"visualDispatchEnabled");
        auto *fpill=child<QLabel>(fw,"autoFollowStatePill");auto *fwarn=child<QLabel>(fw,"autoFollowAlert");
        if(fs) fs->setTimerEnabled(false);
        frame(fw);fgate->setChecked(true);click(fw,"visualArmButton");
        check(fs&&fs->armed(),"fault fixture arms before the safety event");
        if(fs){
            auto *diagnostic=fw.findChild<VisualDiagnosticSession*>();
            if(diagnostic){VisualViewContext stale;stale.gate=DetectionDisplayState::Stale;diagnostic->refresh(stale);}
            // Never ACK the automatic STOP: the retry deadlines latch the alarm.
            QElapsedTimer wait;wait.start();
            while(wait.elapsed()<6000&&!fs->stopTimeoutAlert()){QApplication::processEvents(QEventLoop::AllEvents,20);fs->timerTick();}
            check(fs->stopTimeoutAlert(),"safety STOP retries latch the timeout alarm");
            check(fpill->text()=="FAULT","a latched fault outranks every other state");
            check(!fwarn->isHidden()&&fwarn->text().contains("STOP"),"FAULT shows the alert banner with the cause");
        }
    }
    // 5b. STOPPING: disarmed but the automatic STOP is still unconfirmed.
    {
        test::VisualControllerFixture sc;MainWindow sw(&sc);
        sw.resize(1100,720);sw.show();QApplication::processEvents();
        auto *ss=sw.findChild<VisualDispatchSession*>();auto *sgate=child<QCheckBox>(sw,"visualDispatchEnabled");
        auto *spill=child<QLabel>(sw,"autoFollowStatePill");auto *sarm=child<QPushButton>(sw,"visualArmButton");
        if(ss) ss->setTimerEnabled(false);
        frame(sw);sgate->setChecked(true);
        ss->timerTick();
        check(spill->text()=="READY","stopping fixture starts READY");
        click(sw,"visualArmButton");
        check(ss->armed()&&spill->text()=="ARMED","stopping fixture arms");
        auto *diagnostic=sw.findChild<VisualDiagnosticSession*>();
        VisualViewContext stale;stale.gate=DetectionDisplayState::Stale;diagnostic->refresh(stale);
        check(!ss->armed()&&ss->stopAwaiting(),"STALE disarms and leaves the automatic STOP unconfirmed");
        check(spill->text()=="STOPPING","an unconfirmed automatic STOP shows STOPPING, not READY or DRY RUN");
        check(spill->styleSheet().contains("#F0A92E"),"STOPPING uses the amber fill");
        check(!sarm->isEnabled()&&sarm->text().contains("stopping"),"Arm is refused while the STOP is unconfirmed");
        check(!sc.sends.empty()&&sc.sends.back().second==rb::MotionMode::Stop,"the automatic STOP was sent");
        sc.ack(sc.sends.back().first,CommandTerminalResult::Ok);
        frame(sw);ss->timerTick();
        check(!ss->stopAwaiting()&&spill->text()!="STOPPING","an accepted STOP clears STOPPING");
    }
    // 5c. Pure priority table: FAULT > STOPPING > ARMED > DRY RUN > READY > NOT READY,
    // and READY additionally needs eligibility.
    {
        using rb::ui::AutoFollowState;using rb::ui::AutoFollowStateInput;using rb::ui::classifyAutoFollowState;
        AutoFollowStateInput in;in.enabled=true;in.readinessComplete=true;in.eligible=true;
        check(classifyAutoFollowState(in)==AutoFollowState::Ready,"complete and eligible is READY");
        in.eligible=false;
        check(classifyAutoFollowState(in)==AutoFollowState::NotReady,"a complete checklist without eligibility is NOT READY");
        in.eligible=true;in.readinessComplete=false;
        check(classifyAutoFollowState(in)==AutoFollowState::NotReady,"eligibility without a complete checklist is NOT READY");
        in.readinessComplete=true;in.enabled=false;
        check(classifyAutoFollowState(in)==AutoFollowState::DryRun,"switch off is DRY RUN");
        in.armed=true;
        check(classifyAutoFollowState(in)==AutoFollowState::Armed,"armed outranks DRY RUN");
        in.stopping=true;
        check(classifyAutoFollowState(in)==AutoFollowState::Stopping,"STOPPING outranks ARMED");
        in.stopping=true;in.armed=false;in.enabled=false;
        check(classifyAutoFollowState(in)==AutoFollowState::Stopping,"STOPPING outranks DRY RUN and READY");
        in.fault=true;
        check(classifyAutoFollowState(in)==AutoFollowState::Fault,"FAULT outranks STOPPING");
        check(std::string(rb::ui::autoFollowStateText(AutoFollowState::Stopping))=="STOPPING","state text");
    }
    // 6. Turning the feature off returns to DRY RUN and clears the checklist.
    gate->setChecked(false);
    check(pill->text()=="DRY RUN"&&checklist->isHidden(),"feature off returns to DRY RUN with no checklist");
}

void coveredOperatorStopUi()
{
    test::VisualControllerFixture controller;
    MainWindow window(&controller);
    auto *session = arm(window);
    if (!session || !session->armed()) return;
    session->disarm();
    controller.ack(controller.sends.back().first, CommandTerminalResult::Ok, 0xff);
    const auto text = child<QLabel>(window, "visualDispatchStatus")->text();
    check(text.contains("operator STOP: OK (covered)") && !text.contains("OUTCOME_UNKNOWN"),
          "UI distinguishes covered operator STOP from unconfirmed outcome");
}


// Task 06 PR-D: axis selector, Pitch/Both checklist, Depth Sensor bar and Zero, CSV depth provider.
DepthControlSample depthSampleAt(double metres,bool fresh=true,bool zeroed=true)
{
    DepthControlSample s;s.rawDepthM=metres+0.115;if(zeroed)s.calibratedDepthM=metres;s.ageMs=fresh?20:5000;s.fresh=fresh;return s;
}
bool logContains(MainWindow &w,const QString &text)
{
    for(auto *edit:w.findChildren<QPlainTextEdit*>()) if(edit->toPlainText().contains(text)) return true;
    return false;
}

void axisSelectorAndChecklist()
{
    test::VisualControllerFixture c;MainWindow w(&c);
    w.resize(1100,720);w.show();QApplication::processEvents();
    auto *session=w.findChild<VisualDispatchSession*>();auto *gate=child<QCheckBox>(w,"visualDispatchEnabled");
    auto *yaw=child<QPushButton>(w,"autoFollowAxisYaw");auto *pitch=child<QPushButton>(w,"autoFollowAxisPitch");
    auto *both=child<QPushButton>(w,"autoFollowAxisBoth");auto *checklist=child<QLabel>(w,"autoFollowChecklist");
    auto *pill=child<QLabel>(w,"autoFollowStatePill");auto *action=child<QPushButton>(w,"visualArmButton");
    check(session&&gate&&yaw&&pitch&&both&&checklist&&pill&&action,"axis widgets exist");
    if(!(session&&gate&&yaw&&pitch&&both&&checklist&&pill&&action))return;
    session->setTimerEnabled(false);
    check(yaw->isChecked()&&!pitch->isChecked()&&!both->isChecked()&&session->axisMode()==VisualAxisMode::Yaw,
          "Yaw is the default axis");
    gate->setChecked(true);c.enabled=0x1b;c.known=0x1b;frame(w);session->timerTick();
    check(pill->text()=="READY","Yaw is READY with the original servo mask and no depth");

    pitch->click();
    check(session->axisMode()==VisualAxisMode::Pitch&&pitch->isChecked()&&!yaw->isChecked(),"clicking Pitch selects the Pitch axis");
    frame(w,2);session->timerTick();
    check(pill->text()=="NOT READY","Pitch without the front axis or depth is NOT READY");
    check(!checklist->isHidden()&&checklist->toolTip().contains("Front axis servo")&&checklist->toolTip().contains("Depth unavailable"),
          "Pitch checklist adds the front axis and depth items");
    c.enabled=0x1f;c.known=0x1f;c.depthSample=depthSampleAt(0.1,true,false);
    frame(w,3);session->timerTick();
    check(checklist->toolTip().contains("Depth not zeroed"),"unzeroed depth is spelled out in the checklist");
    check(action->text().contains("Depth not zeroed")&&!action->isEnabled(),"Arm names the missing depth and stays refused");
    c.depthSample=depthSampleAt(0.1);
    frame(w,4);session->timerTick();
    check(pill->text()=="READY"&&action->isEnabled(),"front axis plus zeroed depth makes Pitch READY");

    both->click();
    check(session->axisMode()==VisualAxisMode::Both&&both->isChecked(),"clicking Both selects the Both axis");

    // Switching while armed stops and disarms.
    yaw->click();c.enabled=0x1b;c.known=0x1b;frame(w,5);session->timerTick();click(w,"visualArmButton");
    check(session->armed(),"armed in Yaw");
    const auto sends=c.sends.size();
    pitch->click();
    check(!session->armed()&&c.sends.size()==sends+1&&c.sends.back().second==rb::MotionMode::Stop,
          "selecting an axis while armed sends one STOP and disarms");
    check(pill->text()!="ARMED","the pill no longer says ARMED");
}

void depthBarAndZeroButton()
{
    test::VisualControllerFixture c;MainWindow w(&c);
    w.resize(1100,720);w.show();QApplication::processEvents();
    auto *session=w.findChild<VisualDispatchSession*>();auto *bar=child<QProgressBar>(w,"autoFollowDepthBar");
    auto *zero=child<QPushButton>(w,"autoFollowZeroDepthButton");auto *gate=child<QCheckBox>(w,"visualDispatchEnabled");
    check(session&&bar&&zero&&gate,"depth widgets exist");if(!(session&&bar&&zero&&gate))return;
    session->setTimerEnabled(false);
    const QString dot=QString(QChar(0x00B7));
    check(bar->format()=="Not zeroed"&&!zero->isEnabled(),"no sample: Not zeroed and Zero disabled");
    c.publishDepth(depthSampleAt(0.1,true,false));
    check(bar->format()=="Not zeroed"&&zero->isEnabled()&&bar->styleSheet().contains("#9AA7B2"),
          "fresh but unzeroed: Not zeroed (grey), Zero enabled");
    c.publishDepth(depthSampleAt(0.0));
    check(bar->format()=="0.00 m "+dot+" SURFACE"&&bar->value()==0&&bar->styleSheet().contains("#F0A92E"),
          "zeroed in air shows 0.00 m SURFACE (amber)");
    c.publishDepth(depthSampleAt(0.15));
    check(bar->format()=="0.15 m "+dot+" NORMAL"&&bar->value()==150&&bar->styleSheet().contains("#2F80C9"),
          "normal depth: value in millimetres, default colour");
    c.publishDepth(depthSampleAt(0.30));
    check(bar->format().contains("SOFT_FLOOR")&&bar->styleSheet().contains("#F0A92E"),"soft floor is amber");
    c.publishDepth(depthSampleAt(0.40));
    check(bar->format().contains("HARD_LIMIT")&&bar->styleSheet().contains("#C53F3F")&&bar->value()==400,"hard limit is red");
    c.publishDepth(depthSampleAt(0.9));
    check(bar->value()==500,"the bar clamps at 0.50 m");
    c.publishDepth(depthSampleAt(0.15,false));
    check(bar->format()=="Stale"&&bar->styleSheet().contains("#9AA7B2"),"a stale sample shows Stale (grey)");

    // Zero button: failure reasons go to the log.
    c.publishDepth(depthSampleAt(0.0,true,false));
    c.zeroResult=false;c.zeroError=QStringLiteral("depth is not steady");
    zero->click();
    check(c.zeroCalls==1&&logContains(w,"Zero depth failed: depth is not steady"),"Zero failure reason is logged");
    c.zeroResult=true;zero->click();
    check(c.zeroCalls==2,"Zero calls the controller");

    // Armed: Zero is disabled.
    c.enabled=0x1b;c.known=0x1b;c.publishDepth(depthSampleAt(0.1));
    frame(w);gate->setChecked(true);session->timerTick();click(w,"visualArmButton");
    check(session->armed(),"armed for the Zero-disable check");
    c.publishDepth(depthSampleAt(0.1));
    check(!zero->isEnabled(),"Zero is disabled while armed");
    click(w,"visualArmButton");
    c.publishDepth(depthSampleAt(0.1));
    check(zero->isEnabled(),"Zero is available again after disarming");
    c.connected=false;c.publishDepth(depthSampleAt(0.1));
    check(!zero->isEnabled(),"Zero needs a connected Remote controller");
}

void csvDepthColumnsAreWired()
{
    test::VisualControllerFixture c;MainWindow w(&c);
    QTemporaryDir dir;check(dir.isValid(),"temp dir");
    auto *logger=w.findChild<VisualCsvLogger*>();
    check(logger!=nullptr,"CSV logger exists");if(!logger)return;
    c.depthSample=depthSampleAt(0.125);
    check(logger->start(dir.path()),"logger starts");
    frame(w);
    logger->flush();
    QFile file(logger->filePath());check(file.open(QIODevice::ReadOnly),"CSV opens");
    const auto lines=QString::fromUtf8(file.readAll()).split(QChar(10),Qt::SkipEmptyParts);
    bool found=false;
    for(const auto &line:lines){
        if(!line.startsWith("frame"))continue;
        const auto cols=line.split(',');
        // depth_raw_m, depth_cal_m, depth_age_ms, envelope_state are the last four columns.
        if(cols.size()==42&&qAbs(cols[38].toDouble()-0.24)<1e-9&&qAbs(cols[39].toDouble()-0.125)<1e-9&&cols[40]=="20") found=true;
    }
    check(found,"frame rows carry the controller's raw/zeroed depth and age");
}

}
int main(int argc,char **argv){QApplication app(argc,argv);autoFollowPanelContract();axisSelectorAndChecklist();depthBarAndZeroButton();csvDepthColumnsAreWired();controls();transientSafetyEvents();replayDesktopTransientStale();coveredOperatorStopUi();manualPaths();keyboardPaths();rawRejection();signAndOffMatrix();enableReleaseBranchesAndAlert();negativeAndTimer();loopbackWindowAndCsv();std::printf("main_window_visual_dispatch_tests: %d failures\n",failures);return failures?1:0;}
