#include "ui/MainWindow.h"
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
    auto *gate=child<QCheckBox>(w,"visualDispatchEnabled");auto *sign=child<QComboBox>(w,"visualTurnSign");
    check(gate&&sign,"gate and separate sign selector exist");if(!gate||!sign)return nullptr;
    gate->setChecked(true);sign->setCurrentIndex(sign->findData(1));click(w,"visualTurnSignConfirm");click(w,"visualArmButton");
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
    auto *gate=child<QCheckBox>(w,"visualDispatchEnabled");auto *sign=child<QComboBox>(w,"visualTurnSign");
    check(gate&&!gate->isChecked(),"feature defaults OFF");check(sign&&sign->currentIndex()==0,"sign has unconfirmed placeholder");
    auto *s=w.findChild<VisualDispatchSession*>();if(!gate||!sign||!s)return;
    gate->setChecked(true);check(!s->armed(),"gate ON does not arm");click(w,"visualArmButton");
    auto *status=child<QLabel>(w,"visualDispatchStatus");
    check(status&&status->text().contains("TURN_SIGN_UNCONFIRMED"),"missing sign rejection visible");
    sign->setCurrentIndex(sign->findData(-1));click(w,"visualArmButton");check(!s->armed(),"selection is not confirmation");
    click(w,"visualTurnSignConfirm");click(w,"visualArmButton");check(s->armed(),"separate confirmation enables arming");
    s->timerTick();check(!c.sends.empty(),"real snapshot drives dispatch");
    if(!c.sends.empty())c.ack(c.sends.back().first,CommandTerminalResult::Ok);
    check(status->text().contains("ACK-confirmed: FORWARD"),"UI displays ACK confirmed actual mode");
    auto *v=w.findChild<VideoView*>();check(v&&!v->visualDiagnosticText().contains("no motion output")&&!v->visualDiagnosticText().contains("not sent"),"enabled video text is truthful");
    check(v&&v->visualDiagnosticText().contains("turn sign: -1 confirmed")
          &&!v->visualDiagnosticText().contains("turn_sign="),"active sign shows operator-confirmed -1 without conflicting preview default");
    check(!child<QCheckBox>(w,"visualCsvEnabled")->text().contains("no motion output"),"enabled CSV label truthful");
    const auto n=c.sends.size();click(w,"visualDisarmButton");check(!s->armed()&&c.sends.size()==n+1,"distinct Disarm sends one operator STOP");
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
        s->timerTick();const auto n=c.sends.size();
        auto *sign=child<QComboBox>(w,"visualTurnSign");sign->setCurrentIndex(sign->findData(-1));
        check(!s->armed()&&!s->confirmedTurnSign()&&c.sends.size()==n+1&&c.sends.back().second==rb::MotionMode::Stop,"UI sign change invalidates confirmation and sends one conditional operator STOP");
        click(w,"visualArmButton");check(!s->armed(),"changed sign requires separate confirmation again");
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

}
int main(int argc,char **argv){QApplication app(argc,argv);controls();coveredOperatorStopUi();manualPaths();keyboardPaths();rawRejection();signAndOffMatrix();enableReleaseBranchesAndAlert();negativeAndTimer();loopbackWindowAndCsv();std::printf("main_window_visual_dispatch_tests: %d failures\n",failures);return failures?1:0;}
