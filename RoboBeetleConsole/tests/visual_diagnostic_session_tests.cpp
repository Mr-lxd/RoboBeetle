#include "vision/VisualDiagnosticSession.h"
#include <QCoreApplication>
#include <QEventLoop>
#include <QTimer>
#include <cmath>
#include <cstdio>

namespace {
using namespace rb::vision;
int failures=0;
void check(bool ok,const char *why) { if(!ok){std::fprintf(stderr,"FAIL: %s\n",why);++failures;} }
DetectionFrame frame(quint64 id,double u=120.0) {
    return {id, 1000+id, {640,480}, {{0,"fish",0.9,{u,240}}}};
}
VisualViewContext accepted(const DetectionFrame &f) {
    return {DetectionDisplayState::Target,selectTargetState(f)};
}
void raceAndTimeout() {
    qint64 now=0;
    VisualDiagnosticSession s({},[&]{return now;});
    s.beginSession(1);
    s.onDetectionArrival(frame(10),accepted(frame(10)));
    check(s.snapshot().state==VisualState::Tracking
          && s.snapshot().command.effective==ProposedCommand::TurnLeft,"first target produces a turn suggestion");
    auto old=s.snapshot().command;
    now=20;
    s.onDetectionArrival(frame(11,480),{DetectionDisplayState::AwaitingVideo,{}});
    check(s.snapshot().awaitingVideo && s.snapshot().state==VisualState::Tracking
          && s.snapshot().command.effective==old.effective
          && s.snapshot().command.ex_f==old.ex_f
          && s.snapshot().command.next.lastSwitchMs==old.next.lastSwitchMs,
          "B1 ahead metadata does not STOP, change EMA or lock dwell");
    now=35;
    s.refresh(accepted(frame(11,480)));
    check(s.snapshot().command.ex_f && std::abs(*s.snapshot().command.ex_f+0.2875)<1e-9
          && s.snapshot().command.effective==ProposedCommand::TurnLeft,
          "15 ms video catchup evaluates the pending ID exactly once without STOP");
    now=40;
    s.refresh(accepted(frame(11,480)));
    check(s.snapshot().command.ex_f && std::abs(*s.snapshot().command.ex_f+0.2875)<1e-9,
          "repaint does not apply EMA again");
    now=50;
    s.onDetectionArrival(frame(12),{DetectionDisplayState::AwaitingVideo,{}});
    now=550;
    s.refresh({DetectionDisplayState::AwaitingVideo,{}});
    check(s.snapshot().state==VisualState::Stale
          && s.snapshot().command.effective==ProposedCommand::Stop,"500 ms missing video causes STALE STOP");
    s.beginSession(2);
    now=600;
    s.onDetectionArrival(frame(1),accepted(frame(1)));
    check(s.snapshot().state==VisualState::Tracking && s.snapshot().sessionId==2,
          "new session accepts reset IDs without old memory");
    now=1000;
    s.onDetectionArrival(frame(1),accepted(frame(1)));
    now=1100;
    s.refresh(accepted(frame(1)));
    check(s.snapshot().state==VisualState::Stale && !s.snapshot().target
          && s.snapshot().command.proposed==ProposedCommand::Stop,"repeated arrival ID does not reset frozen-frame timeout");
}
void automaticWakeup() {
    VisualPolicyConfig c; c.stale_ms=30; c.ui_tick_ms=5;
    VisualDiagnosticSession s(c);
    s.onDetectionArrival(frame(10),accepted(frame(10)));
    QEventLoop loop;
    QTimer::singleShot(60,&loop,&QEventLoop::quit);
    loop.exec();
    check(s.snapshot().state==VisualState::Stale
          && s.snapshot().command.proposed==ProposedCommand::Stop,
          "external Qt wakeup expires frozen detection even with no new video or HTTP event");
}
void configProvenance() {
    qint64 now=0;
    VisualDiagnosticSession a({},[&]{return now;}), b({},[&]{return now;});
    const auto hash=a.snapshot().policyHash;
    check(a.snapshot().policyVersion==QStringLiteral("visual-command-proposal-v1"),
          "snapshot identifies the policy version");
    check(hash.size()==64 && hash==b.snapshot().policyHash,
          "same actual configuration has a deterministic SHA-256");
    VisualPolicyConfig c; c.alpha=0.4;
    VisualDiagnosticSession changed(c,[&]{return now;});
    check(hash!=changed.snapshot().policyHash,"actual parameter change changes provenance hash");
    a.beginSession(42);
    check(a.snapshot().policyHash==hash,"session reset preserves immutable configuration provenance");
}
}
int main(int argc,char **argv){QCoreApplication app(argc,argv);raceAndTimeout();automaticWakeup();configProvenance();return failures?1:0;}
