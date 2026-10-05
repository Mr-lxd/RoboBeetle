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
    return {f.detections.isEmpty() ? DetectionDisplayState::NoTarget : DetectionDisplayState::Target,f};
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
    s.onDetectionArrival(frame(11,135),{DetectionDisplayState::AwaitingVideo,frame(11,135)});
    check(s.snapshot().awaitingVideo && s.snapshot().state==VisualState::Tracking
          && s.snapshot().command.effective==old.effective
          && s.snapshot().command.ex_f==old.ex_f
          && s.snapshot().command.next.lastSwitchMs==old.next.lastSwitchMs,
          "B1 ahead metadata does not STOP, change EMA or lock dwell");
    now=35;
    s.refresh(accepted(frame(11,135)));
    check(s.snapshot().command.ex_f && std::abs(*s.snapshot().command.ex_f+0.6109375)<1e-9
          && s.snapshot().command.effective==ProposedCommand::TurnLeft,
          "15 ms video catchup evaluates the pending ID exactly once without STOP");
    now=40;
    s.refresh(accepted(frame(11,135)));
    check(s.snapshot().command.ex_f && std::abs(*s.snapshot().command.ex_f+0.6109375)<1e-9,
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
    check(a.snapshot().policyVersion==QStringLiteral("visual-command-proposal-v3"),
          "snapshot identifies the policy version");
    check(hash.size()==64 && hash==b.snapshot().policyHash,
          "same actual configuration has a deterministic SHA-256");
    VisualPolicyConfig c; c.alpha=0.4;
    VisualDiagnosticSession changed(c,[&]{return now;});
    check(hash!=changed.snapshot().policyHash,"actual parameter change changes provenance hash");
    for(int i=0;i<3;++i) {
        c={};
        if(i==0)c.gate_px=49;
        if(i==1)c.max_miss_ms=501;
        if(i==2)c.require_same_class=true;
        VisualDiagnosticSession associationConfig(c,[&]{return now;});
        check(hash!=associationConfig.snapshot().policyHash,"each association parameter changes policy hash");
    }
    a.beginSession(42);
    check(a.snapshot().policyHash==hash,"session reset preserves immutable configuration provenance");
}
void associationAndMisses() {
    qint64 now=0;
    VisualDiagnosticSession s({},[&]{return now;});
    auto f=frame(1,450); f.detections.push_back({1,"other",.85,{547,240}});
    s.onDetectionArrival(f,accepted(f));
    check(s.snapshot().associationStatus==AssociationStatus::Acquired,"session acquires initial lock");
    for(quint64 id=2;id<8;++id) {
        now+=40; f.frameId=id; f.detections[0].confidence=id%2?.9:.85;
        f.detections[1].confidence=id%2?.85:.9;
        s.onDetectionArrival(f,accepted(f));
        check(s.snapshot().target && s.snapshot().target->target.originalPoint.x()==450
              && s.snapshot().associationStatus==AssociationStatus::Associated
              && s.snapshot().highestConfidenceTarget->target.originalPoint.x()==(id%2?450:547),
              "session follows lock while highest baseline alternates");
    }
    const auto retained=s.snapshot().command;
    now=300; f=frame(8,547); s.onDetectionArrival(f,accepted(f));
    check(s.snapshot().state==VisualState::NoTarget && !s.snapshot().target
          && s.snapshot().associationStatus==AssociationStatus::Miss
          && s.snapshot().command.proposed==ProposedCommand::Hold
          && s.snapshot().command.effective==retained.effective
          && s.snapshot().command.ex_f==retained.ex_f,"out-of-gate detection is NO_TARGET/HOLD");
    now=600; f=frame(9,547); s.onDetectionArrival(f,accepted(f));
    now=799; s.refresh(accepted(f));
    check(s.snapshot().associationStatus==AssociationStatus::Miss
          && s.snapshot().associationMissMs==499,"duplicate refresh does not restart MISS clock");
    now=800; s.refresh(accepted(f));
    check(s.snapshot().associationStatus==AssociationStatus::Unlocked && !s.snapshot().target
          && s.snapshot().command.ex_f==retained.ex_f,"MISS deadline releases without replay or EMA reset");
    now=840; f=frame(10,547); s.onDetectionArrival(f,accepted(f));
    check(s.snapshot().associationStatus==AssociationStatus::Acquired && s.snapshot().target
          && std::abs(*s.snapshot().command.ex_f-(.3*(227.0/320)+.7*(*retained.ex_f)))<1e-12,
          "next fresh ID reacquires and retains original EMA formula/history");
    s.refresh({DetectionDisplayState::Stale,f});
    check(s.snapshot().associationStatus==AssociationStatus::Unlocked && !s.snapshot().target,
          "STALE releases in same evaluation");
    s.refresh(accepted(f));
    check(!s.snapshot().target,"released sample cannot be reacquired by repeat");
    s.beginSession(42); f=frame(0,547); s.onDetectionArrival(f,accepted(f));
    check(s.snapshot().associationStatus==AssociationStatus::Acquired,"session reset accepts ID zero");
    s.refresh({DetectionDisplayState::InferenceOff,f});
    check(s.snapshot().associationStatus==AssociationStatus::Unlocked
          && s.snapshot().command.proposed==ProposedCommand::Stop,"OFF releases and stops");
}
void overdueCatchupAndLost() {
    qint64 now=0; VisualDiagnosticSession s({},[&]{return now;});
    auto f=frame(1); s.onDetectionArrival(f,accepted(f));
    now=40; f=frame(2,135); s.onDetectionArrival(f,{DetectionDisplayState::AwaitingVideo,f});
    check(s.snapshot().associationStatus==AssociationStatus::Acquired,"awaiting does not associate");
    now=540; s.refresh(accepted(f)); // Late video event before the overdue timer.
    check(s.snapshot().state==VisualState::Stale && !s.snapshot().target
          && s.snapshot().associationStatus==AssociationStatus::Unlocked
          && !s.snapshot().command.ex_f && s.snapshot().command.proposed==ProposedCommand::Stop,
          "single-pass overdue catchup releases transient association without output");
    s.refresh(accepted(f)); check(!s.snapshot().target,"late ID cannot revive on repeat");
    now=600; f=frame(3); s.onDetectionArrival(f,accepted(f));
    for(int i=0;i<=6;++i) {
        now=650+i*250; f=frame(4+i); f.detections.clear(); s.onDetectionArrival(f,accepted(f));
    }
    check(s.snapshot().state==VisualState::Lost && s.snapshot().associationStatus==AssociationStatus::Unlocked
          && s.snapshot().command.proposed==ProposedCommand::Stop,"continuous fresh empty frames become LOST and unlock");
}
void initialAwaitingVideo() {
    qint64 now=0; VisualDiagnosticSession s({},[&]{return now;});
    auto f=frame(1);
    s.onDetectionArrival(f,{DetectionDisplayState::AwaitingVideo,f});
    check(s.snapshot().awaitingVideo && s.snapshot().state==VisualState::Stale,
          "initial awaiting retains initial state without consuming sample");
    now=15; s.refresh(accepted(f));
    check(s.snapshot().target && s.snapshot().associationStatus==AssociationStatus::Acquired
          && s.snapshot().state==VisualState::Tracking,"initial pending ID acquires at first video catchup");
    s.beginSession(2); now=20; f=frame(1);
    s.onDetectionArrival(f,{DetectionDisplayState::Stale,f}); // No video dimensions yet.
    now=35; s.refresh(accepted(f));
    check(s.snapshot().target && s.snapshot().associationStatus==AssociationStatus::Acquired,
          "never-associated startup frame can become usable before watchdog expires");
}
}
int main(int argc,char **argv){QCoreApplication app(argc,argv);raceAndTimeout();automaticWakeup();configProvenance();associationAndMisses();overdueCatchupAndLost();initialAwaitingVideo();return failures?1:0;}
