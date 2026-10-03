#include "vision/TargetAssociation.h"
#include <cmath>
#include <cstdio>
#include <limits>

namespace {
using namespace rb::vision;
int failures = 0;
void check(bool ok, const char *why) {
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", why); ++failures; }
}
DetectionFrame frame(quint64 id, double u, double v = 240) {
    return {id, 1000 + id, {640,480}, {{0,"fish",.89,{u,v}}}};
}
void alternatingAndMoving() {
    VisualPolicyConfig c;
    auto f = frame(1,450);
    f.detections.push_back({1,"other",.85,{547,240}});
    auto r = associateTarget({},f,0,c);
    check(r.selected && r.selected->target.originalPoint.x()==450
          && r.status==AssociationStatus::Acquired && !r.distancePx, "initial highest acquisition");
    for (quint64 i=2;i<22;++i) {
        f.frameId=i;
        f.detections[0].confidence=i%2 ? .89 : .85;
        f.detections[1].confidence=i%2 ? .85 : .89;
        r=associateTarget(r.next,f,static_cast<qint64>(i)*40,c);
        check(r.selected && r.selected->target.originalPoint.x()==450
              && r.status==AssociationStatus::Associated && r.distancePx==0,
              "confidence alternation cannot jump to 547");
        check(selectTargetState(f)->target.originalPoint.x()==(i%2?450:547), "highest baseline alternates");
    }
    r=associateTarget({},frame(1,20),0,c);
    for(quint64 i=1;i<=100;++i) {
        r=associateTarget(r.next,frame(i+1,20+5*i),static_cast<qint64>(i)*40,c);
        check(r.status==AssociationStatus::Associated && r.distancePx==5
              && r.selected->target.originalPoint.x()==20+5*i, "100 incremental five-pixel frames");
    }
}
void missesAndBoundaries() {
    VisualPolicyConfig c;
    auto lock=associateTarget({},frame(1,450),0,c).next;
    auto miss=associateTarget(lock,frame(2,547),100,c);
    check(miss.status==AssociationStatus::Miss && !miss.selected && miss.distancePx==97
          && miss.next.lockedTarget && miss.next.lockedTarget->target.originalPoint.x()==450
          && miss.next.missSinceMs==100, "MISS retains locked position and starts timer");
    auto repeat=associateTarget(miss.next,frame(2,450),200,c);
    check(!repeat.selected && repeat.next.missSinceMs==100, "duplicate cannot replace MISS sample");
    auto before=expireTargetAssociation(repeat.next,599,c);
    check(before.lockedTarget.has_value(), "499 ms preserves lock");
    auto expired=expireTargetAssociation(before,600,c);
    check(!expired.lockedTarget && expired.lastProcessedFrameId==2, "500 ms releases but retains consumed ID");
    check(!associateTarget(expired,frame(2,547),600,c).selected, "old ID cannot reacquire after release");
    auto acquired=associateTarget(expired,frame(3,547),640,c);
    check(acquired.status==AssociationStatus::Acquired && acquired.selected, "next ID reacquires other target");
    lock=associateTarget({},frame(1,100,100),0,c).next;
    for(const auto &p : {QPointF(148,100),QPointF(148.01,100),QPointF(128,136),QPointF(136,136)}) {
        auto r=associateTarget(lock,frame(2,p.x(),p.y()),40,c);
        check(r.status==((p.x()==148.01 || p.x()==136)?AssociationStatus::Miss:AssociationStatus::Associated),
              "48 included, 48.01 excluded, distance uses both axes");
    }
    auto recovered=associateTarget(miss.next,frame(3,455),300,c);
    check(recovered.selected && !recovered.next.missSinceMs, "success ends continuous MISS interval");
}
void tiesValidityAndRelease() {
    VisualPolicyConfig c;
    auto lock=associateTarget({},frame(1,320),0,c).next;
    auto f=frame(2,310);
    f.detections[0].confidence=.85;
    f.detections.push_back({1,"other",.89,{330,240}});
    auto r=associateTarget(lock,f,40,c);
    check(r.selectedDetectionIndex==1, "equal distance prefers higher confidence");
    f.detections[0].confidence=.89;
    r=associateTarget(lock,f,40,c);
    check(r.selectedDetectionIndex==0, "equal distance/confidence retains stream order");
    f=frame(2,325); f.detections[0].classId=9;
    check(associateTarget(lock,f,40,c).selected.has_value(), "default permits class change");
    c.require_same_class=true;
    check(!associateTarget(lock,f,40,c).selected, "same-class option rejects changed class ID");
    c={};
    const auto released=releaseTargetAssociationLock(lock);
    check(!released.lockedTarget && released.lastProcessedFrameId==1
          && !associateTarget(released,frame(1,325),40,c).selected, "release cannot replay consumed ID");
    f=frame(2,std::numeric_limits<double>::quiet_NaN());
    f.detections.push_back({1,"valid",.5,{325,240}});
    check(associateTarget(lock,f,40,c).selectedDetectionIndex==1, "invalid candidate cannot hide valid one");
    f.detections.clear();
    check(associateTarget(lock,f,40,c).status==AssociationStatus::Miss, "empty frame misses");
    for(double gate : {0.0,-1.0,std::numeric_limits<double>::infinity()}) {
        c.gate_px=gate;
        check(!validVisualPolicyConfig(c) && !associateTarget(lock,frame(2,325),40,c).valid,
              "invalid gate is rejected");
    }
    c={}; c.max_miss_ms=0;
    check(!validVisualPolicyConfig(c), "zero miss timeout is invalid");
    c={};
    auto later=associateTarget(lock,frame(2,325),40,c);
    check(!associateTarget(later.next,frame(3,325),39,c).valid, "backward injected time invalid");
    check(!associateTarget({},frame(0,320),-1,c).valid, "negative time invalid");
    f=frame(2,640); f.sourceSize={1280,720};
    check(associateTarget(lock,f,40,c).status==AssociationStatus::Acquired, "new image size releases old coordinate lock");
}
}
int main(){alternatingAndMoving();missesAndBoundaries();tiesValidityAndRelease();return failures?1:0;}
