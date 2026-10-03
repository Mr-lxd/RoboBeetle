#include "vision/VisualTargetStateMachine.h"
#include <cstdio>

namespace {
using namespace rb::vision;
int failures = 0;
void check(bool ok, const char *why) {
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", why); ++failures; }
}
std::optional<TargetState> sample(quint64 id = 10) {
    DetectionFrame f{id, 1000, {640,480}, {{0, "fish", 0.9, {480,240}}}};
    return selectTargetState(f);
}
void deadlinesAndEmptyFrames() {
    VisualPolicyConfig c;
    auto r = advanceVisualTargetState({}, {0,10,DetectionDisplayState::Target,sample()},c);
    check(r.state == VisualState::Tracking && r.frameAdvanced, "first valid target tracks");
    auto m = r.next;
    r = advanceVisualTargetState(m,{400,10,DetectionDisplayState::Target,sample()},c);
    check(!r.frameAdvanced && r.next.lastAdvancedArrivalMs == 0, "same ID does not renew local arrival");
    r = advanceVisualTargetState(r.next,{499,std::nullopt,DetectionDisplayState::Target,sample()},c);
    check(r.state == VisualState::Tracking, "frozen Pi timestamp with fresh HTTP tracks until 499 ms");
    r = advanceVisualTargetState(r.next,{500,std::nullopt,DetectionDisplayState::Target,sample()},c);
    check(r.state == VisualState::Stale && !r.usableTarget, "at 500 ms old target becomes stale");
    r = advanceVisualTargetState(r.next,{501,11,DetectionDisplayState::Target,sample(11)},c);
    check(r.state == VisualState::Tracking, "new increasing ID can recover");
    m = {};
    for (int t=0; t<=1500; t+=100) {
        r = advanceVisualTargetState(m,{t,quint64(t/100),DetectionDisplayState::NoTarget,{}},c);
        check(r.state == (t<1500 ? VisualState::NoTarget : VisualState::Lost),
              "continued fresh empty frames become LOST at 1500 ms");
        m = r.next;
    }
    r = advanceVisualTargetState({}, {0,0,DetectionDisplayState::NoTarget,{}},c);
    r = advanceVisualTargetState(r.next, {500,{},DetectionDisplayState::NoTarget,{}},c);
    check(r.state == VisualState::Stale, "empty result without new frames is STALE before LOST");
}
void awaitingVideoAndPriorities() {
    VisualPolicyConfig c;
    auto r = advanceVisualTargetState({}, {0,10,DetectionDisplayState::Target,sample()},c);
    r = advanceVisualTargetState(r.next,{20,11,DetectionDisplayState::AwaitingVideo,{}},c);
    check(r.awaitingVideo && r.state == VisualState::Tracking
              && r.next.lastAdvancedArrivalMs == 20, "ahead detection preserves TRACKING and renews arrival");
    auto waiting = r.next;
    r = advanceVisualTargetState(waiting,{35,{},DetectionDisplayState::Target,sample(11)},c);
    check(r.state == VisualState::Tracking && !r.awaitingVideo && r.usableTarget,
          "video catches up 15 ms later without STALE");
    r = advanceVisualTargetState(waiting,{519,{},DetectionDisplayState::AwaitingVideo,{}},c);
    check(r.state == VisualState::Tracking && r.awaitingVideo, "pending video allowed through 499 ms");
    r = advanceVisualTargetState(r.next,{520,{},DetectionDisplayState::AwaitingVideo,{}},c);
    check(r.state == VisualState::Stale && !r.awaitingVideo, "no matching video at 500 ms becomes STALE");
    r = advanceVisualTargetState(waiting,{400,12,DetectionDisplayState::AwaitingVideo,{}},c);
    r = advanceVisualTargetState(r.next,{520,13,DetectionDisplayState::AwaitingVideo,{}},c);
    check(r.state == VisualState::Stale, "new ahead IDs cannot perpetually postpone the video deadline");
    r = advanceVisualTargetState(waiting,{400,12,DetectionDisplayState::AwaitingVideo,{}},c);
    r = advanceVisualTargetState(r.next,{521,{},DetectionDisplayState::Target,sample(12)},c);
    check(r.state == VisualState::Stale && !r.usableTarget,
          "late catchup cannot bypass the oldest pending deadline before an overdue timer");
    r = advanceVisualTargetState(waiting,{30,{},DetectionDisplayState::InferenceOff,{}},c);
    check(r.state == VisualState::InferenceOff, "authoritative OFF overrides pending video");
    r = advanceVisualTargetState(waiting,{30,{},DetectionDisplayState::Stale,{}},c);
    check(r.state == VisualState::Stale, "HTTP invalidity overrides pending video");
    r = advanceVisualTargetState({}, {0,0,DetectionDisplayState::AwaitingVideo,{}},c);
    check(r.state == VisualState::Stale && r.awaitingVideo, "cold start waiting cannot invent a target");
    r = advanceVisualTargetState({}, {0,{},DetectionDisplayState::Target,sample()},c);
    check(r.state == VisualState::Stale, "rendered metadata is not an arrival event");
    r = advanceVisualTargetState(waiting,{19,{},DetectionDisplayState::Target,sample()},c);
    check(r.state == VisualState::Stale, "backward monotonic time fails closed");
    c.alpha = 0;
    r = advanceVisualTargetState({}, {0,10,DetectionDisplayState::Target,sample()},c);
    check(r.state == VisualState::Stale, "invalid config fails closed");
}
}
int main() { deadlinesAndEmptyFrames(); awaitingVideoAndPriorities(); return failures ? 1 : 0; }
