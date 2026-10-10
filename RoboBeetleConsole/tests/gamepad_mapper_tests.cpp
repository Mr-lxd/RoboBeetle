#include "input/GamepadMapper.h"
#include <cstdio>
#include <cstdlib>
using namespace rb;
namespace {
int failures = 0;
void expect(bool ok, const char *name) {
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", name); ++failures; }
}
struct Driver {
    GamepadMapper mapper;
    GamepadInput in;
    Driver() {
        in.connected = in.authority = in.enabled = true;
        in.depth = DepthEnvelopeState::Normal;
        expect(!mapper.update(in).command, "enable itself sends nothing");
    }
    GamepadOutput poll(std::int64_t time, bool accepted = true) {
        in.nowMs = time;
        auto out = mapper.update(in);
        in.previousAccepted = out.command ? std::optional<bool>(accepted) : std::nullopt;
        return out;
    }
};
void centerBeforeDriving() {
    GamepadMapper mapper;
    GamepadInput in;
    in.connected = in.authority = in.enabled = true;
    in.depth = DepthEnvelopeState::Normal;
    in.leftY = 1;
    expect(!mapper.update(in).command, "enable with held stick sends nothing");
    in.nowMs = 20;
    expect(!mapper.update(in).command, "held stick waits for center across polls");
    in.b = true;
    expect(mapper.update(in).command == MotionMode::Stop, "B still stops while waiting for center");
    in.b = false; in.leftY = 0; in.rightX = .35;
    mapper.update(in);
    in.leftY = 1; in.nowMs = 40;
    expect(!mapper.update(in).command, "all axes must be strictly below center threshold");
    in.leftY = 0; in.rightX = 0; in.leftX = .4;
    mapper.update(in);
    in.leftY = 1;
    expect(!mapper.update(in).command, "unmapped left horizontal also must center");
    in.leftX = 0; in.leftY = 0;
    expect(!mapper.update(in).command, "center itself sends nothing");
    in.leftY = 1; in.nowMs = 60;
    expect(mapper.update(in).command == MotionMode::Forward, "push after center starts forward");
    in.previousAccepted = true; in.connected = false;
    const auto disconnected = mapper.update(in);
    expect(disconnected.disable && disconnected.command == MotionMode::Stop, "disconnect closes driving");
    in.previousAccepted.reset(); in.connected = true;
    expect(!mapper.update(in).command, "re-enable with held stick sends nothing");
    in.nowMs = 600;
    expect(!mapper.update(in).command, "re-enable requires a new center observation");
}
void directions() {
    for (const auto mode : {MotionMode::Forward, MotionMode::TurnLeft, MotionMode::TurnRight,
                            MotionMode::Ascend, MotionMode::Descend}) {
        Driver d;
        if (mode == MotionMode::Forward) d.in.leftY = 1;
        if (mode == MotionMode::TurnLeft) d.in.rightX = -1;
        if (mode == MotionMode::TurnRight) d.in.rightX = 1;
        if (mode == MotionMode::Ascend) d.in.rightY = 1;
        if (mode == MotionMode::Descend) d.in.rightY = -1;
        expect(d.poll(20).command == mode, "mapped direction");
        expect(!d.poll(600).command, "unchanged mode not repeated");
    }
    Driver d; d.in.leftY = -1; d.in.leftX = 1;
    expect(!d.poll(20).command, "no backward or left horizontal");
}
void hysteresisAndPriority() {
    Driver d; d.in.leftY = .5;
    expect(!d.poll(20).command, "strict push threshold");
    d.in.leftY = .51; expect(d.poll(40).command == MotionMode::Forward, "forward push");
    d.in.leftY = .35; expect(!d.poll(60).command, "hold at center threshold");
    d.in.leftY = .34; expect(d.poll(80).command == MotionMode::Stop, "release below threshold");
    expect(!d.poll(100).command, "center STOP once");
    Driver p; p.in.leftY = 1; p.poll(20); p.in.rightX = -1;
    expect(p.poll(520).command == MotionMode::TurnLeft, "right priority");
    p.in.rightX = 0; expect(p.poll(1020).command == MotionMode::Forward, "right center falls back");
    Driver a; a.in.rightX = .8; a.in.rightY = .7;
    expect(a.poll(20).command == MotionMode::TurnRight, "dominant axis");
    a.in.rightY = 1; expect(!a.poll(600).command, "axis remains locked");
    a.in.rightX = .34; expect(a.poll(620).command == MotionMode::Ascend, "axis releases");
    a.in.rightY = -.8; expect(a.poll(1120).command == MotionMode::Descend, "axis direction reversal");
    Driver v; v.in.rightX = .7; v.in.rightY = -.9;
    expect(v.poll(20).command == MotionMode::Descend, "vertical dominant");
}
void spacingAndLatest() {
    Driver d; d.in.leftY = 1; d.poll(20);
    d.in.rightX = -1; expect(!d.poll(519).command, "minimum 500 ms");
    d.in.rightX = 1; expect(d.poll(520, false).command == MotionMode::TurnRight, "latest at deadline");
    d.in.rightX = 0; d.in.rightY = -1;
    expect(d.poll(540, false).command == MotionMode::Descend, "rejected request replaced");
    d.in.rightY = 0;
    expect(!d.poll(560).command, "no stale retry when back to accepted mode");
    d.in.leftY = 0; expect(d.poll(580).command == MotionMode::Stop, "STOP bypasses interval");
    Driver idle; expect(!idle.poll(20).command, "idle leaves UI motion alone");
    Driver rejected; rejected.in.leftY = 1; rejected.poll(20, false);
    rejected.in.leftY = 0; expect(!rejected.poll(40).command, "unaccepted start owns no motion");
}
void bAndDisable() {
    Driver b; b.in.enabled = false; b.poll(20); b.in.b = true;
    expect(b.poll(40).command == MotionMode::Stop, "B works while disabled with authority");
    expect(!b.poll(60).command, "B press edge only");
    b.in.b = false; b.poll(80); b.in.authority = false; b.in.b = true;
    expect(!b.poll(100).command, "B needs authority");
    Driver driving; driving.in.leftY = 1; driving.poll(20); driving.in.b = true;
    expect(driving.poll(40).command == MotionMode::Stop, "B immediate STOP");
    expect(!driving.poll(600).command, "held stick does not undo B STOP");
    driving.in.b = false; driving.in.leftY = 0; driving.poll(620);
    driving.in.leftY = 1; expect(driving.poll(640).command == MotionMode::Forward, "new stick request after B");
    for (int reason = 0; reason < 3; ++reason) {
        Driver d; d.in.leftY = 1; d.poll(20);
        if (reason == 0) d.in.connected = false;
        if (reason == 1) d.in.enabled = false;
        if (reason == 2) d.in.authority = false;
        auto out = d.poll(40);
        expect(out.command == MotionMode::Stop, "disable/disconnect/loss STOP");
        expect(reason == 1 || out.disable, "automatic close requested");
        expect(!d.poll(60).command, "disable STOP once");
    }
    Driver idle; idle.in.connected = false;
    auto out = idle.poll(20); expect(!out.command && out.disable, "idle disconnect closes without STOP");
}
void depth() {
    for (auto state : {DepthEnvelopeState::Unavailable, DepthEnvelopeState::NotZeroed,
                       DepthEnvelopeState::Surface, DepthEnvelopeState::SoftFloor}) {
        for (double y : {-1., 1.}) {
            Driver d; d.in.depth = state; d.in.rightY = y; d.in.leftY = 1;
            const bool blocked = state == DepthEnvelopeState::Unavailable || state == DepthEnvelopeState::NotZeroed
                || (state == DepthEnvelopeState::Surface && y > 0)
                || (state == DepthEnvelopeState::SoftFloor && y < 0);
            expect(d.poll(20).command == (blocked ? MotionMode::Forward : y > 0 ? MotionMode::Ascend : MotionMode::Descend),
                   "depth gate falls back to forward or permits opposite direction");
        }
    }
    Driver centered; centered.in.rightY = 1; centered.in.depth = DepthEnvelopeState::Surface;
    expect(!centered.poll(20).command, "blocked vertical with no forward stays idle");
    for (auto state : {DepthEnvelopeState::Surface, DepthEnvelopeState::SoftFloor,
                       DepthEnvelopeState::Unavailable}) {
        Driver d; d.in.rightY = state == DepthEnvelopeState::SoftFloor ? -1 : 1;
        d.in.leftY = 1; d.poll(20); d.in.depth = state;
        expect(d.poll(40).command == MotionMode::Stop, "active vertical boundary STOP before fallback");
    }
    Driver desc; desc.in.rightY = -1; desc.poll(20); desc.in.depth = DepthEnvelopeState::Unavailable;
    expect(desc.poll(40).command == MotionMode::Stop, "descending stale depth STOP");
    Driver hard; hard.in.leftY = 1; hard.poll(20); hard.in.depth = DepthEnvelopeState::HardLimit;
    auto out = hard.poll(40); expect(out.command == MotionMode::Stop && out.disable, "hard limit STOP and close");
}
}
int main() {
    centerBeforeDriving(); directions(); hysteresisAndPriority(); spacingAndLatest(); bAndDisable(); depth();
    if (!failures) std::puts("Gamepad mapper rules passed");
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
