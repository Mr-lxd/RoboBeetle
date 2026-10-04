#include "vision/VisualDispatchStateMachine.h"

#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

namespace {
using namespace rb::vision;
int checks = 0;
int failures = 0;
void expect(bool value, const char *message)
{
    ++checks;
    if (!value) { ++failures; std::fprintf(stderr, "FAIL: %s\n", message); }
}
struct FakePort final : VisualCommandSendPort {
    std::vector<DispatchRequest> requests;
    void send(const DispatchRequest &request) override { requests.push_back(request); }
    DispatchRequest last() const { return requests.empty() ? DispatchRequest{0, ProposedCommand::Hold} : requests.back(); }
};
VisualDispatchInput ready(std::int64_t time = 0)
{
    VisualDispatchInput input;
    input.nowMs = time;
    input.linkConnected = true;
    input.controlOwned = true;
    input.enabledMask = 0x001b;
    input.poseKnownMask = 0x001b;
    input.confirmedTurnSign = 1;
    input.state = VisualState::Tracking;
    input.suggestion = ProposedCommand::Forward;
    input.ex = 0.0;
    return input;
}
struct Fixture {
    FakePort port;
    VisualDispatchStateMachine policy;
    VisualDispatchInput input{ready()};
    explicit Fixture(VisualDispatchConfig config = {}) : policy(port, config) {}
    void start()
    {
        expect(policy.arm(input) == ArmReason::Ready && policy.armed(), "eligible explicit arm succeeds");
        policy.evaluate(input);
        expect(port.last().command == ProposedCommand::Forward, "first non-STOP sends immediately");
    }
    void ack(std::int64_t time = 1, DispatchOutcome outcome = DispatchOutcome::Ok)
    { policy.acknowledge(port.last().id, outcome, time); }
    void evaluate(std::int64_t time)
    { input.nowMs = time; policy.evaluate(input); }
    void turn(double error = 0.5)
    { input.suggestion = ProposedCommand::TurnRight; input.ex = error; }
};

void armingConditions()
{
    FakePort port;
    VisualDispatchStateMachine initial(port);
    expect(!initial.armed() && !initial.currentMode(), "default disarmed with no confirmed mode");
    initial.evaluate(ready());
    expect(port.requests.empty(), "default disarmed sends nothing");
    const auto reject = [](VisualDispatchInput input, ArmReason reason) {
        FakePort local;
        VisualDispatchStateMachine p(local);
        expect(p.arm(input) == reason && !p.armed(), "missing arm condition has exact rejection reason");
        expect(std::strcmp(armReasonName(reason), "UNKNOWN") != 0, "rejection reason is displayable");
        p.evaluate(input);
        expect(local.requests.empty(), "rejected arm sends no commands");
    };
    auto input = ready(); input.linkConnected = false; reject(input, ArmReason::LinkDisconnected);
    input = ready(); input.controlOwned = false; reject(input, ArmReason::ControlNotOwned);
    for (const auto mask : {0U, 0x000bU}) {
        input = ready(); input.enabledMask = static_cast<std::uint16_t>(mask); reject(input, ArmReason::ServosNotEnabled);
        input = ready(); input.poseKnownMask = static_cast<std::uint16_t>(mask); reject(input, ArmReason::PoseUnknown);
    }
    input = ready(); input.confirmedTurnSign.reset(); reject(input, ArmReason::TurnSignUnconfirmed);
    for (int sign : {0, 2, -2}) { input = ready(); input.confirmedTurnSign = sign; reject(input, ArmReason::TurnSignUnconfirmed); }
    for (auto state : {VisualState::Stale, VisualState::InferenceOff, VisualState::NoTarget, VisualState::Lost}) {
        input = ready(); input.state = state; reject(input, ArmReason::NotTracking);
    }
    input = ready(); input.nowMs = -1; reject(input, ArmReason::InvalidTime);
    VisualDispatchStateMachine invalid(port, {0, 0x001b});
    expect(invalid.arm(ready()) == ArmReason::InvalidConfig, "invalid ACK timeout refuses arm");
    VisualDispatchStateMachine emptyMask(port, {1000, 0});
    expect(emptyMask.arm(ready()) == ArmReason::InvalidConfig, "empty servo requirement cannot bypass eligibility");
}

void ackModeDwellAndBusy()
{
    Fixture f; f.start();
    const auto first = f.port.last();
    expect(!f.policy.currentMode(), "send alone never confirms current mode");
    f.turn(); f.evaluate(100);
    expect(f.port.requests.size() == 1, "pending ACK blocks new non-STOP even on suggestion change");
    f.policy.acknowledge(first.id, DispatchOutcome::Ok, 101);
    expect(f.policy.currentMode() == ProposedCommand::Forward, "only ACK OK updates current mode");
    f.evaluate(999); expect(f.port.requests.size() == 1, "non-STOP dwell rejects 999 ms");
    f.evaluate(1000); expect(f.port.requests.size() == 2 && f.port.last().command == ProposedCommand::TurnRight, "non-STOP dwell permits exactly 1000 ms");
    expect(f.policy.currentMode() == ProposedCommand::Forward, "pending turn leaves confirmed Forward mode");
    f.ack(1001, DispatchOutcome::Busy);
    expect(f.policy.currentMode() == ProposedCommand::Forward, "Busy never changes current mode");
    f.evaluate(1001); expect(f.port.requests.size() == 2, "Busy never immediately retries");
    f.turn(-0.5); f.evaluate(1999); expect(f.port.requests.size() == 2, "Busy respects next dwell window");
    f.evaluate(2000);
    expect(f.port.requests.size() == 3 && f.port.last().command == ProposedCommand::TurnLeft, "Busy next window compares latest suggestion, does not replay old right");
    f.ack(2001); f.evaluate(3000);
    expect(f.port.requests.size() == 3 && f.policy.currentMode() == ProposedCommand::TurnLeft, "unchanged confirmed command is suppressed");

    Fixture reverted; reverted.start(); reverted.ack(); reverted.turn(); reverted.evaluate(1000); reverted.ack(1001, DispatchOutcome::Busy);
    reverted.input.suggestion = ProposedCommand::Forward; reverted.evaluate(2000);
    expect(reverted.port.requests.size() == 2, "Busy with latest suggestion matching confirmed mode sends nothing");
}

void disarmingSafety()
{
    for (int condition = 0; condition < 6; ++condition) {
        Fixture f; f.start(); // The motion request is still pending, within dwell.
        if (condition == 0) f.input.state = VisualState::Stale;
        if (condition == 1) f.input.state = VisualState::InferenceOff;
        if (condition == 2) f.input.linkConnected = false;
        if (condition == 3) f.input.controlOwned = false;
        if (condition == 4) f.input.enabledMask = 0;
        if (condition == 5) f.input.poseKnownMask = 0;
        f.evaluate(20);
        expect(!f.policy.armed() && f.port.requests.size() == 2 && f.port.last().command == ProposedCommand::Stop, "safety loss STOPs in same evaluation, bypasses ACK/dwell and disarms");
        f.input = ready(21); f.policy.evaluate(f.input);
        expect(!f.policy.armed() && f.port.requests.size() == 2, "safety recovery never implicitly rearms");
    }
    Fixture disconnected; disconnected.start(); disconnected.input.linkConnected = false; disconnected.evaluate(20); disconnected.evaluate(5000);
    expect(disconnected.port.requests.size() == 2 && disconnected.policy.stopTimeoutCount() == 0, "link loss stops STOP retry and timeout counting");
    disconnected.input = ready(5001); disconnected.input.enabledMask = 0; disconnected.input.poseKnownMask = 0;
    expect(disconnected.policy.arm(disconnected.input) == ArmReason::ServosNotEnabled, "after disconnect enabled/pose clearing blocks rearm");
}

void targetLossRecoveryAndStopGate()
{
    for (auto state : {VisualState::Lost}) {
        Fixture f; f.start(); f.ack();
        f.input.state = state; f.evaluate(100);
        const auto stop = f.port.last();
        expect(f.policy.armed() && stop.command == ProposedCommand::Stop, "target loss STOP keeps armed");
        expect(f.policy.currentMode() == ProposedCommand::Forward, "STOP submission does not update mode before OK");
        f.evaluate(101); f.input.state = VisualState::Lost; f.evaluate(102);
        expect(f.port.requests.size() == 2, "continuous LOST stop episode sends only once");
        f.policy.acknowledge(stop.id, DispatchOutcome::Ok, 110);
        expect(f.policy.currentMode() == ProposedCommand::Stop, "STOP OK confirms current mode");
        f.input.state = VisualState::Tracking; f.evaluate(300);
        expect(f.port.requests.size() == 2, "TRACKING recovery during return-to-zero cannot start");
        f.evaluate(1109); expect(f.port.requests.size() == 2, "STOP accepted +999 blocks START");
        f.evaluate(1110); expect(f.port.requests.size() == 3 && f.port.last().command == ProposedCommand::Forward, "STOP accepted +1000 resumes same Forward suggestion");
        f.ack(1111); f.input.state = state; f.evaluate(1120);
        expect(f.port.requests.size() == 4 && f.port.last().command == ProposedCommand::Stop, "fresh stop episode sends again despite dwell");
    }
}

void noTargetHoldsConfirmedModeOnly()
{
    Fixture loss; loss.start(); loss.ack();
    loss.input.state = VisualState::NoTarget;
    loss.input.suggestion = ProposedCommand::Stop; // State takes precedence over suggestion.
    for (auto time : {100, 500, 1599}) {
        loss.evaluate(time);
        expect(loss.port.requests.size() == 1 && loss.policy.armed() &&
                   loss.policy.currentMode() == ProposedCommand::Forward,
               "confirmed Forward holds through 1499 ms NO_TARGET without any send");
    }
    loss.input.state = VisualState::Lost; loss.input.suggestion = ProposedCommand::Hold; loss.evaluate(1600);
    expect(loss.port.requests.size() == 2 && loss.port.last().command == ProposedCommand::Stop &&
               loss.policy.armed(), "only LOST ends the grace period with STOP and preserves arming");

    Fixture recover; recover.start(); recover.ack();
    recover.input.state = VisualState::NoTarget; recover.turn(); recover.evaluate(100);
    recover.evaluate(1599);
    recover.input.state = VisualState::Tracking; recover.input.suggestion = ProposedCommand::Forward;
    recover.evaluate(1600);
    expect(recover.port.requests.size() == 1 && recover.policy.armed(),
           "NO_TARGET recovery to unchanged confirmed Forward never resends");

    for (bool noTarget : {true, false}) {
        Fixture fresh;
        expect(fresh.policy.arm(fresh.input) == ArmReason::Ready, "fresh eligible arming has no assumed confirmed mode");
        fresh.input.state = noTarget ? VisualState::NoTarget : VisualState::Tracking;
        fresh.input.suggestion = ProposedCommand::Hold;
        fresh.evaluate(10);
        expect(fresh.port.requests.size() == 1 && fresh.port.last().command == ProposedCommand::Stop && fresh.policy.armed(),
               "NO_TARGET/HOLD without any confirmed mode immediately STOPs, never guesses");

        Fixture manual; manual.start(); manual.ack(); manual.policy.manualInput(ManualInputKind::Stop);
        manual.input = ready(10); expect(manual.policy.arm(manual.input) == ArmReason::Ready, "manual takeover requires explicit rearm");
        manual.input.state = noTarget ? VisualState::NoTarget : VisualState::Tracking;
        manual.input.suggestion = ProposedCommand::Hold; manual.evaluate(11);
        expect(manual.port.requests.size() == 2 && manual.port.last().command == ProposedCommand::Stop,
               "manual rearm invalidates old confirmed mode: NO_TARGET/HOLD STOP bypasses dwell");

        Fixture reconnect; reconnect.start(); reconnect.ack(); reconnect.input.linkConnected = false; reconnect.evaluate(10);
        reconnect.input = ready(11); expect(reconnect.policy.arm(reconnect.input) == ArmReason::Ready, "fresh post-disconnect evidence permits explicit rearm");
        reconnect.input.state = noTarget ? VisualState::NoTarget : VisualState::Tracking;
        reconnect.input.suggestion = ProposedCommand::Hold; reconnect.evaluate(12);
        expect(reconnect.port.requests.size() == 3 && reconnect.port.last().command == ProposedCommand::Stop,
               "link-loss rearm cannot HOLD old confirmed mode: sends immediate STOP");
    }
}

void noTargetStillEnforcesPendingMotionTimeout()
{
    Fixture f; f.start(); f.ack(); f.turn(); f.evaluate(1000);
    const auto motion = f.port.last();
    f.input.state = VisualState::NoTarget; f.evaluate(1001); f.evaluate(1999);
    expect(f.port.requests.size() == 2 && f.policy.armed(), "NO_TARGET holds confirmed mode before pending motion deadline");
    f.evaluate(2000);
    expect(f.port.requests.size() == 3 && f.port.last().command == ProposedCommand::Stop && !f.policy.armed(),
           "pending motion timeout during NO_TARGET immediately STOPs and disarms");
    f.policy.acknowledge(motion.id, DispatchOutcome::Ok, 2001);
    expect(f.policy.currentMode() == ProposedCommand::Forward, "late motion OK after timeout STOP cannot replace confirmed Forward");
}

void disconnectedStopDoesNotSurviveReconnect()
{
    Fixture f; f.start(); f.ack(); f.input.state = VisualState::Lost; f.evaluate(10);
    const auto oldStop = f.port.last();
    f.input.linkConnected = false; f.input.controlOwned = false; f.evaluate(20);
    f.input = ready(2000); // Reconnected + reacquired authority, not automatically armed.
    f.policy.evaluate(f.input);
    f.policy.acknowledge(oldStop.id, DispatchOutcome::Ok, 2001);
    f.evaluate(4000);
    expect(f.port.requests.size() == 2 && !f.policy.armed() &&
               f.policy.currentMode() == ProposedCommand::Forward,
           "reconnect past STOP timeout never revives its retries or accepts old STOP OK");
}

void manualTakeover()
{
    for (auto kind : {ManualInputKind::Motion, ManualInputKind::Stop}) for (bool pending : {false, true}) {
        Fixture f; f.start(); const auto old = f.port.last();
        if (!pending) { f.ack(); f.turn(); f.evaluate(100); }
        f.policy.manualInput(kind);
        expect(!f.policy.armed(), "manual motion including STOP immediately disarms");
        f.policy.acknowledge(old.id, DispatchOutcome::Ok, 101);
        f.evaluate(3000);
        expect(f.port.requests.size() == 1 && !f.policy.armed(), "manual takeover blocks sends despite late ACK and expired dwell");
    }
    Fixture ordinary; ordinary.start(); ordinary.ack(); ordinary.policy.manualInput(ManualInputKind::NonMotion); ordinary.turn(); ordinary.evaluate(1000);
    expect(ordinary.policy.armed() && ordinary.port.requests.size() == 2, "non-motion manual input does not take over");
    Fixture retries; retries.start(); retries.input.state = VisualState::Stale; retries.evaluate(20); const auto stop = retries.port.last();
    retries.policy.manualInput(ManualInputKind::Stop); retries.policy.acknowledge(stop.id, DispatchOutcome::Ok, 21); retries.evaluate(10000);
    expect(retries.port.requests.size() == 2 && !retries.policy.currentMode(), "manual STOP takeover cancels all automatic STOP retries and ACK associations");
}

void nonStopUnknownAndTimeout()
{
    Fixture late; late.start(); const auto expiredMotion = late.port.last();
    late.policy.acknowledge(expiredMotion.id, DispatchOutcome::Ok, 1000);
    expect(!late.policy.armed() && late.port.requests.size() == 2 &&
               late.port.last().command == ProposedCommand::Stop && !late.policy.currentMode(),
           "late OK arriving at expired non-STOP deadline cannot bypass timeout STOP");
    Fixture timeout; timeout.start(); timeout.evaluate(999);
    expect(timeout.port.requests.size() == 1 && !timeout.policy.currentMode(), "unacknowledged non-STOP not timed out at 999 ms");
    timeout.evaluate(1000);
    expect(timeout.port.requests.size() == 2 && timeout.port.last().command == ProposedCommand::Stop && !timeout.policy.armed(), "non-STOP timeout sends STOP immediately and disarms");
    expect(!timeout.policy.currentMode(), "timeout never confirms a motion mode");
    Fixture unknown; unknown.start(); unknown.ack(10, DispatchOutcome::OutcomeUnknown);
    expect(!unknown.policy.armed() && unknown.port.requests.size() == 2 && unknown.port.last().command == ProposedCommand::Stop && !unknown.policy.currentMode(), "OutcomeUnknown immediately STOPs and disarms without confirming mode");
    Fixture rejected; rejected.start(); rejected.ack(10, DispatchOutcome::Rejected);
    expect(!rejected.policy.armed() && rejected.port.last().command == ProposedCommand::Stop, "other non-STOP rejection fails closed");
}

void stopRetriesAnyOkAndStaleMotion()
{
    for (bool ackFirst : {true, false}) {
        Fixture f; f.start(); const auto motion = f.port.last();
        f.input.state = VisualState::Lost; f.evaluate(10); const auto first = f.port.last();
        f.policy.acknowledge(motion.id, DispatchOutcome::Ok, 20);
        expect(!f.policy.currentMode(), "STOP invalidates earlier non-STOP late OK");
        f.evaluate(1009); expect(f.port.requests.size() == 2, "STOP timeout waits its full configured interval");
        f.evaluate(1010); const auto retry = f.port.last();
        expect(f.port.requests.size() == 3 && retry.command == ProposedCommand::Stop && retry.id != first.id, "STOP timeout resends with distinct request ID");
        expect(f.policy.stopTimeoutCount() == 1 && !f.policy.stopTimeoutAlert(), "first STOP timeout is counted without alert");
        f.policy.acknowledge(ackFirst ? first.id : retry.id, DispatchOutcome::Ok, 1020);
        expect(f.policy.currentMode() == ProposedCommand::Stop, "any OK for same STOP episode confirms it");
        f.policy.acknowledge(motion.id, DispatchOutcome::Ok, 1021);
        f.evaluate(2019); expect(f.port.requests.size() == 3, "OK ends retries, stale motion cannot overwrite STOP");
        f.policy.acknowledge(ackFirst ? retry.id : first.id, DispatchOutcome::Ok, 2019);
        f.input.state = VisualState::Tracking; f.evaluate(2020);
        expect(f.port.requests.size() == 4 && f.port.last().command == ProposedCommand::Forward, "duplicate episode OK cannot extend accepted STOP dwell deadline");
    }
    Fixture alert; alert.start(); alert.input.state = VisualState::Stale; alert.evaluate(10);
    alert.evaluate(1010); alert.evaluate(2010); alert.evaluate(3010);
    expect(alert.port.requests.size() == 5 && alert.policy.stopTimeoutCount() == 3 && alert.policy.stopTimeoutAlert(), "three consecutive STOP timeouts expose latched operator alert");
    alert.ack(3011); alert.evaluate(10000);
    expect(alert.port.requests.size() == 5, "STOP OK ends retry even after safety disarm");
    for (bool authority : {true, false}) {
        Fixture loss; loss.start(); loss.input.state = VisualState::Lost; loss.evaluate(10);
        if (authority) loss.input.controlOwned = false; else loss.input.linkConnected = false;
        loss.evaluate(1010); loss.evaluate(9000);
        expect(loss.port.requests.size() == 2 && !loss.policy.armed(), "link or authority loss terminates existing STOP retries");
    }
    Fixture configured({200, 0x001b}); configured.start(); configured.input.state = VisualState::Lost; configured.evaluate(10); configured.evaluate(209);
    expect(configured.port.requests.size() == 2, "custom STOP timeout waits boundary minus one");
    configured.evaluate(210); expect(configured.port.requests.size() == 3, "custom STOP timeout applies at boundary");
}

void directionMappingAndHold()
{
    for (int sign : {1, -1}) for (double error : {0.5, -0.5}) for (auto suggestion : {ProposedCommand::TurnLeft, ProposedCommand::TurnRight}) {
        Fixture f; f.input.confirmedTurnSign = sign; f.input.ex = error; f.input.suggestion = suggestion;
        expect(f.policy.arm(f.input) == ArmReason::Ready, "explicitly confirmed +1/-1 direction arms");
        f.policy.evaluate(f.input);
        expect(f.port.last().command == (error * sign > 0 ? ProposedCommand::TurnRight : ProposedCommand::TurnLeft), "direction derives from sign(ex) times confirmed turn_sign, not old suggestion mapping");
    }
    Fixture hold; hold.start(); hold.ack(); hold.input.suggestion = ProposedCommand::Hold; hold.evaluate(1000);
    expect(hold.port.requests.size() == 1, "HOLD emits no command");
    for (double error : {0.0, std::numeric_limits<double>::quiet_NaN()}) {
        Fixture invalid; invalid.start(); invalid.ack(); invalid.turn(error); invalid.evaluate(1000);
        expect(!invalid.policy.armed() && invalid.port.last().command == ProposedCommand::Stop, "invalid turning error fails closed");
    }
}

void additionalAckAndStopBoundaries()
{
    for (bool authority : {true, false}) {
        Fixture lostArm; lostArm.start(); lostArm.input.state = VisualState::Stale; lostArm.evaluate(10);
        const auto oldStop = lostArm.port.last();
        auto loss = ready(11);
        if (authority) loss.controlOwned = false; else loss.linkConnected = false;
        expect(lostArm.policy.arm(loss) == (authority ? ArmReason::ControlNotOwned : ArmReason::LinkDisconnected),
               "disarmed pending-STOP arm attempt still rejects lost link/authority");
        lostArm.policy.acknowledge(oldStop.id, DispatchOutcome::Ok, 12);
        lostArm.input = ready(2000); lostArm.policy.evaluate(lostArm.input);
        expect(!lostArm.policy.currentMode() && lostArm.port.requests.size() == 2,
               "loss through rejected arm must cancel disarmed STOP ACK and retries");
    }
    Fixture failedArmClock; failedArmClock.start();
    auto staleArm = ready(500); staleArm.state = VisualState::Stale;
    expect(failedArmClock.policy.arm(staleArm) == ArmReason::NotTracking,
           "unsafe arm attempt refuses TRACKING requirement");
    expect(failedArmClock.policy.arm(ready(100)) == ArmReason::InvalidTime && !failedArmClock.policy.armed(),
           "rejected arm observes valid clock before safety STOP so time cannot go backwards");
    Fixture reversedTime; reversedTime.input.nowMs = 100; reversedTime.start();
    auto earlier = reversedTime.input; earlier.nowMs = 99;
    expect(reversedTime.policy.arm(earlier) == ArmReason::InvalidTime &&
               !reversedTime.policy.armed() && reversedTime.port.last().command == ProposedCommand::Stop,
           "already-armed decreasing-time arm request must fail closed");
    Fixture lateBusy; lateBusy.start(); lateBusy.ack(1000, DispatchOutcome::Busy);
    expect(!lateBusy.policy.armed() && lateBusy.port.last().command == ProposedCommand::Stop &&
               !lateBusy.policy.currentMode(),
           "late Busy cannot bypass a non-STOP timeout");
    Fixture stopBusy; stopBusy.start(); stopBusy.ack();
    stopBusy.input.state = VisualState::Lost; stopBusy.evaluate(100);
    stopBusy.ack(101, DispatchOutcome::Busy);
    expect(stopBusy.policy.currentMode() == ProposedCommand::Forward,
           "STOP Busy never updates current mode");
    stopBusy.evaluate(1099);
    expect(stopBusy.port.requests.size() == 2, "STOP Busy cannot trigger immediate retry");
    stopBusy.evaluate(1100);
    expect(stopBusy.port.requests.size() == 3 && stopBusy.policy.currentMode() == ProposedCommand::Forward,
           "STOP timeout resends but still does not update mode");

    Fixture pendingStop; pendingStop.start(); pendingStop.ack();
    pendingStop.input.state = VisualState::Lost; pendingStop.evaluate(100);
    const auto firstStop = pendingStop.port.last();
    pendingStop.input.state = VisualState::Tracking; pendingStop.turn(); pendingStop.evaluate(1099);
    expect(pendingStop.port.requests.size() == 2, "pending STOP ACK blocks recovered non-STOP after previous dwell");
    pendingStop.evaluate(1100);
    expect(pendingStop.port.requests.size() == 3 && pendingStop.port.last().command == ProposedCommand::Stop,
           "recovery while STOP unconfirmed retries only STOP");
    pendingStop.policy.acknowledge(firstStop.id, DispatchOutcome::Ok, 1101);
    pendingStop.evaluate(2100); expect(pendingStop.port.requests.size() == 3, "conservative STOP ACK receipt gate blocks +999");
    pendingStop.evaluate(2101);
    expect(pendingStop.port.requests.size() == 4 && pendingStop.port.last().command == ProposedCommand::TurnRight,
           "recovery sends latest turn at STOP accepted +1000");
    pendingStop.ack(2102); pendingStop.turn(-0.5); pendingStop.evaluate(3100);
    expect(pendingStop.port.requests.size() == 4, "max gate also enforces last non-STOP send +999");
    pendingStop.evaluate(3101); expect(pendingStop.port.requests.size() == 5, "max gate permits last non-STOP send +1000");

    Fixture staleEpisode; staleEpisode.start(); staleEpisode.input.state = VisualState::Lost; staleEpisode.evaluate(10);
    const auto oldStop = staleEpisode.port.last(); staleEpisode.ack(11);
    staleEpisode.input.state = VisualState::Tracking; staleEpisode.evaluate(1011); staleEpisode.ack(1012);
    staleEpisode.input.state = VisualState::Lost; staleEpisode.evaluate(1020);
    staleEpisode.policy.acknowledge(oldStop.id, DispatchOutcome::Ok, 1021);
    staleEpisode.evaluate(2020);
    expect(staleEpisode.port.requests.size() == 5, "OK from preceding STOP episode cannot confirm current episode");

    Fixture noBurst; noBurst.start(); noBurst.input.state = VisualState::Stale; noBurst.evaluate(10);
    noBurst.evaluate(10000); noBurst.evaluate(10000);
    expect(noBurst.port.requests.size() == 3 && noBurst.policy.stopTimeoutCount() == 1,
           "long evaluation pause never bursts multiple STOP retries");
    Fixture directionLost; directionLost.start(); directionLost.ack(); directionLost.input.confirmedTurnSign.reset(); directionLost.evaluate(20);
    expect(!directionLost.policy.armed() && directionLost.port.last().command == ProposedCommand::Stop,
           "loss of explicit direction confirmation while armed fails closed");
}
} // namespace

int main()
{
    armingConditions(); ackModeDwellAndBusy(); disarmingSafety();
    targetLossRecoveryAndStopGate(); manualTakeover(); nonStopUnknownAndTimeout();
    noTargetHoldsConfirmedModeOnly(); noTargetStillEnforcesPendingMotionTimeout(); disconnectedStopDoesNotSurviveReconnect();
    stopRetriesAnyOkAndStaleMotion(); directionMappingAndHold();
    additionalAckAndStopBoundaries();
    std::printf("Visual dispatch: %d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
