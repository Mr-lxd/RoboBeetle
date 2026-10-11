#include "vision/VisualApproachPolicy.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace rb::vision;
namespace {
int failures = 0;
void check(bool value, const char *name) { if (!value) { ++failures; std::cerr << "FAIL: " << name << '\n'; } }
bool near(double a, double b) { return std::abs(a-b) < 1e-9; }
VisualApproachResult step(const VisualApproachMemory &m, std::int64_t now, std::uint64_t frame,
                          std::optional<double> area, const VisualApproachConfig &c,
                          bool available = true, std::uint64_t session = 1) {
    return evaluateVisualApproach(m, {now,session,frame,area,available,500}, c);
}
void configAndMapping() {
    VisualApproachConfig c;
    check(validVisualApproachConfig(c), "provisional defaults valid");
    check(c.slowArea == 8 && c.resumeArea == 14 && c.stopArea == 20, "review defaults");
    auto bad = c; bad.alphaArea = 0; check(!validVisualApproachConfig(bad), "alpha zero rejected");
    bad = c; bad.stopArea = 577; check(!validVisualApproachConfig(bad), "area beyond grid rejected");
    bad = c; bad.resumeArea = bad.stopArea; check(!validVisualApproachConfig(bad), "ordered hysteresis required");
    bad = c; bad.tTurnMin = .61; check(!validVisualApproachConfig(bad), "turn floor within max");
    bad = c; bad.kTurn = std::numeric_limits<double>::quiet_NaN(); check(!validVisualApproachConfig(bad), "NaN gain rejected");
    bad = c; bad.resumeDwellMs = -1; check(!validVisualApproachConfig(bad), "negative dwell rejected");
    check(mapVisualSetpoint(ProposedCommand::Forward, 0,c).throttle == 600, "low segment");
    check(mapVisualSetpoint(ProposedCommand::Forward, 8,c).throttle == 600, "slow inclusive");
    check(mapVisualSetpoint(ProposedCommand::Forward, 14,c).throttle == 375, "linear segment");
    check(mapVisualSetpoint(ProposedCommand::Forward, 20,c).throttle == 150, "stop inclusive before dwell");
    check(mapVisualSetpoint(ProposedCommand::Forward, 576,c).throttle == 150, "upper segment");
    auto left = mapVisualSetpoint(ProposedCommand::TurnLeft, 20,c);
    check(left.throttle == 300 && left.turn == -1000 && left.pitch == 0, "left sign and turn floor");
    auto right = mapVisualSetpoint(ProposedCommand::TurnRight, 8,c);
    check(right.throttle == 600 && right.turn == 1000 && right.pitch == 0, "right sign");
    auto up = mapVisualSetpoint(ProposedCommand::Ascend, 20,c);
    check(up.throttle == 150 && up.turn == 0 && up.pitch == 500, "ascend sign without yaw floor");
    auto down = mapVisualSetpoint(ProposedCommand::Descend, 20,c);
    check(down.pitch == -500 && down.turn == 0, "descend sign mutually exclusive axes");
    auto stopped = mapVisualSetpoint(ProposedCommand::Stop, 8,c);
    check(stopped.throttle == 0 && stopped.turn == 0 && stopped.pitch == 0, "stop zero");
    auto arrived = mapVisualSetpoint(ProposedCommand::TurnLeft, 8,c,true);
    check(arrived.throttle == 0 && arrived.turn == 0 && arrived.pitch == 0, "arrived eccentric zero");
    check(mapVisualSetpoint(ProposedCommand::Forward, std::numeric_limits<double>::quiet_NaN(),c).throttle == 0, "nonfinite area safe zero");
}
void firstFrameAndEma() {
    VisualApproachConfig c;
    auto r = step({},0,1,20,c);
    check(r.arrivedEnter && r.next.state == ApproachState::Arrived, "first at stop immediate arrived");
    r = step({},0,1,8,c);
    check(r.observationUsed && r.area_f && r.area_f && near(*r.area_f,8), "first value exact");
    r = step(r.next,50,1,20,c);
    check(!r.observationUsed && r.area_f && near(*r.area_f,8), "same frame no EMA repeat");
    r = step(r.next,100,2,18,c);
    check(r.observationUsed && r.area_f && near(*r.area_f,11), "next frame EMA once");
    r = step(r.next,150,3,20,c,false);
    check(!r.observationUsed && r.area_f && near(*r.area_f,11), "awaiting video ignores value");
    r = step(r.next,200,3,20,c);
    check(r.observationUsed && r.area_f && near(*r.area_f,13.7), "same awaited frame consumed on availability");
    r = step(r.next,250,3,20,c,true,2);
    check(r.arrivedEnter && r.area_f && near(*r.area_f,20), "new session clears EMA and immediate arrive");
    r = step({},300,1,21,c);
    check(r.arrivedEnter, "fresh memory reacquisition immediate arrive");
}
void dwellAndHysteresis() {
    VisualApproachConfig c; c.alphaArea=1;
    auto r = step({},0,1,8,c);
    r = step(r.next,100,2,20,c);
    check(!r.arrivedEnter, "later threshold starts dwell");
    r = step(r.next,1099,3,20,c);
    check(r.next.state == ApproachState::Approach, "arrive before exact dwell");
    r = step(r.next,1100,3,20,c);
    check(r.arrivedEnter, "same fresh frame completes exact arrive dwell");
    r = step(r.next,1200,4,14,c);
    check(!r.next.resumeSinceMs, "resume strict threshold");
    r = step(r.next,1300,5,13,c);
    r = step(r.next,2299,6,13,c);
    check(r.next.state == ApproachState::Arrived, "resume before exact dwell");
    r = step(r.next,2300,6,13,c);
    check(r.arrivedExit && r.next.state == ApproachState::Approach, "pure resume exact dwell");
    r = step(r.next,2400,7,20,c);
    r = step(r.next,2500,8,19,c);
    check(!r.next.arriveSinceMs, "below stop breaks arrive dwell");
    r = step(r.next,2600,9,20,c);
    r = step(r.next,3000,10,std::nullopt,c);
    check(!r.next.arriveSinceMs, "missing resets arrive candidate");
    r = step(r.next,3500,11,20,c);
    check(!r.arrivedEnter, "missing interrupts continuity");
    r = step(r.next,4500,11,20,c);
    check(r.arrivedEnter, "new continuous dwell after missing");
    r = step(r.next,4600,12,13,c);
    r = step(r.next,4700,12,std::nullopt,c,false);
    check(!r.next.resumeSinceMs, "no target resets resume dwell");
    r = step(r.next,5600,13,13,c);
    check(!r.arrivedExit, "no target interrupts resume continuity");
}
void missingArea() {
    VisualApproachConfig c;
    auto r = step({},0,1,10,c);
    r = step(r.next,100,2,std::nullopt,c);
    check(r.area_f && r.area_f && near(*r.area_f,10) && r.next.missingSinceMs == 100, "missing preserves EMA starts timer");
    r = step(r.next,600,2,std::nullopt,c);
    check(!r.areaMissingLost && r.area_f, "500ms inclusive grace");
    r = step(r.next,601,2,std::nullopt,c);
    check(r.areaMissingLost && !r.area_f && r.next.state == ApproachState::Approach, "over 500 loses clears area");
    r = step(r.next,1000,2,std::nullopt,c);
    check(!r.areaMissingLost, "missing lost event emitted once");
    r = step(r.next,1100,3,20,c);
    check(r.arrivedEnter && r.area_f && near(*r.area_f,20), "missing recovery first value immediate arrive");
    r = step(r.next,1200,4,std::numeric_limits<double>::quiet_NaN(),c);
    check(r.next.missingSinceMs == 1200, "invalid selected area treated missing");
    r = step(r.next,1300,5,14,c);
    check(!r.next.missingSinceMs && !r.next.areaMissingLost, "valid area clears missing episode");
    r = step({},0,1,std::nullopt,c,false);
    r = step(r.next,2000,2,std::nullopt,c,false);
    check(!r.areaMissingLost && !r.next.missingSinceMs, "no target never starts area missing timer");
    r = step({},0,1,10,c);
    r = step(r.next,100,2,std::nullopt,c);
    r = step(r.next,700,3,std::nullopt,c,false);
    check(!r.areaMissingLost && !r.next.missingSinceMs, "no target suspends selected missing episode");
}
}
int main() {
    configAndMapping(); firstFrameAndEma(); dwellAndHysteresis(); missingArea();
    if (failures) { std::cerr << failures << " checks failed\n"; return EXIT_FAILURE; }
    std::cout << "PASS visual_approach_policy_tests\n"; return EXIT_SUCCESS;
}

