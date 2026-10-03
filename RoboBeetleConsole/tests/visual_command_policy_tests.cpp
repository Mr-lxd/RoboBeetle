#include "vision/VisualCommandPolicy.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

namespace {
using namespace rb::vision;
int failures = 0;
int checks = 0;

void expect(bool condition, const char *message)
{
    ++checks;
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

bool near(std::optional<double> value, double expected)
{
    return value && std::abs(*value - expected) < 1e-12;
}

VisualPolicyConfig directConfig()
{
    VisualPolicyConfig config;
    config.alpha = 1.0;
    config.min_dwell_ms = 0;
    return config;
}

VisualCommandResult track(const VisualCommandMemory &memory, std::int64_t time,
                          std::uint64_t frame, double ex, const VisualPolicyConfig &config)
{
    return evaluateVisualCommand(memory, {time, VisualState::Tracking, frame, ex}, config);
}

void initialSampleAndPerFrameEma()
{
    VisualPolicyConfig config;
    auto result = track({}, 0, 0, 0.0, config);
    expect(near(result.ex_f, 0.0) && result.effective == ProposedCommand::Forward,
           "first sample including ID zero initializes EMA and accepts first suggestion");
    expect(result.next.lastSwitchMs == 0 && result.next.lastFilteredFrameId == 0,
           "first effective change records time and consumed frame ID");
    result = track(result.next, 10, 1, 1.0, config);
    expect(near(result.ex_f, 0.3), "second sample applies alpha once");
    expect(result.dwellBlocked && result.effective == ProposedCommand::Forward
           && result.proposed == ProposedCommand::Forward && std::abs(result.yaw_cmd - 0.3) < 1e-12,
           "dwell retains discrete suggestion while continuous yaw advances");
    const auto unchanged = result.next;
    result = track(result.next, 20, 1, -1.0, config);
    expect(near(result.ex_f, 0.3) && result.next.lastFilteredFrameId == 1,
           "same ID never resamples even if supplied error changes");
    result = track(result.next, 30, 0, -1.0, config);
    expect(near(result.ex_f, 0.3) && result.next.lastFilteredFrameId == 1,
           "older ID never resamples or lowers consumed ID floor");
    expect(result.next.lastSwitchMs == unchanged.lastSwitchMs,
           "duplicate or unchanged outputs do not reset switch time");
    result = track(result.next, 40, 2, 1.0, config);
    expect(near(result.ex_f, 0.51), "third new sample applies EMA to previous filtered error");
    result = track(result.next, 1000, 2, 1.0, config);
    expect(result.effective == ProposedCommand::TurnRight && !result.dwellBlocked
           && near(result.ex_f, 0.51), "timer evaluation can release dwell without resampling");
    const auto nonzero = track({}, 0, 3, -0.8, config);
    expect(near(nonzero.ex_f, -0.8) && nonzero.effective == ProposedCommand::TurnLeft,
           "first nonzero sample initializes directly and first turn bypasses dwell");
}

void strictHysteresisBothDirections()
{
    const auto config = directConfig();
    struct Case { double ex; ProposedCommand expected; };
    const Case cases[] = {
        {0.24, ProposedCommand::Forward}, {0.25, ProposedCommand::Forward},
        {0.26, ProposedCommand::TurnRight}, {0.20, ProposedCommand::TurnRight},
        {0.12, ProposedCommand::TurnRight}, {-0.25, ProposedCommand::TurnRight},
        {-0.26, ProposedCommand::TurnLeft}, {-0.20, ProposedCommand::TurnLeft},
        {-0.12, ProposedCommand::TurnLeft}, {0.25, ProposedCommand::TurnLeft},
        {0.26, ProposedCommand::TurnRight}, {0.119, ProposedCommand::Forward},
        {-0.25, ProposedCommand::Forward}, {-0.26, ProposedCommand::TurnLeft},
        {-0.119, ProposedCommand::Forward},
    };
    VisualCommandMemory memory;
    std::uint64_t frame = 0;
    for (const auto &item : cases) {
        const auto result = track(memory, static_cast<std::int64_t>(frame), frame, item.ex, config);
        expect(result.effective == item.expected && result.proposed == item.expected,
               "strict hysteresis table respects direction and on/off equality");
        expect(!result.dwellBlocked, "zero dwell never blocks a valid ordinary change");
        memory = result.next;
        ++frame;
    }
}

void dwellAndHolding()
{
    auto config = directConfig();
    config.min_dwell_ms = 1000;
    const auto first = track({}, 0, 10, 0.8, config);
    auto result = track(first.next, 999, 11, -0.8, config);
    expect(result.effective == ProposedCommand::TurnRight && result.dwellBlocked
           && result.next.lastSwitchMs == 0, "opposite turn blocked just before dwell deadline");
    result = track(result.next, 1000, 11, -0.8, config);
    expect(result.effective == ProposedCommand::TurnLeft && !result.dwellBlocked
           && result.next.lastSwitchMs == 1000, "opposite turn accepted exactly at dwell deadline");
    result = track(result.next, 1999, 12, 0.0, config);
    expect(result.effective == ProposedCommand::TurnLeft && result.dwellBlocked,
           "turn to forward uses ordinary dwell");
    result = track(result.next, 2000, 12, 0.0, config);
    expect(result.effective == ProposedCommand::Forward && result.next.lastSwitchMs == 2000,
           "turn to forward accepted at dwell deadline");
    const auto held = evaluateVisualCommand(first.next,
        {500, VisualState::NoTarget, 100, std::numeric_limits<double>::quiet_NaN()}, config);
    expect(held.proposed == ProposedCommand::Hold && held.effective == ProposedCommand::TurnRight,
           "NO_TARGET proposes HOLD retaining actual effective suggestion");
    expect(near(held.ex_f, 0.8) && std::abs(held.yaw_cmd - 0.8) < 1e-12
           && held.next.lastFilteredFrameId == 10 && held.next.lastSwitchMs == 0,
           "HOLD retains EMA yaw consumed ID and switch time without reading error");
    expect(held.next.lastEvaluatedMs == 500 && !held.dwellBlocked,
           "HOLD advances evaluation time without a dwell change");
    const auto recovered = track(held.next, 1000, 11, -0.8, config);
    expect(recovered.effective == ProposedCommand::TurnLeft,
           "HOLD does not postpone the ordinary switch deadline");
    const auto initialHold = evaluateVisualCommand({}, {0, VisualState::NoTarget, {}, {}}, config);
    expect(initialHold.proposed == ProposedCommand::Hold && initialHold.effective == ProposedCommand::Stop
           && !initialHold.ex_f && initialHold.yaw_cmd == 0.0 && !initialHold.next.lastSwitchMs,
           "initial HOLD retains default STOP and unknown filter");
}

void priorityStopsAndNewSampleRecovery()
{
    auto config = directConfig();
    config.min_dwell_ms = 1000;
    const auto first = track({}, 0, 10, 0.8, config);
    for (const auto state : {VisualState::Stale, VisualState::InferenceOff, VisualState::Lost}) {
        auto result = evaluateVisualCommand(first.next, {1, state, 11, -0.8}, config);
        expect(result.proposed == ProposedCommand::Stop && result.effective == ProposedCommand::Stop
               && !result.dwellBlocked && !result.ex_f && result.yaw_cmd == 0.0,
               "all priority stop states bypass dwell and clear filtered output");
        expect(result.next.lastFilteredFrameId == 10 && result.next.lastSwitchMs == 1
               && result.next.lastEvaluatedMs == 1, "priority STOP retains consumed ID and records actual switch");
        result = evaluateVisualCommand(result.next, {2, state, {}, {}}, config);
        expect(result.next.lastSwitchMs == 1, "repeated priority STOP never resets switch time");
        result = track(result.next, 1001, 10, 0.8, config);
        expect(result.effective == ProposedCommand::Stop && result.proposed == ProposedCommand::Stop
               && result.waitingForSample && !result.ex_f && result.yaw_cmd == 0.0,
               "consumed ID cannot reinitialize a cleared filter after dwell expires");
        result = track(result.next, 1002, 9, -0.8, config);
        expect(result.waitingForSample && !result.ex_f && result.next.lastFilteredFrameId == 10,
               "older ID remains waiting after STOP without lowering floor");
        result = track(result.next, 1003, 11, -0.8, config);
        expect(result.effective == ProposedCommand::TurnLeft && near(result.ex_f, -0.8)
               && !result.waitingForSample, "new ID reinitializes EMA directly after STOP");
    }
    auto stopped = evaluateVisualCommand(first.next, {1, VisualState::Stale, {}, {}}, config);
    auto recovering = track(stopped.next, 2, 11, -0.8, config);
    expect(recovering.effective == ProposedCommand::Stop && recovering.dwellBlocked
           && near(recovering.ex_f, -0.8) && recovering.next.lastSwitchMs == 1,
           "new sample recovery respects dwell since latest priority STOP");
    recovering = track(recovering.next, 1000, 11, -0.8, config);
    expect(recovering.effective == ProposedCommand::Stop && recovering.dwellBlocked,
           "recovery remains blocked one millisecond before STOP dwell deadline");
    recovering = track(recovering.next, 1001, 11, -0.8, config);
    expect(recovering.effective == ProposedCommand::TurnLeft && !recovering.dwellBlocked,
           "recovery switches exactly at STOP dwell deadline without extra EMA");
    const auto reset = track({}, 1002, 0, -0.5, config);
    expect(reset.effective == ProposedCommand::TurnLeft && near(reset.ex_f, -0.5),
           "explicit memory reset accepts low ID and discards old dwell");
}

void imageYawSignAndSaturation()
{
    auto config = directConfig();
    config.turn_sign = -1;
    config.K_yaw = 2.0;
    for (const double ex : {-0.8, 0.8}) {
        const auto result = track({}, 0, 0, ex, config);
        expect(result.effective == (ex > 0 ? ProposedCommand::TurnLeft : ProposedCommand::TurnRight),
               "turn_sign changes only discrete turn direction");
        expect(result.yaw_cmd == (ex > 0 ? 1.0 : -1.0) && near(result.ex_f, ex),
               "yaw saturates in image sign independent of turn_sign");
    }
    config.K_yaw = 0.0;
    expect(track({}, 0, 0, 0.8, config).yaw_cmd == 0.0, "zero gain keeps discrete turn but yaw zero");
    config.K_yaw = std::numeric_limits<double>::max();
    expect(track({}, 0, 0, 1.0, config).yaw_cmd == 1.0, "large finite gain remains clamped");
}

void invalidInputsAndTime()
{
    const auto config = directConfig();
    const auto first = track({}, 100, 10, 0.8, config);
    for (const double ex : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity(), -1.01, 1.01}) {
        const auto result = track(first.next, 101, 11, ex, config);
        expect(result.effective == ProposedCommand::Stop && !result.ex_f && result.yaw_cmd == 0.0,
               "nonfinite or out of normalized range input yields STOP with no filter");
        expect(result.next.lastFilteredFrameId == 10, "invalid sample never consumes a new frame ID");
    }
    for (const auto time : {-1LL, 99LL}) {
        const auto result = track(first.next, time, 11, -0.8, config);
        expect(result.effective == ProposedCommand::Stop && !result.ex_f
               && result.next.lastEvaluatedMs == 100 && result.next.lastSwitchMs == 100,
               "negative or backward time stops without advancing either clock");
    }
    expect(track(first.next, 100, 11, -0.8, config).effective == ProposedCommand::TurnLeft,
           "equal monotonic time is allowed");
    const auto missing = evaluateVisualCommand(first.next, {101, VisualState::Tracking, {}, {}}, config);
    expect(missing.effective == ProposedCommand::Stop && !missing.ex_f,
           "TRACKING with no target sample cannot invent a filtered error");
    const auto maxTime = track(first.next, std::numeric_limits<std::int64_t>::max(), 11, -0.8, config);
    expect(maxTime.effective == ProposedCommand::TurnLeft, "largest monotonic timestamp avoids dwell overflow");
    VisualPolicyConfig invalid[] = {config, config, config, config, config, config, config, config,
                                    config, config, config, config, config, config};
    invalid[0].alpha = 0.0; invalid[1].alpha = 1.1;
    invalid[2].alpha = std::numeric_limits<double>::quiet_NaN();
    invalid[3].e_off = invalid[3].e_on; invalid[4].e_off = -0.1;
    invalid[5].e_on = 1.1; invalid[6].min_dwell_ms = -1;
    invalid[7].stale_ms = 0; invalid[8].lost_ms = 0;
    invalid[9].K_yaw = -1.0; invalid[10].K_yaw = std::numeric_limits<double>::infinity();
    invalid[11].turn_sign = 0; invalid[12].ui_tick_ms = 0;
    invalid[13].e_on = std::numeric_limits<double>::infinity();
    for (const auto &bad : invalid) {
        const auto result = track(first.next, 101, 11, -0.8, bad);
        expect(result.effective == ProposedCommand::Stop && !result.ex_f && result.yaw_cmd == 0.0,
               "invalid config returns fail-closed STOP instead of clipping parameters");
        expect(result.next.lastFilteredFrameId == 10 && result.next.lastSwitchMs == 101,
               "valid-time config failure retains ID floor and records effective STOP change");
    }
}

void displayNames()
{
    const struct { VisualState state; const char *name; } states[] = {
        {VisualState::Tracking, "TRACKING"}, {VisualState::NoTarget, "NO_TARGET"},
        {VisualState::Lost, "LOST"}, {VisualState::Stale, "STALE"},
        {VisualState::InferenceOff, "INFERENCE_OFF"},
    };
    for (const auto &item : states)
        expect(std::strcmp(visualStateName(item.state), item.name) == 0, "visual state name matches display");
    const struct { ProposedCommand command; const char *name; } commands[] = {
        {ProposedCommand::Forward, "FORWARD"}, {ProposedCommand::TurnLeft, "TURN_LEFT"},
        {ProposedCommand::TurnRight, "TURN_RIGHT"}, {ProposedCommand::Stop, "STOP"},
        {ProposedCommand::Hold, "HOLD"},
    };
    for (const auto &item : commands)
        expect(std::strcmp(proposedCommandName(item.command), item.name) == 0, "proposal name matches display");
    expect(std::strcmp(visualStateName(static_cast<VisualState>(999)), "UNKNOWN") == 0,
           "unknown visual enum has deterministic display fallback");
    expect(std::strcmp(proposedCommandName(static_cast<ProposedCommand>(999)), "UNKNOWN") == 0,
           "unknown proposal enum has deterministic display fallback");
}
} // namespace

int main()
{
    initialSampleAndPerFrameEma();
    strictHysteresisBothDirections();
    dwellAndHolding();
    priorityStopsAndNewSampleRecovery();
    imageYawSignAndSaturation();
    invalidInputsAndTime();
    displayNames();
    std::printf("visual_command_policy_tests: %d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
