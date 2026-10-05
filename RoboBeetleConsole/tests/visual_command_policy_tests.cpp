#include "vision/VisualCommandPolicy.h"

#include <cmath>
#include <cstdio>
#include <algorithm>
#include <cstring>
#include <limits>
#include <optional>
#include <string>

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
        {ProposedCommand::Hold, "HOLD"}, {ProposedCommand::Ascend, "ASCEND"},
        {ProposedCommand::Descend, "DESCEND"},
    };
    for (const auto &item : commands)
        expect(std::strcmp(proposedCommandName(item.command), item.name) == 0, "proposal name matches display");
    expect(std::strcmp(visualStateName(static_cast<VisualState>(999)), "UNKNOWN") == 0,
           "unknown visual enum has deterministic display fallback");
    expect(std::strcmp(proposedCommandName(static_cast<ProposedCommand>(999)), "UNKNOWN") == 0,
           "unknown proposal enum has deterministic display fallback");
}

// ---- Yaw regression: reference copy of the pre-pitch implementation ------------
// Copied verbatim from the version before Task 06 PR-C (modulo the input/memory
// types that gained fields the reference ignores). Yaw mode must match it exactly.
namespace reference {
bool validError(double ex) noexcept { return std::isfinite(ex) && ex >= -1.0 && ex <= 1.0; }
bool validEffective(ProposedCommand command) noexcept
{
    return command == ProposedCommand::Forward || command == ProposedCommand::TurnLeft
        || command == ProposedCommand::TurnRight || command == ProposedCommand::Stop;
}
ProposedCommand turnFor(double error, int turnSign) noexcept
{
    const bool right = (error > 0.0) == (turnSign > 0);
    return right ? ProposedCommand::TurnRight : ProposedCommand::TurnLeft;
}
VisualCommandResult evaluate(const VisualCommandMemory &previous, const VisualCommandInput &input,
                             const VisualPolicyConfig &config)
{
    VisualCommandResult result;
    result.next = previous;
    const bool validTime = input.nowMs >= 0
        && (!previous.lastEvaluatedMs
            || (*previous.lastEvaluatedMs >= 0 && input.nowMs >= *previous.lastEvaluatedMs))
        && (!previous.lastSwitchMs
            || (*previous.lastSwitchMs >= 0 && input.nowMs >= *previous.lastSwitchMs));
    if (validTime) result.next.lastEvaluatedMs = input.nowMs;
    const auto stop = [&]() {
        result.next.filteredEx.reset();
        result.next.effective = ProposedCommand::Stop;
        if (validTime && previous.effective != ProposedCommand::Stop) result.next.lastSwitchMs = input.nowMs;
        return result;
    };
    if (!validTime || !validVisualPolicyConfig(config) || !validEffective(previous.effective)
        || (previous.filteredEx && !validError(*previous.filteredEx))) return stop();
    switch (input.state) {
    case VisualState::Stale:
    case VisualState::InferenceOff:
    case VisualState::Lost:
        return stop();
    case VisualState::NoTarget:
        result.proposed = ProposedCommand::Hold;
        result.effective = previous.effective;
        result.ex_f = previous.filteredEx;
        result.yaw_cmd = result.ex_f ? std::clamp(config.K_yaw * *result.ex_f, -1.0, 1.0) : 0.0;
        return result;
    case VisualState::Tracking:
        break;
    default:
        return stop();
    }
    if (!input.usableFrameId || !input.ex || !validError(*input.ex)) return stop();
    if (!previous.lastFilteredFrameId || *input.usableFrameId > *previous.lastFilteredFrameId) {
        result.next.filteredEx = previous.filteredEx
            ? config.alpha * *input.ex + (1.0 - config.alpha) * *previous.filteredEx
            : *input.ex;
        result.next.lastFilteredFrameId = input.usableFrameId;
    }
    if (!result.next.filteredEx) {
        result.waitingForSample = true;
        return stop();
    }
    result.ex_f = result.next.filteredEx;
    result.yaw_cmd = std::clamp(config.K_yaw * *result.ex_f, -1.0, 1.0);
    const double magnitude = std::abs(*result.ex_f);
    ProposedCommand candidate = previous.effective;
    const bool wasTurning = previous.effective == ProposedCommand::TurnLeft
        || previous.effective == ProposedCommand::TurnRight;
    if (!wasTurning) {
        candidate = magnitude > config.e_on ? turnFor(*result.ex_f, config.turn_sign) : ProposedCommand::Forward;
    } else if (magnitude < config.e_off) {
        candidate = ProposedCommand::Forward;
    } else if (magnitude > config.e_on) {
        candidate = turnFor(*result.ex_f, config.turn_sign);
    }
    if (candidate != previous.effective) {
        result.dwellBlocked = previous.lastSwitchMs && input.nowMs - *previous.lastSwitchMs < config.min_dwell_ms;
        if (!result.dwellBlocked) {
            result.next.effective = candidate;
            result.next.lastSwitchMs = input.nowMs;
        }
    }
    result.effective = result.next.effective;
    result.proposed = result.effective;
    return result;
}
} // namespace reference

bool sameOptional(const std::optional<double> &a, const std::optional<double> &b)
{
    if (a.has_value() != b.has_value()) return false;
    return !a || *a == *b || (std::isnan(*a) && std::isnan(*b));
}

bool sameResult(const VisualCommandResult &a, const VisualCommandResult &b)
{
    return a.proposed == b.proposed && a.effective == b.effective && sameOptional(a.ex_f, b.ex_f)
        && a.yaw_cmd == b.yaw_cmd && a.dwellBlocked == b.dwellBlocked
        && a.waitingForSample == b.waitingForSample
        && a.next.lastEvaluatedMs == b.next.lastEvaluatedMs
        && sameOptional(a.next.filteredEx, b.next.filteredEx)
        && a.next.lastFilteredFrameId == b.next.lastFilteredFrameId
        && a.next.effective == b.next.effective && a.next.lastSwitchMs == b.next.lastSwitchMs;
}

struct Lcg {
    std::uint64_t s{0x9E3779B97F4A7C15ULL};
    std::uint32_t next() { s = s * 6364136223846793005ULL + 1442695040888963407ULL; return static_cast<std::uint32_t>(s >> 33); }
    double unit() { return next() / 2147483648.0; }                // [0,1)
    double sym(double r) { return (unit() * 2.0 - 1.0) * r; }
};

void yawMatchesReferenceOnRandomSequences()
{
    const VisualPolicyConfig configs[] = {VisualPolicyConfig{}, directConfig(), [] {
        VisualPolicyConfig c; c.alpha = 0.6; c.e_on = 0.4; c.e_off = 0.1; c.min_dwell_ms = 300; c.turn_sign = -1; return c; }()};
    int steps = 0;
    bool allSame = true;
    for (const auto &config : configs) {
        Lcg rng;
        VisualCommandMemory memoryNew, memoryRef;
        std::int64_t now = 0;
        std::uint64_t frame = 0;
        for (int i = 0; i < 12000; ++i) {
            now += static_cast<std::int64_t>(rng.next() % 130);
            if (rng.next() % 4000 == 0) now = std::max<std::int64_t>(0, now - 500); // clock regression
            const auto roll = rng.next() % 100;
            const VisualState state = roll < 72 ? VisualState::Tracking : roll < 84 ? VisualState::NoTarget
                : roll < 91 ? VisualState::Lost : roll < 96 ? VisualState::Stale : VisualState::InferenceOff;
            const auto frameRoll = rng.next() % 10;
            if (frameRoll < 7) frame += 1 + rng.next() % 3;            // new frame, with gaps
            else if (frameRoll == 8 && frame > 2) frame -= 2;          // older frame ID
            // frameRoll 7/9: same ID
            double ex = rng.sym(1.0) * (rng.next() % 9 == 0 ? 1.3 : 1.0);  // sometimes out of range
            if (rng.next() % 50 == 0) ex = std::numeric_limits<double>::quiet_NaN();
            std::optional<double> exOpt = rng.next() % 40 == 0 ? std::nullopt : std::optional<double>(ex);
            std::optional<std::uint64_t> idOpt = rng.next() % 60 == 0 ? std::nullopt : std::optional<std::uint64_t>(frame);
            // The vertical channel carries junk in Yaw mode and must change nothing.
            std::optional<double> ey = rng.next() % 3 == 0 ? std::nullopt : std::optional<double>(rng.sym(1.4));
            if (rng.next() % 30 == 0) ey = std::numeric_limits<double>::quiet_NaN();
            VisualCommandInput input{now, state, idOpt, exOpt, ey, VisualAxisMode::Yaw};
            const auto a = evaluateVisualCommand(memoryNew, input, config);
            const auto b = reference::evaluate(memoryRef, input, config);
            if (!sameResult(a, b)) {
                allSame = false;
                std::fprintf(stderr, "Yaw divergence at step %d (config %d)\n", i, static_cast<int>(&config - configs));
                break;
            }
            memoryNew = a.next;
            memoryRef = b.next;
            ++steps;
        }
        if (!allSame) break;
    }
    expect(allSame && steps >= 36000, "Yaw output matches the pre-pitch implementation on 36,000 random steps");
}

void yawMatchesReferenceOnDesktopReplay()
{
    // Operator CSV from the desktop STALE incident: state and ex_f per row.
    std::string path = __FILE__;
    path.replace(path.rfind("visual_command_policy_tests.cpp"), std::string("visual_command_policy_tests.cpp").size(),
                 "fixtures/visual-dispatch-transient-stale.csv");
    std::FILE *file = std::fopen(path.c_str(), "rb");
    expect(file != nullptr, "desktop replay fixture opens");
    if (!file) return;
    char line[512];
    (void)std::fgets(line, sizeof line, file); // header
    VisualCommandMemory memoryNew, memoryRef;
    VisualPolicyConfig config;
    int rows = 0;
    bool same = true;
    std::uint64_t frame = 0;
    while (std::fgets(line, sizeof line, file)) {
        long relative = 0; char kind[32] = {}, state[32] = {}, proposed[32] = {}, exf[32] = {}; int awaiting = 0;
        // relative_ms,row_kind,state,proposed_command,ex_f,awaiting_video
        if (std::sscanf(line, "%ld,%31[^,],%31[^,],%31[^,],%31[^,],%d", &relative, kind, state, proposed, exf, &awaiting) < 5)
            if (std::sscanf(line, "%ld,%31[^,],%31[^,],%31[^,],,%d", &relative, kind, state, proposed, &awaiting) < 5) continue;
        VisualState vs = std::strcmp(state, "TRACKING") == 0 ? VisualState::Tracking
            : std::strcmp(state, "NO_TARGET") == 0 ? VisualState::NoTarget
            : std::strcmp(state, "LOST") == 0 ? VisualState::Lost
            : std::strcmp(state, "STALE") == 0 ? VisualState::Stale : VisualState::InferenceOff;
        if (awaiting) continue;
        std::optional<double> ex = exf[0] ? std::optional<double>(std::atof(exf)) : std::nullopt;
        ++frame;
        VisualCommandInput input{relative, vs, frame, ex ? ex : std::optional<double>(0.0), std::nullopt, VisualAxisMode::Yaw};
        const auto a = evaluateVisualCommand(memoryNew, input, config);
        const auto b = reference::evaluate(memoryRef, input, config);
        if (!sameResult(a, b)) { same = false; break; }
        memoryNew = a.next; memoryRef = b.next;
        ++rows;
    }
    std::fclose(file);
    expect(same && rows >= 30, "Yaw output matches the pre-pitch implementation on the desktop CSV replay");
}

// ---- Pitch / Both ----------------------------------------------------------------
VisualCommandResult axisStep(const VisualCommandMemory &memory, std::int64_t time, std::uint64_t frame,
                             double ex, double ey, VisualAxisMode axis, const VisualPolicyConfig &config)
{
    return evaluateVisualCommand(memory, {time, VisualState::Tracking, frame, ex, ey, axis}, config);
}

void pitchSignAndHysteresis()
{
    auto config = directConfig();
    config.alpha_y = 1.0;
    auto r = axisStep({}, 0, 0, 0.0, 0.6, VisualAxisMode::Pitch, config);
    expect(r.effective == ProposedCommand::Descend, "target below centre (ey>0) descends with pitch_sign +1");
    expect(near(r.ey_f, 0.6), "ey_f is reported");
    r = axisStep({}, 0, 0, 0.0, -0.6, VisualAxisMode::Pitch, config);
    expect(r.effective == ProposedCommand::Ascend, "target above centre (ey<0) ascends with pitch_sign +1");
    auto flipped = config;
    flipped.pitch_sign = -1;
    r = axisStep({}, 0, 0, 0.0, 0.6, VisualAxisMode::Pitch, flipped);
    expect(r.effective == ProposedCommand::Ascend, "pitch_sign -1 reverses the mapping");

    // Strict hysteresis: on at >0.25, stays until <0.12, 0.25 exactly does not start.
    VisualCommandMemory m;
    std::uint64_t f = 0;
    std::int64_t t = 0;
    const auto step = [&](double ey) { r = axisStep(m, t += 10, f++, 0.0, ey, VisualAxisMode::Pitch, config); m = r.next; return r.effective; };
    expect(step(0.25) == ProposedCommand::Forward, "ey == e_on does not start a dive");
    expect(step(0.26) == ProposedCommand::Descend, "ey > e_on starts a dive");
    expect(step(0.2) == ProposedCommand::Descend, "inside the band the dive continues");
    expect(step(0.12) == ProposedCommand::Descend, "ey == e_off still dives");
    expect(step(0.11) == ProposedCommand::Forward, "ey < e_off ends the dive");
    expect(step(0.2) == ProposedCommand::Forward, "inside the band a stopped dive does not restart");
    expect(step(-0.3) == ProposedCommand::Ascend, "overshoot to the other side flips directly");
    expect(step(0.3) == ProposedCommand::Descend, "and back");
}

void pitchIgnoresEx()
{
    auto config = directConfig();
    config.alpha_y = 1.0;
    const auto r = axisStep({}, 0, 0, 0.95, 0.0, VisualAxisMode::Pitch, config);
    expect(r.effective == ProposedCommand::Forward, "Pitch ignores a large ex");
}

void pitchRequiresEy()
{
    auto config = directConfig();
    for (const auto &bad : {std::optional<double>{}, std::optional<double>(std::nan("")), std::optional<double>(1.5)}) {
        const auto r = evaluateVisualCommand({}, {0, VisualState::Tracking, 1, 0.0, bad, VisualAxisMode::Pitch}, config);
        expect(r.effective == ProposedCommand::Stop, "Pitch without a valid ey stops");
        const auto both = evaluateVisualCommand({}, {0, VisualState::Tracking, 1, 0.0, bad, VisualAxisMode::Both}, config);
        expect(both.effective == ProposedCommand::Stop, "Both without a valid ey stops");
        const auto yaw = evaluateVisualCommand({}, {0, VisualState::Tracking, 1, 0.0, bad, VisualAxisMode::Yaw}, config);
        expect(yaw.effective == ProposedCommand::Forward, "Yaw does not care about ey");
    }
}

void bothHorizontalFirst()
{
    auto config = directConfig();
    config.alpha_y = 1.0;
    VisualCommandMemory m;
    std::uint64_t f = 0;
    std::int64_t t = 0;
    VisualCommandResult r;
    const auto step = [&](double ex, double ey) { r = axisStep(m, t += 10, f++, ex, ey, VisualAxisMode::Both, config); m = r.next; return r.effective; };
    expect(step(0.9, 0.9) == ProposedCommand::TurnRight, "a large ex wins over a large ey");
    expect(step(0.2, 0.9) == ProposedCommand::TurnRight, "the turn continues inside its hysteresis band");
    expect(step(0.05, 0.9) == ProposedCommand::Descend, "once the turn ends the vertical command applies");
    expect(step(0.05, 0.05) == ProposedCommand::Forward, "and then neither axis asks for anything");
    expect(step(-0.9, 0.9) == ProposedCommand::TurnLeft, "the turn pre-empts a vertical command again");
}

void verticalStateIsFreshAfterYawFrames()
{
    auto config = directConfig();
    config.alpha_y = 1.0;
    VisualCommandMemory m;
    std::uint64_t f = 0;
    std::int64_t t = 0;
    // 20 frames in Yaw mode with the target far below centre: Yaw output is Forward ...
    for (int i = 0; i < 20; ++i) {
        const auto r = axisStep(m, t += 10, f++, 0.0, 0.8, VisualAxisMode::Yaw, config);
        expect(r.effective == ProposedCommand::Forward, "Yaw ignores ey");
        m = r.next;
    }
    expect(m.verticalCandidate == ProposedCommand::Descend && near(m.filteredEy, 0.8),
           "the vertical state tracks ey while in Yaw mode");
    // ... and the first Pitch frame can use it immediately.
    const auto first = axisStep(m, t += 10, f++, 0.0, 0.8, VisualAxisMode::Pitch, config);
    expect(first.effective == ProposedCommand::Descend, "switching to Pitch needs no warm-up");
}

void verticalDwellAndHold()
{
    VisualPolicyConfig config;     // default 1000 ms dwell
    config.alpha = 1.0;
    config.alpha_y = 1.0;
    auto r = axisStep({}, 0, 0, 0.0, 0.8, VisualAxisMode::Pitch, config);
    expect(r.effective == ProposedCommand::Descend, "first command is accepted");
    r = axisStep(r.next, 400, 1, 0.0, -0.8, VisualAxisMode::Pitch, config);
    expect(r.dwellBlocked && r.effective == ProposedCommand::Descend, "min_dwell limits vertical flips too");
    r = axisStep(r.next, 1100, 2, 0.0, -0.8, VisualAxisMode::Pitch, config);
    expect(!r.dwellBlocked && r.effective == ProposedCommand::Ascend, "after the dwell the flip is allowed");
    const auto hold = evaluateVisualCommand(r.next, {1200, VisualState::NoTarget, std::nullopt, std::nullopt,
                                                     std::nullopt, VisualAxisMode::Pitch}, config);
    expect(hold.proposed == ProposedCommand::Hold && hold.effective == ProposedCommand::Ascend
               && near(hold.ey_f, -0.8), "NO_TARGET holds the vertical command and keeps ey_f");
    const auto lost = evaluateVisualCommand(r.next, {1200, VisualState::Lost, std::nullopt, std::nullopt,
                                                     std::nullopt, VisualAxisMode::Pitch}, config);
    expect(lost.effective == ProposedCommand::Stop && !lost.next.filteredEy
               && lost.next.verticalCandidate == ProposedCommand::Forward, "LOST stops and clears the vertical state");
}

void verticalEmaIsPerFrame()
{
    VisualPolicyConfig config;
    config.alpha_y = 0.5;
    auto r = axisStep({}, 0, 0, 0.0, 0.8, VisualAxisMode::Pitch, config);
    expect(near(r.ey_f, 0.8), "first ey initialises the EMA");
    r = axisStep(r.next, 10, 1, 0.0, 0.0, VisualAxisMode::Pitch, config);
    expect(near(r.ey_f, 0.4), "second frame applies alpha_y once");
    r = axisStep(r.next, 20, 1, 0.0, -1.0, VisualAxisMode::Pitch, config);
    expect(near(r.ey_f, 0.4), "the same frame ID never resamples");
}

void pitchConfigValidation()
{
    VisualPolicyConfig c;
    expect(validVisualPolicyConfig(c) && c.pitch_sign == 1 && VisualPolicyConfig::configuredPitchSign == 1,
           "default pitch parameters are valid and pitch_sign is +1");
    auto bad = c; bad.alpha_y = 0.0; expect(!validVisualPolicyConfig(bad), "alpha_y must be positive");
    bad = c; bad.alpha_y = 1.1; expect(!validVisualPolicyConfig(bad), "alpha_y must not exceed one");
    bad = c; bad.ey_off = bad.ey_on; expect(!validVisualPolicyConfig(bad), "ey_off must be below ey_on");
    bad = c; bad.ey_on = 1.5; expect(!validVisualPolicyConfig(bad), "ey_on must not exceed one");
    bad = c; bad.pitch_sign = 0; expect(!validVisualPolicyConfig(bad), "pitch_sign must be +-1");
    expect(std::strcmp(proposedCommandName(ProposedCommand::Ascend), "ASCEND") == 0
               && std::strcmp(proposedCommandName(ProposedCommand::Descend), "DESCEND") == 0,
           "ASCEND/DESCEND display names");
    expect(std::string(VisualPolicyConfig::policy_version) == "visual-command-proposal-v3", "policy version bumped");
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
    yawMatchesReferenceOnRandomSequences();
    yawMatchesReferenceOnDesktopReplay();
    pitchSignAndHysteresis();
    pitchIgnoresEx();
    pitchRequiresEy();
    bothHorizontalFirst();
    verticalStateIsFreshAfterYawFrames();
    verticalDwellAndHold();
    verticalEmaIsPerFrame();
    pitchConfigValidation();
    std::printf("visual_command_policy_tests: %d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
