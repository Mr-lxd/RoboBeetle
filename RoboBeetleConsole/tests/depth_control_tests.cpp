#include "robot/DepthControl.h"

#include <cstdio>
#include <string>

namespace {
int failures = 0;
void expect(bool c, const char *m)
{
    if (!c) { std::fprintf(stderr, "FAIL: %s\n", m); ++failures; }
}

using rb::DepthControlConfig;
using rb::DepthControlSample;
using rb::DepthEnvelopeMemory;
using rb::DepthEnvelopeState;

rb::DepthEnvelopeResult eval(double d, bool latched = false, const DepthControlConfig &cfg = {})
{
    DepthControlSample s;
    s.rawDepthM = d;
    s.calibratedDepthM = d;
    s.ageMs = 0;
    s.fresh = true;
    return rb::evaluateDepthEnvelope(s, DepthEnvelopeMemory{latched}, cfg);
}

void boundaries()
{
    expect(eval(0.020).state == DepthEnvelopeState::Surface, "0.020 is Surface");
    expect(eval(0.021).state == DepthEnvelopeState::Normal, "0.021 is Normal");
    expect(eval(-0.05).state == DepthEnvelopeState::Surface, "negative is Surface");
    expect(eval(0.399).state == DepthEnvelopeState::Normal, "0.399 Normal");
    expect(eval(0.400).state == DepthEnvelopeState::SoftFloor, "0.400 SoftFloor");
    expect(eval(0.400).next.softFloorLatched, "0.400 latches");
    expect(eval(0.499).state == DepthEnvelopeState::SoftFloor, "0.499 SoftFloor");
    expect(eval(0.500).state == DepthEnvelopeState::HardLimit, "0.500 HardLimit");
}

void hysteresis()
{
    expect(eval(0.371, true).state == DepthEnvelopeState::SoftFloor, "latched 0.371 stays");
    expect(eval(0.371, true).next.softFloorLatched, "latched 0.371 keeps latch");
    expect(eval(0.370, true).state == DepthEnvelopeState::Normal, "latched 0.370 releases");
    expect(!eval(0.370, true).next.softFloorLatched, "latched 0.370 clears latch");
    expect(eval(0.369, true).state == DepthEnvelopeState::Normal, "latched 0.369 releases");
    expect(eval(0.380, false).state == DepthEnvelopeState::Normal, "unlatched 0.380 Normal");
    expect(eval(0.010, true).state == DepthEnvelopeState::Surface, "release into Surface");
    expect(eval(0.55, true).next.softFloorLatched, "hard limit keeps latch");
    {
        auto hard = eval(0.52, false);
        expect(hard.state == DepthEnvelopeState::HardLimit, "0.52 HardLimit");
        expect(hard.next.softFloorLatched, "hard limit sets latch");
        auto back = eval(0.38, hard.next.softFloorLatched);
        expect(back.state == DepthEnvelopeState::SoftFloor, "0.52 -> 0.38 is SoftFloor");
    }
    expect(eval(0.30, true).state == DepthEnvelopeState::Normal, "0.30 after hard releases");
}

void unavailable()
{
    DepthControlConfig cfg;
    const DepthEnvelopeMemory latched{true};
    auto r = rb::evaluateDepthEnvelope(std::nullopt, latched, cfg);
    expect(r.state == DepthEnvelopeState::Unavailable && r.next.softFloorLatched,
           "no sample: Unavailable, latch kept");
    DepthControlSample s;
    s.calibratedDepthM = 0.1;
    s.fresh = false;
    r = rb::evaluateDepthEnvelope(s, latched, cfg);
    expect(r.state == DepthEnvelopeState::Unavailable && r.next.softFloorLatched,
           "stale: Unavailable, latch kept");
    s.fresh = true;
    s.calibratedDepthM = std::nullopt;
    r = rb::evaluateDepthEnvelope(s, latched, cfg);
    expect(r.state == DepthEnvelopeState::NotZeroed && r.next.softFloorLatched,
           "not zeroed: NotZeroed, latch kept");
}

template <typename F>
void bad(F mutate, const char *m)
{
    DepthControlConfig c;
    mutate(c);
    expect(!rb::validDepthControlConfig(c), m);
    expect(eval(0.2, false, c).state == DepthEnvelopeState::Unavailable, m);
}

void invalidConfig()
{
    expect(rb::validDepthControlConfig(DepthControlConfig{}), "default config valid");
    bad([](DepthControlConfig &c) { c.surfaceMarginM = -0.01; }, "negative surface");
    bad([](DepthControlConfig &c) { c.surfaceMarginM = 0.37; }, "surface == softRelease");
    bad([](DepthControlConfig &c) { c.softReleaseM = 0.40; }, "softRelease == softMax");
    bad([](DepthControlConfig &c) { c.softMaxM = 0.50; }, "softMax == hardMax");
    bad([](DepthControlConfig &c) { c.nominalSamplePeriodMs = 0; }, "zero period");
    bad([](DepthControlConfig &c) { c.controlFreshMs = 2 * c.nominalSamplePeriodMs + 199; },
        "fresh below 2*period+200");
    bad([](DepthControlConfig &c) { c.zeroMinSamples = 2; }, "too few zero samples");
    bad([](DepthControlConfig &c) { c.zeroMaxRangeM = 0.0; }, "zero range");
    DepthControlConfig ok;
    ok.controlFreshMs = 2 * ok.nominalSamplePeriodMs + 200;
    expect(rb::validDepthControlConfig(ok), "fresh == 2*period+200 valid");
}

void names()
{
    expect(std::string(rb::depthEnvelopeStateName(DepthEnvelopeState::HardLimit)) == "HARD_LIMIT",
           "state name");
}
} // namespace

int main()
{
    boundaries();
    hysteresis();
    unavailable();
    invalidConfig();
    names();
    if (failures == 0) std::puts("depth_control_tests passed");
    return failures == 0 ? 0 : 1;
}
