#include "robot/YawIntegrator.h"
#include <cmath>
#include <cstdio>
#include <numbers>

namespace {
int failures = 0;
void check(bool ok, const char *why)
{
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", why); ++failures; }
}
bool near(std::optional<double> value, double expected, double tolerance = 1e-9)
{
    return value && std::abs(*value - expected) < tolerance;
}

void constantRateIntegratesOnTheMcuClock()
{
    rb::YawIntegrator yaw;
    // Pi mapping is offset from the MCU clock; integration uses the MCU step.
    for (int ms = 0; ms <= 1000; ms += 50) yaw.add(5000.0 + ms, ms, 10.0);
    const auto at = yaw.at(6000.0, 2500.0);
    check(at && near(at->instDeg, 10.0), "10 deg/s for 1 s integrates to 10 deg");
    check(at && !at->meanDeg, "no mean before a full period of history");
    check(near(yaw.at(5525.0, 2500.0)->instDeg, 5.25), "lookup interpolates between samples");
    check(near(yaw.at(6100.0, 2500.0)->instDeg, 11.0), "short extrapolation uses the last rate");
    check(!yaw.at(6151.0, 2500.0), "no extrapolation past 150 ms");
    check(!yaw.at(4999.0, 2500.0), "no yaw before the first sample");
}

void swayCancelsWhileATurnIsKept()
{
    rb::YawIntegrator yaw;
    constexpr double period = 2400.0, sway = 4.0, turn = 3.0; // deg, deg/s
    const double w = 2.0 * std::numbers::pi / period;
    // yaw(t) = turn*t + sway*sin(w t)  ->  rate = turn + sway*w*1000*cos(w t)
    for (int ms = 0; ms <= 8000; ms += 10)
        yaw.add(ms, ms, turn + sway * w * 1000.0 * std::cos(w * ms));
    double worst = 0.0;
    for (double t = 3000.0; t <= 8000.0; t += 137.0) {
        const auto at = yaw.at(t, period);
        const double expectedSway = sway * std::sin(w * t);
        if (!at || !at->meanDeg) { worst = 1e9; break; }
        worst = std::max(worst, std::abs((at->instDeg - *at->meanDeg) - expectedSway));
        worst = std::max(worst, std::abs(*at->meanDeg - turn * t / 1000.0));
    }
    check(worst < 0.02, "mean heading keeps the turn and the sway term matches the oscillation");
}

void gapsAndResetsStartNewEpochs()
{
    rb::YawIntegrator yaw;
    yaw.add(0, 0, 10.0);
    yaw.add(50, 50, 10.0);
    const auto first = yaw.epoch();
    yaw.add(60, 50, 99.0); // duplicate MCU time
    yaw.add(40, 40, 99.0); // reordered
    check(near(yaw.at(50, 2500)->instDeg, 0.5) && yaw.epoch() == first,
          "duplicate and reordered samples are ignored");
    yaw.add(400, 400, 10.0); // 350 ms gap
    check(yaw.epoch() == first + 1 && near(yaw.at(400, 2500)->instDeg, 0.0),
          "a sample gap restarts integration in a new epoch");
    check(!yaw.at(300, 2500), "history before the gap is gone");
    yaw.reset();
    check(yaw.epoch() == first + 2 && !yaw.at(400, 2500), "reset clears and advances the epoch");
    yaw.add(500, 500, 10.0);
    yaw.add(560, 550, 10.0); // Pi mapping stepped forward more than the MCU
    yaw.add(555, 600, 10.0); // ... then back: lookup axis stays monotonic
    check(near(yaw.at(530, 2500)->instDeg, 0.25, 1e-6) && near(yaw.at(560, 2500)->instDeg, 1.0, 1e-6),
          "a Pi clock step back cannot reorder the lookup axis; the newest sample wins");
    yaw.add(1e9, 700, std::nan(""));
    check(near(yaw.at(560, 2500)->instDeg, 1.0, 1e-6), "non-finite samples are ignored");
}
} // namespace

int main()
{
    constantRateIntegratesOnTheMcuClock();
    swayCancelsWhileATurnIsKept();
    gapsAndResetsStartNewEpochs();
    std::printf("yaw_integrator_tests: %d failures\n", failures);
    return failures ? 1 : 0;
}
