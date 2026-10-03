#include "vision/VisualPolicyConfig.h"
#include <cstdio>
#include <limits>

int main()
{
    using namespace rb::vision;
    int failures = 0;
    auto check = [&](bool value, const char *why) {
        if (!value) { std::fprintf(stderr, "FAIL: %s\n", why); ++failures; }
    };
    VisualPolicyConfig c;
    check(validVisualPolicyConfig(c), "provisional defaults are valid");
    c.turn_sign = -1;
    check(validVisualPolicyConfig(c), "inverted display direction is valid");
    c.turn_sign = 0;
    check(!validVisualPolicyConfig(c), "direction must be +1 or -1");
    c = {}; c.e_off = c.e_on;
    check(!validVisualPolicyConfig(c), "hysteresis thresholds must be ordered");
    c = {}; c.alpha = 0;
    check(!validVisualPolicyConfig(c), "zero alpha is invalid");
    c = {}; c.stale_ms = 0;
    check(!validVisualPolicyConfig(c), "watchdog needs positive duration");
    c = {}; c.min_dwell_ms = -1;
    check(!validVisualPolicyConfig(c), "negative dwell is invalid");
    c = {}; c.K_yaw = std::numeric_limits<double>::infinity();
    check(!validVisualPolicyConfig(c), "infinite gain is invalid");
    return failures ? 1 : 0;
}
