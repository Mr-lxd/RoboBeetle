#pragma once

#include <cmath>
#include <cstdint>
#include <string_view>

namespace rb::vision {

// Independent display suggestions; these are never dispatched to the robot.
enum class VisualState { Tracking, NoTarget, Lost, Stale, InferenceOff };
enum class ProposedCommand { Forward, TurnLeft, TurnRight, Stop, Hold };

struct VisualPolicyConfig {
    // All values [Provisional]. EMA is per accepted frame, not per timer tick.
    double alpha{0.3};
    double e_on{0.25};
    double e_off{0.12};
    std::int64_t min_dwell_ms{1000};
    std::int64_t stale_ms{500};
    std::int64_t lost_ms{1500};
    double K_yaw{1.0};
    int turn_sign{1}; // [Provisional] Physical turn direction remains unverified.
    int ui_tick_ms{50};
    static constexpr std::string_view policy_version{"visual-command-proposal-v1"};
};

[[nodiscard]] inline bool validVisualPolicyConfig(const VisualPolicyConfig &c) noexcept
{
    return std::isfinite(c.alpha) && c.alpha > 0.0 && c.alpha <= 1.0
        && std::isfinite(c.e_on) && std::isfinite(c.e_off)
        && c.e_off >= 0.0 && c.e_off < c.e_on && c.e_on <= 1.0
        && c.min_dwell_ms >= 0 && c.stale_ms > 0 && c.lost_ms > 0
        && std::isfinite(c.K_yaw) && c.K_yaw >= 0.0
        && (c.turn_sign == 1 || c.turn_sign == -1) && c.ui_tick_ms > 0;
}

} // namespace rb::vision
