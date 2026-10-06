#pragma once

#include <cmath>
#include <cstdint>
#include <string_view>

namespace rb::vision {

// Independent display suggestions; these are never dispatched to the robot.
enum class VisualState { Tracking, NoTarget, Lost, Stale, InferenceOff };
// Ascend/Descend are appended so existing numeric values do not move.
enum class ProposedCommand { Forward, TurnLeft, TurnRight, Stop, Hold, Ascend, Descend };
// Which axis the automatic follower acts on. Yaw reproduces the original behaviour.
enum class VisualAxisMode { Yaw, Pitch, Both };

struct VisualPolicyConfig {
    // All values [Provisional]. EMA is per accepted frame, not per timer tick.
    double alpha{0.3};
    double e_on{0.25};
    double e_off{0.12};
    std::int64_t min_dwell_ms{1000};
    std::int64_t stale_ms{500};
    std::int64_t lost_ms{1500};
    double K_yaw{1.0};
    // [Provisional] Physical turn direction remains unverified. The sign is a
    // build-time constant: the operator no longer selects or confirms it.
    static constexpr int configuredTurnSign{+1};
    int turn_sign{configuredTurnSign};
    // Vertical axis. Separate from the ex parameters; initial values equal them. [Provisional]
    double alpha_y{0.3};
    double ey_on{0.25};
    double ey_off{0.12};
    static constexpr int configuredPitchSign{+1}; // ey > 0 (target below centre) -> DESCEND.
    int pitch_sign{configuredPitchSign};
    int ui_tick_ms{50};
    double gate_px{48.0}; // [Provisional] Original image pixels.
    std::int64_t max_miss_ms{500}; // [Provisional] Local monotonic milliseconds.
    bool require_same_class{false}; // [Provisional] Labels can change on one object.
    static constexpr std::string_view policy_version{"visual-command-proposal-v3"};
};

// File recording limits are independent of command-policy provenance.
struct VisualCsvConfig {
    static constexpr std::string_view schema_version{"visual-csv-v5"};
    std::int64_t max_file_bytes{32 * 1024 * 1024};
    int flush_ms{250};
};

[[nodiscard]] inline bool validVisualPolicyConfig(const VisualPolicyConfig &c) noexcept
{
    return std::isfinite(c.alpha) && c.alpha > 0.0 && c.alpha <= 1.0
        && std::isfinite(c.e_on) && std::isfinite(c.e_off)
        && c.e_off >= 0.0 && c.e_off < c.e_on && c.e_on <= 1.0
        && c.min_dwell_ms >= 0 && c.stale_ms > 0 && c.lost_ms > 0
        && std::isfinite(c.K_yaw) && c.K_yaw >= 0.0
        && (c.turn_sign == 1 || c.turn_sign == -1) && c.ui_tick_ms > 0
        && std::isfinite(c.alpha_y) && c.alpha_y > 0.0 && c.alpha_y <= 1.0
        && std::isfinite(c.ey_on) && std::isfinite(c.ey_off)
        && c.ey_off >= 0.0 && c.ey_off < c.ey_on && c.ey_on <= 1.0
        && (c.pitch_sign == 1 || c.pitch_sign == -1)
        && std::isfinite(c.gate_px) && c.gate_px > 0.0 && c.max_miss_ms > 0;
}

} // namespace rb::vision
