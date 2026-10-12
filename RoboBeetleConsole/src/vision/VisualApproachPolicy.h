#pragma once

#include "vision/VisualPolicyConfig.h"

#include <optional>


namespace rb::vision {

struct VisualApproachConfig {
    double alphaArea{0.30};
    double slowArea{8.0};
    double resumeArea{14.0};
    double stopArea{20.0};
    std::int64_t arriveDwellMs{1000};
    std::int64_t resumeDwellMs{1000};
    double tMax{0.60};
    double tMin{0.15};
    double tTurnMin{0.30};
    double kTurn{1.0};
    double kPitch{0.50};
};

[[nodiscard]] bool validVisualApproachConfig(const VisualApproachConfig &config) noexcept;

enum class ApproachState { Approach, Arrived };
[[nodiscard]] const char *approachStateName(ApproachState state) noexcept;

struct VisualSetpoint {
    unsigned short throttle{0};
    short turn{0};
    short pitch{0};
};

// Hold is resolved by the caller using its previous complete setpoint.
// Stop and Arrived always return zero, including when the target is eccentric.
[[nodiscard]] VisualSetpoint mapVisualSetpoint(ProposedCommand command, double area_f,
                                             const VisualApproachConfig &config,
                                             bool arrived = false) noexcept;

struct VisualApproachMemory {
    ApproachState state{ApproachState::Approach};
    std::optional<double> filteredArea;
    std::optional<std::uint64_t> sessionId;
    std::optional<std::uint64_t> lastFrameId;
    std::optional<std::int64_t> arriveSinceMs;
    std::optional<std::int64_t> resumeSinceMs;
    std::optional<std::int64_t> missingSinceMs;
    bool areaMissingLost{false};
};

struct VisualApproachInput {
    std::int64_t nowMs{0};
    std::uint64_t sessionId{0};
    std::uint64_t frameId{0};
    std::optional<double> areaCells;
    // True only for selected valid TRACKING observation with video available.
    // False clears dwell, but does not start the selected-target area-missing timer.
    bool observationAvailable{false};
    std::int64_t maxMissMs{500};
};

struct VisualApproachResult {
    VisualApproachMemory next;
    std::optional<double> area_f;
    bool arrivedEnter{false};
    bool arrivedExit{false};
    bool areaMissingLost{false}; // One event per uninterrupted missing episode.
    bool observationUsed{false}; // New valid frame consumed into EMA.
};

// Pure update. Fresh memory resets Arm/reacquisition; a session change also resets.
// Resume reports only area dwell; STOP acceptance age and STOPPED gate the caller.
[[nodiscard]] VisualApproachResult evaluateVisualApproach(
    const VisualApproachMemory &previous, const VisualApproachInput &input,
    const VisualApproachConfig &config);

} // namespace rb::vision

