#pragma once

#include "vision/VisualPolicyConfig.h"

#include <optional>

namespace rb::vision {

struct VisualCommandMemory {
    std::optional<std::int64_t> lastEvaluatedMs;
    std::optional<double> filteredEx;
    std::optional<std::uint64_t> lastFilteredFrameId;
    ProposedCommand effective{ProposedCommand::Stop}; // Never Hold.
    std::optional<std::int64_t> lastSwitchMs;
    // Vertical channel. Updated on every usable frame whatever the axis mode, so
    // switching Yaw -> Both starts from fresh state.
    std::optional<double> filteredEy;
    ProposedCommand verticalCandidate{ProposedCommand::Forward}; // Forward / Ascend / Descend.
    // Set while filteredEx holds the yaw-stabilised bearing (ex minus the gyro
    // yaw at capture) of that epoch; unset while it holds plain image ex.
    std::optional<std::uint32_t> filterYawEpoch;
    std::optional<double> lastYawMeanEx; // Latest mean-heading term of that epoch.
};

struct VisualCommandInput {
    std::int64_t nowMs;
    VisualState state;
    std::optional<std::uint64_t> usableFrameId;
    std::optional<double> ex;
    std::optional<double> ey;
    VisualAxisMode axis{VisualAxisMode::Yaw};
    // Gyro yaw in ex units (yaw_ex_per_deg * degrees). sampleYawEx belongs to
    // the usable frame's capture; meanYawEx to the newest capture evaluated.
    std::optional<std::uint32_t> yawEpoch;
    std::optional<double> sampleYawEx;
    std::optional<double> meanYawEx;
};

struct VisualCommandResult {
    VisualCommandMemory next;
    std::optional<double> ex_f;
    std::optional<double> ey_f;
    double yaw_cmd{0.0};
    ProposedCommand proposed{ProposedCommand::Stop};
    ProposedCommand effective{ProposedCommand::Stop};
    bool dwellBlocked{false};
    bool waitingForSample{false};
};

[[nodiscard]] VisualCommandResult evaluateVisualCommand(
    const VisualCommandMemory &previous, const VisualCommandInput &input,
    const VisualPolicyConfig &config);

[[nodiscard]] const char *visualStateName(VisualState state) noexcept;
[[nodiscard]] const char *proposedCommandName(ProposedCommand command) noexcept;

} // namespace rb::vision
