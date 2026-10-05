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
};

struct VisualCommandInput {
    std::int64_t nowMs;
    VisualState state;
    std::optional<std::uint64_t> usableFrameId;
    std::optional<double> ex;
    std::optional<double> ey;
    VisualAxisMode axis{VisualAxisMode::Yaw};
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
