#pragma once

#include "vision/TargetState.h"
#include "vision/VisualPolicyConfig.h"

namespace rb::vision {

enum class AssociationStatus { Unlocked, Acquired, Associated, Miss };

struct AssociationSelection {
    std::optional<TargetState> selected;
    std::optional<qsizetype> selectedDetectionIndex;
    AssociationStatus status{AssociationStatus::Unlocked};
    std::optional<double> distancePx;
};

struct TargetAssociationMemory {
    std::optional<TargetState> lockedTarget;
    std::optional<quint64> lastProcessedFrameId;
    std::optional<qint64> missSinceMs;
    std::optional<qint64> lastEvaluatedMs;
    AssociationSelection cached;
};

struct TargetAssociationResult : AssociationSelection {
    TargetAssociationMemory next;
    bool valid{true};
};

[[nodiscard]] TargetAssociationResult associateTarget(
    const TargetAssociationMemory &memory, const DetectionFrame &frame,
    qint64 nowMs, const VisualPolicyConfig &config);
[[nodiscard]] TargetAssociationMemory expireTargetAssociation(
    const TargetAssociationMemory &memory, qint64 nowMs, const VisualPolicyConfig &config);
[[nodiscard]] TargetAssociationMemory releaseTargetAssociationLock(
    const TargetAssociationMemory &memory);
[[nodiscard]] const char *associationStatusName(AssociationStatus status) noexcept;

} // namespace rb::vision
