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
    // Horizontal shift of the lock point predicted from gyro yaw since the lock.
    std::optional<double> predictedShiftPx;
};

struct TargetAssociationMemory {
    std::optional<TargetState> lockedTarget;
    std::optional<YawReference> lockYaw; // Yaw at the locked target's capture.
    std::optional<quint64> lastProcessedFrameId;
    std::optional<qint64> missSinceMs;
    std::optional<qint64> lastEvaluatedMs;
    AssociationSelection cached;
};

struct TargetAssociationResult : AssociationSelection {
    TargetAssociationMemory next;
    bool valid{true};
};

// yaw is the body yaw at this frame's capture. With a same-epoch lock yaw the
// gate is centred on the gyro-predicted point and the lock is held longer.
[[nodiscard]] TargetAssociationResult associateTarget(
    const TargetAssociationMemory &memory, const DetectionFrame &frame,
    qint64 nowMs, const VisualPolicyConfig &config,
    const std::optional<YawReference> &yaw = std::nullopt);
[[nodiscard]] qint64 associationMissLimitMs(const TargetAssociationMemory &memory,
                                            const VisualPolicyConfig &config) noexcept;
[[nodiscard]] TargetAssociationMemory expireTargetAssociation(
    const TargetAssociationMemory &memory, qint64 nowMs, const VisualPolicyConfig &config);
[[nodiscard]] TargetAssociationMemory releaseTargetAssociationLock(
    const TargetAssociationMemory &memory);
[[nodiscard]] const char *associationStatusName(AssociationStatus status) noexcept;

} // namespace rb::vision
