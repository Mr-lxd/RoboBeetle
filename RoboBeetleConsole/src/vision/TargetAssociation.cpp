#include "vision/TargetAssociation.h"
#include <cmath>

namespace rb::vision {
namespace {
bool validTime(const TargetAssociationMemory &m, qint64 now)
{
    return now >= 0 && (!m.lastEvaluatedMs || now >= *m.lastEvaluatedMs);
}
}
TargetAssociationMemory releaseTargetAssociationLock(const TargetAssociationMemory &previous)
{
    auto m = previous;
    m.lockedTarget.reset();
    m.missSinceMs.reset();
    m.cached = {};
    // Preserve processed IDs: a released lock cannot replay its old sample.
    return m;
}
TargetAssociationMemory expireTargetAssociation(const TargetAssociationMemory &previous,
    qint64 now, const VisualPolicyConfig &config)
{
    if (!validTime(previous, now) || !validVisualPolicyConfig(config)) {
        return releaseTargetAssociationLock(previous);
    }
    auto m = previous;
    if (m.missSinceMs && now - *m.missSinceMs >= config.max_miss_ms) {
        m = releaseTargetAssociationLock(m);
    }
    m.lastEvaluatedMs = now;
    return m;
}
TargetAssociationResult associateTarget(const TargetAssociationMemory &previous,
    const DetectionFrame &frame, qint64 now, const VisualPolicyConfig &config)
{
    TargetAssociationResult r;
    r.next = previous;
    if (!validTime(previous, now) || !validVisualPolicyConfig(config)) {
        r.next = releaseTargetAssociationLock(previous);
        r.valid = false;
        return r;
    }
    r.next = expireTargetAssociation(previous, now, config);
    auto &m = r.next;
    if (m.lastProcessedFrameId && frame.frameId <= *m.lastProcessedFrameId) {
        static_cast<AssociationSelection &>(r) = m.cached;
        return r;
    }
    if (m.lockedTarget && m.lockedTarget->sourceSize != frame.sourceSize) {
        m = releaseTargetAssociationLock(m);
    }
    std::optional<TargetState> candidate;
    std::optional<double> distance;
    std::optional<qsizetype> index;
    for (qsizetype i = 0; i < frame.detections.size(); ++i) {
        auto t = targetStateAt(frame, i);
        if (!t) continue;
        if (!m.lockedTarget) {
            if (!candidate || t->target.confidence > candidate->target.confidence) {
                candidate = std::move(t); index = i;
            }
        } else {
            const auto &lock = m.lockedTarget->target;
            if (config.require_same_class && t->target.classId != lock.classId) continue;
            const auto delta = t->target.originalPoint - lock.originalPoint;
            const double d = std::hypot(delta.x(), delta.y());
            if (!distance || d < *distance
                || (d == *distance && t->target.confidence > candidate->target.confidence)) {
                distance = d; candidate = std::move(t); index = i;
            }
        }
    }
    r.distancePx = distance;
    if (candidate && (!m.lockedTarget || (distance && *distance <= config.gate_px))) {
        r.status = m.lockedTarget ? AssociationStatus::Associated : AssociationStatus::Acquired;
        r.selected = candidate;
        r.selectedDetectionIndex = index;
        m.lockedTarget = std::move(candidate);
        m.missSinceMs.reset();
    } else {
        r.status = AssociationStatus::Miss;
        if (!m.missSinceMs) m.missSinceMs = now;
    }
    m.lastProcessedFrameId = frame.frameId;
    m.cached = static_cast<const AssociationSelection &>(r);
    return r;
}
const char *associationStatusName(AssociationStatus status) noexcept
{
    switch (status) {
    case AssociationStatus::Acquired: return "ACQUIRED";
    case AssociationStatus::Associated: return "ASSOCIATED";
    case AssociationStatus::Miss: return "MISS";
    default: return "UNLOCKED";
    }
}
} // namespace rb::vision
