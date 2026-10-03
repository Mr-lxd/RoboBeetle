#include "vision/VisualDiagnosticSession.h"
#include <QCryptographicHash>
#include <algorithm>

namespace rb::vision {
namespace {
QString configHash(const VisualPolicyConfig &c)
{
    // Ordered text, locale independent and full double precision; never hash padding.
    const QByteArray canonical = (QString::fromLatin1(c.policy_version.data(),
            static_cast<qsizetype>(c.policy_version.size())) + '\n'
        + QString::number(c.alpha, 'g', 17) + '\n'
        + QString::number(c.e_on, 'g', 17) + '\n'
        + QString::number(c.e_off, 'g', 17) + '\n'
        + QString::number(c.min_dwell_ms) + '\n'
        + QString::number(c.stale_ms) + '\n'
        + QString::number(c.lost_ms) + '\n'
        + QString::number(c.K_yaw, 'g', 17) + '\n'
        + QString::number(c.turn_sign) + '\n'
        + QString::number(c.ui_tick_ms) + '\n'
        + QString::number(c.gate_px, 'g', 17) + '\n'
        + QString::number(c.max_miss_ms) + '\n'
        + QString::number(c.require_same_class ? 1 : 0) + '\n').toUtf8();
    return QString::fromLatin1(QCryptographicHash::hash(canonical,
        QCryptographicHash::Sha256).toHex());
}
}
VisualDiagnosticSession::VisualDiagnosticSession(VisualPolicyConfig config, NowMs nowMs,
                                                QObject *parent)
    : QObject(parent), config_(config), nowMs_(std::move(nowMs))
{
    clock_.start();
    if (!nowMs_) { nowMs_ = [this] { return clock_.elapsed(); }; }
    wakeup_.setSingleShot(true);
    wakeup_.setTimerType(Qt::PreciseTimer);
    connect(&wakeup_, &QTimer::timeout, this, [this] { evaluate(std::nullopt); });
    beginSession(0);
}
void VisualDiagnosticSession::beginSession(quint64 sessionId)
{
    targetMemory_ = {};
    commandMemory_ = {};
    associationMemory_ = {};
    context_ = {};
    snapshot_ = {};
    snapshot_.sessionId = sessionId;
    snapshot_.policyVersion = QString::fromLatin1(config_.policy_version.data(),
        static_cast<qsizetype>(config_.policy_version.size()));
    snapshot_.policyHash = configHash(config_);
    evaluate(std::nullopt);
}
void VisualDiagnosticSession::onDetectionArrival(const DetectionFrame &frame,
                                                 const VisualViewContext &context)
{
    if (targetMemory_.highestArrivedFrameId
        && frame.frameId <= *targetMemory_.highestArrivedFrameId) {
        evaluate(std::nullopt);
        return;
    }
    snapshot_.frameId = frame.frameId;
    snapshot_.captureTimestampNs = frame.captureTimestampNs;
    snapshot_.nDetections = frame.detections.size();
    snapshot_.detectionFrame = frame;
    context_ = context;
    context_.frame = frame;
    evaluate(frame.frameId);
}
void VisualDiagnosticSession::refresh(const VisualViewContext &context)
{
    context_ = context;
    evaluate(std::nullopt);
}
void VisualDiagnosticSession::evaluate(std::optional<quint64> arrivedId)
{
    const auto now = nowMs_();
    auto gate = context_.gate;
    std::optional<TargetState> selected;
    snapshot_.highestConfidenceTarget.reset();
    // Review S1: raw safety/wait gates bypass association; one state-machine call below.
    if (gate == DetectionDisplayState::Target || gate == DetectionDisplayState::NoTarget) {
        if (!context_.frame || !snapshot_.detectionFrame
            || context_.frame->frameId != snapshot_.frameId) {
            gate = DetectionDisplayState::Stale;
        } else {
            associationMemory_ = expireTargetAssociation(associationMemory_, now, config_);
            const auto &frame = *snapshot_.detectionFrame;
            if (!associationMemory_.lastProcessedFrameId
                || frame.frameId > *associationMemory_.lastProcessedFrameId) {
                const auto associated = associateTarget(associationMemory_, frame, now, config_);
                associationMemory_ = associated.next;
                if (!associated.valid) gate = DetectionDisplayState::Stale;
            }
            if (gate != DetectionDisplayState::Stale) {
                selected = associationMemory_.cached.selected;
                gate = selected ? DetectionDisplayState::Target : DetectionDisplayState::NoTarget;
                snapshot_.highestConfidenceTarget = selectTargetState(frame);
            }
        }
    }
    const auto state = advanceVisualTargetState(targetMemory_,
        {now, arrivedId, gate, selected}, config_);
    targetMemory_ = state.next;
    if (state.state == VisualState::Stale || state.state == VisualState::InferenceOff
        || state.state == VisualState::Lost) {
        // A skipped/overdue ID is consumed too: later video/status cannot resurrect it.
        associationMemory_ = releaseTargetAssociationLock(associationMemory_);
        if (snapshot_.frameId) associationMemory_.lastProcessedFrameId = snapshot_.frameId;
        snapshot_.highestConfidenceTarget.reset();
    }
    snapshot_.associationStatus = associationMemory_.cached.status;
    snapshot_.associationDistancePx = associationMemory_.cached.distancePx;
    snapshot_.selectedDetectionIndex = associationMemory_.cached.selectedDetectionIndex;
    snapshot_.associationMissMs = associationMemory_.missSinceMs && now >= *associationMemory_.missSinceMs
        ? std::optional<qint64>(now - *associationMemory_.missSinceMs) : std::nullopt;
    snapshot_.localMonoMs = now;
    snapshot_.state = state.state;
    snapshot_.awaitingVideo = state.awaitingVideo;
    snapshot_.target = state.usableTarget;
    snapshot_.turnSign = config_.turn_sign;
    if (!state.awaitingVideo) {
        snapshot_.command = evaluateVisualCommand(commandMemory_,
            {now, state.state,
             state.usableTarget ? std::optional<std::uint64_t>(state.usableTarget->frameId) : std::nullopt,
             state.usableTarget ? std::optional<double>(state.usableTarget->ex) : std::nullopt}, config_);
        commandMemory_ = snapshot_.command.next;
    }
    emit diagnosticChanged(snapshot_);
    const int tick = validVisualPolicyConfig(config_) ? config_.ui_tick_ms : VisualPolicyConfig{}.ui_tick_ms;
    qint64 delay = tick;
    if (state.nextDeadlineMs && *state.nextDeadlineMs > now) {
        delay = std::min(delay, *state.nextDeadlineMs - now);
    }
    if (!state.awaitingVideo && associationMemory_.missSinceMs
        && now >= *associationMemory_.missSinceMs) {
        const auto remaining = config_.max_miss_ms - (now - *associationMemory_.missSinceMs);
        if (remaining > 0) delay = std::min<qint64>(delay, remaining);
    }
    wakeup_.start(static_cast<int>(std::max<qint64>(1, delay)));
}
} // namespace rb::vision
