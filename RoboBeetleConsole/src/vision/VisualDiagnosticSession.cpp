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
        + QString::number(c.ui_tick_ms) + '\n').toUtf8();
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
    context_ = context;
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
    const auto state = advanceVisualTargetState(targetMemory_,
        {now, arrivedId, context_.gate, context_.selected}, config_);
    targetMemory_ = state.next;
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
    wakeup_.start(static_cast<int>(std::max<qint64>(1, delay)));
}
} // namespace rb::vision
