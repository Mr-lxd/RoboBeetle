#pragma once

#include "vision/VisualCommandPolicy.h"
#include "vision/VisualTargetStateMachine.h"
#include "vision/TargetAssociation.h"
#include <QElapsedTimer>
#include <QObject>
#include <QTimer>
#include <functional>

namespace rb::vision {

struct VisualViewContext {
    DetectionDisplayState gate{DetectionDisplayState::Stale};
    std::optional<DetectionFrame> frame;
};

struct VisualDiagnosticSnapshot {
    quint64 sessionId{0};
    qint64 localMonoMs{0};
    std::optional<quint64> frameId;
    std::optional<quint64> captureTimestampNs;
    qsizetype nDetections{0};
    std::optional<TargetState> target;
    VisualState state{VisualState::Stale};
    VisualCommandResult command;
    bool awaitingVideo{false};
    int turnSign{1};
    VisualAxisMode axis{VisualAxisMode::Yaw};
    int pitchSign{1};
    QString policyVersion;
    QString policyHash;
    AssociationStatus associationStatus{AssociationStatus::Unlocked};
    std::optional<double> associationDistancePx;
    std::optional<qint64> associationMissMs;
    std::optional<qsizetype> selectedDetectionIndex;
    std::optional<TargetState> highestConfidenceTarget;
    // Gyro yaw at the newest capture: sway is instantaneous minus mean heading;
    // the shift is the gyro-predicted move of the lock point in pixels.
    std::optional<double> yawSwayDeg;
    std::optional<double> associationPredictedShiftPx;
    std::optional<DetectionFrame> detectionFrame;
};

class VisualDiagnosticSession final : public QObject {
    Q_OBJECT
public:
    using NowMs = std::function<qint64()>;
    // Body yaw at a Pi capture timestamp (ns); empty when gyro is unavailable.
    using YawProvider = std::function<std::optional<YawReference>(quint64 captureNs)>;
    explicit VisualDiagnosticSession(VisualPolicyConfig config = {}, NowMs nowMs = {},
                                     QObject *parent = nullptr);
    [[nodiscard]] qint64 monotonicNowMs() const { return nowMs_(); }
    void beginSession(quint64 sessionId);
    void onDetectionArrival(const DetectionFrame &frame, const VisualViewContext &context);
    void refresh(const VisualViewContext &context);
    void setAxisMode(VisualAxisMode axis);
    void setYawProvider(YawProvider provider) { yawProvider_ = std::move(provider); }
    [[nodiscard]] VisualAxisMode axisMode() const noexcept { return axis_; }
    [[nodiscard]] const VisualDiagnosticSnapshot &snapshot() const noexcept { return snapshot_; }
signals:
    void diagnosticChanged(const rb::vision::VisualDiagnosticSnapshot &snapshot);
private:
    void evaluate(std::optional<quint64> arrivedId);
    VisualPolicyConfig config_;
    VisualAxisMode axis_{VisualAxisMode::Yaw};
    NowMs nowMs_;
    QElapsedTimer clock_;
    QTimer wakeup_;
    VisualTargetMemory targetMemory_;
    VisualCommandMemory commandMemory_;
    TargetAssociationMemory associationMemory_;
    VisualViewContext context_;
    VisualDiagnosticSnapshot snapshot_;
    YawProvider yawProvider_;
    std::optional<YawReference> latestYaw_; // At the newest arrived capture.
};

} // namespace rb::vision
