#pragma once

#include "vision/VisualDiagnosticSession.h"
#include "vision/VisualDispatchSession.h"
#include <QFile>
#include <QObject>
#include <QTimer>

namespace rb::vision {

// Owned and used by the GUI thread, like VisualDiagnosticSession.
class VisualCsvLogger final : public QObject {
    Q_OBJECT
public:
    explicit VisualCsvLogger(VisualCsvConfig config = {}, QObject *parent = nullptr);
    ~VisualCsvLogger() override;
    [[nodiscard]] static QString defaultDirectory();
    bool start(const QString &directory);
    void record(const VisualDiagnosticSnapshot &snapshot);
    void recordDispatch(const VisualDispatchRecord &record);
    void stop();
    void flush();
    // Source of the depth_* / envelope_state columns (blank when unset).
    void setDepthInfoProvider(std::function<VisualDepthCsvInfo()> provider) { depthInfo_ = std::move(provider); }
    [[nodiscard]] bool isRecording() const { return recording_; }
    [[nodiscard]] QString filePath() const { return file_.fileName(); }
    [[nodiscard]] QString lastError() const { return lastError_; }
signals:
    void recordingChanged();
private:
    bool append(const QByteArray &bytes);
    void fail(const QString &reason);
    QByteArray row(const VisualDiagnosticSnapshot &snapshot, bool transition,
                   std::optional<qint64> arrivalMs = std::nullopt) const;
    bool finishPending(qint64 evaluationMs);
    VisualCsvConfig config_;
    std::function<VisualDepthCsvInfo()> depthInfo_;
    QFile file_;
    QTimer timer_;
    QString lastError_;
    bool recording_{false};
    qint64 bytesWritten_{0};
    std::optional<quint64> sessionId_;
    std::optional<quint64> highestFrameId_;
    // One unresolved frame. Its ID is already in highestFrameId_.
    std::optional<VisualDiagnosticSnapshot> pendingFrame_;
    qint64 pendingArrivalMs_{0};
    qint64 lastEvaluationMs_{0};
    // Observe arrival even while recording is off; enabling CSV must not retime a cached ID.
    std::optional<quint64> observedSessionId_;
    std::optional<quint64> observedFrameId_;
    qint64 observedArrivalMs_{0};
    std::optional<VisualState> previousState_;
    std::optional<ProposedCommand> previousEffective_;
};

} // namespace rb::vision
