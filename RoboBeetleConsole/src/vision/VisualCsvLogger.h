#pragma once

#include "vision/VisualDiagnosticSession.h"
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
    void stop();
    void flush();
    [[nodiscard]] bool isRecording() const { return recording_; }
    [[nodiscard]] QString filePath() const { return file_.fileName(); }
    [[nodiscard]] QString lastError() const { return lastError_; }
signals:
    void recordingChanged();
private:
    bool append(const QByteArray &bytes);
    void fail(const QString &reason);
    QByteArray row(const VisualDiagnosticSnapshot &snapshot, bool transition) const;
    VisualCsvConfig config_;
    QFile file_;
    QTimer timer_;
    QString lastError_;
    bool recording_{false};
    qint64 bytesWritten_{0};
    std::optional<quint64> sessionId_;
    std::optional<quint64> highestFrameId_;
    std::optional<VisualState> previousState_;
    std::optional<ProposedCommand> previousEffective_;
};

} // namespace rb::vision
