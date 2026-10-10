#pragma once
#include "robot/MotionStateMonitor.h"
#include <QFile>
#include "robobeetle/protocol/cpg_parameters.hpp"
#include <QObject>
#include <QTimer>

namespace rb {
class MotionStateCsvLogger final : public QObject {
    Q_OBJECT
public:
    explicit MotionStateCsvLogger(QObject *parent = nullptr);
    ~MotionStateCsvLogger() override;
    bool start(const QString &directory, const QString &sessionId);
    void record(const MotionStateRecord &);
    void recordParameters(quint32 epoch, quint64 piRxMs, const robobeetle::protocol::CpgParametersSnapshot &);
    // Empty sample columns distinguish end/epoch totals from gyro rows.
    void recordTotals(const MotionStateMonitor &);
    void stop();
    bool isRecording() const { return file_.isOpen(); }
    QString filePath() const { return file_.fileName(); }
    QString lastError() const { return error_; }
signals:
    void failed();
private:
    bool append(const QByteArray &);
    void fail(const QString &);
    QFile file_;
    QTimer timer_;
    QString session_, error_;
    qint64 bytes_{};
    std::optional<std::pair<quint32,quint16>> recordedParameters_;
};
} // namespace rb
