#include "vision/VisualCsvLogger.h"
#include <QDateTime>
#include <QDir>
#include <QStandardPaths>
#include <QStringList>
#include <QUuid>

namespace rb::vision {
namespace {
const QByteArray header = "row_kind,local_mono_ms,frame_id,capture_ts_ns,n_detections,sel_class,sel_conf,u,v,ex,ey,ex_f,yaw_cmd,state,proposed_command,effective_command,policy_version,policy_hash,session_id,awaiting_video\n";

QString escaped(QString field)
{
    if (field.contains(QLatin1Char(',')) || field.contains(QLatin1Char('"'))
        || field.contains(QLatin1Char('\r')) || field.contains(QLatin1Char('\n'))) {
        field.replace(QStringLiteral("\""), QStringLiteral("\"\""));
        return QLatin1Char('"') + field + QLatin1Char('"');
    }
    return field;
}
QString number(double value)
{
    // QString::number uses the C locale regardless of the user's locale.
    return QString::number(value, 'g', 17);
}
}

VisualCsvLogger::VisualCsvLogger(VisualCsvConfig config, QObject *parent)
    : QObject(parent), config_(config)
{
    timer_.setTimerType(Qt::PreciseTimer);
    connect(&timer_, &QTimer::timeout, this, &VisualCsvLogger::flush);
}

VisualCsvLogger::~VisualCsvLogger()
{
    timer_.stop();
    if (file_.isOpen()) {
        // Destruction closes pending output without notifying a disappearing UI.
        if (!file_.flush()) lastError_ = QStringLiteral("CSV flush failed: %1").arg(file_.errorString());
        file_.close();
    }
}

QString VisualCsvLogger::defaultDirectory()
{
    return QDir(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation))
        .filePath(QStringLiteral("RoboBeetle/visual-logs"));
}

bool VisualCsvLogger::start(const QString &directory)
{
    if (recording_) return true;
    lastError_.clear();
    if (config_.max_file_bytes < header.size()) {
        fail(QStringLiteral("CSV file size limit is smaller than the header (%1 bytes).")
             .arg(header.size()));
        return false;
    }
    if (config_.flush_ms <= 0) {
        fail(QStringLiteral("CSV flush interval must be positive."));
        return false;
    }
    const QString requested = directory.isEmpty() ? defaultDirectory() : directory;
    if (!QDir().mkpath(requested)) {
        fail(QStringLiteral("Cannot create CSV directory: %1").arg(requested));
        return false;
    }
    const QString name = QStringLiteral("visual-%1-%2.csv")
        .arg(QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMdd'T'HHmmsszzz'Z'")),
             QUuid::createUuid().toString(QUuid::WithoutBraces));
    file_.setFileName(QDir(requested).absoluteFilePath(name));
    // NewOnly rejects an existing name without truncating it.
    if (!file_.open(QIODevice::WriteOnly | QIODevice::NewOnly)) {
        fail(QStringLiteral("Cannot open CSV file %1: %2").arg(file_.fileName(), file_.errorString()));
        return false;
    }
    bytesWritten_ = 0;
    sessionId_.reset();
    highestFrameId_.reset();
    previousState_.reset();
    previousEffective_.reset();
    if (!append(header)) return false;
    recording_ = true;
    timer_.start(config_.flush_ms);
    emit recordingChanged();
    return true;
}

bool VisualCsvLogger::append(const QByteArray &bytes)
{
    if (bytes.size() > config_.max_file_bytes - bytesWritten_) {
        fail(QStringLiteral("CSV file size limit reached (%1 bytes); recording stopped.")
             .arg(config_.max_file_bytes));
        return false;
    }
    if (file_.write(bytes) != bytes.size()) {
        fail(QStringLiteral("CSV write failed: %1").arg(file_.errorString()));
        return false;
    }
    bytesWritten_ += bytes.size();
    return true;
}

QByteArray VisualCsvLogger::row(const VisualDiagnosticSnapshot &s, bool transition) const
{
    QStringList fields;
    fields.reserve(20);
    fields << (transition ? QStringLiteral("transition") : QStringLiteral("frame"))
           << QString::number(s.localMonoMs);
    fields << (!transition && s.frameId ? QString::number(*s.frameId) : QString{})
           << (!transition && s.captureTimestampNs ? QString::number(*s.captureTimestampNs) : QString{})
           << (!transition ? QString::number(s.nDetections) : QString{});
    if (!transition && !s.awaitingVideo && s.target) {
        const auto &target = *s.target;
        fields << target.target.className << number(target.target.confidence)
               << number(target.target.originalPoint.x()) << number(target.target.originalPoint.y())
               << number(target.ex) << number(target.ey);
    } else {
        for (int i = 0; i < 6; ++i) fields << QString{};
    }
    fields << (s.command.ex_f ? number(*s.command.ex_f) : QString{})
           << number(s.command.yaw_cmd)
           << QString::fromLatin1(visualStateName(s.state))
           << QString::fromLatin1(proposedCommandName(s.command.proposed))
           << QString::fromLatin1(proposedCommandName(s.command.effective))
           << s.policyVersion << s.policyHash << QString::number(s.sessionId)
           << (s.awaitingVideo ? QStringLiteral("1") : QStringLiteral("0"));
    for (QString &field : fields) field = escaped(std::move(field));
    return (fields.join(QLatin1Char(',')) + QLatin1Char('\n')).toUtf8();
}

void VisualCsvLogger::record(const VisualDiagnosticSnapshot &s)
{
    if (!recording_) return;
    if (!sessionId_ || *sessionId_ != s.sessionId) {
        sessionId_ = s.sessionId;
        highestFrameId_.reset();
    }
    if (s.frameId && (!highestFrameId_ || *s.frameId > *highestFrameId_)) {
        if (!append(row(s, false))) return;
        highestFrameId_ = s.frameId;
    }
    if (previousState_ && (*previousState_ != s.state || *previousEffective_ != s.command.effective)) {
        if (!append(row(s, true))) return;
    }
    previousState_ = s.state;
    previousEffective_ = s.command.effective;
}

void VisualCsvLogger::fail(const QString &reason)
{
    lastError_ = reason;
    timer_.stop();
    file_.close();
    recording_ = false;
    emit recordingChanged();
}

void VisualCsvLogger::flush()
{
    if (recording_ && !file_.flush()) {
        fail(QStringLiteral("CSV flush failed: %1").arg(file_.errorString()));
    }
}

void VisualCsvLogger::stop()
{
    if (!recording_) return;
    timer_.stop();
    flush();
    if (!recording_) return; // Flush failure already closed and signalled.
    file_.close();
    recording_ = false;
    emit recordingChanged();
}
} // namespace rb::vision
