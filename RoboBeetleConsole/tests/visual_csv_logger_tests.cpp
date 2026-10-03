#include "vision/VisualCsvLogger.h"
#include <QCoreApplication>
#include <QDir>
#include <QEventLoop>
#include <QFileInfo>
#include <QLocale>
#include <QRegularExpression>
#include <QStandardPaths>
#include <QTemporaryDir>
#include <cstdio>
#ifdef Q_OS_WIN
#include <windows.h>
#endif

namespace {
using namespace rb::vision;
int failures = 0;
const QByteArray header = "row_kind,local_mono_ms,frame_id,capture_ts_ns,n_detections,sel_class,sel_conf,u,v,ex,ey,ex_f,yaw_cmd,state,proposed_command,effective_command,policy_version,policy_hash,session_id,awaiting_video\n";
void check(bool ok, const char *why) {
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", why); ++failures; }
}
QByteArray contents(const QString &path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) return {};
    return file.readAll();
}
// Parse complete RFC-style quoted records, including embedded CR/LF.
QList<QStringList> csv(const QByteArray &bytes) {
    const QString text = QString::fromUtf8(bytes);
    QList<QStringList> rows;
    QStringList fields;
    QString field;
    bool quoted = false;
    for (qsizetype i = 0; i < text.size(); ++i) {
        const QChar c = text[i];
        if (c == QLatin1Char('"')) {
            if (quoted && i + 1 < text.size() && text[i + 1] == QLatin1Char('"')) {
                field += c; ++i;
            } else quoted = !quoted;
        } else if (!quoted && c == QLatin1Char(',')) {
            fields.append(field); field.clear();
        } else if (!quoted && c == QLatin1Char('\n')) {
            fields.append(field); rows.append(fields); fields.clear(); field.clear();
        } else field += c;
    }
    return rows;
}
VisualDiagnosticSnapshot sample(quint64 id = 10, quint64 session = 1) {
    VisualDiagnosticSnapshot s;
    s.sessionId = session;
    s.localMonoMs = 123;
    s.frameId = id;
    s.captureTimestampNs = 1000 + id;
    s.nDetections = 2;
    s.target = TargetState{id, 1000 + id, {640, 480},
        {3, QStringLiteral("fish"), 0.875, {120.5, 240.25}}, -0.6234375, 0.0010416666666667};
    s.state = VisualState::Tracking;
    s.command.ex_f = -0.25;
    s.command.yaw_cmd = -0.25;
    s.command.proposed = ProposedCommand::TurnLeft;
    s.command.effective = ProposedCommand::TurnLeft;
    s.policyVersion = QStringLiteral("visual-command-proposal-v1");
    s.policyHash = QString(64, QLatin1Char('a'));
    return s;
}
bool begin(VisualCsvLogger &logger, const QString &directory) {
    const bool ok = logger.start(directory);
    check(ok, "start opens a fresh CSV in the requested directory");
    return ok;
}
void defaultsAndFirstFrame() {
    QTemporaryDir dir;
    check(dir.isValid(), "temporary directory exists");
    VisualCsvConfig config;
    check(config.max_file_bytes == 32 * 1024 * 1024 && config.flush_ms == 250,
          "CSV defaults use a 32 MiB including-header cap and 250 ms flush");
    check(VisualCsvLogger::defaultDirectory() == QDir(QStandardPaths::writableLocation(
        QStandardPaths::DocumentsLocation)).filePath(QStringLiteral("RoboBeetle/visual-logs")),
        "default directory is Documents/RoboBeetle/visual-logs");
    VisualCsvLogger logger;
    int changes = 0;
    QObject::connect(&logger, &VisualCsvLogger::recordingChanged, [&] { ++changes; });
    logger.record(sample()); logger.flush(); logger.stop();
    check(!logger.isRecording() && logger.filePath().isEmpty() && logger.lastError().isEmpty()
          && changes == 0, "recording is off until start and idle operations do nothing");
    if (!begin(logger, QDir(dir.path()).filePath(QStringLiteral("nested/logs")))) return;
    check(logger.isRecording() && changes == 1 && logger.lastError().isEmpty(),
          "successful start emits one recording state change");
    logger.record(sample()); logger.flush();
    const auto bytes = contents(logger.filePath());
    check(bytes.startsWith(header), "header has the exact ordered 20 columns");
    const auto rows = csv(bytes);
    check(rows.size() == 2, "first target snapshot produces exactly one frame without initial transition");
    if (rows.size() >= 2) {
        check(rows[1].size() == 20, "frame has exactly 20 columns");
        if (rows[1].size() == 20) check(rows[1][0] == "frame" && rows[1][1] == "123"
            && rows[1][2] == "10" && rows[1][3] == "1010" && rows[1][4] == "2"
            && rows[1][5] == "fish" && rows[1][6] == "0.875" && rows[1][7] == "120.5"
            && rows[1][8] == "240.25" && rows[1][11] == "-0.25"
            && rows[1][13] == "TRACKING" && rows[1][14] == "TURN_LEFT"
            && rows[1][15] == "TURN_LEFT" && rows[1][18] == "1" && rows[1][19] == "0",
            "frame carries diagnostic values and policy/session provenance");
    }
    logger.stop(); logger.stop();
    check(!logger.isRecording() && changes == 2, "stop is idempotent and emits only one state change");
}
void frameDeduplicationAndSessionReset() {
    QTemporaryDir dir; VisualCsvLogger logger;
    if (!begin(logger, dir.path())) return;
    logger.record(sample(10)); logger.record(sample(10)); logger.record(sample(9));
    logger.record(sample(11)); logger.record(sample(0, 2)); logger.record(sample(0, 2));
    logger.record(sample(1, 2)); logger.stop();
    const auto rows = csv(contents(logger.filePath()));
    check(rows.size() == 5, "duplicate and old IDs are suppressed but a new session resets the ID floor including zero");
    if (rows.size() == 5) check(rows[1][2] == "10" && rows[2][2] == "11"
        && rows[3][2] == "0" && rows[3][18] == "2" && rows[4][2] == "1",
        "accepted frame rows preserve the actual frame/session IDs");
}
void transitionsAndHold() {
    QTemporaryDir dir; VisualCsvLogger logger;
    if (!begin(logger, dir.path())) return;
    auto s = sample(); s.frameId.reset(); s.captureTimestampNs.reset(); s.target.reset();
    logger.record(s); // Transition baseline does not require a frame.
    s.state = VisualState::NoTarget; s.command.proposed = ProposedCommand::Hold;
    logger.record(s); logger.record(s);
    s.command.proposed = ProposedCommand::Stop; logger.record(s); // proposal alone is not a transition
    s.command.effective = ProposedCommand::Forward; logger.record(s);
    s.state = VisualState::Lost; s.command.effective = ProposedCommand::Stop; logger.record(s);
    logger.stop();
    const auto rows = csv(contents(logger.filePath()));
    check(rows.size() == 4, "timer-only state/effective changes produce transitions; repeats/proposal-only changes do not");
    if (rows.size() == 4) {
        for (int r = 1; r < rows.size(); ++r) {
            check(rows[r].size() == 20 && rows[r][0] == "transition", "transition retains exact column count");
            for (int c = 2; c <= 10; ++c) check(rows[r][c].isEmpty(), "transition frame/target columns stay empty");
        }
        check(rows[1][14] == "HOLD" && rows[1][15] == "TURN_LEFT" && rows[1][11] == "-0.25",
              "HOLD records the retained effective command and filtered error");
    }
}
void awaitingVideoAndMissingTarget() {
    QTemporaryDir dir; VisualCsvLogger logger;
    if (!begin(logger, dir.path())) return;
    auto s = sample(); logger.record(s);
    s.frameId = 11; s.captureTimestampNs = 1011; s.awaitingVideo = true;
    logger.record(s); // Deliberately leave an old target in snapshot: logger must blank it.
    s.awaitingVideo = false; logger.record(s); // same ID when video catches up
    s.command.effective = ProposedCommand::Forward; logger.record(s);
    s.frameId = 12; s.target.reset(); logger.record(s);
    logger.stop();
    const auto rows = csv(contents(logger.filePath()));
    check(rows.size() == 5, "awaiting-video ID records once, video catchup can still produce a true transition");
    if (rows.size() == 5) {
        check(rows[2][0] == "frame" && rows[2][2] == "11" && rows[2][19] == "1"
              && rows[2][11] == "-0.25" && rows[2][13] == "TRACKING",
              "awaiting-video frame retains old state/filter and marks exclusion flag");
        for (int c = 5; c <= 10; ++c) check(rows[2][c].isEmpty() && rows[4][c].isEmpty(),
            "awaiting-video and null target frame raw target fields are blank");
        check(rows[3][0] == "transition" && rows[3][15] == "FORWARD", "catchup effective change is retained");
    }
}
void escapingAndLocale() {
    QTemporaryDir dir; VisualCsvLogger logger;
    if (!begin(logger, dir.path())) return;
    const QLocale oldLocale;
    QLocale::setDefault(QLocale(QLocale::German, QLocale::Germany));
    auto s = sample(); s.target->target.className = QString::fromUtf8("鱼,\"blue\"\r\nnext");
    s.policyVersion = QStringLiteral("v,\"1\"\nline");
    s.policyHash = QStringLiteral("hash\rline");
    logger.record(s); logger.stop(); QLocale::setDefault(oldLocale);
    const auto bytes = contents(logger.filePath()); const auto rows = csv(bytes);
    check(bytes.contains(QString::fromUtf8("鱼").toUtf8()) && bytes.contains("\"\"blue\"\""),
          "CSV is UTF-8 and doubles embedded quotes");
    check(rows.size() == 2 && rows[1].size() == 20, "embedded commas/quotes/newlines preserve one CSV record");
    if (rows.size() == 2 && rows[1].size() == 20) check(rows[1][5] == s.target->target.className
        && rows[1][16] == s.policyVersion && rows[1][17] == s.policyHash
        && rows[1][6] == "0.875" && rows[1][7] == "120.5", "escaped text roundtrips and numbers ignore system locale");
}
void periodicAndDestructorFlush() {
    QTemporaryDir dir; QString path; int changes = 0;
    {
        VisualCsvLogger logger;
        if (!begin(logger, dir.path())) return;
        QObject::connect(&logger, &VisualCsvLogger::recordingChanged, [&] { ++changes; });
        path = logger.filePath(); logger.record(sample());
        QEventLoop loop; QTimer::singleShot(300, &loop, &QEventLoop::quit); loop.exec();
        check(csv(contents(path)).size() == 2, "250 ms timer flush makes pending rows externally readable after 300 ms");
        logger.record(sample(11));
    }
    check(csv(contents(path)).size() == 3, "destructor flushes and closes the final pending row");
    check(changes == 0, "destructor closes without emitting recordingChanged");
}
void uniqueFilesPreserveExisting() {
    QTemporaryDir dir;
    const QString sentinelPath = QDir(dir.path()).filePath(QStringLiteral("visual-existing.csv"));
    QFile sentinel(sentinelPath); check(sentinel.open(QIODevice::WriteOnly), "sentinel file opens");
    sentinel.write("preserve me"); sentinel.close();
    VisualCsvLogger logger;
    if (!begin(logger, dir.path())) return;
    const QString first = logger.filePath(); logger.record(sample()); logger.stop();
    const QByteArray original = contents(first);
    if (!begin(logger, dir.path())) return;
    const QString second = logger.filePath(); logger.stop();
    check(first != second && contents(first) == original && contents(sentinelPath) == "preserve me",
          "restart uses a unique new file and preserves every existing file");
    const QRegularExpression name(QStringLiteral("^visual-\\d{8}T\\d{9}Z-[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}\\.csv$"));
    check(name.match(QFileInfo(first).fileName()).hasMatch(), "filename identifies UTC milliseconds and UUID");
}
void capAndErrors() {
    QTemporaryDir dir;
    VisualCsvConfig tiny; tiny.max_file_bytes = header.size() - 1;
    VisualCsvLogger tooSmall(tiny);
    check(!tooSmall.start(dir.path()) && !tooSmall.isRecording() && !tooSmall.lastError().isEmpty(),
          "header larger than cap fails explicitly before recording");
    check(QDir(dir.path()).entryList(QDir::Files).isEmpty(), "header-too-large rejection creates no output file");
    VisualCsvConfig exact; exact.max_file_bytes = header.size();
    VisualCsvLogger capped(exact);
    if (begin(capped, dir.path())) {
        int changes = 0;
        QObject::connect(&capped, &VisualCsvLogger::recordingChanged, [&] { ++changes; });
        capped.record(sample());
        const QString reason = capped.lastError(); capped.flush(); capped.stop(); capped.record(sample(11));
        check(!capped.isRecording() && !reason.isEmpty() && reason == capped.lastError() && changes == 1,
              "cap stops recording once and retains its explicit error after idle operations");
        check(contents(capped.filePath()) == header, "cap includes header and rejects whole rows without partial data or rollover");
        check(begin(capped, dir.path()) && capped.lastError().isEmpty(), "a deliberate restart clears the previous error");
        capped.stop();
    }
    VisualCsvLogger reference;
    if (begin(reference, dir.path())) {
        reference.record(sample()); reference.stop();
        const QByteArray oneRow = contents(reference.filePath());
        VisualCsvConfig oneRowCap; oneRowCap.max_file_bytes = oneRow.size();
        VisualCsvLogger exactRow(oneRowCap);
        if (begin(exactRow, dir.path())) {
            exactRow.record(sample());
            check(exactRow.isRecording(), "a complete row that fits the cap exactly is accepted");
            exactRow.record(sample(11));
            check(!exactRow.isRecording() && contents(exactRow.filePath()) == oneRow,
                  "next overflowing row is rejected before any bytes are written");
        }
    }
    const QString blocker = QDir(dir.path()).filePath(QStringLiteral("blocker"));
    QFile file(blocker); check(file.open(QIODevice::WriteOnly), "directory blocker opens"); file.write("x"); file.close();
    VisualCsvLogger badDirectory;
    check(!badDirectory.start(QDir(blocker).filePath(QStringLiteral("child")))
        && !badDirectory.isRecording() && !badDirectory.lastError().isEmpty(),
        "invalid requested directory reports error and remains stopped");
}
void operatingSystemIoFailure(bool flushFailure) {
#ifdef Q_OS_WIN
    QTemporaryDir dir; VisualCsvLogger logger;
    if (!begin(logger, dir.path())) return;
    logger.flush(); // Commit the header before faulting the next buffered write.
    const QString path = logger.filePath();
    const HANDLE blocker = CreateFileW(reinterpret_cast<LPCWSTR>(path.utf16()),
        GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
        nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    check(blocker != INVALID_HANDLE_VALUE, "native handle opens the real CSV for I/O failure test");
    if (blocker == INVALID_HANDLE_VALUE) return;
    // The independent handle's byte-range reservation makes QFile's real write fail.
    const bool locked = LockFile(blocker, 0, 0, MAXDWORD, 0) != 0;
    check(locked, "native file range can block QFile's pending disk write");
    if (locked) {
        int changes = 0;
        QObject::connect(&logger, &VisualCsvLogger::recordingChanged, [&] { ++changes; });
        auto s = sample();
        if (!flushFailure) s.policyVersion = QString(40 * 1024, QLatin1Char('x'));
        logger.record(s); logger.flush();
        const QString reason = logger.lastError(); logger.flush(); logger.stop();
        const QString prefix = flushFailure ? QStringLiteral("CSV flush failed:")
                                            : QStringLiteral("CSV write failed:");
        check(!logger.isRecording() && reason.startsWith(prefix)
              && logger.lastError() == reason && changes == 1,
              "real OS write/flush failure stops once and retains its explicit reason");
        UnlockFile(blocker, 0, 0, MAXDWORD, 0);
    }
    CloseHandle(blocker);
    check(contents(path) == header, "failed I/O creates no rollover and preserves previously committed header");
#else
    (void)flushFailure;
#endif
}
} // namespace
int main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    defaultsAndFirstFrame(); frameDeduplicationAndSessionReset(); transitionsAndHold();
    awaitingVideoAndMissingTarget(); escapingAndLocale(); periodicAndDestructorFlush();
    uniqueFilesPreserveExisting(); capAndErrors();
    operatingSystemIoFailure(true); operatingSystemIoFailure(false);
    std::fprintf(stdout, "visual_csv_logger_tests: %d failure(s)\n", failures);
    return failures ? 1 : 0;
}
