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
#include <cmath>
#ifdef Q_OS_WIN
#include <windows.h>
#endif

namespace {
using namespace rb::vision;
int failures = 0;
const QByteArray header = "row_kind,local_mono_ms,arrival_mono_ms,frame_id,capture_ts_ns,n_detections,sel_class,sel_conf,u,v,ex,ey,ex_f,yaw_cmd,state,proposed_command,effective_command,policy_version,policy_hash,session_id,awaiting_video,schema_version,assoc_status,assoc_dist_px,hc_u,hc_v,hc_conf,src_w,src_h,dets,policy_request_id,request_id,dispatch_command,dispatch_result,ack_rtt_ms,axis_mode,ey_f,pitch_sign,depth_raw_m,depth_cal_m,depth_age_ms,envelope_state,sel_area_cells\n";
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
        {3, QStringLiteral("fish"), 0.875, {120.5, 240.25}, 5}, -0.6234375, 0.0010416666666667};
    s.state = VisualState::Tracking;
    s.command.ex_f = -0.25;
    s.command.yaw_cmd = -0.25;
    s.command.proposed = ProposedCommand::TurnLeft;
    s.command.effective = ProposedCommand::TurnLeft;
    s.policyVersion = QStringLiteral("visual-command-proposal-v2");
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
    check(bytes.startsWith(header), "header has the exact ordered 43 columns");
    const auto rows = csv(bytes);
    check(rows.size() == 2, "first target snapshot produces exactly one frame without initial transition");
    if (rows.size() >= 2) {
        check(rows[1].size() == 43, "frame has exactly 43 columns");
        if (rows[1].size() == 43) check(rows[1][0] == "frame" && rows[1][1] == "123"
            && rows[1][3] == "10" && rows[1][4] == "1010" && rows[1][5] == "2"
            && rows[1][6] == "fish" && rows[1][7] == "0.875" && rows[1][8] == "120.5"
            && rows[1][9] == "240.25" && rows[1][12] == "-0.25"
            && rows[1][14] == "TRACKING" && rows[1][15] == "TURN_LEFT"
            && rows[1][16] == "TURN_LEFT" && rows[1][19] == "1" && rows[1][20] == "0",
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
    if (rows.size() == 5) check(rows[1][3] == "10" && rows[2][3] == "11"
        && rows[3][3] == "0" && rows[3][19] == "2" && rows[4][3] == "1",
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
            check(rows[r].size() == 43 && rows[r][0] == "transition", "transition retains exact column count");
            for (int c = 2; c <= 11; ++c) check(rows[r][c].isEmpty(), "transition frame/target columns stay empty");
        }
        check(rows[1][15] == "HOLD" && rows[1][16] == "TURN_LEFT" && rows[1][12] == "-0.25",
              "HOLD records the retained effective command and filtered error");
    }
}
void awaitingVideoAndMissingTarget() {
    QTemporaryDir dir; VisualCsvLogger logger;
    if (!begin(logger, dir.path())) return;
    auto s = sample(); logger.record(s);
    s.frameId = 11; s.captureTimestampNs = 1011; s.awaitingVideo = true;
    logger.record(s);
    logger.flush();
    check(csv(contents(logger.filePath())).size() == 2, "pending frame is not written at first arrival");
    s.awaitingVideo = false; s.localMonoMs += 15;
    s.target->frameId = 11;
    s.target->ex = 0.5; s.target->target.originalPoint.setX(480.0);
    s.command.ex_f = -0.025; s.command.yaw_cmd = -0.025;
    logger.record(s); logger.record(s); // first catchup only, never a duplicate
    s.command.effective = ProposedCommand::Forward; logger.record(s);
    s.frameId = 12; s.target.reset(); logger.record(s);
    logger.stop();
    const auto rows = csv(contents(logger.filePath()));
    check(rows.size() == 5, "awaiting-video ID records once, video catchup can still produce a true transition");
    if (rows.size() == 5 && rows[2].size() == 43 && rows[4].size() == 43) {
        check(rows[2][0] == "frame" && rows[2][3] == "11" && rows[2][20] == "0"
              && rows[2][6] == "fish" && rows[2][10].toDouble() == 0.5
              && std::abs(rows[2][12].toDouble() + 0.025) < 1e-12
              && rows[2][1].toLongLong() - rows[2][2].toLongLong() == 15,
              "15 ms catchup writes one fully evaluated sample with first arrival time");
        for (int c = 6; c <= 11; ++c) check(rows[4][c].isEmpty(),
            "null target frame raw target fields are blank");
        check(rows[3][0] == "transition" && rows[3][16] == "FORWARD", "catchup effective change is retained");
    }
}

void pendingReplacementAndInvalidation() {
    for (const auto state : {VisualState::Tracking, VisualState::Stale, VisualState::InferenceOff}) {
        QTemporaryDir dir; VisualCsvLogger logger;
        if (!begin(logger, dir.path())) return;
        auto s = sample(); s.awaitingVideo = true;
        logger.record(s);
        s.localMonoMs += 200; logger.record(s); // repeat does not renew arrival
        s.localMonoMs = 623;
        s.awaitingVideo = false;
        s.state = state;
        if (state == VisualState::Tracking) {
            s.frameId = 11; s.target->frameId = 11;
        } else {
            s.target.reset(); s.command.ex_f.reset(); s.command.yaw_cmd = 0;
            s.command.proposed = s.command.effective = ProposedCommand::Stop;
        }
        logger.record(s); logger.record(s); logger.stop();
        const auto rows = csv(contents(logger.filePath()));
        check(rows.size() == 3, "replacement or invalidation produces exactly two ordered rows");
        if (rows.size() != 3 || rows[1].size() != 43 || rows[2].size() != 43) continue;
        check(rows[1][0] == "frame" && rows[1][3] == "10" && rows[1][20] == "1"
              && rows[1][1] == "623" && rows[1][2] == "123",
              "fallback row precedes replacement/transition and retains original 500 ms arrival gap");
        for (int c = 6; c <= 12; ++c) check(rows[1][c].isEmpty(),
            "unprocessed pending frame has no selection, raw errors or filtered error");
        if (state == VisualState::Tracking) {
            check(rows[2][0] == "frame" && rows[2][3] == "11" && rows[2][20] == "0"
                  && rows[2][1] == rows[2][2], "new ID follows flushed pending ID at its own arrival");
        } else {
            check(rows[2][0] == "transition" && rows[2][14] == visualStateName(state)
                  && rows[2][16] == "STOP", "expired pending row precedes STALE/OFF STOP transition");
            for (int c = 2; c <= 11; ++c) check(rows[2][c].isEmpty(),
                "transition leaves arrival timestamp and all frame fields empty");
        }
    }
    QTemporaryDir dir; VisualCsvLogger logger;
    if (!begin(logger, dir.path())) return;
    auto s = sample(); s.awaitingVideo = true; logger.record(s);
    auto next = sample(0, 2); next.localMonoMs = 150;
    logger.record(next); logger.stop();
    const auto rows = csv(contents(logger.filePath()));
    check(rows.size() == 3 && rows[1].size() == 43 && rows[2].size() == 43 && rows[1][3] == "10" && rows[1][19] == "1"
          && rows[1][20] == "1" && rows[1][1] == "150" && rows[1][2] == "123"
          && rows[2][3] == "0" && rows[2][19] == "2",
          "session reset flushes the old pending ID before a low ID from the new session");
}

void csvReproducesEma() {
    QTemporaryDir dir; VisualCsvLogger logger;
    if (!begin(logger, dir.path())) return;
    VisualCommandMemory memory;
    VisualPolicyConfig policy;
    auto s = sample(1); s.localMonoMs = 0;
    auto evaluated = [&](quint64 id, qint64 now, double ex) {
        s.frameId = id; s.captureTimestampNs = 1000 + id; s.localMonoMs = now;
        s.awaitingVideo = false; s.target = sample(id).target;
        s.target->ex = ex; s.target->target.originalPoint.setX(320 * (1 + ex));
        s.command = evaluateVisualCommand(memory, {now, VisualState::Tracking, id, ex}, policy);
        memory = s.command.next; logger.record(s);
    };
    evaluated(1, 0, -0.6);
    s.frameId = 2; s.localMonoMs = 40; s.awaitingVideo = true; logger.record(s);
    evaluated(2, 55, -0.2);
    evaluated(3, 80, 0.1);
    s.frameId = 4; s.localMonoMs = 120; s.awaitingVideo = true; logger.record(s);
    evaluated(5, 160, 0.8); // ID4 was never used by policy; exclude its fallback row.
    logger.stop();
    const auto rows = csv(contents(logger.filePath()));
    std::optional<double> recomputed;
    int accepted = 0, awaiting = 0;
    for (qsizetype i = 1; i < rows.size(); ++i) {
        const auto &r = rows[i];
        if (r.size() != 43 || r[0] != "frame") continue;
        if (r[20] == "1") { ++awaiting; continue; }
        bool exOk = false, filteredOk = false;
        const double ex = r[10].toDouble(&exOk), filtered = r[12].toDouble(&filteredOk);
        recomputed = recomputed ? 0.3 * ex + 0.7 * *recomputed : ex;
        check(exOk && filteredOk && std::abs(filtered - *recomputed) < 1e-12,
              "CSV accepted ex sequence reproduces the controller EMA at alpha=0.3");
        if (r[3] == "2") check(r[1].toLongLong() - r[2].toLongLong() == 15,
            "EMA chain preserves arrival time for deferred evaluated frame");
        ++accepted;
    }
    check(accepted == 4 && awaiting == 1, "all four controller-used samples exist once, plus one excluded awaiting frame");
}

void closingPendingFrame() {
    for (bool explicitStop : {true, false}) {
        QTemporaryDir dir; QString path; int changes = 0;
        {
            VisualCsvLogger logger;
            if (!begin(logger, dir.path())) return;
            QObject::connect(&logger, &VisualCsvLogger::recordingChanged, [&] { ++changes; });
            path = logger.filePath();
            auto s = sample(); s.awaitingVideo = true; logger.record(s);
            s.localMonoMs += 10; logger.record(s);
            if (explicitStop) { logger.stop(); logger.stop(); }
        }
        const auto rows = csv(contents(path));
        check(rows.size() == 2 && rows[1].size() == 43 && rows[1][20] == "1"
              && rows[1][1] == "133" && rows[1][2] == "123",
              "stop or destructor flushes unresolved pending frame once at the last evaluation time");
        check(changes == (explicitStop ? 1 : 0),
              "explicit stop emits once; destructor-only pending finalization emits no UI signal");
    }
}

void arrivalBeforeRecording() {
    for (bool awaiting : {false, true}) {
        QTemporaryDir dir; VisualCsvLogger logger;
        auto s = sample(); s.awaitingVideo = awaiting;
        logger.record(s); // Display remains active while CSV is off.
        s.localMonoMs = 133; logger.record(s);
        if (!begin(logger, dir.path())) return;
        s.localMonoMs = 143; logger.record(s);
        if (awaiting) {
            s.localMonoMs = 158; s.awaitingVideo = false; logger.record(s);
        }
        logger.stop();
        const auto rows = csv(contents(logger.filePath()));
        check(rows.size() == 2 && rows[1].size() == 43 && rows[1][2] == "123"
              && rows[1][1] == (awaiting ? "158" : "143"),
              "enabling CSV preserves the ID arrival already observed while recording was off");
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
    check(rows.size() == 2 && rows[1].size() == 43, "embedded commas/quotes/newlines preserve one CSV record");
    if (rows.size() == 2 && rows[1].size() == 43) check(rows[1][6] == s.target->target.className
        && rows[1][17] == s.policyVersion && rows[1][18] == s.policyHash
        && rows[1][7] == "0.875" && rows[1][8] == "120.5", "escaped text roundtrips and numbers ignore system locale");
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
void associationRowsFromSession() {
    QTemporaryDir dir; VisualCsvLogger logger;
    if (!begin(logger,dir.path())) return;
    qint64 now=0; VisualDiagnosticSession session({},[&]{return now;});
    QObject::connect(&session,&VisualDiagnosticSession::diagnosticChanged,
                     [&](const auto &s){logger.record(s);});
    auto frame=[](quint64 id,double u=450) {
        return DetectionFrame{id,1000+id,{640,480},{{0,"fish",.89,{u,240}},{1,"other",.85,{547,240}}}};
    };
    auto f=frame(1); session.onDetectionArrival(f,{DetectionDisplayState::Target,f});
    now=40; f=frame(2); f.detections[0].confidence=.85; f.detections[1].confidence=.89;
    session.onDetectionArrival(f,{DetectionDisplayState::AwaitingVideo,f});
    now=55; session.refresh({DetectionDisplayState::Target,f});
    session.refresh({DetectionDisplayState::Target,f});
    now=80; f=frame(3,455); session.onDetectionArrival(f,{DetectionDisplayState::Target,f});
    now=120; f=frame(4,460); session.onDetectionArrival(f,{DetectionDisplayState::AwaitingVideo,f});
    now=160; f=frame(5,460); session.onDetectionArrival(f,{DetectionDisplayState::Target,f});
    now=200; f=frame(6,547); f.detections.removeLast();
    session.onDetectionArrival(f,{DetectionDisplayState::Target,f});
    now=450; f=frame(7); session.onDetectionArrival(f,{DetectionDisplayState::AwaitingVideo,f});
    now=950; session.refresh({DetectionDisplayState::AwaitingVideo,f});
    logger.stop(); const auto rows=csv(contents(logger.filePath()));
    std::optional<double> ema; int accepted=0,waiting=0,frame2=0;
    for(qsizetype i=1;i<rows.size();++i) {
        const auto &r=rows[i];
        check(r.size() == 43,"association schema has forty-three columns");
        if(r.size() != 43)continue;
        check(r[21]=="visual-csv-v5","every row including transition identifies schema");
        if(r[0]=="transition") {
            for(int col=23;col<30;++col)check(r[col].isEmpty(),"transition excludes new frame fields");
            continue;
        }
        check(r[27]=="640" && r[28]=="480" && !r[29].isEmpty(),"frame and discarded awaiting retain dimensions/all detections");
        for(const auto &entry:r[29].split(';')) {
            const auto values=entry.split(':');
            check(values.size()==5,"dets entries are replayable classId:confidence:u:v:area");
        }
        if(r[20]=="1") {
            ++waiting;
            for(int col=22;col<=26;++col)check(r[col].isEmpty(),"unprocessed awaiting has no association or highest selection");
            if(r[3]=="7")check(i+1<rows.size() && rows[i+1][0]=="transition"
                              && rows[i+1][14]=="STALE","500 ms waiting row precedes STALE transition");
            continue;
        }
        if(r[3]=="2") {
            ++frame2;
            check(r[22]=="ASSOCIATED" && r[8].toDouble()==450 && r[24].toDouble()==547
                  && r[1].toLongLong()-r[2].toLongLong()==15,"15 ms catchup logs actual lock and jumping hc once");
        }
        if(r[3]=="6")check(r[22]=="MISS" && r[8].isEmpty() && r[10].isEmpty()
                           && r[24].toDouble()==547 && r[15]=="HOLD","MISS logs hc but does not substitute target");
        if(!r[10].isEmpty()) {
            const double ex=r[10].toDouble(); ema=ema?.3*ex+.7*(*ema):ex;
            check(std::abs(r[12].toDouble()-*ema)<1e-12,"actual association session CSV preserves EMA chain");
            ++accepted;
        }
    }
    check(frame2==1 && accepted==4 && waiting==2,"once-per-ID evaluated and discarded frame counts");
}

void dispatchRowsPreserveDiagnostics() {
    QTemporaryDir dir; VisualCsvLogger logger;
    if (!begin(logger, dir.path())) return;
    auto s = sample(); s.awaitingVideo = true;
    logger.record(s);
    logger.recordDispatch({124, 91, 501, ProposedCommand::Forward, "SENT", 0xff, -1});
    logger.recordDispatch({130, 91, 501, ProposedCommand::Forward, "OK", 0, 6});
    logger.recordDispatch({140, 92, 502, ProposedCommand::Stop, "SENT", 0xff, -1});
    logger.recordDispatch({150, 92, 502, ProposedCommand::Stop, "BUSY", 7, 10});
    logger.recordDispatch({160, 93, 503, ProposedCommand::Stop, "SENT", 0xff, -1});
    logger.recordDispatch({170, 93, 503, ProposedCommand::Stop, "REJECTED", 6, 10});
    logger.recordDispatch({180, 94, 504, ProposedCommand::Stop, "SENT", 0xff, -1});
    logger.recordDispatch({190, 94, 504, ProposedCommand::Stop, "OUTCOME_UNKNOWN", 0xff, -1});
    logger.recordDispatch({200, 0, std::nullopt, ProposedCommand::Stop, "LOCAL_REJECTED", 0xff, -1});
    s.awaitingVideo = false; s.localMonoMs = 210;
    logger.record(s); logger.record(s); // cached ID is still logged exactly once
    s.state = VisualState::Lost; s.localMonoMs = 220; logger.record(s);
    logger.stop();
    const auto rows = csv(contents(logger.filePath()));
    check(rows.size() == 12, "nine dispatch events plus unresolved frame and one transition");
    if (rows.size() != 12) return;
    const auto col = [&](const char *name) { return rows[0].indexOf(QString::fromLatin1(name)); };
    for (const auto &r : rows) check(r.size() == 43, "every v5 record has 43 columns");
    check(rows[1][col("policy_request_id")] == "91" && rows[1][col("request_id")] == "501"
          && rows[1][col("dispatch_command")] == "FORWARD" && rows[1][col("dispatch_result")] == "SENT"
          && rows[1][col("ack_rtt_ms")].isEmpty(), "send row records policy and wire IDs with no invented RTT");
    check(rows[2][col("dispatch_result")] == "OK" && rows[2][col("ack_rtt_ms")] == "6",
          "terminal row retains measured ACK RTT");
    check(rows[8][col("dispatch_result")] == "OUTCOME_UNKNOWN" && rows[8][col("ack_rtt_ms")].isEmpty(),
          "unmeasured unknown leaves RTT blank");
    check(rows[9][col("policy_request_id")] == "0" && rows[9][col("request_id")].isEmpty()
          && rows[9][col("dispatch_result")] == "LOCAL_REJECTED", "operator ID zero and local rejection distinguished");
    check(rows[10][col("row_kind")] == "frame" && rows[10][col("arrival_mono_ms")] == "123"
          && rows[10][col("local_mono_ms")] == "210", "dispatch does not retime or consume pending frame");
    for (int i : {10,11}) for (int c = 30; c < 35; ++c)
        check(rows[i][c].isEmpty(), "diagnostic rows leave all dispatch columns blank");
    check(rows[11][col("row_kind")] == "transition", "dispatch preserves diagnostic transition baseline");
    for (int i=1; i<rows.size(); ++i) check(rows[i][col("schema_version")] == "visual-csv-v5", "all rows carry v5 schema");
}

// Task 06 PR-C: schema v4 appends axis, vertical and depth columns after the 35 v3 columns.
void pitchAndDepthColumns() {
    QTemporaryDir dir;
    VisualCsvLogger logger;
    if (!begin(logger, dir.path())) return;
    // No provider: depth columns stay blank, axis columns are always filled.
    auto yaw = sample(10);
    logger.record(yaw);
    // With a provider and a pitch snapshot.
    logger.setDepthInfoProvider([] {
        VisualDepthCsvInfo info;
        info.rawM = 0.25; info.calibratedM = 0.125; info.ageMs = 87; info.envelope = QStringLiteral("NORMAL");
        return info;
    });
    auto pitch = sample(11);
    pitch.localMonoMs = 223;
    pitch.axis = VisualAxisMode::Pitch;
    pitch.pitchSign = 1;
    pitch.command.ey_f = 0.5;
    pitch.command.proposed = ProposedCommand::Descend;
    pitch.command.effective = ProposedCommand::Descend;
    logger.record(pitch);
    logger.flush();
    const auto rows = csv(contents(logger.filePath()));
    check(rows.size() >= 4, "pitch fixture produced frame and transition rows");
    if (rows.size() < 4) return;
    const auto col = [&](const char *name) { return rows[0].indexOf(QString::fromLatin1(name)); };
    check(rows[0].size() == 43 && col("axis_mode") == 35 && col("envelope_state") == 41 && col("sel_area_cells") == 42,
          "v4 appends seven columns after ack_rtt_ms without moving the old ones");
    check(rows[1][col("axis_mode")] == "YAW" && rows[1][col("pitch_sign")] == "1"
              && rows[1][col("ey_f")].isEmpty() && rows[1][col("depth_raw_m")].isEmpty()
              && rows[1][col("envelope_state")].isEmpty(),
          "no depth provider leaves the depth columns blank");
    int frames = 0;
    for (const auto &row : rows) {
        if (row.size() != 43 || row[0] != "frame" || row[col("axis_mode")] != "PITCH") continue;
        ++frames;
        check(row[col("ey_f")] == "0.5" && row[col("pitch_sign")] == "1"
                  && row[col("depth_raw_m")] == "0.25" && row[col("depth_cal_m")] == "0.125"
                  && row[col("depth_age_ms")] == "87" && row[col("envelope_state")] == "NORMAL"
                  && row[col("effective_command")] == "DESCEND",
              "a pitch frame carries ey_f, pitch_sign and the depth columns");
    }
    check(frames == 1, "exactly one pitch frame row");
    check(rows[1][col("sel_area_cells")] == "5",
          "frame row carries the selected target's area");
    for (const auto &row : rows) {
        if (row.size() == 43 && row[0] == "transition")
            check(row[col("axis_mode")] == "YAW" || row[col("axis_mode")] == "PITCH",
                  "transition rows carry the axis too");
    }
    // Dispatch/outcome rows leave the v4 columns blank.
    logger.recordDispatch({300, 7, 5u, ProposedCommand::Descend, QStringLiteral("SENT"), 0xff, -1});
    logger.flush();
    const auto after = csv(contents(logger.filePath()));
    const auto &last = after.last();
    bool blank = last.size() == 43 && last[0] == "dispatch" && last[col("dispatch_command")] == "DESCEND";
    for (int c = 35; c < 43 && last.size() == 43; ++c) blank = blank && last[c].isEmpty();
    check(blank, "dispatch rows record DESCEND and leave all v4 columns blank");
}

int main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    pitchAndDepthColumns();
    dispatchRowsPreserveDiagnostics();
    associationRowsFromSession();
    defaultsAndFirstFrame(); frameDeduplicationAndSessionReset(); transitionsAndHold();
    awaitingVideoAndMissingTarget(); escapingAndLocale(); periodicAndDestructorFlush();
    pendingReplacementAndInvalidation(); csvReproducesEma(); closingPendingFrame();
    arrivalBeforeRecording();
    uniqueFilesPreserveExisting(); capAndErrors();
    operatingSystemIoFailure(true); operatingSystemIoFailure(false);
    std::fprintf(stdout, "visual_csv_logger_tests: %d failure(s)\n", failures);
    return failures ? 1 : 0;
}
