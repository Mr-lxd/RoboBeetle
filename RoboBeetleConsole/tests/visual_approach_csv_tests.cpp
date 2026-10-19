#include "vision/VisualCsvLogger.h"
#include <QCoreApplication>
#include <QTemporaryDir>
#include <cstdio>

using namespace rb::vision;
int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    QTemporaryDir dir;
    int failures = 0;
    auto check = [&](bool ok, const char *why) {
        if (!ok) {
            std::fprintf(stderr, "FAIL: %s\n", why);
            ++failures;
        }
    };
    VisualCsvLogger logger;
    check(logger.start(dir.path()), "start logger");
    VisualDiagnosticSnapshot s;
    s.sessionId = 1;
    s.frameId = 1;
    s.localMonoMs = 50;
    s.state = VisualState::Tracking;
    s.command.proposed = s.command.effective = ProposedCommand::Forward;
    VisualApproachCsvInfo info;
    info.proportional = true;
    info.dryRun = true;
    info.area = 12;
    info.state = "APPROACH";
    info.setpoint = VisualSetpoint{450, 0, 500};
    info.frameId = 1;
    info.sessionId = 1;
    logger.record(s, info);
    for (int i = 0; i < 20; ++i) {
        s.localMonoMs += 50;
        logger.record(s, info);
    }
    auto event = info;
    event.event = "ARRIVED_ENTER";
    event.state = "ARRIVED";
    event.setpoint = VisualSetpoint{};
    event.nowMs = 1100;
    logger.recordApproachEvent(event);
    info.dryRun = false;
    info.submitted = true;
    logger.recordDispatch({1200, 3, 3, ProposedCommand::Forward, "SENT", 0xff, -1, info});
    logger.recordDispatch({1300, 3, 3, ProposedCommand::Forward, "OK", 0, 100, info});
    logger.stop();
    QFile f(logger.filePath());
    check(f.open(QIODevice::ReadOnly), "open CSV");
    const auto lines = f.readAll().trimmed().split('\n');
    check(lines.size() == 5, "same-frame timer evaluations do not add periodic rows");
    if (lines.size() == 5) {
        auto header = lines[0].split(',');
        check(header.size() == 59, "unified v8 has 59 columns");
        check(header[45] == "pitch" && header[54] == "pitch_setpoint", "IMU pitch and target pitch distinct");
        auto frame = lines[1].split(',');
        auto e = lines[2].split(',');
        auto sent = lines[3].split(',');
        auto outcome = lines[4].split(',');
        check(frame.size() == 59 && frame[21] == "visual-csv-v8" && frame[48] == "PROPORTIONAL" &&
                  frame[49] == "1" && frame[50] == "12" && frame[51] == "APPROACH" && frame[52] == "450" &&
                  frame[54] == "500" && frame[55] == "0",
              "paired Dry run frame values");
        check(e[0] == "event" && e[51] == "ARRIVED" && e[56] == "ARRIVED_ENTER", "sparse arrival event");
        check(sent[55] == "1" && outcome[55] == "0",
              "submission success does not imply outcome row submitted again");
    }
    return failures ? 1 : 0;
}
