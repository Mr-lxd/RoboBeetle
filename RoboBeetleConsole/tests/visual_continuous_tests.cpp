#include "helpers/VisualControllerFixture.h"
#include "vision/VisualCsvLogger.h"
#include "vision/VisualDispatchSession.h"
#include <QCoreApplication>
#include <QFile>
#include <QTemporaryDir>
#include <cstdio>

using namespace rb;
using namespace rb::vision;
namespace
{
int failures = 0;
void check(bool condition, const char *message)
{
    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", message);
    }
}
void mapping()
{
    VisualApproachConfig c;
    const auto map = [&](double ex, double ey, VisualAxisMode axis = VisualAxisMode::Both,
                         DepthEnvelopeState depth = DepthEnvelopeState::Normal, int ts = 1, int ps = 1) {
        return mapContinuousVisualSetpoint(5, ex, ey, axis, ts, ps, depth, c);
    };
    auto v = map(.05, -.05);
    check(v.throttle == 600 && v.turn == 0 && v.pitch == 0, "inclusive deadzone");
    v = map(.225, .225);
    check(v.turn == 500 && v.pitch == -250, "linear errors right/down and Both simultaneous");
    v = map(-.8, -.8);
    check(v.turn == -1000 && v.pitch == 500, "saturation left/up");
    v = map(.225, .225, VisualAxisMode::Yaw);
    check(v.turn == 500 && v.pitch == 0, "Yaw masks pitch");
    v = map(.225, .225, VisualAxisMode::Pitch);
    check(v.turn == 0 && v.pitch == -250, "Pitch masks turn");
    v = map(.4, .4, VisualAxisMode::Both, DepthEnvelopeState::Normal, -1, -1);
    check(v.turn == -1000 && v.pitch == 500, "configured signs applied");
    check(map(0, -.4, VisualAxisMode::Both, DepthEnvelopeState::Surface).pitch == 0,
          "Surface clamps positive pitch");
    check(map(0, .4, VisualAxisMode::Both, DepthEnvelopeState::SoftFloor).pitch == 0,
          "SoftFloor clamps negative pitch");
    c.tMin = .05;
    c.tTurnMin = .4;
    v = mapContinuousVisualSetpoint(20, .225, 0, VisualAxisMode::Yaw, 1, 1, DepthEnvelopeState::Normal, c);
    check(v.throttle == 200, "turn floor is weighted by absolute normalized turn");
    v = mapContinuousVisualSetpoint(20, 1, 1, VisualAxisMode::Both, 1, 1, DepthEnvelopeState::Normal, c,
                                    true);
    check(v.throttle == 0 && v.turn == 0 && v.pitch == 0, "Arrived has all-zero setpoint");
}
struct Controller : test::VisualControllerFixture {
    bool prop{};
    int starts{}, stops{}, streams{};
    VisualSetpoint latest{};
    Controller()
    {
        enabled = known = 0x1f;
        depthSample = DepthControlSample{.2, .1, 0, true};
    }
    bool hasCpgSchema2() const override { return true; }
    quint8 cpgFeatureLevel() const override { return 2; }
    bool proportionalActive() const override { return prop; }
    std::optional<quint32> submitVisualProportionalStart(const ProportionalConfig &) override
    {
        ++starts;
        return next++;
    }
    std::optional<quint32> submitVisualProportionalStop() override
    {
        ++stops;
        prop = motionActive = false;
        return next++;
    }
    bool setProportionalInput(quint16 t, qint16 u, qint16 p) override
    {
        ++streams;
        latest = {t, u, p};
        return prop;
    }
};
VisualDiagnosticSnapshot sample()
{
    VisualDiagnosticSnapshot s;
    s.sessionId = 1;
    s.frameId = 1;
    s.state = VisualState::Tracking;
    s.target = TargetState{1, 1, {640, 480}, {1, "fish", .99, {320, 240}, 5}, 0, 0};
    s.command.proposed = s.command.effective = ProposedCommand::TurnLeft;
    s.command.ex_f = .225;
    s.command.ey_f = .225;
    return s;
}
void sessionAndCsv()
{
    Controller c;
    auto snapshot = sample();
    qint64 now = 0;
    VisualDispatchSession s(&c, [&] { return std::optional{snapshot}; }, [&] { return now; });
    s.setTimerEnabled(false);
    s.setExecutionMode(VisualExecutionMode::Continuous);
    s.setAxisMode(VisualAxisMode::Both);
    s.setFeatureEnabled(true);
    bool parametersLogged = false;
    QObject::connect(&s, &VisualDispatchSession::approachEvent, [&](const VisualApproachCsvInfo &i) {
        if (i.event == "CONFIG")
            parametersLogged = i.detail.contains("e_dead=") && i.detail.contains("e_full=");
    });
    s.setApproachConfig(s.approachConfig());
    check(parametersLogged, "CONFIG includes Continuous thresholds");
    check(s.arm() == ArmReason::Ready && c.starts == 1, "Continuous uses ordinary proportional Start");
    c.prop = c.motionActive = true;
    c.ack(1, CommandTerminalResult::Ok);
    for (now = 50; now <= 250; now += 50)
        s.timerTick();
    check(c.streams == 5 && c.latest.turn == 500 && c.latest.pitch == -250,
          "Continuous refreshes each evaluation and ignores discrete selection/dwell");
    snapshot.command.ex_f = -.225;
    now = 300;
    s.timerTick();
    check(c.latest.turn == -500, "Continuous sign change has no command-switch dwell");
    c.publishDepth(DepthControlSample{.26, .26, 0, true});
    check(s.armed() && c.stops == 1, "shared SoftFloor stops previously submitted forbidden pitch and keeps Arm");
    c.publishDepth(DepthControlSample{.36, .36, 0, true});
    check(!s.armed() && c.stops == 1, "shared HardLimit safety stops and disarms");

    for (const auto mode : {VisualExecutionMode::Proportional, VisualExecutionMode::Continuous}) {
        Controller dry;
        auto frame = sample();
        qint64 time = 0;
        VisualDispatchSession session(&dry, [&] { return std::optional{frame}; }, [&] { return time; });
        session.setTimerEnabled(false);
        session.setExecutionMode(mode);
        session.setDryRun(true);
        session.setFeatureEnabled(true);
        check(session.arm() == ArmReason::Ready, "dry session arms");
        check(session.approachInfo().area.has_value(), "armed session has computed area");
        session.disarm();
        const auto info = session.approachInfo();
        check(!info.area && info.state.isEmpty() && !info.setpoint,
              "unarmed no-STOP info excludes old execution values");
        QTemporaryDir dir;
        VisualCsvLogger logger;
        check(logger.start(dir.path()), "CSV starts");
        logger.record(frame, info);
        logger.stop();
        QFile file(logger.filePath());
        check(file.open(QIODevice::ReadOnly), "CSV opens");
        const auto rows = file.readAll().trimmed().split('\n');
        check(rows.size() == 2, "one unarmed frame row");
        if (rows.size() != 2)
            continue;
        const auto fields = rows[1].split(',');
        check(fields.size() == 59, "v8 column count");
        if (fields.size() != 59)
            continue;
        check(fields[48] == (mode == VisualExecutionMode::Continuous ? "CONTINUOUS" : "PROPORTIONAL") &&
                  fields[49] == "1",
              "mode and dry flag retained while unarmed");
        for (int col = 50; col <= 54; ++col)
            check(fields[col].isEmpty(), "unarmed frame execution column empty");
        check(dry.starts + dry.stops + dry.streams == 0, "Continuous Dry run has zero controller calls");
    }
}
void continuousHoldTracksHeldError()
{
    Controller c;
    auto snapshot = sample();
    qint64 now = 0;
    VisualDispatchSession s(&c, [&] { return std::optional{snapshot}; }, [&] { return now; });
    s.setTimerEnabled(false);
    s.setExecutionMode(VisualExecutionMode::Continuous);
    s.setAxisMode(VisualAxisMode::Yaw);
    s.setFeatureEnabled(true);
    check(s.arm() == ArmReason::Ready, "continuous arms");
    c.prop = c.motionActive = true;
    c.ack(1, CommandTerminalResult::Ok);
    now = 50;
    s.timerTick();
    check(c.latest.turn == 500, "tracked error streams its turn");
    // Detector misses: the policy's held ex_f follows the gyro heading.
    snapshot.state = VisualState::NoTarget;
    snapshot.target.reset();
    snapshot.command.proposed = ProposedCommand::Hold;
    snapshot.command.ex_f = .1;
    now = 100;
    s.timerTick();
    check(c.latest.turn == 143 && c.latest.throttle >= 150 && s.armed() && c.stops == 0,
          "NO_TARGET hold eases the turn to the dead-reckoned error without stopping");
    snapshot.command.ex_f.reset();
    now = 150;
    s.timerTick();
    check(c.latest.turn == 143 && c.stops == 0, "a hold without ex_f keeps the last setpoint");
    now = 1700;
    s.timerTick();
    check(c.stops == 1, "NO_TARGET beyond the lost time still stops");
}
} // namespace
int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    mapping();
    sessionAndCsv();
    continuousHoldTracksHeldError();
    return failures ? 1 : 0;
}
