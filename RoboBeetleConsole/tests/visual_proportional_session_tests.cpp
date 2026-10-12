#include "helpers/VisualControllerFixture.h"
#include "vision/VisualDispatchSession.h"
#include <QCoreApplication>
#include <cstdio>

using namespace rb;
using namespace rb::vision;
namespace
{
int failures = 0;
void check(bool value, const char *name)
{
    if (!value) {
        std::fprintf(stderr, "FAIL: %s\n", name);
        ++failures;
    }
}
struct Controller : test::VisualControllerFixture {
    bool prop{}, pending{}, reject{};
    int starts{}, stops{}, streams{};
    quint8 reason{};
    VisualSetpoint latest{};
    Controller() { enabled = known = 0x1f; }
    bool hasCpgSchema2() const override { return true; }
    quint8 cpgFeatureLevel() const override { return 2; }
    bool proportionalActive() const override { return prop; }
    bool proportionalPending() const override { return pending; }
    quint8 proportionalStopReason() const override { return reason; }
    std::optional<quint32> submitVisualProportionalStart(const ProportionalConfig &) override
    {
        ++starts;
        if (reject)
            return {};
        pending = true;
        return next++;
    }
    std::optional<quint32> submitVisualProportionalStop() override
    {
        ++stops;
        pending = prop = false;
        motionActive = false;
        return next++;
    }
    bool setProportionalInput(quint16 t, qint16 u, qint16 p) override
    {
        ++streams;
        latest = {t, u, p};
        return prop;
    }
    void started(quint32 id = 1)
    {
        prop = true;
        pending = false;
        motionActive = true;
        ack(id, CommandTerminalResult::Ok);
    }
};
VisualDiagnosticSnapshot sample(quint64 frame, int area = 5)
{
    VisualDiagnosticSnapshot s;
    s.sessionId = 1;
    s.frameId = frame;
    s.state = VisualState::Tracking;
    s.target = TargetState{frame, frame, {640, 480}, {0, "fish", .99, {320, 240}, area}, 0, 0};
    s.command.proposed = s.command.effective = ProposedCommand::Forward;
    return s;
}
struct Fixture {
    Controller c;
    qint64 now{};
    VisualDiagnosticSnapshot s = sample(1);
    VisualDispatchSession session{&c, [this] { return std::optional{s}; }, [this] { return now; }};
    Fixture()
    {
        session.setTimerEnabled(false);
        session.setExecutionMode(VisualExecutionMode::Proportional);
        session.setFeatureEnabled(true);
    }
    void arm() { check(session.arm() == ArmReason::Ready, "eligible proportional Arm"); }
    void tick(qint64 n)
    {
        now = n;
        session.timerTick();
    }
};
void streamAndGrace()
{
    Fixture f;
    f.arm();
    f.tick(0);
    check(f.c.starts == 1, "ordinary Start submitted");
    f.c.started();
    int before = f.c.streams;
    for (int n = 50; n <= 250; n += 50)
        f.tick(n);
    check(f.c.streams == before + 5, "every evaluation refreshes latest slot");
    auto previous = f.c.latest;
    f.s.state = VisualState::NoTarget;
    f.s.target.reset();
    f.tick(300);
    f.tick(1750);
    check(f.c.latest.throttle == previous.throttle && f.session.armed(),
          "NO_TARGET grace keeps complete setpoint");
    f.s.state = VisualState::Lost;
    f.tick(1800);
    check(f.c.stops == 1 && f.session.armed(), "LOST STOP keeps armed");
}
void safetyAndDry()
{
    Fixture f;
    f.session.setDryRun(true);
    f.c.connected = f.c.active = false;
    f.c.enabled = f.c.known = 0;
    f.arm();
    f.tick(0);
    f.tick(50);
    f.s = sample(2, 40);
    f.tick(100);
    f.s = sample(3, 40);
    f.tick(150);
    f.tick(1150);
    check(f.session.approachInfo().state == "ARRIVED", "Dry run reaches arrived");
    f.session.manualInput(ManualInputKind::Motion);
    f.session.disarm();
    check(f.c.starts + f.c.stops + f.c.streams == 0 && f.c.sends.empty(),
          "Dry run invokes no controller send ports");
    Fixture large;
    large.s = sample(1, 20);
    large.arm();
    large.tick(0);
    check(large.c.starts == 0 && large.session.approachInfo().state == "ARRIVED",
          "initial large target never starts");
    Fixture stale;
    stale.arm();
    stale.tick(0);
    stale.c.started();
    stale.s.state = VisualState::Stale;
    stale.tick(50);
    check(!stale.session.armed() && stale.c.stops == 1, "STALE stops and disarms");
    Fixture manual;
    manual.arm();
    manual.tick(0);
    manual.c.started();
    manual.session.manualInput(ManualInputKind::Motion);
    manual.tick(50);
    check(!manual.session.armed() && manual.c.stops == 1, "manual takeover sends one STOP");
    Fixture failed;
    failed.c.reject = true;
    failed.arm();
    failed.tick(0);
    check(!failed.session.armed() && failed.c.stops == 1, "local Start failure STOP and disarms");
}
void areaAndDepth()
{
    Fixture f;
    int missingEvents = 0;
    QObject::connect(&f.session, &VisualDispatchSession::approachEvent, [&](const VisualApproachCsvInfo &i) {
        if (i.event == "AREA_MISSING_LOST")
            ++missingEvents;
    });
    f.arm();
    f.tick(0);
    f.c.started();
    f.s.target->target.areaCells.reset();
    f.s.target->frameId = 2;
    f.tick(50);
    f.tick(550);
    check(f.c.stops == 0, "missing area boundary retains latest");
    f.tick(551);
    check(f.c.stops == 1 && f.session.armed(), "missing area over500ms is LOST");
    f.tick(1200);
    f.tick(2000);
    check(missingEvents == 1, "one event per continuous missing-area episode");
    Fixture d;
    d.session.setAxisMode(VisualAxisMode::Pitch);
    d.c.depthSample = DepthControlSample{.2, .1, 0, true};
    d.s.command.proposed = ProposedCommand::Ascend;
    d.arm();
    d.tick(0);
    d.c.started();
    d.tick(50);
    check(d.c.latest.pitch > 0, "ASCEND positive pitch");
    d.s.command.proposed = ProposedCommand::Forward;
    d.c.publishDepth(DepthControlSample{.12, .01, 0, true});
    check(d.c.stops == 1 && d.session.armed(), "old in-flight ASCEND causes immediate boundary STOP");
    Fixture pending;
    pending.session.setAxisMode(VisualAxisMode::Pitch);
    pending.c.depthSample = DepthControlSample{.2, .1, 0, true};
    pending.s.command.proposed = ProposedCommand::Ascend;
    pending.arm();
    pending.tick(0);
    pending.s.command.proposed = ProposedCommand::Forward;
    pending.c.publishDepth(DepthControlSample{.12, .01, 0, true});
    check(pending.c.stops == 1 && pending.session.armed(),
          "pending ASCEND safety STOP bypasses switching dwell");
    pending.c.started();
    pending.tick(100);
    check(pending.c.streams == 0, "late Start ACK cannot send forbidden pitch");
    Fixture timeout;
    timeout.arm();
    timeout.tick(0);
    timeout.c.started();
    timeout.c.reason = 3;
    timeout.c.prop = false; // Remote has qualified telemetry and ended this session.
    emit timeout.c.proportionalTelemetryChanged();
    check(!timeout.session.armed() && timeout.c.stops == 1, "firmware setpoint timeout STOP disarms");
}
void stopSettlementAndUnknown()
{
    Fixture f;
    auto config = f.session.approachConfig();
    config.alphaArea = 1.0;
    f.session.setApproachConfig(config);
    f.s = sample(1, 20);
    f.arm(); // Initial ARRIVED sends STOP while the controller is already idle.
    f.s = sample(2, 5);
    f.tick(50);
    f.tick(1050);
    check(f.c.starts == 0, "resume waits for STOP acceptance");
    f.c.ack(1, CommandTerminalResult::Ok);
    f.tick(2000);
    check(f.c.starts == 0, "resume waits full second after STOP OK");
    f.c.motionActive = true;
    f.tick(2050);
    check(f.c.starts == 0, "resume also waits for actual STOPPED");
    f.c.motionActive = false;
    f.tick(2100);
    check(f.c.starts == 1 && f.session.approachInfo().state == "APPROACH",
          "settled STOP and complete dwell permit new Start");

    Fixture unknown;
    unknown.arm();
    unknown.c.ack(1, CommandTerminalResult::OutcomeUnknown);
    check(!unknown.session.armed() && unknown.c.stops == 1, "unknown Start outcome stops and disarms");

    Fixture staleReason;
    staleReason.c.reason = 3;
    staleReason.arm();
    emit staleReason.c.proportionalTelemetryChanged();
    check(staleReason.c.stops == 0 && staleReason.session.armed(),
          "previous-session timeout does not cancel pending Start");
    staleReason.c.started();
    emit staleReason.c.proportionalTelemetryChanged();
    check(staleReason.c.stops == 0 && staleReason.session.armed(),
          "stale stopped telemetry before current proportional sample is ignored");
}
void videoWaitPreservesAreaEpisodes()
{
    Fixture missing;
    missing.arm();
    missing.c.started();
    missing.s.target->target.areaCells.reset();
    missing.tick(50);
    missing.s.awaitingVideo = true;
    missing.tick(300);
    missing.s.awaitingVideo = false;
    missing.tick(551);
    check(missing.c.stops == 1, "video catch-up does not reset missing-area timeout");

    Fixture arrived;
    auto config = arrived.session.approachConfig();
    config.alphaArea = 1.0;
    arrived.session.setApproachConfig(config);
    arrived.arm();
    arrived.c.started();
    arrived.s = sample(2, 20);
    arrived.tick(50);
    arrived.s.awaitingVideo = true;
    arrived.tick(300);
    arrived.s.awaitingVideo = false;
    arrived.tick(1050);
    check(arrived.c.stops == 1 && arrived.session.approachInfo().state == "ARRIVED",
          "video catch-up does not reset arrival dwell");
}
} // namespace
int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    streamAndGrace();
    safetyAndDry();
    areaAndDepth();
    stopSettlementAndUnknown();
    videoWaitPreservesAreaEpisodes();
    return failures ? 1 : 0;
}
