#include "robot/MotionStateMonitor.h"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <iostream>

rb::MotionStateTelemetry fragment(unsigned seq, unsigned index, unsigned count,
                                 unsigned tx, quint64 rx, unsigned samples = 1) {
    rb::MotionStateBatch batch;
    batch.batch_seq = seq; batch.fragment_index = index; batch.fragment_count = count;
    batch.mcu_tx_ms = tx; batch.sample_count = samples;
    for (unsigned i = 0; i < samples; ++i) {
        batch.samples[i].mcu_ms = tx - 50 + i * 10;
        batch.samples[i].gyro_valid = true;
    }
    return {7, rx, 3, rb::motionStateBytes(*rb::encodeMotionStateBatch(batch))};
}
void testPeriodRetention() {
    rb::MotionStateMonitor monitor;
    unsigned seq = 0;
    auto sample = [&](unsigned ms, unsigned state, unsigned version, unsigned backend = 1) {
        rb::MotionStateBatch batch;
        batch.batch_seq = seq++;
        batch.schema_version = 2;
        batch.fragment_count = batch.sample_count = 1;
        batch.mcu_tx_ms = ms;
        auto &s = batch.samples[0];
        s.mcu_ms = ms;
        s.backend = backend;
        s.state = state;
        s.phase_valid = state == 1;
        s.phase_u16 = (ms % 2000) * 65536 / 2000;
        s.cpg_param_version = version;
        monitor.accept({7, ms + 10000, 0, rb::motionStateBytes(*rb::encodeMotionStateBatch(batch))});
    };
    for (unsigned ms = 0; ms <= 6000; ms += 500) sample(ms, 1, 1);
    assert(monitor.measuredCpgPeriod() && std::abs(*monitor.measuredCpgPeriod() - 2.0) < 1e-9);
    sample(6500, 0, 1);
    assert(monitor.measuredCpgPeriod() && std::abs(*monitor.measuredCpgPeriod() - 2.0) < 1e-9);
    sample(7000, 0, 2);
    assert(!monitor.measuredCpgPeriod());
    for (unsigned ms = 7500; ms <= 14000; ms += 500) sample(ms, 1, 2);
    assert(monitor.measuredCpgPeriod());
    sample(14500, 0, 2, 0);
    assert(!monitor.measuredCpgPeriod());
}
void testYawAtCapture() {
    rb::MotionStateMonitor monitor;
    unsigned seq = 0;
    auto gyro = [&](unsigned ms, qint16 tenthDps, quint32 epoch = 7) {
        rb::MotionStateBatch batch;
        batch.batch_seq = seq++;
        batch.fragment_count = batch.sample_count = 1;
        batch.mcu_tx_ms = ms;
        batch.samples[0].mcu_ms = ms;
        batch.samples[0].gyro_valid = true;
        batch.samples[0].gyro_tenth_dps[2] = tenthDps;
        monitor.accept({epoch, ms + 10000, 0, rb::motionStateBytes(*rb::encodeMotionStateBatch(batch))});
    };
    for (unsigned ms = 0; ms <= 3000; ms += 50) gyro(ms, 100); // 10 deg/s
    const auto yaw = monitor.yawAtCapture(13000ULL * 1000000ULL);
    assert(yaw && std::abs(yaw->instDeg - 30.0) < 1e-9);
    // Default 2.5 s period without a measured CPG period; a steady turn is kept.
    assert(yaw->meanDeg && std::abs(*yaw->meanDeg - 30.0) < 1e-9);
    assert(!monitor.yawAtCapture(9000ULL * 1000000ULL));
    const auto epoch = yaw->epoch;
    gyro(3050, 100, 8); // gateway link epoch change resets the monitor
    const auto after = monitor.yawAtCapture(13050ULL * 1000000ULL);
    assert(after && after->epoch != epoch && std::abs(after->instDeg) < 1e-9);
    monitor.reset();
    gyro(3100, 100, 8);
    assert(monitor.yawAtCapture(13100ULL * 1000000ULL)->epoch > after->epoch);
}
int main() {
    testPeriodRetention();
    testYawAtCapture();
    rb::MotionStateMonitor m;
    auto first = fragment(10, 0, 3, 1000, 11008, 2);
    assert(m.accept(first).size() == 2); // usable samples do not await missing fragments
    assert(m.accept(first).empty()); // duplicate is not a new gyro
    assert(m.accept(fragment(10, 2, 3, 1001, 11009)).size() == 1);
    m.accept(fragment(12, 1, 3, 1100, 11108));
    assert(m.batchGapTotal() == 1 && m.fragmentGapTotal() == 1);
    m.finishBatch(); // an actual stream end establishes leading/trailing losses
    assert(m.fragmentGapTotal() == 3);
    rb::MotionStateMonitor clock;
    clock.accept(fragment(0, 0, 1, 1000, 11008));
    assert(clock.offsetMs() && *clock.offsetMs() == 10008);
    clock.accept(fragment(1, 0, 1, 2000, 12020));
    assert(*clock.offsetMs() == 10008); // queue delay does not drag minimum upward
    auto rows = clock.accept(fragment(2, 0, 1, 3000, 13005));
    assert(*clock.offsetMs() == 10005 && rows[0].samplePiMs == 12955);
    assert(std::abs(rows[0].clockErrorMs - 50.0 / 11.52) < 0.00001);
    assert(!clock.atCapture(10957000000ULL));
    assert(clock.atCapture(12955000000ULL)->samplePiMs == 12955);
    clock.accept(fragment(3, 0, 1, 14000, 24030, 0));
    assert(*clock.offsetMs() == 10030); // anchors expire after 10 s in Pi time
    std::cout << "motion reassembly/loss and 10 s offset estimator passed\n";
}
