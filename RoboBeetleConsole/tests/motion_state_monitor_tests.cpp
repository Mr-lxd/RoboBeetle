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
int main() {
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
