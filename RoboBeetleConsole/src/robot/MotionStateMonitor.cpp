#include "robot/MotionStateMonitor.h"
#include <algorithm>
#include <bit>
#include <limits>

namespace rb {
void MotionStateMonitor::reset() { *this = {}; }
void MotionStateMonitor::finishBatch() {
    if (batchSeq_ && !batchFinished_) {
        fragmentGaps_ += fragmentCount_ - std::popcount(seenFragments_);
        batchFinished_ = true;
    }
}
std::optional<double> MotionStateMonitor::offsetMs() const {
    if (anchors_.empty()) return {};
    return double(std::min_element(anchors_.begin(), anchors_.end(),
        [](const auto &a, const auto &b) { return a.offsetMs < b.offsetMs; })->offsetMs);
}
std::vector<MotionStateRecord> MotionStateMonitor::accept(const MotionStateTelemetry &t) {
    const auto b = robobeetle::protocol::decode_motion_state_batch(t.batch_payload);
    if (!b || t.pi_rx_ms > quint64(std::numeric_limits<qint64>::max())) return {};
    if (epoch_ && *epoch_ != t.link_epoch) { finishBatch(); reset(); }
    epoch_ = t.link_epoch;
    if (batchSeq_ && b->batch_seq != *batchSeq_) {
        const quint16 step = quint16(b->batch_seq - *batchSeq_);
        if (step >= 32768) return {}; // late duplicate from a completed older batch
        finishBatch();
        batchGaps_ += step - 1;
        seenFragments_ = 0;
    }
    if (!batchSeq_ || b->batch_seq != *batchSeq_) {
        batchSeq_ = b->batch_seq;
        fragmentCount_ = b->fragment_count;
        batchFinished_ = false;
    }
    if (b->fragment_count != fragmentCount_) return {};
    const quint32 bit = quint32(1U << b->fragment_index);
    if (seenFragments_ & bit) return {};
    seenFragments_ |= bit;
    if (std::popcount(seenFragments_) == fragmentCount_) batchFinished_ = true;
    if (!lastTx_) unwrappedTx_ = b->mcu_tx_ms;
    else {
        const auto delta = qint32(b->mcu_tx_ms - *lastTx_);
        if (delta < 0) {
            // MCU restart without a gateway epoch update: retire old clock/history.
            anchors_.clear(); history_.clear(); unwrappedTx_ = b->mcu_tx_ms;
        } else unwrappedTx_ += delta;
    }
    lastTx_ = b->mcu_tx_ms;
    const auto cutoff = t.pi_rx_ms > 10000 ? t.pi_rx_ms - 10000 : 0;
    while (!anchors_.empty() && anchors_.front().piRxMs < cutoff) anchors_.pop_front();
    // Keep raw minimum. It includes unknown queue/transport bias. wireMs is only
    // the selected anchor's known 115200 8N1 wire-time reference, NOT an error bound.
    anchors_.push_back({t.pi_rx_ms, qint64(t.pi_rx_ms) - unwrappedTx_,
                       (28.0 + (b->schema_version==2?35.0:22.0) * b->sample_count) / 11.52});
    while (anchors_.size() > 4096) anchors_.pop_front();
    const auto anchor = std::min_element(anchors_.begin(), anchors_.end(),
        [](const auto &a, const auto &c) { return a.offsetMs < c.offsetMs; });
    samplerDrops_ = b->sampler_drop_total;
    gatewayDrops_ = t.gateway_drop_total;
    std::vector<MotionStateRecord> records;
    for (unsigned i = 0; i < b->sample_count; ++i) {
        const auto &s = b->samples[i];
        const qint64 sampleMcu = unwrappedTx_ + qint32(s.mcu_ms - b->mcu_tx_ms);
        if (sampleMcu < 0) continue;
        MotionStateRecord r{s, t.link_epoch, b->mcu_tx_ms, b->sampler_drop_total,
            t.gateway_drop_total, b->batch_seq, b->fragment_index, quint8(i),
            quint64(sampleMcu), t.pi_rx_ms, double(sampleMcu + anchor->offsetMs),
            anchor->wireMs, batchGaps_, fragmentGaps_};
        if (s.backend!=1 || s.state!=1 || !s.phase_valid || (periodVersion_ && *periodVersion_!=s.cpg_param_version)) {
            previousPhase_.reset(); previousPhaseMs_.reset(); lastWrapMs_.reset(); periods_.clear(); discardFirstPeriod_=true;
        }
        periodVersion_=s.cpg_param_version;
        if (s.backend==1 && s.state==1 && s.phase_valid) {
            if (previousPhase_ && s.phase_u16<*previousPhase_) {
                const double delta=65536.0-*previousPhase_+s.phase_u16;
                const double wrap=*previousPhaseMs_+(double(sampleMcu)-*previousPhaseMs_)*(65536.0-*previousPhase_)/delta;
                if (lastWrapMs_) {
                    if (discardFirstPeriod_) discardFirstPeriod_=false;
                    else { periods_.push_back((wrap-*lastWrapMs_)/1000); if (periods_.size()>3) periods_.pop_front(); }
                }
                lastWrapMs_=wrap;
            }
            previousPhase_=s.phase_u16; previousPhaseMs_=double(sampleMcu);
        }
        records.push_back(r);
        history_.push_back(r);
    }
    while (!history_.empty() && history_.front().samplePiMs < double(cutoff)) history_.pop_front();
    while (history_.size() > 4096) history_.pop_front();
    return records;
}
std::optional<MotionStateRecord> MotionStateMonitor::atCapture(quint64 captureNs) const {
    const double captureMs = double(captureNs) / 1e6;
    std::optional<MotionStateRecord> nearest;
    for (const auto &r : history_)
        if (r.samplePiMs <= captureMs && (!nearest || r.samplePiMs >= nearest->samplePiMs)) nearest = r;
    return nearest;
}
std::optional<double> MotionStateMonitor::measuredCpgPeriod() const {
    if (periods_.size()<3) return {};
    return (periods_[0]+periods_[1]+periods_[2])/3;
}
double MotionStateMonitor::gyroRateHz() const {
    double first = 0, last = 0;
    unsigned count = 0;
    for (const auto &r : history_) if (r.sample.gyro_valid) {
        if (!count) first = r.samplePiMs;
        last = r.samplePiMs; ++count;
    }
    return count > 1 && last > first ? (count - 1) * 1000.0 / (last - first) : 0;
}
} // namespace rb
