#pragma once
#include "protocol/MotionStateCodec.h"
#include <deque>
#include <vector>

namespace rb {
struct MotionStateRecord {
    MotionStateSample sample;
    quint32 linkEpoch{}, mcuTxMs{}, samplerDropTotal{}, gatewayDropTotal{};
    quint16 batchSeq{};
    quint8 fragmentIndex{}, sampleIndex{};
    quint64 mcuUnwrappedMs{}, piRxMs{};
    double samplePiMs{}, clockErrorMs{};
    quint64 batchGapTotal{}, fragmentGapTotal{};
};
class MotionStateMonitor {
public:
    // GUI-thread, recording only. No Console clock or control state is consulted.
    std::vector<MotionStateRecord> accept(const MotionStateTelemetry &);
    void finishBatch();
    void reset();
    std::optional<double> offsetMs() const;
    std::optional<MotionStateRecord> atCapture(quint64 captureNs) const;
    double gyroRateHz() const;
    quint64 batchGapTotal() const { return batchGaps_; }
    quint64 fragmentGapTotal() const { return fragmentGaps_; }
    quint32 samplerDropTotal() const { return samplerDrops_; }
    quint32 gatewayDropTotal() const { return gatewayDrops_; }
    std::optional<quint32> linkEpoch() const { return epoch_; }
private:
    struct Anchor { quint64 piRxMs; qint64 offsetMs; double wireMs; };
    std::deque<Anchor> anchors_;
    std::deque<MotionStateRecord> history_;
    std::optional<quint32> epoch_, lastTx_;
    qint64 unwrappedTx_{};
    std::optional<quint16> batchSeq_;
    quint16 seenFragments_{};
    quint8 fragmentCount_{};
    bool batchFinished_{true};
    quint64 batchGaps_{}, fragmentGaps_{};
    quint32 samplerDrops_{}, gatewayDrops_{};
};
} // namespace rb
