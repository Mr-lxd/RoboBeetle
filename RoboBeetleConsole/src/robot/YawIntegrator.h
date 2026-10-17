#pragma once
#include <QtGlobal>
#include <deque>
#include <optional>

namespace rb {
struct YawEstimate {
    quint32 epoch{0};
    double instDeg{0};
    // Causal mean heading over one gait period plus half the period's net turn,
    // so periodic sway cancels while a steady turn is preserved. Absent until a
    // full period of history exists.
    std::optional<double> meanDeg;
};

// Integrates gyro_z (deg/s) on the MCU clock and answers queries on the Pi
// clock that frame capture timestamps use. Absolute yaw is only meaningful
// within one epoch; a reset or a sample gap starts a new epoch.
class YawIntegrator {
public:
    static constexpr double kMaxGapMs = 250.0;
    static constexpr double kMaxExtrapolateMs = 150.0;
    static constexpr double kHistoryMs = 10000.0;

    void reset();
    void add(double samplePiMs, qint64 mcuMs, double gyroZDps);
    [[nodiscard]] std::optional<YawEstimate> at(double piMs, double periodMs) const;
    [[nodiscard]] quint32 epoch() const noexcept { return epoch_; }

private:
    struct Point { double piMs; double yawDeg; double rateDps; };
    [[nodiscard]] std::optional<double> yawAt(double piMs) const;
    std::deque<Point> points_;
    std::optional<qint64> lastMcuMs_;
    quint32 epoch_{0};
};
} // namespace rb
