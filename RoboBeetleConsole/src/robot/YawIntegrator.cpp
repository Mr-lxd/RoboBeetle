#include "robot/YawIntegrator.h"
#include <algorithm>
#include <cmath>

namespace rb {
void YawIntegrator::reset()
{
    points_.clear();
    lastMcuMs_.reset();
    ++epoch_;
}

void YawIntegrator::add(double samplePiMs, qint64 mcuMs, double gyroZDps)
{
    if (!std::isfinite(samplePiMs) || !std::isfinite(gyroZDps)) return;
    if (lastMcuMs_ && mcuMs <= *lastMcuMs_) return; // duplicate or reordered sample
    if (lastMcuMs_ && double(mcuMs - *lastMcuMs_) > kMaxGapMs) reset();
    if (points_.empty()) {
        points_.push_back({samplePiMs, 0.0, gyroZDps});
    } else {
        const auto &p = points_.back();
        const double dtS = double(mcuMs - *lastMcuMs_) / 1000.0;
        // The Pi mapping can step back when a lower-offset anchor arrives; keep
        // the lookup axis monotonic so interpolation stays well defined.
        points_.push_back({std::max(samplePiMs, p.piMs), p.yawDeg + 0.5 * (p.rateDps + gyroZDps) * dtS,
                           gyroZDps});
    }
    lastMcuMs_ = mcuMs;
    while (points_.size() > 1 && points_.front().piMs < points_.back().piMs - kHistoryMs)
        points_.pop_front();
}

std::optional<double> YawIntegrator::yawAt(double piMs) const
{
    if (points_.empty() || piMs < points_.front().piMs) return {};
    const auto &last = points_.back();
    if (piMs >= last.piMs) {
        if (piMs - last.piMs > kMaxExtrapolateMs) return {};
        return last.yawDeg + last.rateDps * (piMs - last.piMs) / 1000.0;
    }
    const auto next = std::upper_bound(points_.begin(), points_.end(), piMs,
        [](double t, const Point &p) { return t < p.piMs; });
    const auto &b = *next;
    const auto &a = *std::prev(next);
    if (b.piMs == a.piMs) return b.yawDeg;
    return a.yawDeg + (b.yawDeg - a.yawDeg) * (piMs - a.piMs) / (b.piMs - a.piMs);
}

std::optional<YawEstimate> YawIntegrator::at(double piMs, double periodMs) const
{
    const auto inst = yawAt(piMs);
    if (!inst) return {};
    YawEstimate result{epoch_, *inst, std::nullopt};
    const double start = piMs - periodMs;
    if (!(periodMs > 0) || start < points_.front().piMs) return result;
    const auto startYaw = yawAt(start);
    if (!startYaw) return result;
    // Trapezoid integral of the piecewise-linear yaw over [start, piMs].
    double area = 0.0, t = start, y = *startYaw;
    for (const auto &p : points_) {
        if (p.piMs <= start) continue;
        if (p.piMs >= piMs) break;
        area += 0.5 * (y + p.yawDeg) * (p.piMs - t);
        t = p.piMs;
        y = p.yawDeg;
    }
    area += 0.5 * (y + *inst) * (piMs - t);
    result.meanDeg = area / periodMs + 0.5 * (*inst - *startYaw);
    return result;
}
} // namespace rb
