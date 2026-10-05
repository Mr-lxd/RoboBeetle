#include "robot/DepthControl.h"

#include <cmath>

namespace rb {

bool validDepthControlConfig(const DepthControlConfig &c) noexcept
{
    const bool finite = std::isfinite(c.surfaceMarginM) && std::isfinite(c.softReleaseM)
        && std::isfinite(c.softMaxM) && std::isfinite(c.hardMaxM)
        && std::isfinite(c.zeroMaxRangeM);
    return finite
        && c.surfaceMarginM >= 0.0
        && c.surfaceMarginM < c.softReleaseM
        && c.softReleaseM < c.softMaxM
        && c.softMaxM < c.hardMaxM
        && c.nominalSamplePeriodMs > 0
        && c.controlFreshMs >= 2 * c.nominalSamplePeriodMs + 200
        && c.zeroMinSamples >= 3
        && c.zeroMaxRangeM > 0.0;
}

DepthEnvelopeResult evaluateDepthEnvelope(const std::optional<DepthControlSample> &sample,
                                          const DepthEnvelopeMemory &previous,
                                          const DepthControlConfig &config)
{
    if (!validDepthControlConfig(config) || !sample || !sample->fresh)
        return {DepthEnvelopeState::Unavailable, previous};
    if (!sample->calibratedDepthM)
        return {DepthEnvelopeState::NotZeroed, previous};

    const double d = *sample->calibratedDepthM;
    if (!std::isfinite(d))
        return {DepthEnvelopeState::Unavailable, previous};
    if (d >= config.hardMaxM)
        return {DepthEnvelopeState::HardLimit, DepthEnvelopeMemory{true}};

    if ((previous.softFloorLatched && d > config.softReleaseM) || d >= config.softMaxM)
        return {DepthEnvelopeState::SoftFloor, DepthEnvelopeMemory{true}};

    const DepthEnvelopeMemory cleared{false};
    if (d <= config.surfaceMarginM)
        return {DepthEnvelopeState::Surface, cleared};
    return {DepthEnvelopeState::Normal, cleared};
}

const char *depthEnvelopeStateName(DepthEnvelopeState state) noexcept
{
    switch (state) {
    case DepthEnvelopeState::Unavailable: return "UNAVAILABLE";
    case DepthEnvelopeState::NotZeroed: return "NOT_ZEROED";
    case DepthEnvelopeState::Surface: return "SURFACE";
    case DepthEnvelopeState::Normal: return "NORMAL";
    case DepthEnvelopeState::SoftFloor: return "SOFT_FLOOR";
    case DepthEnvelopeState::HardLimit: return "HARD_LIMIT";
    }
    return "UNKNOWN";
}

} // namespace rb
