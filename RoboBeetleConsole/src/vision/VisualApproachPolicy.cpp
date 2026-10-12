#include "vision/VisualApproachPolicy.h"

#include <algorithm>
#include <cmath>

namespace rb::vision {
namespace {
bool unitPositive(double value) noexcept
{
    return std::isfinite(value) && value > 0.0 && value <= 1.0;
}
void clearDwell(VisualApproachMemory &memory)
{
    memory.arriveSinceMs.reset();
    memory.resumeSinceMs.reset();
}
bool dwellComplete(std::optional<std::int64_t> &since, std::int64_t now,
                   std::int64_t duration)
{
    if (!since || now < *since) since = now;
    return now - *since >= duration;
}
}

bool validVisualApproachConfig(const VisualApproachConfig &c) noexcept
{
    return unitPositive(c.alphaArea)
        && std::isfinite(c.slowArea) && std::isfinite(c.resumeArea)
        && std::isfinite(c.stopArea) && c.slowArea > 0.0
        && c.slowArea < c.resumeArea && c.resumeArea < c.stopArea
        && c.stopArea <= 576.0
        && c.arriveDwellMs >= 0 && c.resumeDwellMs >= 0
        && unitPositive(c.tMin) && unitPositive(c.tMax)
        && std::isfinite(c.tTurnMin)
        && c.tMin <= c.tTurnMin && c.tTurnMin <= c.tMax
        && unitPositive(c.kTurn) && unitPositive(c.kPitch);
}

const char *approachStateName(ApproachState state) noexcept
{
    return state == ApproachState::Arrived ? "ARRIVED" : "APPROACH";
}

VisualSetpoint mapVisualSetpoint(ProposedCommand command, double area_f,
                                const VisualApproachConfig &c, bool arrived) noexcept
{
    if (arrived || command == ProposedCommand::Stop || command == ProposedCommand::Hold
        || !validVisualApproachConfig(c) || !std::isfinite(area_f) || area_f < 0.0) {
        return {};
    }
    double throttle = c.tMax;
    if (area_f >= c.stopArea) {
        throttle = c.tMin;
    } else if (area_f > c.slowArea) {
        throttle -= (c.tMax - c.tMin) * (area_f - c.slowArea) / (c.stopArea - c.slowArea);
    }
    VisualSetpoint result;
    switch (command) {
    case ProposedCommand::TurnLeft:
        result.turn = static_cast<short>(-std::lround(1000.0 * c.kTurn));
        throttle = std::max(throttle, c.tTurnMin);
        break;
    case ProposedCommand::TurnRight:
        result.turn = static_cast<short>(std::lround(1000.0 * c.kTurn));
        throttle = std::max(throttle, c.tTurnMin);
        break;
    case ProposedCommand::Ascend:
        result.pitch = static_cast<short>(std::lround(1000.0 * c.kPitch));
        break;
    case ProposedCommand::Descend:
        result.pitch = static_cast<short>(-std::lround(1000.0 * c.kPitch));
        break;
    case ProposedCommand::Forward:
        break;
    default:
        return {};
    }
    result.throttle = static_cast<unsigned short>(std::lround(1000.0 * throttle));
    return result;
}

bool validContinuousVisualConfig(const VisualApproachConfig &c) noexcept
{
    return validVisualApproachConfig(c) && std::isfinite(c.eDead) && std::isfinite(c.eFull)
        && c.eDead >= 0 && c.eDead < c.eFull && c.eFull <= 1;
}

VisualSetpoint mapContinuousVisualSetpoint(double area_f, double ex_f, double ey_f,
    VisualAxisMode axis, int turnSign, int pitchSign, DepthEnvelopeState depth,
    const VisualApproachConfig &c, bool arrived) noexcept
{
    if (arrived || !validContinuousVisualConfig(c) || !std::isfinite(ex_f) || !std::isfinite(ey_f)) return {};
    const auto response = [&c](double error) {
        return std::copysign(std::clamp((std::abs(error) - c.eDead) / (c.eFull - c.eDead), 0.0, 1.0), error);
    };
    const double turn = axis == VisualAxisMode::Pitch ? 0 : turnSign * response(ex_f);
    double pitch = axis == VisualAxisMode::Yaw ? 0 : -pitchSign * response(ey_f) * c.kPitch;
    if ((depth == DepthEnvelopeState::Surface && pitch > 0)
        || (depth == DepthEnvelopeState::SoftFloor && pitch < 0)) pitch = 0;
    auto result = mapVisualSetpoint(ProposedCommand::Forward, area_f, c);
    result.throttle = std::max(result.throttle,
        static_cast<unsigned short>(std::lround(1000 * c.tTurnMin * std::abs(turn))));
    result.turn = static_cast<short>(std::lround(1000 * turn));
    result.pitch = static_cast<short>(std::lround(1000 * pitch));
    return result;
}

VisualApproachResult evaluateVisualApproach(const VisualApproachMemory &previous,
                                          const VisualApproachInput &input,
                                          const VisualApproachConfig &config)
{
    VisualApproachResult result;
    result.next = previous;
    auto &memory = result.next;
    if (!memory.sessionId || *memory.sessionId != input.sessionId) {
        memory = {};
        memory.sessionId = input.sessionId;
    }
    if (!validVisualApproachConfig(config) || input.maxMissMs <= 0
        || !input.observationAvailable) {
        clearDwell(memory);
        // NO_TARGET/video waiting belongs to caller freshness/lost grace, not
        // the selected-target missing-area episode.
        memory.missingSinceMs.reset();
        result.area_f = memory.filteredArea;
        return result;
    }
    const bool validArea = input.areaCells && std::isfinite(*input.areaCells)
        && *input.areaCells > 0.0 && *input.areaCells <= 576.0;
    if (!validArea) {
        clearDwell(memory);
        if (!memory.missingSinceMs || input.nowMs < *memory.missingSinceMs) {
            memory.missingSinceMs = input.nowMs;
        }
        if (!memory.areaMissingLost
            && input.nowMs - *memory.missingSinceMs > input.maxMissMs) {
            memory.areaMissingLost = true;
            memory.filteredArea.reset();
            memory.lastFrameId.reset();
            memory.state = ApproachState::Approach;
            result.areaMissingLost = true;
        }
        result.area_f = memory.filteredArea;
        return result;
    }
    memory.missingSinceMs.reset();
    memory.areaMissingLost = false;
    const bool firstArea = !memory.filteredArea;
    if (!memory.lastFrameId || *memory.lastFrameId != input.frameId) {
        memory.filteredArea = firstArea ? *input.areaCells
            : config.alphaArea * *input.areaCells
                + (1.0 - config.alphaArea) * *memory.filteredArea;
        memory.lastFrameId = input.frameId;
        result.observationUsed = true;
    }
    result.area_f = memory.filteredArea;
    if (!memory.filteredArea) return result;
    if (memory.state == ApproachState::Approach) {
        memory.resumeSinceMs.reset();
        if (*memory.filteredArea >= config.stopArea) {
            if (firstArea || dwellComplete(memory.arriveSinceMs, input.nowMs, config.arriveDwellMs)) {
                memory.state = ApproachState::Arrived;
                clearDwell(memory);
                result.arrivedEnter = true;
            }
        } else {
            memory.arriveSinceMs.reset();
        }
    } else {
        memory.arriveSinceMs.reset();
        if (*memory.filteredArea < config.resumeArea) {
            if (dwellComplete(memory.resumeSinceMs, input.nowMs, config.resumeDwellMs)) {
                memory.state = ApproachState::Approach;
                clearDwell(memory);
                result.arrivedExit = true;
            }
        } else {
            memory.resumeSinceMs.reset();
        }
    }
    return result;
}

} // namespace rb::vision
