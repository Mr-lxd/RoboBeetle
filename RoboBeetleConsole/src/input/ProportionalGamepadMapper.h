#pragma once
#include "input/GamepadMapper.h"
#include <algorithm>
#include <cmath>
namespace rb
{
struct ProportionalGamepadOutput
{
    bool start{}, stream{}, stop{}, disable{};
    unsigned short throttle{};
    short turn{}, pitch{};
};
class ProportionalGamepadMapper
{
  public:
    double deadzone{.15}, gamma{1.5};
    void requireCenter() { awaitingCenter_ = true; }
    static double shape(double x, double d, double g)
    {
        return std::copysign(std::pow(std::max(0., (std::min(1., std::abs(x)) - d) / (1 - d)), g),
                             x);
    }
    ProportionalGamepadOutput update(const GamepadInput &i, bool active, bool pending)
    {
        ProportionalGamepadOutput o;
        const bool b = i.connected && i.b && !bDown_;
        bDown_ = i.connected && i.b;
        auto stop = [&]
        {
            requireCenter();
            o.stop = true;
            return o;
        };
        if (b && i.authority)
        {
            return stop();
        }
        const bool allowed = i.enabled && i.connected && i.authority;
        if (!allowed)
        {
            if (wasEnabled_)
            {
                o.disable = i.enabled;
                wasEnabled_ = false;
                if (active || pending)
                    return stop();
            }
            requireCenter();
            return o;
        }
        if (!wasEnabled_)
        {
            wasEnabled_ = true;
            requireCenter();
        }
        if (i.depth == DepthEnvelopeState::HardLimit)
        {
            o.disable = true;
            return stop();
        }
        if (pending)
            return o;
        const bool centered = std::abs(i.leftX) <= deadzone && std::abs(i.leftY) <= deadzone &&
                              std::abs(i.rightX) <= deadzone && std::abs(i.rightY) <= deadzone;
        if (active && centered)
            return stop();
        if (awaitingCenter_)
        {
            if (centered && !i.b)
                awaitingCenter_ = false;
            return o;
        }
        o.throttle = static_cast<unsigned short>(
            std::lround(1000 * shape(std::max(0., i.leftY), deadzone, gamma)));
        o.turn = static_cast<short>(std::lround(1000 * shape(i.rightX, deadzone, gamma)));
        o.pitch = static_cast<short>(std::lround(1000 * shape(i.rightY, deadzone, gamma)));
        if (i.depth == DepthEnvelopeState::Unavailable ||
            i.depth == DepthEnvelopeState::NotZeroed ||
            (i.depth == DepthEnvelopeState::Surface && o.pitch > 0) ||
            (i.depth == DepthEnvelopeState::SoftFloor && o.pitch < 0))
            o.pitch = 0;
        o.start = !active && (o.throttle != 0 || o.pitch != 0);
        o.stream = active;
        return o;
    }

  private:
    bool awaitingCenter_{true}, wasEnabled_{false}, bDown_{false};
};
} // namespace rb
