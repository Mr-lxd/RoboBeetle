#include "input/GamepadMapper.h"
#include <cmath>

namespace rb {
GamepadOutput GamepadMapper::update(const GamepadInput &in)
{
    if (offered_ && in.previousAccepted.value_or(false) && *offered_ != MotionMode::Stop) {
        active_ = *offered_;
        lastStartAt_ = offeredAt_;
    }
    offered_.reset();
    const bool bPressed = in.connected && in.b && !bWasDown_;
    bWasDown_ = in.connected && in.b;
    auto offer = [&](MotionMode mode, bool disable = false) {
        offered_ = mode;
        offeredAt_ = in.nowMs;
        if (mode == MotionMode::Stop) active_ = MotionMode::Stop;
        return GamepadOutput{mode, disable};
    };
    const bool allowed = in.enabled && in.connected && in.authority;
    if (!allowed) {
        const bool disable = in.enabled && (!in.connected || !in.authority);
        enabled_ = false;
        rightAxis_ = Axis::None;
        forward_ = false;
        bSuppressed_.reset();
        if (active_ != MotionMode::Stop || (bPressed && in.authority)) return offer(MotionMode::Stop, disable);
        return {{}, disable};
    }
    if (in.depth == DepthEnvelopeState::HardLimit) {
        enabled_ = false;
        rightAxis_ = Axis::None;
        forward_ = false;
        bSuppressed_.reset();
        return offer(MotionMode::Stop, true);
    }
    const bool justEnabled = !enabled_;
    enabled_ = true;
    if (justEnabled) awaitingCenter_ = true;
    if (awaitingCenter_ && std::abs(in.leftX) < .35 && std::abs(in.leftY) < .35
        && std::abs(in.rightX) < .35 && std::abs(in.rightY) < .35) {
        awaitingCenter_ = false;
    }
    forward_ = forward_ ? in.leftY >= .35 : in.leftY > .5;
    if (rightAxis_ != Axis::None) {
        const double value = rightAxis_ == Axis::Horizontal ? in.rightX : in.rightY;
        if (std::abs(value) < .35) rightAxis_ = Axis::None;
        else if (std::abs(value) > .5) rightDirection_ = value > 0 ? 1 : -1;
    }
    if (rightAxis_ == Axis::None && (std::abs(in.rightX) > .5 || std::abs(in.rightY) > .5)) {
        rightAxis_ = std::abs(in.rightX) >= std::abs(in.rightY) ? Axis::Horizontal : Axis::Vertical;
        rightDirection_ = (rightAxis_ == Axis::Horizontal ? in.rightX : in.rightY) > 0 ? 1 : -1;
    }
    MotionMode wanted = forward_ ? MotionMode::Forward : MotionMode::Stop;
    if (rightAxis_ == Axis::Horizontal) wanted = rightDirection_ > 0 ? MotionMode::TurnRight : MotionMode::TurnLeft;
    if (rightAxis_ == Axis::Vertical) {
        const bool usable = in.depth != DepthEnvelopeState::Unavailable && in.depth != DepthEnvelopeState::NotZeroed;
        if (usable && rightDirection_ > 0 && in.depth != DepthEnvelopeState::Surface) wanted = MotionMode::Ascend;
        if (usable && rightDirection_ < 0 && in.depth != DepthEnvelopeState::SoftFloor) wanted = MotionMode::Descend;
    }
    if (bPressed) {
        bSuppressed_ = wanted;
        return offer(MotionMode::Stop);
    }
    if (bSuppressed_) {
        if (*bSuppressed_ == wanted) return {};
        bSuppressed_.reset();
    }
    if (justEnabled || awaitingCenter_) return {}; // B remains available before this stick gate
    const bool vertical = active_ == MotionMode::Ascend || active_ == MotionMode::Descend;
    if ((vertical && (in.depth == DepthEnvelopeState::Unavailable || in.depth == DepthEnvelopeState::NotZeroed))
        || (active_ == MotionMode::Ascend && in.depth == DepthEnvelopeState::Surface)
        || (active_ == MotionMode::Descend && in.depth == DepthEnvelopeState::SoftFloor)) {
        return offer(MotionMode::Stop);
    }
    if (wanted == active_) return {};
    if (wanted == MotionMode::Stop) {
        if (active_ != MotionMode::Stop) return offer(MotionMode::Stop);
        return {};
    }
    if (lastStartAt_ && in.nowMs - *lastStartAt_ < 500) return {};
    return offer(wanted);
}
} // namespace rb
