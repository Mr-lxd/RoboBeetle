#pragma once
#include "robot/RobotCommand.h"
#include "robot/DepthControl.h"
#include <optional>
#include <cstdint>

namespace rb {
struct GamepadInput {
    double leftX{0}, leftY{0}, rightX{0}, rightY{0}; // up/right positive
    bool b{false}, connected{false}, enabled{false}, authority{false};
    DepthEnvelopeState depth{DepthEnvelopeState::Unavailable};
    std::int64_t nowMs{0};
    std::optional<bool> previousAccepted;
};
struct GamepadOutput {
    std::optional<MotionMode> command;
    bool disable{false};
};
class GamepadMapper {
public:
    GamepadOutput update(const GamepadInput &input);
private:
    enum class Axis { None, Horizontal, Vertical };
    Axis rightAxis_{Axis::None};
    int rightDirection_{0};
    bool forward_{false}, bWasDown_{false}, enabled_{false};
    MotionMode active_{MotionMode::Stop};
    std::optional<MotionMode> offered_;
    std::int64_t offeredAt_{0};
    std::optional<std::int64_t> lastStartAt_;
    std::optional<MotionMode> bSuppressed_;
};
} // namespace rb
