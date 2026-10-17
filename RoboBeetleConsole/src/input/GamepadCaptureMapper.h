#pragma once

namespace rb {
struct GamepadCaptureOutput {
    bool snapshot{false};
    bool toggleRecording{false};
};
// X takes a snapshot, Y toggles recording. Each fires once per press; a button
// already held when the pad (re)connects only arms after it is released.
class GamepadCaptureMapper {
public:
    GamepadCaptureOutput update(bool connected, bool x, bool y)
    {
        GamepadCaptureOutput out;
        if (!connected) {
            xDown_ = yDown_ = true;
            return out;
        }
        out.snapshot = x && !xDown_;
        out.toggleRecording = y && !yDown_;
        xDown_ = x;
        yDown_ = y;
        return out;
    }

private:
    bool xDown_{true}, yDown_{true};
};
} // namespace rb
