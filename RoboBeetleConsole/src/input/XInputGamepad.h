#pragma once
#include <QLibrary>

namespace rb {
struct XInputGamepadState {
    bool connected{false};
    int slot{0}; // user-facing slots 1..4
    double leftX{0}, leftY{0}, rightX{0}, rightY{0};
    bool b{false};
};
class XInputGamepad {
public:
    XInputGamepad();
    XInputGamepadState poll() const;
private:
    QLibrary library_;
    QFunctionPointer getState_{nullptr};
};
} // namespace rb
