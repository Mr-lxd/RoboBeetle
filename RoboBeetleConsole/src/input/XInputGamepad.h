#pragma once
#include <QLibrary>
#include <QElapsedTimer>

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
    XInputGamepadState poll();
private:
    QElapsedTimer scanClock_;
    qint64 lastScanMs_{-2000};
    int connectedSlot_{-1}; // XInput slots 0..3; -1 means disconnected
    QLibrary library_;
    QFunctionPointer getState_{nullptr};
};
} // namespace rb
