#include "input/XInputGamepad.h"
#if defined(Q_OS_WIN)
#include <windows.h>
#include <Xinput.h>
#endif

namespace rb {
XInputGamepad::XInputGamepad()
{
    scanClock_.start();
#if defined(Q_OS_WIN)
    for (const auto *name : {"xinput1_4", "xinput9_1_0"}) {
        library_.setFileName(QString::fromLatin1(name));
        if (library_.load()) {
            getState_ = library_.resolve("XInputGetState");
            if (getState_) break;
            library_.unload();
        }
    }
#endif
}
XInputGamepadState XInputGamepad::poll()
{
#if defined(Q_OS_WIN)
    if (getState_) {
        using GetState = DWORD (WINAPI *)(DWORD, XINPUT_STATE *);
        const auto getState = reinterpret_cast<GetState>(getState_);
        const auto now = scanClock_.elapsed();
        auto readSlot = [&](DWORD index) -> XInputGamepadState {
            XINPUT_STATE state{};
            if (getState(index, &state) != ERROR_SUCCESS) return {};
            auto axis = [](SHORT value) { return value >= 0 ? value / 32767.0 : value / 32768.0; };
            return {true, static_cast<int>(index + 1),
                axis(state.Gamepad.sThumbLX), axis(state.Gamepad.sThumbLY),
                axis(state.Gamepad.sThumbRX), axis(state.Gamepad.sThumbRY),
                (state.Gamepad.wButtons & XINPUT_GAMEPAD_B) != 0};
        };
        if (connectedSlot_ >= 0) {
            const auto state = readSlot(static_cast<DWORD>(connectedSlot_));
            if (!state.connected) {
                connectedSlot_ = -1;
                lastScanMs_ = now;
            }
            return state; // expose disconnect immediately, without switching pads
        }
        if (now - lastScanMs_ < 2000) return {};
        lastScanMs_ = now;
        for (DWORD index = 0; index < XUSER_MAX_COUNT; ++index) {
            const auto state = readSlot(index);
            if (!state.connected) continue;
            connectedSlot_ = static_cast<int>(index);
            return state;
        }
    }
#endif
    return {};
}
} // namespace rb
