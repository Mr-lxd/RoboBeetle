#include "input/XInputGamepad.h"
#if defined(Q_OS_WIN)
#include <windows.h>
#include <Xinput.h>
#endif

namespace rb {
XInputGamepad::XInputGamepad()
{
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
XInputGamepadState XInputGamepad::poll() const
{
#if defined(Q_OS_WIN)
    if (getState_) {
        using GetState = DWORD (WINAPI *)(DWORD, XINPUT_STATE *);
        const auto getState = reinterpret_cast<GetState>(getState_);
        for (DWORD index = 0; index < XUSER_MAX_COUNT; ++index) {
            XINPUT_STATE state{};
            if (getState(index, &state) != ERROR_SUCCESS) continue;
            auto axis = [](SHORT value) { return value >= 0 ? value / 32767.0 : value / 32768.0; };
            return {true, static_cast<int>(index + 1),
                axis(state.Gamepad.sThumbLX), axis(state.Gamepad.sThumbLY),
                axis(state.Gamepad.sThumbRX), axis(state.Gamepad.sThumbRY),
                (state.Gamepad.wButtons & XINPUT_GAMEPAD_B) != 0};
        }
    }
#endif
    return {};
}
} // namespace rb
