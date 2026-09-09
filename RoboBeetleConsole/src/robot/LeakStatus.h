#pragma once

#include <QMetaType>
#include <QString>
#include <QtGlobal>

namespace rb {

enum class LeakState : quint8 {
    Unknown = 0,
    Dry = 1,
    Wet = 2,
};

constexpr bool isValidLeakState(quint8 value)
{
    return value <= static_cast<quint8>(LeakState::Wet);
}

inline LeakState leakStateFromByte(quint8 value)
{
    return isValidLeakState(value)
        ? static_cast<LeakState>(value)
        : LeakState::Unknown;
}

inline QString leakStateDisplayText(LeakState state)
{
    switch (state) {
    case LeakState::Unknown:
        return QStringLiteral("Leak: Unknown");
    case LeakState::Dry:
        return QStringLiteral("Leak: Dry");
    case LeakState::Wet:
        return QStringLiteral("LEAK DETECTED");
    }
    return QStringLiteral("Leak: Unknown");
}

} // namespace rb

Q_DECLARE_METATYPE(rb::LeakState)
