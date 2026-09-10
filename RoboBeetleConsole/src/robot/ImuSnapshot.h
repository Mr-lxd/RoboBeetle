#pragma once

#include <QByteArray>
#include <QByteArrayView>
#include <QString>
#include <QtGlobal>

#include <array>
#include <optional>

namespace rb {

struct ImuDiagnostics {
    quint32 rxByteCount{0};
    quint32 headerCount{0};
    quint32 validFrameCount{0};
    quint32 checksumErrorCount{0};
    quint32 ringOverflowCount{0};
    quint32 rearmFailureCount{0};
    quint32 uartErrorCount{0};
    quint32 magFrameCount{0};
    quint32 unsupportedFrameCount{0};

    bool operator==(const ImuDiagnostics &) const = default;
};

struct ImuSnapshot {
    static constexpr qsizetype PayloadSize = 56;
    static constexpr quint8 SchemaVersion = 0x01;
    static constexpr quint8 AccValid = 0x01;
    static constexpr quint8 GyroValid = 0x02;
    static constexpr quint8 AngleValid = 0x04;
    static constexpr quint8 ValidityMask = AccValid | GyroValid | AngleValid;

    static constexpr qint16 AccMinMg = -16000;
    static constexpr qint16 AccMaxMg = 16000;
    static constexpr qint16 GyroMinDecidps = -20000;
    static constexpr qint16 GyroMaxDecidps = 20000;
    static constexpr qint16 AngleMinCentidegrees = -18000;
    static constexpr qint16 AngleMaxCentidegrees = 18000;

    quint8 validityFlags{0};
    std::array<qint16, 3> accMg{};
    std::array<qint16, 3> gyroDecidps{};
    std::array<qint16, 3> angleCentidegrees{};
    ImuDiagnostics diagnostics{};

    [[nodiscard]] bool accValid() const { return (validityFlags & AccValid) != 0; }
    [[nodiscard]] bool gyroValid() const { return (validityFlags & GyroValid) != 0; }
    [[nodiscard]] bool angleValid() const { return (validityFlags & AngleValid) != 0; }

    static QByteArray encodePayload(const ImuSnapshot &snapshot);
    static std::optional<ImuSnapshot> decodePayload(QByteArrayView payload,
                                                     QString *detail = nullptr);
};

} // namespace rb
