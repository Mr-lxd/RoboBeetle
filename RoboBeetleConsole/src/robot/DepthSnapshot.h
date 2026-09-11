#pragma once

#include <QByteArray>
#include <QByteArrayView>
#include <QString>
#include <QtGlobal>

#include <optional>

namespace rb {

struct DepthDiagnostics {
    quint32 rxByteCount{0};
    quint32 validLineCount{0};
    quint32 parseErrorCount{0};
    quint32 overlongLineCount{0};
    quint32 ringOverflowCount{0};
    quint32 hardRearmFailureCount{0};
    quint32 uartErrorCount{0};

    bool operator==(const DepthDiagnostics &) const = default;
};

struct DepthSnapshot {
    static constexpr qsizetype PayloadSize = 38;
    static constexpr quint8 SchemaVersion = 0x01;
    static constexpr quint8 DepthValid = 0x01;
    static constexpr quint8 TemperatureValid = 0x02;
    static constexpr quint8 ValidityMask = DepthValid | TemperatureValid;
    static constexpr quint16 UnknownSampleAgeMs = 0xffff;

    quint8 validityFlags{0};
    qint32 depthMm{0};
    qint16 temperatureCentiC{0};
    quint16 sampleAgeMs{UnknownSampleAgeMs};
    DepthDiagnostics diagnostics{};

    [[nodiscard]] bool depthValid() const
    {
        return (validityFlags & DepthValid) != 0;
    }

    [[nodiscard]] bool temperatureValid() const
    {
        return (validityFlags & TemperatureValid) != 0;
    }

    static QByteArray encodePayload(const DepthSnapshot &snapshot);
    static std::optional<DepthSnapshot> decodePayload(QByteArrayView payload,
                                                       QString *detail = nullptr);
};

} // namespace rb
