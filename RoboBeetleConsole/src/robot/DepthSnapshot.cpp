#include "robot/DepthSnapshot.h"

#include <limits>
#include <optional>

namespace rb {
namespace {

constexpr qsizetype kDepthOffset = 2;
constexpr qsizetype kTemperatureOffset = 6;
constexpr qsizetype kSampleAgeOffset = 8;
constexpr qsizetype kRxByteCountOffset = 10;
constexpr qsizetype kValidLineCountOffset = 14;
constexpr qsizetype kParseErrorCountOffset = 18;
constexpr qsizetype kOverlongLineCountOffset = 22;
constexpr qsizetype kRingOverflowCountOffset = 26;
constexpr qsizetype kHardRearmFailureCountOffset = 30;
constexpr qsizetype kUartErrorCountOffset = 34;

void writeLe16(QByteArray &payload, qsizetype offset, quint16 value)
{
    payload[offset] = static_cast<char>(value & 0xffU);
    payload[offset + 1] = static_cast<char>((value >> 8U) & 0xffU);
}

void writeLe32(QByteArray &payload, qsizetype offset, quint32 value)
{
    payload[offset] = static_cast<char>(value & 0xffU);
    payload[offset + 1] = static_cast<char>((value >> 8U) & 0xffU);
    payload[offset + 2] = static_cast<char>((value >> 16U) & 0xffU);
    payload[offset + 3] = static_cast<char>((value >> 24U) & 0xffU);
}

quint16 readLe16(QByteArrayView payload, qsizetype offset)
{
    return static_cast<quint16>(static_cast<quint8>(payload[offset]))
        | (static_cast<quint16>(static_cast<quint8>(payload[offset + 1])) << 8U);
}

quint32 readLe32(QByteArrayView payload, qsizetype offset)
{
    return static_cast<quint32>(static_cast<quint8>(payload[offset]))
        | (static_cast<quint32>(static_cast<quint8>(payload[offset + 1])) << 8U)
        | (static_cast<quint32>(static_cast<quint8>(payload[offset + 2])) << 16U)
        | (static_cast<quint32>(static_cast<quint8>(payload[offset + 3])) << 24U);
}

qint16 readSignedLe16(QByteArrayView payload, qsizetype offset)
{
    const quint16 raw = readLe16(payload, offset);
    if ((raw & 0x8000U) == 0) {
        return static_cast<qint16>(raw);
    }

    const quint16 magnitude = static_cast<quint16>((~raw) + 1U);
    if (magnitude == 0x8000U) {
        return std::numeric_limits<qint16>::min();
    }
    return static_cast<qint16>(-static_cast<qint16>(magnitude));
}

qint32 readSignedLe32(QByteArrayView payload, qsizetype offset)
{
    const quint32 raw = readLe32(payload, offset);
    if ((raw & 0x80000000U) == 0) {
        return static_cast<qint32>(raw);
    }

    const quint32 magnitude = (~raw) + 1U;
    if (magnitude == 0x80000000U) {
        return std::numeric_limits<qint32>::min();
    }
    return static_cast<qint32>(-static_cast<qint32>(magnitude));
}

void setDetail(QString *detail, const QString &message)
{
    if (detail != nullptr) {
        *detail = message;
    }
}

bool hasReservedFlags(quint8 flags)
{
    return (flags & ~DepthSnapshot::ValidityMask) != 0;
}

} // namespace

QByteArray DepthSnapshot::encodePayload(const DepthSnapshot &snapshot)
{
    if (hasReservedFlags(snapshot.validityFlags)) {
        return {};
    }

    QByteArray payload(PayloadSize, '\0');
    payload[0] = static_cast<char>(SchemaVersion);
    payload[1] = static_cast<char>(snapshot.validityFlags);

    if (snapshot.depthValid()) {
        writeLe32(payload, kDepthOffset, static_cast<quint32>(snapshot.depthMm));
    }
    if (snapshot.temperatureValid()) {
        writeLe16(payload, kTemperatureOffset,
                  static_cast<quint16>(snapshot.temperatureCentiC));
    }
    const quint16 sampleAge = snapshot.depthValid()
        ? snapshot.sampleAgeMs
        : UnknownSampleAgeMs;
    writeLe16(payload, kSampleAgeOffset, sampleAge);
    writeLe32(payload, kRxByteCountOffset, snapshot.diagnostics.rxByteCount);
    writeLe32(payload, kValidLineCountOffset, snapshot.diagnostics.validLineCount);
    writeLe32(payload, kParseErrorCountOffset, snapshot.diagnostics.parseErrorCount);
    writeLe32(payload, kOverlongLineCountOffset,
              snapshot.diagnostics.overlongLineCount);
    writeLe32(payload, kRingOverflowCountOffset,
              snapshot.diagnostics.ringOverflowCount);
    writeLe32(payload, kHardRearmFailureCountOffset,
              snapshot.diagnostics.hardRearmFailureCount);
    writeLe32(payload, kUartErrorCountOffset, snapshot.diagnostics.uartErrorCount);
    return payload;
}

std::optional<DepthSnapshot> DepthSnapshot::decodePayload(QByteArrayView payload,
                                                            QString *detail)
{
    if (payload.size() != PayloadSize) {
        setDetail(detail, QStringLiteral("DepthSnapshot payload must be exactly 38 bytes"));
        return std::nullopt;
    }
    if (static_cast<quint8>(payload[0]) != SchemaVersion) {
        setDetail(detail, QStringLiteral("Unsupported DepthSnapshot schema version"));
        return std::nullopt;
    }

    const quint8 validityFlags = static_cast<quint8>(payload[1]);
    if (hasReservedFlags(validityFlags)) {
        setDetail(detail, QStringLiteral("DepthSnapshot has reserved validity flags"));
        return std::nullopt;
    }

    const qint32 depthMm = readSignedLe32(payload, kDepthOffset);
    const qint16 temperatureCentiC = readSignedLe16(payload, kTemperatureOffset);
    const quint16 sampleAgeMs = readLe16(payload, kSampleAgeOffset);
    if (((validityFlags & DepthValid) == 0 && depthMm != 0)
        || ((validityFlags & TemperatureValid) == 0 && temperatureCentiC != 0)) {
        setDetail(detail, QStringLiteral("DepthSnapshot has nonzero invalid values"));
        return std::nullopt;
    }
    if ((validityFlags & DepthValid) == 0
        && sampleAgeMs != UnknownSampleAgeMs) {
        setDetail(detail,
                  QStringLiteral("DepthSnapshot without valid depth must use unknown age"));
        return std::nullopt;
    }

    DepthSnapshot snapshot;
    snapshot.validityFlags = validityFlags;
    snapshot.depthMm = snapshot.depthValid() ? depthMm : 0;
    snapshot.temperatureCentiC = snapshot.temperatureValid() ? temperatureCentiC : 0;
    snapshot.sampleAgeMs = sampleAgeMs;

    snapshot.diagnostics.rxByteCount = readLe32(payload, kRxByteCountOffset);
    snapshot.diagnostics.validLineCount = readLe32(payload, kValidLineCountOffset);
    snapshot.diagnostics.parseErrorCount = readLe32(payload, kParseErrorCountOffset);
    snapshot.diagnostics.overlongLineCount = readLe32(payload, kOverlongLineCountOffset);
    snapshot.diagnostics.ringOverflowCount = readLe32(payload, kRingOverflowCountOffset);
    snapshot.diagnostics.hardRearmFailureCount =
        readLe32(payload, kHardRearmFailureCountOffset);
    snapshot.diagnostics.uartErrorCount = readLe32(payload, kUartErrorCountOffset);
    setDetail(detail, {});
    return snapshot;
}

} // namespace rb
