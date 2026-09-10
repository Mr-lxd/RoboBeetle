#include "robot/ImuSnapshot.h"

namespace rb {
namespace {

constexpr qsizetype kAccOffset = 2;
constexpr qsizetype kGyroOffset = 8;
constexpr qsizetype kAngleOffset = 14;

void writeLe16(QByteArray &payload, qsizetype offset, qint16 value)
{
    const quint16 raw = static_cast<quint16>(value);
    payload[offset] = static_cast<char>(raw & 0xffU);
    payload[offset + 1] = static_cast<char>((raw >> 8U) & 0xffU);
}

void writeLe32(QByteArray &payload, qsizetype offset, quint32 value)
{
    payload[offset] = static_cast<char>(value & 0xffU);
    payload[offset + 1] = static_cast<char>((value >> 8U) & 0xffU);
    payload[offset + 2] = static_cast<char>((value >> 16U) & 0xffU);
    payload[offset + 3] = static_cast<char>((value >> 24U) & 0xffU);
}

qint16 readLe16(QByteArrayView payload, qsizetype offset)
{
    const quint16 raw = static_cast<quint8>(payload[offset])
        | (static_cast<quint16>(static_cast<quint8>(payload[offset + 1])) << 8U);
    return static_cast<qint16>(raw);
}

quint32 readLe32(QByteArrayView payload, qsizetype offset)
{
    return static_cast<quint32>(static_cast<quint8>(payload[offset]))
        | (static_cast<quint32>(static_cast<quint8>(payload[offset + 1])) << 8U)
        | (static_cast<quint32>(static_cast<quint8>(payload[offset + 2])) << 16U)
        | (static_cast<quint32>(static_cast<quint8>(payload[offset + 3])) << 24U);
}

bool allZero(const std::array<qint16, 3> &values)
{
    return values[0] == 0 && values[1] == 0 && values[2] == 0;
}

bool inRange(const std::array<qint16, 3> &values, qint16 minimum, qint16 maximum)
{
    for (const qint16 value : values) {
        if (value < minimum || value > maximum) {
            return false;
        }
    }
    return true;
}

bool validDomain(const std::array<qint16, 3> &values,
                 quint8 flags,
                 quint8 flag,
                 qint16 minimum,
                 qint16 maximum)
{
    if ((flags & flag) == 0) {
        return allZero(values);
    }
    return inRange(values, minimum, maximum);
}

void setDetail(QString *detail, const QString &message)
{
    if (detail != nullptr) {
        *detail = message;
    }
}

} // namespace

QByteArray ImuSnapshot::encodePayload(const ImuSnapshot &snapshot)
{
    if ((snapshot.validityFlags & ~ValidityMask) != 0
        || !validDomain(snapshot.accMg, snapshot.validityFlags, AccValid,
                        AccMinMg, AccMaxMg)
        || !validDomain(snapshot.gyroDecidps, snapshot.validityFlags, GyroValid,
                        GyroMinDecidps, GyroMaxDecidps)
        || !validDomain(snapshot.angleCentidegrees, snapshot.validityFlags, AngleValid,
                        AngleMinCentidegrees, AngleMaxCentidegrees)) {
        return {};
    }

    QByteArray payload(PayloadSize, '\0');
    payload[0] = static_cast<char>(SchemaVersion);
    payload[1] = static_cast<char>(snapshot.validityFlags);

    if (snapshot.accValid()) {
        for (qsizetype index = 0; index < 3; ++index) {
            writeLe16(payload, kAccOffset + index * 2, snapshot.accMg[index]);
        }
    }
    if (snapshot.gyroValid()) {
        for (qsizetype index = 0; index < 3; ++index) {
            writeLe16(payload, kGyroOffset + index * 2, snapshot.gyroDecidps[index]);
        }
    }
    if (snapshot.angleValid()) {
        for (qsizetype index = 0; index < 3; ++index) {
            writeLe16(payload, kAngleOffset + index * 2,
                      snapshot.angleCentidegrees[index]);
        }
    }

    writeLe32(payload, 20, snapshot.diagnostics.rxByteCount);
    writeLe32(payload, 24, snapshot.diagnostics.headerCount);
    writeLe32(payload, 28, snapshot.diagnostics.validFrameCount);
    writeLe32(payload, 32, snapshot.diagnostics.checksumErrorCount);
    writeLe32(payload, 36, snapshot.diagnostics.ringOverflowCount);
    writeLe32(payload, 40, snapshot.diagnostics.rearmFailureCount);
    writeLe32(payload, 44, snapshot.diagnostics.uartErrorCount);
    writeLe32(payload, 48, snapshot.diagnostics.magFrameCount);
    writeLe32(payload, 52, snapshot.diagnostics.unsupportedFrameCount);
    return payload;
}

std::optional<ImuSnapshot> ImuSnapshot::decodePayload(QByteArrayView payload,
                                                       QString *detail)
{
    if (payload.size() != PayloadSize) {
        setDetail(detail, QStringLiteral("ImuSnapshot payload must be exactly 56 bytes"));
        return std::nullopt;
    }
    if (static_cast<quint8>(payload[0]) != SchemaVersion) {
        setDetail(detail, QStringLiteral("Unsupported ImuSnapshot schema version"));
        return std::nullopt;
    }

    ImuSnapshot snapshot;
    snapshot.validityFlags = static_cast<quint8>(payload[1]);
    if ((snapshot.validityFlags & ~ValidityMask) != 0) {
        setDetail(detail, QStringLiteral("ImuSnapshot has reserved validity flags"));
        return std::nullopt;
    }

    for (qsizetype index = 0; index < 3; ++index) {
        snapshot.accMg[index] = readLe16(payload, kAccOffset + index * 2);
        snapshot.gyroDecidps[index] = readLe16(payload, kGyroOffset + index * 2);
        snapshot.angleCentidegrees[index] = readLe16(payload, kAngleOffset + index * 2);
    }
    if (!validDomain(snapshot.accMg, snapshot.validityFlags, AccValid,
                     AccMinMg, AccMaxMg)
        || !validDomain(snapshot.gyroDecidps, snapshot.validityFlags, GyroValid,
                        GyroMinDecidps, GyroMaxDecidps)
        || !validDomain(snapshot.angleCentidegrees, snapshot.validityFlags, AngleValid,
                        AngleMinCentidegrees, AngleMaxCentidegrees)) {
        setDetail(detail, QStringLiteral("ImuSnapshot fixed-point values are invalid"));
        return std::nullopt;
    }

    snapshot.diagnostics.rxByteCount = readLe32(payload, 20);
    snapshot.diagnostics.headerCount = readLe32(payload, 24);
    snapshot.diagnostics.validFrameCount = readLe32(payload, 28);
    snapshot.diagnostics.checksumErrorCount = readLe32(payload, 32);
    snapshot.diagnostics.ringOverflowCount = readLe32(payload, 36);
    snapshot.diagnostics.rearmFailureCount = readLe32(payload, 40);
    snapshot.diagnostics.uartErrorCount = readLe32(payload, 44);
    snapshot.diagnostics.magFrameCount = readLe32(payload, 48);
    snapshot.diagnostics.unsupportedFrameCount = readLe32(payload, 52);
    setDetail(detail, {});
    return snapshot;
}

} // namespace rb
