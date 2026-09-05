#pragma once

#include "protocol/Packet.h"

#include <QByteArray>
#include <QByteArrayView>

namespace rb {

class PacketCodec final {
public:
    static constexpr quint8 Magic0 = 0x52;
    static constexpr quint8 Magic1 = 0x42;
    static constexpr quint8 Version = 0x02;
    static constexpr qsizetype MaxPayloadSize = 64;

    static QByteArray encodeLogical(const Packet &packet);
    static QByteArray encodeWire(const Packet &packet);
    static DecodeResult decodeLogical(QByteArrayView logicalFrame);
    static DecodeResult decodeWire(QByteArrayView encodedFrame);

    static QByteArray cobsEncode(QByteArrayView data);
    static bool cobsDecode(QByteArrayView encoded, QByteArray &decoded);
};

QString decodeErrorText(DecodeError error);

} // namespace rb

