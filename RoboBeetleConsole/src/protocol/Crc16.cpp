#include "protocol/Crc16.h"

namespace rb {

quint16 crc16CcittFalse(QByteArrayView data)
{
    quint16 crc = 0xffff;
    for (const char byte : data) {
        crc ^= static_cast<quint16>(static_cast<quint8>(byte)) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) != 0U
                ? static_cast<quint16>((crc << 1U) ^ 0x1021U)
                : static_cast<quint16>(crc << 1U);
        }
    }
    return crc;
}

} // namespace rb

