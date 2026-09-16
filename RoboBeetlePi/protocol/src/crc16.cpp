#include "robobeetle/protocol/crc16.hpp"

namespace robobeetle::protocol {

std::uint16_t crc16_ccitt_false(const Byte *data, std::size_t length)
{
    std::uint16_t crc = 0xFFFFU;
    for (std::size_t index = 0U; index < length; ++index) {
        crc ^= static_cast<std::uint16_t>(data[index]) << 8U;
        for (unsigned bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<std::uint16_t>(
                    (crc << 1U) ^ 0x1021U);
            } else {
                crc = static_cast<std::uint16_t>(crc << 1U);
            }
        }
    }
    return crc;
}

std::uint16_t crc16_ccitt_false(const Bytes &data)
{
    return crc16_ccitt_false(data.data(), data.size());
}

} // namespace robobeetle::protocol
