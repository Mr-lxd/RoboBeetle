#pragma once

#include "robobeetle/protocol/frame.hpp"

#include <cstddef>
#include <cstdint>

namespace robobeetle::protocol {

std::uint16_t crc16_ccitt_false(const Byte *data, std::size_t length);
std::uint16_t crc16_ccitt_false(const Bytes &data);

} // namespace robobeetle::protocol
