#pragma once

#include <cstdint>
#include <vector>

namespace robobeetle::protocol {

using Byte = std::uint8_t;
using Bytes = std::vector<Byte>;

// The wire type intentionally remains raw so newer Firmware message IDs can
// be represented before LinkCore learns their semantic meaning.
struct Frame {
    Byte message_type{0U};
    std::uint16_t sequence{0U};
    Bytes payload;
};

inline bool operator==(const Frame &left, const Frame &right)
{
    return left.message_type == right.message_type &&
           left.sequence == right.sequence &&
           left.payload == right.payload;
}

inline bool operator!=(const Frame &left, const Frame &right)
{
    return !(left == right);
}

} // namespace robobeetle::protocol
