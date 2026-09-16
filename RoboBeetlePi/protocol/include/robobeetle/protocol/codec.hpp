#pragma once

#include "robobeetle/protocol/frame.hpp"
#include "robobeetle/protocol/protocol_error.hpp"

#include <cstddef>

namespace robobeetle::protocol {

class Codec final {
public:
    static constexpr Byte Magic0 = 0x52U;
    static constexpr Byte Magic1 = 0x42U;
    static constexpr Byte Version = 0x02U;
    static constexpr std::size_t MaxPayloadSize = 64U;
    static constexpr std::size_t HeaderSize = 8U;
    static constexpr std::size_t CrcSize = 2U;
    static constexpr std::size_t MaxLogicalSize = HeaderSize +
                                                  MaxPayloadSize + CrcSize;
    static constexpr std::size_t MaxEncodedBodySize = MaxLogicalSize + 1U;
    // Includes the trailing 0x00 delimiter appended by encodeWire().
    static constexpr std::size_t MaxWireSize = 76U;

    static Bytes encodeLogical(const Frame &frame);
    static Bytes encodeWire(const Frame &frame);
    // decodeWire receives the COBS body without the trailing 0x00 delimiter.
    static DecodeResult decodeWire(const Bytes &encoded_body);
    static DecodeResult decodeLogical(const Bytes &logical_frame);
};

} // namespace robobeetle::protocol
