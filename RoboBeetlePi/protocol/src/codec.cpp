#include "robobeetle/protocol/codec.hpp"

#include "robobeetle/protocol/cobs.hpp"
#include "robobeetle/protocol/crc16.hpp"

namespace robobeetle::protocol {
namespace {

void append_le16(Bytes &bytes, std::uint16_t value)
{
    bytes.push_back(static_cast<Byte>(value & 0xFFU));
    bytes.push_back(static_cast<Byte>((value >> 8U) & 0xFFU));
}

std::uint16_t read_le16(const Bytes &bytes, std::size_t offset)
{
    return static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(bytes[offset]) |
        (static_cast<std::uint16_t>(bytes[offset + 1U]) << 8U));
}

DecodeResult failure(DecodeError error, const char *detail)
{
    DecodeResult result;
    result.error = error;
    result.detail = detail;
    return result;
}

} // namespace

Bytes Codec::encodeLogical(const Frame &frame)
{
    if (frame.payload.size() > MaxPayloadSize) {
        return {};
    }

    Bytes logical;
    logical.reserve(HeaderSize + frame.payload.size() + CrcSize);
    logical.push_back(Magic0);
    logical.push_back(Magic1);
    logical.push_back(Version);
    logical.push_back(frame.message_type);
    append_le16(logical, frame.sequence);
    append_le16(logical, static_cast<std::uint16_t>(frame.payload.size()));
    logical.insert(logical.end(), frame.payload.begin(), frame.payload.end());
    append_le16(logical, crc16_ccitt_false(logical));
    return logical;
}

Bytes Codec::encodeWire(const Frame &frame)
{
    const Bytes logical = encodeLogical(frame);
    if (logical.empty()) {
        return {};
    }
    Bytes wire = cobs_encode(logical);
    wire.push_back(0U);
    return wire;
}

DecodeResult Codec::decodeWire(const Bytes &encoded_body)
{
    Bytes logical;
    if (!cobs_decode(encoded_body, logical)) {
        return failure(DecodeError::CobsDecodeFailed,
                       "COBS frame is malformed");
    }
    return decodeLogical(logical);
}

DecodeResult Codec::decodeLogical(const Bytes &logical_frame)
{
    const std::size_t fixed_size = HeaderSize + CrcSize;
    if (logical_frame.size() < fixed_size) {
        return failure(DecodeError::TooShort,
                       "Logical frame is shorter than 10 bytes");
    }
    if (logical_frame[0] != Magic0 || logical_frame[1] != Magic1) {
        return failure(DecodeError::InvalidMagic,
                       "Magic must be 0x52 0x42");
    }
    if (logical_frame[2] != Version) {
        return failure(DecodeError::InvalidVersion,
                       "Protocol version must be 0x02");
    }

    const std::uint16_t payload_length = read_le16(logical_frame, 6U);
    if (payload_length > MaxPayloadSize ||
        logical_frame.size() != fixed_size + payload_length) {
        return failure(DecodeError::InvalidLength,
                       "Payload length does not match frame size");
    }

    const std::size_t crc_offset = HeaderSize + payload_length;
    const std::uint16_t received_crc = read_le16(logical_frame, crc_offset);
    const std::uint16_t calculated_crc = crc16_ccitt_false(
        logical_frame.data(), crc_offset);
    if (received_crc != calculated_crc) {
        return failure(DecodeError::CrcMismatch,
                       "CRC-16/CCITT-FALSE mismatch");
    }

    DecodeResult result;
    result.frame.message_type = logical_frame[3];
    result.frame.sequence = read_le16(logical_frame, 4U);
    result.frame.payload.assign(
        logical_frame.begin() + static_cast<std::ptrdiff_t>(HeaderSize),
        logical_frame.begin() + static_cast<std::ptrdiff_t>(HeaderSize + payload_length));
    return result;
}

} // namespace robobeetle::protocol
