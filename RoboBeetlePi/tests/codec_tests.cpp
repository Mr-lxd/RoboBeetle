#include "test_support.hpp"

#include "robobeetle/protocol/codec.hpp"
#include "robobeetle/protocol/cobs.hpp"
#include "robobeetle/protocol/frame.hpp"

namespace rbp2_test {
namespace {

using robobeetle::protocol::Codec;
using robobeetle::protocol::DecodeError;
using robobeetle::protocol::Frame;

std::vector<std::uint8_t> valid_wire()
{
    return Codec::encodeWire({0x01U, 0x1234U, bytes({0x01U, 0x02U, 0x00U, 0x04U})});
}

std::vector<std::uint8_t> wire_from_logical(
    std::vector<std::uint8_t> logical)
{
    return robobeetle::protocol::cobs_encode(logical);
}

} // namespace

void test_codec_boundaries_and_unknown_type()
{
    const Frame empty{0x01U, 0U, {}};
    const auto empty_wire = Codec::encodeWire(empty);
    expect(!empty_wire.empty() && empty_wire.back() == 0U,
           "empty payload frame must encode with delimiter");
    expect(Codec::decodeWire(without_delimiter(empty_wire)).frame == empty,
           "empty payload frame must round-trip");

    const Frame maximum{0x7eU, 0xffffU, std::vector<std::uint8_t>(64U, 0xA5U)};
    const auto maximum_wire = Codec::encodeWire(maximum);
    expect(!maximum_wire.empty(), "64-byte payload must be accepted");
    expect(Codec::MaxLogicalSize == 74U &&
               Codec::MaxEncodedBodySize == 75U &&
               Codec::MaxWireSize == 76U,
           "Protocol V2 maximum-size constants must match the wire contract");
    expect(Codec::MaxLogicalSize == Codec::HeaderSize +
                                      Codec::MaxPayloadSize + Codec::CrcSize &&
               Codec::MaxEncodedBodySize == Codec::MaxLogicalSize + 1U &&
               Codec::MaxWireSize == Codec::MaxEncodedBodySize + 1U,
           "maximum-size constants must remain internally consistent");
    expect(maximum_wire.size() <= Codec::MaxWireSize,
           "maximum legal payload must fit within MaxWireSize");
    expect(without_delimiter(maximum_wire).size() <= Codec::MaxEncodedBodySize,
           "maximum legal payload body must fit within MaxEncodedBodySize");
    for (std::size_t payload_size = 0U;
         payload_size <= Codec::MaxPayloadSize;
         ++payload_size) {
        const Frame legal{0x7eU, static_cast<std::uint16_t>(payload_size),
                          std::vector<std::uint8_t>(payload_size, 0xA5U)};
        const auto legal_wire = Codec::encodeWire(legal);
        expect(!legal_wire.empty() && legal_wire.size() <= Codec::MaxWireSize,
               "every legal payload size must fit within MaxWireSize");
    }
    const auto maximum_decoded = Codec::decodeWire(without_delimiter(maximum_wire));
    expect(maximum_decoded.ok() && maximum_decoded.frame == maximum,
           "64-byte payload must round-trip");

    const Frame too_large{0x01U, 1U, std::vector<std::uint8_t>(65U, 0U)};
    expect(Codec::encodeWire(too_large).empty(),
           "payload larger than 64 bytes must be rejected");

    const auto valid = valid_wire();
    const auto valid_body = without_delimiter(valid);
    const auto valid_logical = Codec::encodeLogical(
        {0x01U, 0x1234U, bytes({0x01U, 0x02U, 0x00U, 0x04U})});

    auto truncated = valid_logical;
    truncated.pop_back();
    expect(Codec::decodeLogical(truncated).error == DecodeError::InvalidLength,
           "truncated logical frame must be rejected");

    auto bad_length = valid_logical;
    bad_length[6] = 0x05U;
    expect(Codec::decodeLogical(bad_length).error == DecodeError::InvalidLength,
           "incorrect payload length must be rejected");

    auto bad_magic = valid_logical;
    bad_magic[0] = 0x58U;
    expect(Codec::decodeLogical(bad_magic).error == DecodeError::InvalidMagic,
           "bad Magic must be rejected");

    auto bad_version = valid_logical;
    bad_version[2] = 0x03U;
    expect(Codec::decodeLogical(bad_version).error == DecodeError::InvalidVersion,
           "bad Version must be rejected");

    auto bad_crc_logical = valid_logical;
    bad_crc_logical.back() ^= 0x01U;
    const auto bad_crc_body = wire_from_logical(bad_crc_logical);
    expect(Codec::decodeWire(bad_crc_body).error == DecodeError::CrcMismatch,
           "CRC corruption must be rejected");

    const auto unknown_logical = Codec::encodeLogical(
        {0x7fU, 0x0000U, bytes({0xAAU})});
    const auto unknown_decoded = Codec::decodeLogical(unknown_logical);
    expect(unknown_decoded.ok(),
           "valid unknown message type must decode at framing level");
    expect(unknown_decoded.frame.message_type == 0x7fU,
           "unknown message type must remain raw");

    auto bad_crc_unknown = unknown_logical;
    bad_crc_unknown[bad_crc_unknown.size() - 1U] ^= 0x01U;
    expect(Codec::decodeWire(wire_from_logical(bad_crc_unknown)).error ==
               DecodeError::CrcMismatch,
           "bad CRC unknown type must remain a CRC error");

    expect(Codec::decodeWire(std::vector<std::uint8_t>{0x03U, 0xFFU}).error ==
               DecodeError::CobsDecodeFailed,
           "malformed COBS must be rejected");
    expect(valid_body.back() != 0U,
           "encoded body passed to decodeWire excludes delimiter");
}

} // namespace rbp2_test
