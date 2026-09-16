#include "test_support.hpp"

#include "robobeetle/protocol/cobs.hpp"
#include "robobeetle/protocol/codec.hpp"
#include "robobeetle/protocol/crc16.hpp"
#include "robobeetle/protocol/protocol_error.hpp"
#include "robobeetle/protocol/stream_decoder.hpp"

namespace rbp2_test {

namespace {

std::vector<std::uint8_t> wire_from_logical(
    std::vector<std::uint8_t> logical)
{
    auto wire = robobeetle::protocol::cobs_encode(logical);
    wire.push_back(0U);
    return wire;
}

void recompute_crc(std::vector<std::uint8_t> &logical)
{
    const auto crc = robobeetle::protocol::crc16_ccitt_false(
        logical.data(), logical.size() - 2U);
    logical[logical.size() - 2U] = static_cast<std::uint8_t>(crc & 0xFFU);
    logical.back() = static_cast<std::uint8_t>((crc >> 8U) & 0xFFU);
}

} // namespace

void test_stream_decoder_contract()
{
    using robobeetle::protocol::Codec;
    using robobeetle::protocol::DecodeError;
    using robobeetle::protocol::StreamDecoder;

    const auto first = Codec::encodeWire({0x01U, 7U, bytes({0x01U, 0U, 0U, 0U})});
    const auto second = Codec::encodeWire({0x14U, 8U, bytes({0x03U, 0U})});

    StreamDecoder decoder;
    expect(decoder.feed(std::vector<std::uint8_t>(first.begin(), first.begin() + 3U)).empty(),
           "fragmented frame must not emit early");
    const auto split = decoder.feed(
        std::vector<std::uint8_t>(first.begin() + 3, first.end()));
    expect(split.size() == 1U && split.front().ok(),
           "fragmented frame must decode after delimiter");

    const auto sticky = decoder.feed(concat(first, second));
    expect(sticky.size() == 2U && sticky[0].ok() && sticky[1].ok(),
           "concatenated frames must emit in order");

    const auto empty_delimiters = decoder.feed(bytes({0U, 0U}));
    expect(empty_delimiters.empty(), "empty delimiters must not emit frames");

    auto bad_crc = first;
    bad_crc[bad_crc.size() - 2U] ^= 0x01U;
    const auto recovered = decoder.feed(concat(bad_crc, first));
    expect(recovered.size() == 2U, "bad frame must not swallow following frame");
    expect(recovered[0].error == DecodeError::CrcMismatch,
           "bad CRC must be reported by StreamDecoder");
    expect(recovered[1].ok(), "valid frame after bad CRC must recover");

    auto bad_magic = Codec::encodeLogical({0x01U, 9U, bytes({0x02U, 0U, 0U, 0U})});
    bad_magic[0] = 0x58U;
    const auto bad_magic_wire = concat(
        robobeetle::protocol::cobs_encode(bad_magic), bytes({0U}));
    expect(decoder.feed(bad_magic_wire).front().error == DecodeError::InvalidMagic,
           "bad Magic must be reported by StreamDecoder");

    const auto malformed_plus_valid = concat(
        std::vector<std::uint8_t>{0x03U, 0xFFU, 0U}, first);
    const auto cobs_recovered = decoder.feed(malformed_plus_valid);
    expect(cobs_recovered.size() == 2U &&
               cobs_recovered[0].error == DecodeError::CobsDecodeFailed &&
               cobs_recovered[1].ok(),
           "malformed COBS must resynchronize at delimiter");

    std::vector<std::uint8_t> overlength(97U, 0x55U);
    overlength.push_back(0U);
    overlength.insert(overlength.end(), first.begin(), first.end());
    const auto overlength_results = decoder.feed(overlength);
    expect(overlength_results.size() == 2U,
           "overlength frame must emit an error then recover");
    expect(overlength_results[0].error == DecodeError::InvalidLength,
           "overlength encoded frame must be rejected");
    expect(overlength_results[1].ok(),
           "frame after overlength discard must decode");

    decoder.feed(std::vector<std::uint8_t>(first.begin(), first.begin() + 4U));
    decoder.reset();
    const auto after_reset = decoder.feed(first);
    expect(after_reset.size() == 1U && after_reset.front().ok(),
           "reset must discard interrupted frame and resynchronize");

    const robobeetle::protocol::Frame expected_following_frame{
        0x14U, 8U, bytes({0x03U, 0U})};

    auto invalid_version_logical = Codec::encodeLogical(
        {0x01U, 12U, bytes({0x02U, 0U, 0U, 0U})});
    invalid_version_logical[2] = 0x03U;
    recompute_crc(invalid_version_logical);
    StreamDecoder version_decoder;
    const auto version_events = version_decoder.feed(concat(
        wire_from_logical(invalid_version_logical), second));
    expect(version_events.size() == 2U,
           "invalid Version must emit an error and recover at delimiter");
    expect(version_events[0].error == DecodeError::InvalidVersion,
           "invalid Version must be reported by StreamDecoder");
    expect(version_events[1].ok() &&
               version_events[1].frame == expected_following_frame,
           "valid frame after invalid Version must decode correctly");

    auto invalid_length_logical = Codec::encodeLogical(
        {0x01U, 13U, bytes({0x04U, 0U, 0U, 0U})});
    invalid_length_logical[6] = 0x05U;
    recompute_crc(invalid_length_logical);
    StreamDecoder length_decoder;
    const auto length_events = length_decoder.feed(concat(
        wire_from_logical(invalid_length_logical), second));
    expect(length_events.size() == 2U,
           "invalid payload Length must emit an error and recover at delimiter");
    expect(length_events[0].error == DecodeError::InvalidLength,
           "invalid payload Length must be reported by StreamDecoder");
    expect(length_events[1].ok() &&
               length_events[1].frame == expected_following_frame,
           "valid frame after invalid payload Length must decode correctly");
}

} // namespace rbp2_test
