#include "test_support.hpp"

#include "robobeetle/protocol/cobs.hpp"

namespace rbp2_test {

void test_cobs_round_trip_and_edges()
{
    using robobeetle::protocol::cobs_decode;
    using robobeetle::protocol::cobs_encode;

    const auto zero_data = bytes({0x00U});
    expect(cobs_encode(zero_data) == bytes({0x01U, 0x01U}),
           "COBS zero byte encoding differs");

    const auto mixed = bytes({0x01U, 0x02U, 0x00U, 0x03U});
    const auto mixed_encoded = cobs_encode(mixed);
    expect(mixed_encoded == bytes({0x03U, 0x01U, 0x02U, 0x02U, 0x03U}),
           "COBS mixed zero encoding differs");
    std::vector<std::uint8_t> mixed_decoded;
    expect(cobs_decode(mixed_encoded, mixed_decoded),
           "COBS mixed vector must decode");
    expect(mixed_decoded == mixed, "COBS mixed vector round-trip differs");

    std::vector<std::uint8_t> long_data(254U, 0x11U);
    const auto long_encoded = cobs_encode(long_data);
    expect(!long_encoded.empty() && long_encoded.front() == 0xFFU,
           "COBS 0xff block must use a 0xff code");
    std::vector<std::uint8_t> long_decoded;
    expect(cobs_decode(long_encoded, long_decoded),
           "COBS 0xff block must decode");
    expect(long_decoded == long_data, "COBS 0xff block round-trip differs");

    std::vector<std::uint8_t> decoded;
    expect(cobs_decode(bytes({0x01U}), decoded),
           "COBS empty logical body uses code 1");
    expect(decoded.empty(), "COBS code 1 must decode to empty data");
    expect(!cobs_decode(std::vector<std::uint8_t>{}, decoded),
           "COBS empty encoded input must fail");
    expect(!cobs_decode(bytes({0x00U}), decoded),
           "COBS zero code must fail");
    expect(!cobs_decode(bytes({0x03U, 0x01U}), decoded),
           "COBS truncated block must fail");
}

} // namespace rbp2_test
