#include "test_support.hpp"

#include "robobeetle/protocol/crc16.hpp"

namespace rbp2_test {

void test_crc_reference_and_edges()
{
    using robobeetle::protocol::crc16_ccitt_false;
    const auto reference = std::vector<std::uint8_t>{
        '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    expect(crc16_ccitt_false(reference) == 0x29B1U,
           "CRC-16/CCITT-FALSE reference value must be 0x29B1");
    expect(crc16_ccitt_false(std::vector<std::uint8_t>{}) == 0xFFFFU,
           "CRC-16/CCITT-FALSE empty input must retain init value");
    expect(crc16_ccitt_false(bytes({0x52U, 0x42U, 0x02U})) ==
               crc16_ccitt_false(bytes({0x52U, 0x42U, 0x02U})),
           "CRC calculation must be deterministic");
}

} // namespace rbp2_test
