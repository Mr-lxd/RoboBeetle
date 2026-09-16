#include "test_support.hpp"

#include <cstdlib>
#include <iostream>

namespace rbp2_test {
void test_existing_golden_vectors();
void test_new_canonical_vectors();
void test_crc_reference_and_edges();
void test_cobs_round_trip_and_edges();
void test_codec_boundaries_and_unknown_type();
void test_stream_decoder_contract();
void test_link_core_contract();
int failures = 0;
} // namespace rbp2_test

int main()
{
    rbp2_test::test_existing_golden_vectors();
    rbp2_test::test_new_canonical_vectors();
    rbp2_test::test_crc_reference_and_edges();
    rbp2_test::test_cobs_round_trip_and_edges();
    rbp2_test::test_codec_boundaries_and_unknown_type();
    rbp2_test::test_stream_decoder_contract();
    rbp2_test::test_link_core_contract();

    if (rbp2_test::failures == 0) {
        std::cout << "All Raspberry Pi Protocol V2 tests passed\n";
        return EXIT_SUCCESS;
    }
    std::cerr << rbp2_test::failures << " test assertions failed\n";
    return EXIT_FAILURE;
}
