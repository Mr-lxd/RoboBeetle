#include "vision/VisionProtocol.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <utility>

namespace {

using namespace rb::vision;

int failures = 0;

void expect(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

VisionFrameHeader goldenHeader()
{
    VisionFrameHeader header;
    header.frameId = 0x0102030405060708ULL;
    header.captureTimestampNs = 0x1112131415161718ULL;
    header.width = 640U;
    header.height = 480U;
    header.payloadSize = 0x00012345U;
    return header;
}

constexpr VisionHeaderWire kGoldenWire{{
    0x52U, 0x42U, 0x56U, 0x53U, 0x01U, 0x20U, 0x01U, 0x00U,
    0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U, 0x07U, 0x08U,
    0x11U, 0x12U, 0x13U, 0x14U, 0x15U, 0x16U, 0x17U, 0x18U,
    0x02U, 0x80U, 0x01U, 0xe0U, 0x00U, 0x01U, 0x23U, 0x45U,
}};

void goldenVectorAndRoundTrip()
{
    const auto encoded = encodeVisionHeader(goldenHeader());
    expect(kVisionHeaderSize == 32U, "RBVS v1 header size is frozen at 32 bytes");
    expect(encoded.error == VisionProtocolError::None && encoded.wire.has_value(),
           "golden semantic header encodes");
    if (!encoded.wire) {
        return;
    }
    expect(*encoded.wire == kGoldenWire,
           "C++ encoder matches the shared Python/C++ golden bytes");

    const auto decoded = decodeVisionHeader(*encoded.wire);
    expect(decoded.error == VisionProtocolError::None && decoded.header.has_value(),
           "golden wire header decodes");
    if (!decoded.header) {
        return;
    }
    const auto expected = goldenHeader();
    expect(decoded.header->frameId == expected.frameId,
           "frame ID round-trips as big-endian uint64");
    expect(decoded.header->captureTimestampNs == expected.captureTimestampNs,
           "capture timestamp round-trips as big-endian uint64");
    expect(decoded.header->width == 640U && decoded.header->height == 480U,
           "dimensions round-trip as big-endian uint16");
    expect(decoded.header->payloadSize == 0x00012345U,
           "payload size round-trips as big-endian uint32");
    expect(decoded.header->codec == PayloadCodec::Jpeg && decoded.header->flags == 0U,
           "v1 codec and reserved flags remain frozen");
}

void malformedRegistryFieldsAreRejected()
{
    const std::array<std::pair<std::size_t, std::uint8_t>, 5U> mutations{{
        {0U, static_cast<std::uint8_t>('X')},
        {4U, 2U},
        {5U, 31U},
        {6U, 2U},
        {7U, 1U},
    }};
    const std::array<VisionProtocolError, 5U> errors{{
        VisionProtocolError::BadMagic,
        VisionProtocolError::UnsupportedVersion,
        VisionProtocolError::UnsupportedHeaderSize,
        VisionProtocolError::UnsupportedCodec,
        VisionProtocolError::NonzeroFlags,
    }};
    for (std::size_t index = 0; index < mutations.size(); ++index) {
        VisionHeaderWire wire = kGoldenWire;
        wire[mutations[index].first] = mutations[index].second;
        const auto result = decodeVisionHeader(wire);
        expect(result.error == errors[index] && !result.header.has_value(),
               "malformed RBVS registry field is rejected before payload handling");
    }
}

void semanticLimitsAreRejected()
{
    VisionFrameHeader invalid = goldenHeader();
    invalid.captureTimestampNs = 0U;
    expect(validateVisionHeader(invalid) == VisionProtocolError::InvalidTimestamp,
           "zero monotonic capture timestamp is rejected");

    invalid = goldenHeader();
    invalid.width = 0U;
    expect(validateVisionHeader(invalid) == VisionProtocolError::InvalidDimensions,
           "zero width is rejected");

    invalid = goldenHeader();
    invalid.payloadSize = 0U;
    expect(validateVisionHeader(invalid) == VisionProtocolError::InvalidPayloadSize,
           "zero JPEG payload is rejected");

    invalid = goldenHeader();
    invalid.payloadSize = kVisionMaxPayloadSize + 1U;
    expect(validateVisionHeader(invalid) == VisionProtocolError::InvalidPayloadSize,
           "payload larger than 4 MiB is rejected before allocation");

    invalid = goldenHeader();
    invalid.width = 8192U;
    invalid.height = 8192U;
    expect(validateVisionHeader(invalid) == VisionProtocolError::InvalidDimensions,
           "decoded image larger than the 4K pixel profile is rejected");
}

void exactHeaderLengthAndZeroFrameId()
{
    const std::span<const std::uint8_t> shortHeader(kGoldenWire.data(), 31U);
    const auto shortResult = decodeVisionHeader(shortHeader);
    expect(shortResult.error == VisionProtocolError::WrongHeaderSize,
           "partial header is not interpreted as a complete RBVS header");

    VisionFrameHeader first = goldenHeader();
    first.frameId = 0U;
    const auto encoded = encodeVisionHeader(first);
    expect(encoded.error == VisionProtocolError::None && encoded.wire.has_value(),
           "frame ID zero is valid at a CameraOwner stream start");
}

} // namespace

int main()
{
    goldenVectorAndRoundTrip();
    malformedRegistryFieldsAreRejected();
    semanticLimitsAreRejected();
    exactHeaderLengthAndZeroFrameId();
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
