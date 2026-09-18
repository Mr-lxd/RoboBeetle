#include "vision/VisionFrameDecoder.h"

#include <QImageReader>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <vector>

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

std::vector<std::uint8_t> fixturePayload()
{
    const auto path = std::filesystem::path(__FILE__).parent_path()
        / "fixtures" / "rbvs_v1_2x2.jpg";
    std::ifstream input(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

std::vector<std::uint8_t> makeFrame(
    std::uint64_t frameId,
    std::uint16_t width,
    std::uint16_t height,
    const std::vector<std::uint8_t> &payload)
{
    VisionFrameHeader header;
    header.frameId = frameId;
    header.captureTimestampNs = frameId + 1U;
    header.width = width;
    header.height = height;
    header.payloadSize = static_cast<std::uint32_t>(payload.size());
    const auto encoded = encodeVisionHeader(header);
    expect(encoded.wire.has_value(), "test RBVS header must encode");

    std::vector<std::uint8_t> wire;
    if (!encoded.wire) {
        return wire;
    }
    wire.insert(wire.end(), encoded.wire->begin(), encoded.wire->end());
    wire.insert(wire.end(), payload.begin(), payload.end());
    return wire;
}

void validJpegIsPreflightedAndDecoded()
{
    VisionFrameDecoder decoder;
    expect(QImageReader::allocationLimit() == kVisionImageAllocationLimitMiB,
           "Vision decoder configures the Qt image allocation limit");

    const auto payload = fixturePayload();
    const auto result = decoder.feed(makeFrame(0U, 2U, 2U, payload));

    expect(!result.fatal && result.error == VisionFrameError::None,
           "valid JPEG frame is not fatal");
    expect(result.latestFrame.has_value(), "valid JPEG frame decodes");
    if (result.latestFrame) {
        expect(result.latestFrame->header.frameId == 0U,
               "decoded frame preserves frame id");
        expect(result.latestFrame->image.width() == 2 &&
                   result.latestFrame->image.height() == 2,
               "decoded image dimensions match RBVS header");
    }
}

void corruptJpegDropsOneFrameAndNextFrameStillDecodes()
{
    VisionFrameDecoder decoder;
    const std::vector<std::uint8_t> corrupt{
        'n', 'o', 't', '-', 'j', 'p', 'e', 'g'
    };
    const auto bad = decoder.feed(makeFrame(1U, 2U, 2U, corrupt));

    expect(!bad.fatal && !bad.latestFrame.has_value(),
           "length-correct corrupt JPEG is dropped without killing stream");
    expect(bad.error == VisionFrameError::JpegHeaderInvalid ||
               bad.error == VisionFrameError::JpegDecodeFailed,
           "corrupt JPEG has a frame-level decode classification");

    const auto good = decoder.feed(makeFrame(2U, 2U, 2U, fixturePayload()));
    expect(!good.fatal && good.latestFrame.has_value() &&
               good.latestFrame->header.frameId == 2U,
           "next higher frame id decodes after corrupt JPEG");
}

void jpegDeclaredDimensionsMustMatchRbvsHeader()
{
    VisionFrameDecoder decoder;
    const auto mismatch = decoder.feed(
        makeFrame(3U, 3U, 2U, fixturePayload()));

    expect(!mismatch.fatal && !mismatch.latestFrame.has_value(),
           "JPEG/header dimension mismatch drops one frame");
    expect(mismatch.error == VisionFrameError::JpegDimensionMismatch,
           "dimension mismatch is classified before full image acceptance");
    expect(decoder.dimensionMismatches() == 1U,
           "dimension mismatch diagnostic increments");
}

void coalescedWireFramesDecodeOnlyNewest()
{
    const auto payload = fixturePayload();
    auto wire = makeFrame(10U, 2U, 2U, payload);
    const auto newer = makeFrame(20U, 2U, 2U, payload);
    wire.insert(wire.end(), newer.begin(), newer.end());

    VisionFrameDecoder decoder;
    const auto result = decoder.feed(wire);

    expect(!result.fatal && result.latestFrame.has_value(),
           "coalesced valid frames produce a decoded frame");
    expect(result.completedWireFrames == 2U && result.replacedWireFrames == 1U,
           "compressed decode input remains latest-only for coalesced frames");
    if (result.latestFrame) {
        expect(result.latestFrame->header.frameId == 20U,
               "only newest coalesced frame reaches JPEG decode");
    }
}

void streamFramingFailureRemainsConnectionFatal()
{
    auto wire = makeFrame(1U, 2U, 2U, fixturePayload());
    wire[0] = static_cast<std::uint8_t>('X');

    VisionFrameDecoder decoder;
    const auto result = decoder.feed(wire);

    expect(result.fatal && result.error == VisionFrameError::StreamFatal,
           "malformed RBVS header remains connection-fatal");
    expect(decoder.streamError() == VisionStreamError::HeaderInvalid &&
               decoder.headerError() == VisionProtocolError::BadMagic,
           "framing error detail is preserved below JPEG layer");
}

void resetClearsSessionDiagnostics()
{
    VisionFrameDecoder decoder;
    const std::vector<std::uint8_t> corrupt{
        'n', 'o', 't', '-', 'j', 'p', 'e', 'g'
    };

    const auto bad = decoder.feed(makeFrame(30U, 2U, 2U, corrupt));
    expect(!bad.fatal && !bad.latestFrame.has_value(),
           "corrupt JPEG increments a session diagnostic before reset");
    expect(decoder.jpegDecodeErrors() == 1U,
           "JPEG diagnostic is nonzero before reset");

    decoder.reset();

    expect(decoder.jpegDecodeErrors() == 0U,
           "reset clears JPEG diagnostics for the new TCP session");
    expect(decoder.dimensionMismatches() == 0U,
           "reset clears dimension diagnostics for the new TCP session");
}

} // namespace

int main()
{
    validJpegIsPreflightedAndDecoded();
    corruptJpegDropsOneFrameAndNextFrameStillDecodes();
    jpegDeclaredDimensionsMustMatchRbvsHeader();
    coalescedWireFramesDecodeOnlyNewest();
    streamFramingFailureRemainsConnectionFatal();
    resetClearsSessionDiagnostics();
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
