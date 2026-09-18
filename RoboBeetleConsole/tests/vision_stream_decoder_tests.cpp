#include "vision/VisionStreamDecoder.h"

#include <QByteArray>
#include <QCryptographicHash>

#include <array>
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

constexpr VisionHeaderWire kFixtureHeader{{
    0x52U, 0x42U, 0x56U, 0x53U, 0x01U, 0x20U, 0x01U, 0x00U,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x01U,
    0x00U, 0x02U, 0x00U, 0x02U, 0x00U, 0x00U, 0x02U, 0xaeU,
}};

std::vector<std::uint8_t> completeFrame(
    std::uint64_t frameId, std::uint64_t timestamp,
    const std::vector<std::uint8_t> &payload)
{
    VisionFrameHeader header;
    header.frameId = frameId;
    header.captureTimestampNs = timestamp;
    header.width = 2U;
    header.height = 2U;
    header.payloadSize = static_cast<std::uint32_t>(payload.size());
    const auto encoded = encodeVisionHeader(header);
    expect(encoded.wire.has_value(), "test frame header must encode");

    std::vector<std::uint8_t> wire;
    if (!encoded.wire) {
        return wire;
    }
    wire.insert(wire.end(), encoded.wire->begin(), encoded.wire->end());
    wire.insert(wire.end(), payload.begin(), payload.end());
    return wire;
}

QByteArray sha256(const std::vector<std::uint8_t> &bytes)
{
    return QCryptographicHash::hash(
        QByteArray(reinterpret_cast<const char *>(bytes.data()),
                   static_cast<qsizetype>(bytes.size())),
        QCryptographicHash::Sha256).toHex();
}

void frozenFullFrameFixtureParsesByteByByte()
{
    const auto payload = fixturePayload();
    expect(payload.size() == 686U, "frozen JPEG fixture has exact size");
    expect(sha256(payload) ==
               QByteArray("742b658462184778ddf5d92207786bb7f08182f75e7f0f5444f8112192b50cd6"),
           "frozen JPEG fixture SHA-256 matches contract");

    std::vector<std::uint8_t> wire(kFixtureHeader.begin(), kFixtureHeader.end());
    wire.insert(wire.end(), payload.begin(), payload.end());
    expect(sha256(wire) ==
               QByteArray("635bb1a0371f9c9c4242f2924a4c0ffb0590e3edff16945313a89407520bdfa6"),
           "frozen full-frame SHA-256 matches contract");

    VisionStreamDecoder decoder;
    std::optional<VisionWireFrame> completed;
    for (const auto byte : wire) {
        const std::array<std::uint8_t, 1U> one{{byte}};
        auto result = decoder.feed(one);
        expect(result.status == VisionFeedStatus::Ok,
               "bytewise fragmented golden frame remains valid");
        if (result.latestFrame) {
            completed = std::move(result.latestFrame);
        }
    }

    expect(completed.has_value(), "golden frame completes after final payload byte");
    if (completed) {
        expect(completed->header.frameId == 0U &&
                   completed->header.captureTimestampNs == 1U,
               "golden frame metadata is preserved");
        expect(completed->header.width == 2U && completed->header.height == 2U,
               "golden frame dimensions are preserved");
        expect(completed->jpegPayload == payload,
               "golden JPEG payload is preserved byte-for-byte");
    }
    expect(decoder.finish() == VisionStreamError::None,
           "clean frame boundary is a clean connection finish");
}

void coalescedFramesKeepOnlyNewestCompleteFrame()
{
    const auto payload = fixturePayload();
    auto first = completeFrame(1U, 10U, payload);
    auto second = completeFrame(4U, 20U, payload);
    first.insert(first.end(), second.begin(), second.end());

    VisionStreamDecoder decoder;
    const auto result = decoder.feed(first);
    expect(result.status == VisionFeedStatus::Ok,
           "coalesced frames parse successfully");
    expect(result.completedFrames == 2U && result.replacedFrames == 1U,
           "coalesced feed reports one latest-frame replacement");
    expect(result.latestFrame && result.latestFrame->header.frameId == 4U,
           "coalesced feed exposes only the newest complete frame");
    expect(decoder.lastFrameId() == 4U,
           "sequence state advances across an allowed frame-id gap");
}

void duplicateAndBackwardIdsAreConnectionFatal()
{
    const auto payload = fixturePayload();
    VisionStreamDecoder duplicate;
    const auto first = completeFrame(7U, 10U, payload);
    expect(duplicate.feed(first).status == VisionFeedStatus::Ok,
           "initial sequence frame is accepted");
    expect(duplicate.feed(first).status == VisionFeedStatus::Fatal &&
               duplicate.fatalError() == VisionStreamError::FrameIdNotIncreasing,
           "duplicate frame id is connection-fatal");

    VisionStreamDecoder backward;
    expect(backward.feed(completeFrame(9U, 10U, payload)).status ==
               VisionFeedStatus::Ok,
           "backward test initial frame is accepted");
    expect(backward.feed(completeFrame(8U, 20U, payload)).status ==
               VisionFeedStatus::Fatal &&
               backward.fatalError() == VisionStreamError::FrameIdNotIncreasing,
           "backward frame id is connection-fatal");

    backward.reset();
    expect(backward.feed(completeFrame(0U, 30U, payload)).status ==
               VisionFeedStatus::Ok,
           "new connection reset permits frame id zero again");
}

void partialConnectionCloseIsTruncated()
{
    VisionStreamDecoder headerDecoder;
    expect(headerDecoder.feed(std::span(kFixtureHeader).first(12U)).status ==
               VisionFeedStatus::Ok,
           "partial header is buffered");
    expect(headerDecoder.finish() == VisionStreamError::TruncatedFrame,
           "disconnect in partial header is truncated");

    const auto payload = fixturePayload();
    std::vector<std::uint8_t> partial(kFixtureHeader.begin(), kFixtureHeader.end());
    partial.insert(partial.end(), payload.begin(), payload.begin() + 50);

    VisionStreamDecoder payloadDecoder;
    expect(payloadDecoder.feed(partial).status == VisionFeedStatus::Ok &&
               payloadDecoder.bufferedPayloadBytes() == 50U,
           "partial payload is buffered within its bounded declared size");
    expect(payloadDecoder.finish() == VisionStreamError::TruncatedFrame,
           "disconnect in partial JPEG payload is truncated");
}

void invalidHeaderFailsBeforePayloadAllocation()
{
    VisionHeaderWire badMagic = kFixtureHeader;
    badMagic[0] = static_cast<std::uint8_t>('X');

    VisionStreamDecoder decoder;
    const auto bad = decoder.feed(badMagic);
    expect(bad.status == VisionFeedStatus::Fatal &&
               decoder.fatalError() == VisionStreamError::HeaderInvalid &&
               decoder.headerError() == VisionProtocolError::BadMagic,
           "bad magic becomes a classified connection-fatal framing error");
    expect(decoder.bufferedPayloadBytes() == 0U,
           "bad header never allocates payload bytes");

    VisionHeaderWire tooLarge = kFixtureHeader;
    tooLarge[28] = 0x00U;
    tooLarge[29] = 0x40U;
    tooLarge[30] = 0x00U;
    tooLarge[31] = 0x01U;

    VisionStreamDecoder largeDecoder;
    const auto large = largeDecoder.feed(tooLarge);
    expect(large.status == VisionFeedStatus::Fatal &&
               largeDecoder.headerError() == VisionProtocolError::InvalidPayloadSize,
           "payload over 4 MiB is rejected from the header");
    expect(largeDecoder.bufferedPayloadBytes() == 0U,
           "oversized payload is rejected before payload allocation");
}

void maximumFrameIdCannotWrapInsideConnection()
{
    const auto payload = fixturePayload();
    VisionStreamDecoder decoder;
    expect(decoder.feed(completeFrame(UINT64_MAX, 1U, payload)).status ==
               VisionFeedStatus::Ok,
           "UINT64_MAX is a valid first frame id");
    expect(decoder.feed(completeFrame(0U, 2U, payload)).status ==
               VisionFeedStatus::Fatal &&
               decoder.fatalError() == VisionStreamError::FrameIdNotIncreasing,
           "frame id wrap requires a new TCP connection");
}

} // namespace

int main()
{
    frozenFullFrameFixtureParsesByteByByte();
    coalescedFramesKeepOnlyNewestCompleteFrame();
    duplicateAndBackwardIdsAreConnectionFatal();
    partialConnectionCloseIsTruncated();
    invalidHeaderFailsBeforePayloadAllocation();
    maximumFrameIdCannotWrapInsideConnection();
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
