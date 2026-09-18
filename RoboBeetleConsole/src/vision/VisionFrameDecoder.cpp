#include "vision/VisionFrameDecoder.h"

#include <QBuffer>
#include <QByteArray>
#include <QImageReader>
#include <QIODevice>
#include <QSize>

namespace rb::vision {

VisionFrameDecoder::VisionFrameDecoder()
{
    QImageReader::setAllocationLimit(kVisionImageAllocationLimitMiB);
}

VisionFrameFeedResult VisionFrameDecoder::feed(
    std::span<const std::uint8_t> data)
{
    auto wireResult = streamDecoder_.feed(data);
    VisionFrameFeedResult result;
    result.completedWireFrames = wireResult.completedFrames;
    result.replacedWireFrames = wireResult.replacedFrames;

    if (wireResult.status == VisionFeedStatus::Fatal) {
        result.fatal = true;
        result.error = VisionFrameError::StreamFatal;
        return result;
    }
    if (!wireResult.latestFrame.has_value()) {
        return result;
    }

    const VisionWireFrame &wireFrame = *wireResult.latestFrame;
    const auto &payload = wireFrame.jpegPayload;
    QByteArray payloadView = QByteArray::fromRawData(
        reinterpret_cast<const char *>(payload.data()),
        static_cast<qsizetype>(payload.size()));
    QBuffer buffer(&payloadView);
    if (!buffer.open(QIODevice::ReadOnly)) {
        ++jpegDecodeErrors_;
        result.error = VisionFrameError::JpegHeaderInvalid;
        return result;
    }

    QImageReader reader(&buffer, QByteArrayLiteral("JPEG"));
    const QSize declaredSize = reader.size();
    if (!declaredSize.isValid() ||
        declaredSize.width() <= 0 || declaredSize.height() <= 0) {
        ++jpegDecodeErrors_;
        result.error = VisionFrameError::JpegHeaderInvalid;
        return result;
    }

    const std::uint64_t decodedPixels =
        static_cast<std::uint64_t>(declaredSize.width()) *
        static_cast<std::uint64_t>(declaredSize.height());
    if (decodedPixels > kVisionMaxDecodedPixels ||
        declaredSize.width() != static_cast<int>(wireFrame.header.width) ||
        declaredSize.height() != static_cast<int>(wireFrame.header.height)) {
        ++dimensionMismatches_;
        result.error = VisionFrameError::JpegDimensionMismatch;
        return result;
    }

    QImage image = reader.read();
    if (image.isNull()) {
        ++jpegDecodeErrors_;
        result.error = VisionFrameError::JpegDecodeFailed;
        return result;
    }
    if (image.width() != declaredSize.width() ||
        image.height() != declaredSize.height()) {
        ++dimensionMismatches_;
        result.error = VisionFrameError::JpegDimensionMismatch;
        return result;
    }

    result.latestFrame = DecodedVisionFrame{
        wireFrame.header,
        std::move(image),
    };
    return result;
}

} // namespace rb::vision
