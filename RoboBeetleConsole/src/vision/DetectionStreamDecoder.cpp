#include "vision/DetectionStreamDecoder.h"

#include "vision/VisionProtocol.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>

#include <cmath>
#include <limits>
#include <optional>

namespace rb::vision {
namespace {

std::optional<quint64> readUInt64(const QJsonValue &value, bool positive)
{
    if (!value.isDouble()) {
        return std::nullopt;
    }
    const double number = value.toDouble();
    if (!std::isfinite(number) || number < 0.0 || std::floor(number) != number) {
        return std::nullopt;
    }
    if (number > static_cast<double>(std::numeric_limits<qint64>::max())) {
        return std::nullopt;
    }
    const qint64 integer = value.toInteger(-1);
    if (integer < 0 || (positive && integer == 0)) {
        return std::nullopt;
    }
    return static_cast<quint64>(integer);
}

std::optional<int> readInt(const QJsonValue &value, int minimum, int maximum)
{
    const auto integer = readUInt64(value, false);
    if (!integer.has_value()
        || *integer < static_cast<quint64>(minimum)
        || *integer > static_cast<quint64>(maximum)) {
        return std::nullopt;
    }
    return static_cast<int>(*integer);
}

std::optional<double> readFiniteDouble(const QJsonValue &value)
{
    if (!value.isDouble()) {
        return std::nullopt;
    }
    const double number = value.toDouble();
    return std::isfinite(number) ? std::optional<double>(number) : std::nullopt;
}

std::optional<DetectionFrame> parseLine(const QByteArray &line, QString *error)
{
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(line, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
        *error = QStringLiteral("Detection metadata is not a JSON object");
        return std::nullopt;
    }

    const QJsonObject object = document.object();
    if (object.value(QStringLiteral("type")).toString()
        != QStringLiteral("detections")) {
        *error = QStringLiteral("Detection metadata type is unsupported");
        return std::nullopt;
    }
    const auto version = readInt(
        object.value(QStringLiteral("version")),
        kDetectionStreamVersion,
        kDetectionStreamVersion);
    if (!version.has_value()) {
        *error = QStringLiteral("Detection metadata version is unsupported");
        return std::nullopt;
    }
    if (object.value(QStringLiteral("coordinate_space")).toString()
        != QStringLiteral("original_frame_pixels")) {
        *error = QStringLiteral("Detection coordinate space is unsupported");
        return std::nullopt;
    }

    const auto frameId = readUInt64(
        object.value(QStringLiteral("frame_id")), false);
    const auto captureTimestampNs = readUInt64(
        object.value(QStringLiteral("capture_timestamp_ns")), true);
    const auto width = readInt(
        object.value(QStringLiteral("width")), 1, kVisionMaxDimension);
    const auto height = readInt(
        object.value(QStringLiteral("height")), 1, kVisionMaxDimension);
    if (!frameId.has_value() || !captureTimestampNs.has_value()
        || !width.has_value() || !height.has_value()) {
        *error = QStringLiteral("Detection frame identity or dimensions are invalid");
        return std::nullopt;
    }
    if (static_cast<quint64>(*width) * static_cast<quint64>(*height)
        > kVisionMaxDecodedPixels) {
        *error = QStringLiteral("Detection source pixel count is unsupported");
        return std::nullopt;
    }

    const QJsonValue detectionsValue = object.value(QStringLiteral("detections"));
    if (!detectionsValue.isArray()) {
        *error = QStringLiteral("Detection metadata detections must be an array");
        return std::nullopt;
    }
    const QJsonArray array = detectionsValue.toArray();
    if (array.size() > kDetectionStreamMaxDetections) {
        *error = QStringLiteral("Detection metadata has too many detections");
        return std::nullopt;
    }

    DetectionFrame frame;
    frame.frameId = *frameId;
    frame.captureTimestampNs = *captureTimestampNs;
    frame.sourceSize = QSize(*width, *height);
    frame.detections.reserve(array.size());

    for (const QJsonValue &entry : array) {
        if (!entry.isObject()) {
            *error = QStringLiteral("Detection entry must be an object");
            return std::nullopt;
        }
        const QJsonObject detection = entry.toObject();
        const auto classId = readInt(
            detection.value(QStringLiteral("class_id")),
            0,
            std::numeric_limits<int>::max());
        const QJsonValue classNameValue =
            detection.value(QStringLiteral("class_name"));
        const auto confidence = readFiniteDouble(
            detection.value(QStringLiteral("confidence")));
        const auto originalX = readFiniteDouble(
            detection.value(QStringLiteral("original_x")));
        const auto originalY = readFiniteDouble(
            detection.value(QStringLiteral("original_y")));
        std::optional<int> areaCells;
        if (detection.contains(QStringLiteral("component_area_cells"))) {
            areaCells = readInt(
                detection.value(QStringLiteral("component_area_cells")),
                1,
                std::numeric_limits<int>::max());
            if (!areaCells.has_value()) {
                *error = QStringLiteral("Detection component area is invalid");
                return std::nullopt;
            }
        }
        if (!classId.has_value() || !classNameValue.isString()
            || !confidence.has_value() || !originalX.has_value()
            || !originalY.has_value()) {
            *error = QStringLiteral("Detection entry has invalid fields");
            return std::nullopt;
        }
        const QString className = classNameValue.toString();
        const qsizetype nameBytes = className.toUtf8().size();
        if (nameBytes <= 0 || nameBytes > kDetectionStreamMaxClassNameBytes) {
            *error = QStringLiteral("Detection class name is invalid");
            return std::nullopt;
        }
        if (*confidence < 0.0 || *confidence > 1.0) {
            *error = QStringLiteral("Detection confidence is outside 0..1");
            return std::nullopt;
        }
        if (*originalX < 0.0 || *originalX > static_cast<double>(*width - 1)
            || *originalY < 0.0
            || *originalY > static_cast<double>(*height - 1)) {
            *error = QStringLiteral("Detection centroid is outside the source frame");
            return std::nullopt;
        }

        DetectionObservation observation;
        observation.classId = *classId;
        observation.className = className;
        observation.confidence = *confidence;
        observation.originalPoint = QPointF(*originalX, *originalY);
        observation.areaCells = areaCells;
        frame.detections.push_back(std::move(observation));
    }
    return frame;
}

} // namespace

DetectionDecodeResult DetectionStreamDecoder::feed(const QByteArray &bytes)
{
    DetectionDecodeResult result;
    if (bytes.isEmpty()) {
        return result;
    }

    buffer_.append(bytes);
    while (true) {
        const qsizetype newline = buffer_.indexOf('\n');
        if (newline < 0) {
            if (buffer_.size() > kDetectionStreamMaxLineBytes) {
                result.fatal = true;
                result.error = QStringLiteral("Detection metadata line exceeds 64 KiB");
                reset();
            }
            return result;
        }
        if (newline >= kDetectionStreamMaxLineBytes) {
            result.fatal = true;
            result.error = QStringLiteral("Detection metadata line exceeds 64 KiB");
            reset();
            return result;
        }

        QByteArray line = buffer_.left(newline);
        buffer_.remove(0, newline + 1);
        if (line.endsWith('\r')) {
            line.chop(1);
        }
        if (line.isEmpty()) {
            result.fatal = true;
            result.error = QStringLiteral("Detection metadata line must not be empty");
            reset();
            return result;
        }

        QString error;
        auto frame = parseLine(line, &error);
        if (!frame.has_value()) {
            result.fatal = true;
            result.error = error;
            reset();
            return result;
        }
        result.frames.push_back(std::move(*frame));
    }
}

void DetectionStreamDecoder::reset()
{
    buffer_.clear();
}

} // namespace rb::vision
