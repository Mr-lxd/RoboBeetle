#include "vision/DetectionMetadata.h"
#include "vision/DetectionStreamDecoder.h"

#include <QByteArray>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <cstdio>
#include <cstdlib>

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

QByteArray line(
    quint64 frameId = 42,
    quint64 timestampNs = 1'000'000,
    int width = 640,
    int height = 480,
    QJsonArray detections = {})
{
    QJsonObject object;
    object.insert(QStringLiteral("type"), QStringLiteral("detections"));
    object.insert(QStringLiteral("version"), kDetectionStreamVersion);
    object.insert(QStringLiteral("frame_id"), static_cast<qint64>(frameId));
    object.insert(
        QStringLiteral("capture_timestamp_ns"),
        static_cast<qint64>(timestampNs));
    object.insert(QStringLiteral("width"), width);
    object.insert(QStringLiteral("height"), height);
    object.insert(
        QStringLiteral("coordinate_space"),
        QStringLiteral("original_frame_pixels"));
    object.insert(QStringLiteral("detections"), detections);
    return QJsonDocument(object).toJson(QJsonDocument::Compact) + '\n';
}

QJsonObject detection(
    int classId = 3,
    const QString &name = QStringLiteral("fish"),
    double confidence = 0.875,
    double x = 320.5,
    double y = 207.25)
{
    return QJsonObject{
        {QStringLiteral("class_id"), classId},
        {QStringLiteral("class_name"), name},
        {QStringLiteral("confidence"), confidence},
        {QStringLiteral("original_x"), x},
        {QStringLiteral("original_y"), y},
    };
}

void fragmentedValidRecordParsesOnlyWhenComplete()
{
    DetectionStreamDecoder decoder;
    const QByteArray payload = line(
        42,
        1'000'042,
        640,
        480,
        QJsonArray{detection()});

    auto first = decoder.feed(payload.left(payload.size() / 2));
    expect(!first.fatal && first.frames.isEmpty(),
           "partial NDJSON stays buffered without emitting metadata");
    expect(decoder.hasPartialRecord(),
           "partial NDJSON is observable as buffered state");

    auto second = decoder.feed(payload.mid(payload.size() / 2));
    expect(!second.fatal && second.frames.size() == 1,
           "completed NDJSON emits exactly one detection frame");
    expect(!decoder.hasPartialRecord(),
           "complete NDJSON leaves no partial record");
    if (second.frames.size() == 1) {
        const auto &frame = second.frames.front();
        expect(frame.frameId == 42U
                   && frame.captureTimestampNs == 1'000'042U,
               "frame identity is parsed exactly");
        expect(frame.sourceSize == QSize(640, 480),
               "source dimensions are parsed");
        expect(frame.detections.size() == 1,
               "one detection is parsed");
        if (frame.detections.size() == 1) {
            const auto &item = frame.detections.front();
            expect(item.classId == 3
                       && item.className == QStringLiteral("fish"),
                   "class fields are parsed");
            expect(item.confidence == 0.875,
                   "confidence is parsed");
            expect(item.originalPoint == QPointF(320.5, 207.25),
                   "original-frame centroid is parsed");
        }
    }
}

void multipleRecordsPreserveWireOrder()
{
    DetectionStreamDecoder decoder;
    auto result = decoder.feed(
        line(8, 1000) + line(9, 1100) + line(12, 1200));

    expect(!result.fatal && result.frames.size() == 3,
           "one TCP read may contain multiple metadata records");
    if (result.frames.size() == 3) {
        expect(result.frames.at(0).frameId == 8U
                   && result.frames.at(1).frameId == 9U
                   && result.frames.at(2).frameId == 12U,
               "decoder preserves metadata wire order");
    }
}

void emptyDetectionArrayIsValid()
{
    DetectionStreamDecoder decoder;
    auto result = decoder.feed(line(5, 9999));

    expect(!result.fatal && result.frames.size() == 1,
           "zero-detection result remains a valid metadata frame");
    if (result.frames.size() == 1) {
        expect(result.frames.front().detections.isEmpty(),
               "empty detections can clear old overlay text");
    }
}

void areaCellsIsOptionalPositiveInteger()
{
    const auto decode = [](const QJsonValue &area, bool include) {
        QJsonObject d = detection();
        if (include) d.insert(QStringLiteral("component_area_cells"), area);
        DetectionStreamDecoder decoder;
        return decoder.feed(line(1, 100, 640, 480, QJsonArray{d}));
    };
    auto present = decode(4, true);
    expect(!present.fatal && present.frames.size() == 1
               && present.frames.front().detections.front().areaCells == std::optional<int>(4),
           "component_area_cells is decoded");
    auto missing = decode(0, false);
    expect(!missing.fatal && missing.frames.size() == 1
               && !missing.frames.front().detections.front().areaCells.has_value(),
           "a missing component_area_cells is accepted as unknown");
    for (const QJsonValue &bad : {QJsonValue(0), QJsonValue(-1), QJsonValue(2.5), QJsonValue(QStringLiteral("3")),
                                  QJsonValue(true), QJsonValue(QJsonValue::Null)}) {
        expect(decode(bad, true).fatal, "invalid component_area_cells is fatal protocol input");
    }
}

void invalidSchemaFailsClosed()
{
    {
        DetectionStreamDecoder decoder;
        QByteArray payload = line();
        payload.replace(
            QByteArrayLiteral("\"version\":1"),
            QByteArrayLiteral("\"version\":2"));
        auto result = decoder.feed(payload);
        expect(result.fatal,
               "unsupported detection metadata version fails closed");
    }
    {
        DetectionStreamDecoder decoder;
        QByteArray payload = line(
            1, 100, 640, 480,
            QJsonArray{detection(0, QStringLiteral("fish"), 1.1)});
        auto result = decoder.feed(payload);
        expect(result.fatal,
               "confidence outside 0..1 is fatal protocol input");
    }
    {
        DetectionStreamDecoder decoder;
        QByteArray payload = line(
            1, 100, 640, 480,
            QJsonArray{detection(0, QStringLiteral("fish"), 0.8, 640.0, 2.0)});
        auto result = decoder.feed(payload);
        expect(result.fatal,
               "centroid outside source frame is fatal protocol input");
    }
    {
        DetectionStreamDecoder decoder;
        QJsonArray items;
        for (int i = 0; i < 257; ++i) {
            items.append(detection());
        }
        auto result = decoder.feed(line(1, 100, 640, 480, items));
        expect(result.fatal,
               "more than 256 detections is rejected");
    }
    {
        DetectionStreamDecoder decoder;
        auto result = decoder.feed(
            QByteArray(kDetectionStreamMaxLineBytes + 1, 'x'));
        expect(result.fatal,
               "unterminated metadata larger than 64 KiB is fatal");
    }
    {
        DetectionStreamDecoder decoder;
        QByteArray oversized(kDetectionStreamMaxLineBytes, 'x');
        oversized.append('\n');
        auto result = decoder.feed(oversized);
        expect(result.fatal,
               "64 KiB body plus newline exceeds the full-record wire limit");
    }
}

void freshnessGateMatchesFrozenUiContract()
{
    DetectionFrame frame;
    frame.frameId = 10;
    frame.captureTimestampNs = 2'000'000'000ULL;
    frame.sourceSize = QSize(640, 480);
    frame.detections.push_back(
        DetectionObservation{
            0,
            QStringLiteral("fish"),
            0.9,
            QPointF(100.0, 120.0)});

    expect(detectionOverlayRenderable(
               frame, QSize(640, 480),
               2'000'000'000ULL, true, true),
           "same-time running metadata is renderable");
    expect(detectionOverlayRenderable(
               frame, QSize(640, 480),
               3'500'000'000ULL, true, true),
           "exactly 1500 ms old metadata remains renderable");
    expect(!detectionOverlayRenderable(
               frame, QSize(640, 480),
               3'500'000'001ULL, true, true),
           "metadata older than 1500 ms is suppressed");
    expect(!detectionOverlayRenderable(
               frame, QSize(640, 480),
               1'999'999'999ULL, true, true),
           "metadata newer than the displayed video frame is suppressed");
    expect(detectionOverlayState(frame, QSize(640, 480),
                                1'999'999'999ULL, true, true)
               == DetectionDisplayState::AwaitingVideo,
           "a valid detection ahead of video is waiting, not immediately stale");
    expect(detectionOverlayState(frame, QSize(640, 480), 0U, true, true)
               == DetectionDisplayState::Stale,
           "zero video timestamp is not the ahead-of-video race");
    expect(!detectionOverlayRenderable(
               frame, QSize(320, 240),
               2'100'000'000ULL, true, true),
           "dimension mismatch suppresses overlay");
    expect(!detectionOverlayRenderable(
               frame, QSize(640, 480),
               2'100'000'000ULL, false, true),
           "non-running inference suppresses overlay");
    expect(!detectionOverlayRenderable(
               frame, QSize(640, 480),
               2'100'000'000ULL, true, false),
           "stale HTTP status suppresses overlay");

    frame.detections.clear();
    expect(detectionOverlayState(frame, QSize(640, 480),
                                2'100'000'000ULL, true, true)
               == DetectionDisplayState::NoTarget,
           "fresh empty result is NO_TARGET");
    expect(detectionOverlayState(frame, QSize(640, 480),
                                3'500'000'001ULL, true, true)
               == DetectionDisplayState::Stale,
           "expired empty result is STALE rather than NO_TARGET");
    expect(detectionOverlayState(frame, QSize(640, 480),
                                2'100'000'000ULL, false, true)
               == DetectionDisplayState::InferenceOff,
           "fresh off inference is INFERENCE_OFF rather than NO_TARGET");
    expect(detectionOverlayState(frame, QSize(640, 480),
                                2'100'000'000ULL, false, false)
               == DetectionDisplayState::Stale,
           "stale HTTP status cannot assert authoritative INFERENCE_OFF");
    expect(!detectionOverlayRenderable(
               frame, QSize(640, 480),
               2'100'000'000ULL, true, true),
           "zero detections suppresses old overlay text");
}

} // namespace

int main()
{
    fragmentedValidRecordParsesOnlyWhenComplete();
    multipleRecordsPreserveWireOrder();
    emptyDetectionArrayIsValid();
    areaCellsIsOptionalPositiveInteger();
    invalidSchemaFailsClosed();
    freshnessGateMatchesFrozenUiContract();
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
