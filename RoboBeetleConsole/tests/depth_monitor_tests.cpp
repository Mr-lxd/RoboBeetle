#include "protocol/Packet.h"
#include "robot/DepthMonitor.h"
#include "robot/DepthSnapshot.h"

#include <QByteArray>
#include <QCoreApplication>

#include <cstdio>
#include <optional>

namespace {

int failures = 0;

void expect(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

rb::MessageType depthMessageType()
{
    return rb::MessageType::DepthSnapshot;
}

rb::DepthSnapshot validSnapshot()
{
    rb::DepthSnapshot snapshot;
    snapshot.validityFlags = rb::DepthSnapshot::DepthValid
        | rb::DepthSnapshot::TemperatureValid;
    snapshot.depthMm = -12345;
    snapshot.temperatureCentiC = -678;
    snapshot.sampleAgeMs = 0x1234;
    snapshot.diagnostics.rxByteCount = 0x01020304U;
    snapshot.diagnostics.validLineCount = 0x11121314U;
    snapshot.diagnostics.parseErrorCount = 0x21222324U;
    snapshot.diagnostics.overlongLineCount = 0x31323334U;
    snapshot.diagnostics.ringOverflowCount = 0x41424344U;
    snapshot.diagnostics.hardRearmFailureCount = 0x51525354U;
    snapshot.diagnostics.uartErrorCount = 0x61626364U;
    return snapshot;
}

void expectSameSnapshot(const rb::DepthSnapshot &actual,
                        const rb::DepthSnapshot &expected)
{
    expect(actual.validityFlags == expected.validityFlags,
           "validity flags differ after round trip");
    expect(actual.depthMm == expected.depthMm,
           "depth fixed-point value differs after round trip");
    expect(actual.temperatureCentiC == expected.temperatureCentiC,
           "temperature fixed-point value differs after round trip");
    expect(actual.sampleAgeMs == expected.sampleAgeMs,
           "sample age differs after round trip");
    expect(actual.diagnostics == expected.diagnostics,
           "diagnostics differ after round trip");
}

void testPayloadRoundTripAndGoldenBytes()
{
    const rb::DepthSnapshot expected = validSnapshot();
    const QByteArray payload = rb::DepthSnapshot::encodePayload(expected);
    const QByteArray golden = QByteArray::fromHex(
        "0103c7cfffff5afd3412040302011413121124232221"
        "34333231444342415453525164636261");

    expect(payload.size() == rb::DepthSnapshot::PayloadSize,
           "DepthSnapshot must use one fixed 38-byte payload");
    expect(payload == golden,
           "DepthSnapshot little-endian golden payload differs");

    const std::optional<rb::DepthSnapshot> decoded =
        rb::DepthSnapshot::decodePayload(payload);
    expect(decoded.has_value(), "golden DepthSnapshot payload should decode");
    if (decoded.has_value()) {
        expectSameSnapshot(*decoded, expected);
    }
}

void testPayloadValidationAndInvalidFieldZeroRules()
{
    const rb::DepthSnapshot expected = validSnapshot();
    QByteArray payload = rb::DepthSnapshot::encodePayload(expected);
    QString detail;

    expect(!rb::DepthSnapshot::decodePayload(
                payload.left(rb::DepthSnapshot::PayloadSize - 1), &detail)
                .has_value()
               && !detail.isEmpty(),
           "short DepthSnapshot payload must be rejected with detail");

    payload = rb::DepthSnapshot::encodePayload(expected);
    payload[0] = static_cast<char>(0x02);
    expect(!rb::DepthSnapshot::decodePayload(payload).has_value(),
           "unknown DepthSnapshot schema must be rejected");

    payload = rb::DepthSnapshot::encodePayload(expected);
    payload[1] = static_cast<char>(0x04);
    expect(!rb::DepthSnapshot::decodePayload(payload).has_value(),
           "reserved DepthSnapshot flags must be rejected");

    payload = rb::DepthSnapshot::encodePayload(expected);
    payload[1] = static_cast<char>(rb::DepthSnapshot::TemperatureValid);
    payload[2] = 1;
    expect(!rb::DepthSnapshot::decodePayload(payload).has_value(),
           "nonzero depth data for invalid depth must be rejected");

    payload = rb::DepthSnapshot::encodePayload(expected);
    payload[1] = static_cast<char>(rb::DepthSnapshot::DepthValid);
    payload[6] = 1;
    expect(!rb::DepthSnapshot::decodePayload(payload).has_value(),
           "nonzero temperature data for invalid temperature must be rejected");

    rb::DepthSnapshot invalidForEncoding = expected;
    invalidForEncoding.validityFlags = rb::DepthSnapshot::TemperatureValid;
    invalidForEncoding.depthMm = 1;
    const QByteArray scrubbedPayload =
        rb::DepthSnapshot::encodePayload(invalidForEncoding);
    expect(scrubbedPayload.size() == rb::DepthSnapshot::PayloadSize
               && scrubbedPayload.sliced(2, 4) == QByteArray(4, '\0'),
           "encoder must scrub nonzero invalid depth data");

    rb::DepthSnapshot noValidSnapshot;
    noValidSnapshot.sampleAgeMs = 0;
    const QByteArray noValidPayload =
        rb::DepthSnapshot::encodePayload(noValidSnapshot);
    expect(noValidPayload.size() == rb::DepthSnapshot::PayloadSize
               && noValidPayload.sliced(8, 2)
                      == QByteArray::fromHex("ffff"),
           "snapshot with no valid measurements must encode unknown age");

    QByteArray invalidNoValidAge(rb::DepthSnapshot::PayloadSize, '\0');
    invalidNoValidAge[0] = static_cast<char>(rb::DepthSnapshot::SchemaVersion);
    invalidNoValidAge[8] = 0;
    invalidNoValidAge[9] = 0;
    expect(!rb::DepthSnapshot::decodePayload(invalidNoValidAge).has_value(),
           "snapshot with no valid measurements must reject known age");

    rb::DepthSnapshot partial;
    partial.validityFlags = rb::DepthSnapshot::TemperatureValid;
    partial.temperatureCentiC = 2534;
    partial.sampleAgeMs = rb::DepthSnapshot::UnknownSampleAgeMs;
    const QByteArray partialPayload = rb::DepthSnapshot::encodePayload(partial);
    expect(partialPayload.size() == rb::DepthSnapshot::PayloadSize,
           "partially valid DepthSnapshot must retain fixed size");
    expect(partialPayload.sliced(2, 4) == QByteArray(4, '\0'),
           "invalid depth must encode as four zero bytes");
    const std::optional<rb::DepthSnapshot> partialDecoded =
        rb::DepthSnapshot::decodePayload(partialPayload);
    expect(partialDecoded.has_value()
               && !partialDecoded->depthValid()
               && partialDecoded->depthMm == 0
               && partialDecoded->temperatureCentiC == 2534
               && partialDecoded->sampleAgeMs
                      == rb::DepthSnapshot::UnknownSampleAgeMs,
           "partially valid fields and unknown age must decode");

    partial.sampleAgeMs = 1234;
    const QByteArray temperatureOnlyPayload =
        rb::DepthSnapshot::encodePayload(partial);
    expect(temperatureOnlyPayload.sliced(8, 2)
               == QByteArray::fromHex("ffff"),
           "temperature-only data must not create a depth sample age");
}

void testMonitorLifecycleUsesLocalArrivalFreshness()
{
    rb::DepthMonitor monitor;
    rb::DepthSnapshot expected = validSnapshot();
    expected.sampleAgeMs = 0xfffe;
    const QByteArray payload = rb::DepthSnapshot::encodePayload(expected);

    expect(monitor.state().status == rb::DepthStatus::Unknown
               && !monitor.state().snapshot.has_value()
               && monitor.state().lastReceivedAtMs == -1,
           "depth monitor must start Unknown with no live values");

    monitor.handlePacket({rb::MessageType::Heartbeat, 1, {}}, 100);
    expect(monitor.state().status == rb::DepthStatus::Unknown,
           "non-depth packets must not affect the depth monitor");

    monitor.handlePacket({depthMessageType(), 2, payload}, 1000);
    expect(monitor.state().status == rb::DepthStatus::Receiving
               && monitor.state().snapshot.has_value()
               && monitor.state().lastReceivedAtMs == 1000
               && monitor.state().snapshot->sampleAgeMs == 0xfffe,
           "valid depth packet must enter Receiving and retain sample age");

    monitor.tick(4499);
    expect(monitor.state().status == rb::DepthStatus::Receiving,
           "local arrival must remain fresh before the stale threshold");
    monitor.tick(4500);
    expect(monitor.state().status == rb::DepthStatus::Stale
               && !monitor.state().snapshot.has_value(),
           "stale depth values must be cleared after 3500 ms without telemetry");

    monitor.handlePacket({depthMessageType(), 3, payload}, 5000);
    expect(monitor.state().status == rb::DepthStatus::Receiving
               && monitor.state().snapshot.has_value(),
           "valid depth packet must recover from Stale");

    monitor.handlePacket({depthMessageType(), 4,
                          payload.left(rb::DepthSnapshot::PayloadSize - 1)},
                         5100);
    expect(monitor.state().status == rb::DepthStatus::Error
               && !monitor.state().snapshot.has_value()
               && monitor.state().lastReceivedAtMs == -1
               && !monitor.state().error.isEmpty(),
           "malformed depth payload must enter Error and clear values");

    monitor.handlePacket({depthMessageType(), 5, payload}, 5200);
    monitor.handleLivenessLost();
    expect(monitor.state().status == rb::DepthStatus::Unknown
               && !monitor.state().snapshot.has_value()
               && monitor.state().lastReceivedAtMs == -1,
           "liveness loss must clear depth values and return Unknown");

    monitor.handlePacket({depthMessageType(), 6, payload}, 5300);
    monitor.handleTransportState(rb::TransportState::Disconnected);
    expect(monitor.state().status == rb::DepthStatus::Unknown
               && !monitor.state().snapshot.has_value(),
           "disconnect must clear depth values and return Unknown");
}

void testStatusText()
{
    expect(rb::depthStatusText(rb::DepthStatus::Unknown) == QStringLiteral("Unknown"),
           "Unknown depth status text must be stable");
    expect(rb::depthStatusText(rb::DepthStatus::Receiving)
               == QStringLiteral("Receiving"),
           "Receiving depth status text must be stable");
    expect(rb::depthStatusText(rb::DepthStatus::Stale) == QStringLiteral("Stale"),
           "Stale depth status text must be stable");
    expect(rb::depthStatusText(rb::DepthStatus::Error) == QStringLiteral("Error"),
           "Error depth status text must be stable");
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    testPayloadRoundTripAndGoldenBytes();
    testPayloadValidationAndInvalidFieldZeroRules();
    testMonitorLifecycleUsesLocalArrivalFreshness();
    testStatusText();

    if (failures == 0) {
        qInfo("All Qt depth monitor tests passed");
    }
    return failures == 0 ? 0 : 1;
}
