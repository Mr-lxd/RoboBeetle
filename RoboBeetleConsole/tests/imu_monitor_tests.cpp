#include "protocol/PacketCodec.h"
#include "robot/ImuMonitor.h"
#include "robot/ImuSnapshot.h"

#include <QByteArray>
#include <QCoreApplication>

#include <array>
#include <optional>
#include <cstdio>

namespace {

int failures = 0;

void expect(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

rb::ImuSnapshot sampleSnapshot()
{
    rb::ImuSnapshot snapshot;
    snapshot.validityFlags = rb::ImuSnapshot::AccValid
        | rb::ImuSnapshot::GyroValid | rb::ImuSnapshot::AngleValid;
    snapshot.accMg = {1234, -2500, 1};
    snapshot.gyroDecidps = {123, -24, 0};
    snapshot.angleCentidegrees = {1234, -235, 0};
    snapshot.diagnostics.rxByteCount = 0x51525354U;
    snapshot.diagnostics.headerCount = 0x01020304U;
    snapshot.diagnostics.validFrameCount = 0x11121314U;
    snapshot.diagnostics.checksumErrorCount = 0x21222324U;
    snapshot.diagnostics.ringOverflowCount = 0x61626364U;
    snapshot.diagnostics.rearmFailureCount = 0x71727374U;
    snapshot.diagnostics.uartErrorCount = 0x81828384U;
    snapshot.diagnostics.magFrameCount = 0x31323334U;
    snapshot.diagnostics.unsupportedFrameCount = 0x41424344U;
    return snapshot;
}

void expectSameSnapshot(const rb::ImuSnapshot &actual,
                        const rb::ImuSnapshot &expected)
{
    expect(actual.validityFlags == expected.validityFlags,
           "validity flags differ after round trip");
    expect(actual.accMg == expected.accMg,
           "Acc fixed-point values differ after round trip");
    expect(actual.gyroDecidps == expected.gyroDecidps,
           "Gyro fixed-point values differ after round trip");
    expect(actual.angleCentidegrees == expected.angleCentidegrees,
           "Angle fixed-point values differ after round trip");
    expect(actual.diagnostics.rxByteCount == expected.diagnostics.rxByteCount
               && actual.diagnostics.headerCount == expected.diagnostics.headerCount
               && actual.diagnostics.validFrameCount == expected.diagnostics.validFrameCount
               && actual.diagnostics.checksumErrorCount == expected.diagnostics.checksumErrorCount
               && actual.diagnostics.ringOverflowCount == expected.diagnostics.ringOverflowCount
               && actual.diagnostics.rearmFailureCount == expected.diagnostics.rearmFailureCount
               && actual.diagnostics.uartErrorCount == expected.diagnostics.uartErrorCount
               && actual.diagnostics.magFrameCount == expected.diagnostics.magFrameCount
               && actual.diagnostics.unsupportedFrameCount == expected.diagnostics.unsupportedFrameCount,
           "diagnostics differ after round trip");
}

void test_payload_round_trip_and_golden_bytes()
{
    const rb::ImuSnapshot expected = sampleSnapshot();
    const QByteArray payload = rb::ImuSnapshot::encodePayload(expected);
    const QByteArray golden = QByteArray::fromHex(
        "0107d2043cf601007b00e8ff0000d20415ff0000"
        "5453525104030201141312112423222164636261"
        "74737271848382813433323144434241");

    expect(payload.size() == rb::ImuSnapshot::PayloadSize,
           "ImuSnapshot must use one fixed 56-byte payload");
    expect(payload == golden,
           "ImuSnapshot little-endian golden payload differs");

    const std::optional<rb::ImuSnapshot> decoded =
        rb::ImuSnapshot::decodePayload(payload);
    expect(decoded.has_value(), "golden ImuSnapshot payload should decode");
    if (decoded.has_value()) {
        expectSameSnapshot(*decoded, expected);
    }

    const rb::Packet packet{rb::MessageType::ImuSnapshot, 0x1234, payload};
    const QByteArray wire = rb::PacketCodec::encodeWire(packet);
    const QByteArray goldenWire = QByteArray::fromHex(
        "0852420221341238080107d2043cf601027b03e8ff0105d20415ff0127"
        "5453525104030201141312112423222164636261747372718483828134"
        "333231444342415c1f00");
    expect(wire == goldenWire,
           "ImuSnapshot Protocol V2 golden wire frame differs");
    const rb::DecodeResult wireDecoded = rb::PacketCodec::decodeWire(
        wire.first(wire.size() - 1));
    expect(wireDecoded.ok() && wireDecoded.packet == packet,
           "ImuSnapshot Protocol V2 wire round trip differs");
}

void test_payload_validation_and_partial_validity()
{
    const rb::ImuSnapshot expected = sampleSnapshot();
    QByteArray payload = rb::ImuSnapshot::encodePayload(expected);
    QString detail;

    payload[0] = static_cast<char>(0x02);
    expect(!rb::ImuSnapshot::decodePayload(payload, &detail).has_value()
               && !detail.isEmpty(),
           "unknown ImuSnapshot schema must be rejected");

    payload = rb::ImuSnapshot::encodePayload(expected);
    payload[1] = static_cast<char>(0x80);
    expect(!rb::ImuSnapshot::decodePayload(payload).has_value(),
           "reserved validity flags must be rejected");

    payload = rb::ImuSnapshot::encodePayload(expected);
    payload[1] = static_cast<char>(rb::ImuSnapshot::GyroValid
                                   | rb::ImuSnapshot::AngleValid);
    payload[2] = 1;
    expect(!rb::ImuSnapshot::decodePayload(payload).has_value(),
           "nonzero fixed-point data for invalid Acc must be rejected");

    payload = rb::ImuSnapshot::encodePayload(expected);
    payload[2] = static_cast<char>(0x81);
    payload[3] = static_cast<char>(0x3e);
    expect(!rb::ImuSnapshot::decodePayload(payload).has_value(),
           "Acc values beyond the documented range must be rejected");

    rb::ImuSnapshot partial = expected;
    partial.validityFlags = rb::ImuSnapshot::GyroValid;
    partial.accMg = {};
    partial.angleCentidegrees = {};
    payload = rb::ImuSnapshot::encodePayload(partial);
    expect(payload.size() == rb::ImuSnapshot::PayloadSize,
           "partial-valid snapshot must retain fixed payload size");
    const std::optional<rb::ImuSnapshot> decoded =
        rb::ImuSnapshot::decodePayload(payload);
    expect(decoded.has_value() && decoded->gyroDecidps == expected.gyroDecidps,
           "partial-valid Gyro domain should remain readable");
}

void test_monitor_status_lifecycle()
{
    rb::ImuMonitor monitor;
    const QByteArray payload = rb::ImuSnapshot::encodePayload(sampleSnapshot());

    expect(monitor.state().status == rb::ImuStatus::Unknown
               && !monitor.state().snapshot.has_value(),
           "IMU monitor must start Unknown with no live values");

    monitor.handlePacket({rb::MessageType::ImuSnapshot, 7, payload}, 100);
    expect(monitor.state().status == rb::ImuStatus::Receiving
               && monitor.state().snapshot.has_value()
               && monitor.state().lastReceivedAtMs == 100,
           "valid IMU packet must enter Receiving");

    monitor.tick(3599);
    expect(monitor.state().status == rb::ImuStatus::Receiving,
           "IMU must not become stale before the threshold");
    monitor.tick(3600);
    expect(monitor.state().status == rb::ImuStatus::Stale
               && !monitor.state().snapshot.has_value(),
           "stale IMU values must be invalidated");

    monitor.handlePacket({rb::MessageType::ImuSnapshot, 8, payload}, 4000);
    expect(monitor.state().status == rb::ImuStatus::Receiving
               && monitor.state().snapshot.has_value(),
           "a valid packet must recover from Stale");

    monitor.handlePacket({rb::MessageType::ImuSnapshot, 9, payload.left(10)}, 4100);
    expect(monitor.state().status == rb::ImuStatus::Error
               && !monitor.state().snapshot.has_value(),
           "malformed IMU payload must enter Error and clear values");

    monitor.handlePacket({rb::MessageType::ImuSnapshot, 10, payload}, 4200);
    monitor.handleLivenessLost();
    expect(monitor.state().status == rb::ImuStatus::Unknown
               && !monitor.state().snapshot.has_value(),
           "liveness loss must clear IMU live values");

    monitor.handlePacket({rb::MessageType::ImuSnapshot, 11, payload}, 4300);
    monitor.handleTransportState(rb::TransportState::Disconnected);
    expect(monitor.state().status == rb::ImuStatus::Unknown
               && !monitor.state().snapshot.has_value(),
           "disconnect must clear IMU live values");
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    test_payload_round_trip_and_golden_bytes();
    test_payload_validation_and_partial_validity();
    test_monitor_status_lifecycle();

    if (failures == 0) {
        qInfo("All Qt JY901S IMU monitor tests passed");
    }
    return failures == 0 ? 0 : 1;
}
