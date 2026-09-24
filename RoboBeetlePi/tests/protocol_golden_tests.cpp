#include "test_support.hpp"

#include "robobeetle/protocol/codec.hpp"
#include "robobeetle/protocol/frame.hpp"
#include "robobeetle/protocol/message_types.hpp"

namespace rbp2_test {
namespace {

using robobeetle::protocol::Codec;
using robobeetle::protocol::Frame;
using robobeetle::protocol::MessageType;

void expect_vector(const Frame &frame, const std::string &wire_hex)
{
    const auto expected = hex(wire_hex);
    const auto wire = Codec::encodeWire(frame);
    expect(wire == expected, "Protocol V2 wire golden vector differs");

    const auto decoded = Codec::decodeWire(without_delimiter(wire));
    expect(decoded.ok(), "Protocol V2 golden vector must decode");
    expect(decoded.frame == frame, "Protocol V2 golden vector round-trip differs");
}

void expect_vector(const Frame &frame, const std::vector<std::uint8_t> &expected)
{
    const auto wire = Codec::encodeWire(frame);
    expect(wire == expected, "Protocol V2 wire golden vector differs");

    const auto decoded = Codec::decodeWire(without_delimiter(wire));
    expect(decoded.ok(), "Protocol V2 golden vector must decode");
    expect(decoded.frame == frame, "Protocol V2 golden vector round-trip differs");
}

void expect_logical(const Frame &frame, const std::vector<std::uint8_t> &expected)
{
    const auto logical = Codec::encodeLogical(frame);
    expect(logical == expected, "Protocol V2 logical golden vector differs");
    const auto decoded = Codec::decodeLogical(logical);
    expect(decoded.ok(), "Protocol V2 logical golden vector must decode");
    expect(decoded.frame == frame, "Protocol V2 logical golden round-trip differs");
}

} // namespace

void test_existing_golden_vectors()
{
    // Exact vectors already present in Firmware/Console tests.
    expect_vector(
        {static_cast<std::uint8_t>(MessageType::Heartbeat), 1U,
         hex("78563412")},
        "06524202010102040778563412442800");
    expect_vector(
        {static_cast<std::uint8_t>(MessageType::ServoEnable), 2U,
         hex("0100")},
        "065242021002020202010345ad00");
    expect_vector(
        {static_cast<std::uint8_t>(MessageType::ServoDisable), 3U,
         hex("0100")},
        "0652420211030202020103845000");
    expect_vector(
        {static_cast<std::uint8_t>(MessageType::SetServoPwm), 4U,
         hex("0200dc05014006")},
        "0652420212040207020208dc0501400621db00");
    expect_vector(
        {static_cast<std::uint8_t>(MessageType::Neutral), 5U,
         hex("0300")},
        "0652420214050202020303a0c200");
    expect_vector(
        {static_cast<std::uint8_t>(MessageType::SetServoAngle), 6U,
         hex("0100d8dc")},
        "0652420213060204020105d8dcf44a00");
    expect_vector(
        {static_cast<std::uint8_t>(MessageType::SetServoAngle), 7U,
         hex("01000000")},
        "06524202130702040201010103589b00");
    expect_vector(
        {static_cast<std::uint8_t>(MessageType::SetServoAngle), 8U,
         hex("01002823")},
        "06524202130802040201052823d4d900");
    expect_vector(
        {static_cast<std::uint8_t>(MessageType::LeakStatus), 9U,
         hex("00")},
        "06524202200902010103e36100");
    expect_vector(
        {static_cast<std::uint8_t>(MessageType::LeakStatus), 10U,
         hex("01")},
        "06524202200a02010401109f00");
    expect_vector(
        {static_cast<std::uint8_t>(MessageType::LeakStatus), 11U,
         hex("02")},
        "06524202200b02010402220500");
}

void test_new_canonical_vectors()
{
    // These vectors are Pi-side canonical cases for types not covered by all
    // existing repository golden-vector tests. Payload layouts come from the
    // current Firmware protocol/telemetry encoders.
    const Frame ack{0x02U, 0x0042U, hex("34121500")};
    expect_logical(ack, hex("524202024200040034121500c2dc"));
    expect_vector(ack, hex("06524202024202040434121503c2dc00"));

    const Frame motion{0x15U, 0x1234U, hex("010101")};
    expect_logical(motion, hex("5242021534120300010101d489"));
    expect_vector(motion, hex("085242021534120306010101d48900"));

    const Frame gait{0x16U, 0x1235U, hex("01")};
    expect_logical(gait, hex("524202163512010001a5a2"));
    expect_vector(gait, hex("08524202163512010401a5a200"));

    const Frame coordination{0x17U, 0x1236U, hex("01")};
    expect(robobeetle::protocol::is_known_message_type(
               static_cast<std::uint8_t>(MessageType::SetFrontRearCoordination)),
           "FrontRear coordination command type 0x17 is known");
    expect(Codec::decodeLogical(Codec::encodeLogical(coordination)).frame ==
               coordination,
           "FrontRear coordination frame preserves its one-byte value");

    std::vector<std::uint8_t> imu_payload(56U, 0U);
    imu_payload[0] = 1U;
    expect(imu_payload.size() == 56U, "IMU canonical payload must be 56 bytes");
    std::vector<std::uint8_t> expected_imu_wire{
        0x06U, 0x52U, 0x42U, 0x02U, 0x21U, 0x01U, 0x02U, 0x38U, 0x02U,
    };
    expected_imu_wire.insert(expected_imu_wire.end(), 55U, 0x01U);
    expected_imu_wire.insert(expected_imu_wire.end(),
                              {0x03U, 0xBFU, 0x11U, 0x00U});
    const Frame imu{0x21U, 1U, imu_payload};
    std::vector<std::uint8_t> expected_imu_logical{
        0x52U, 0x42U, 0x02U, 0x21U, 0x01U, 0x00U, 0x38U, 0x00U, 0x01U,
        0x00U,
    };
    expected_imu_logical.insert(expected_imu_logical.end(), 54U, 0x00U);
    expected_imu_logical.insert(expected_imu_logical.end(), {0xBFU, 0x11U});
    expect_logical(imu, expected_imu_logical);
    expect_vector(imu, expected_imu_wire);

    const auto depth_payload = hex(
        "0100"
        "00000000"
        "0000"
        "ffff"
        "00000000000000000000000000000000000000000000000000000000");
    expect(depth_payload.size() == 38U, "Depth canonical payload must be 38 bytes");
    const Frame depth{0x22U, 2U, depth_payload};
    std::vector<std::uint8_t> expected_depth_logical{
        0x52U, 0x42U, 0x02U, 0x22U, 0x02U, 0x00U, 0x26U, 0x00U,
    };
    expected_depth_logical.insert(expected_depth_logical.end(),
                                  depth_payload.begin(), depth_payload.end());
    expected_depth_logical.insert(expected_depth_logical.end(), {0x71U, 0xD3U});
    expect_logical(depth, expected_depth_logical);
    expect_vector(
        depth,
        "0652420222020226020101010101010103ffff0101010101010101010101010101010101010101010101010101010371d300");
}

} // namespace rbp2_test
