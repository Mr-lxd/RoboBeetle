#include "rb_protocol_v2.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

_Static_assert(RBP2_RESULT_OK == 0, "ACK result value changed");
_Static_assert(RBP2_RESULT_INVALID_PAYLOAD == 1, "ACK result value changed");
_Static_assert(RBP2_RESULT_HOST_NOT_ALIVE == 2, "ACK result value changed");
_Static_assert(RBP2_RESULT_UNSUPPORTED_SERVO == 3, "ACK result value changed");
_Static_assert(RBP2_RESULT_SERVO_NOT_ENABLED == 4, "ACK result value changed");
_Static_assert(RBP2_RESULT_OUT_OF_RANGE == 5, "ACK result value changed");
_Static_assert(RBP2_RESULT_HARDWARE_FAILURE == 6, "ACK result value changed");

static int failures = 0;

static void expect(int condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

static void expect_golden_vector(
    uint8_t type,
    uint16_t sequence,
    const uint8_t *payload,
    uint16_t payload_length,
    const uint8_t *expected_wire,
    size_t expected_wire_length)
{
    uint8_t wire[RBP2_MAX_WIRE_SIZE];
    rbp2_frame_t decoded;

    size_t wire_length = rbp2_encode_wire(
        type,
        sequence,
        payload,
        payload_length,
        wire,
        sizeof(wire));

    expect(wire_length == expected_wire_length, "golden vector length differs");
    expect(memcmp(wire, expected_wire, expected_wire_length) == 0,
           "golden vector bytes differ");
    expect(rbp2_decode_wire(wire, wire_length - 1U, &decoded) == RBP2_OK,
           "golden vector does not decode");
    expect(decoded.type == type, "decoded message type differs");
    expect(decoded.sequence == sequence, "decoded sequence differs");
    expect(decoded.payload_length == payload_length, "decoded payload length differs");
    expect(memcmp(decoded.payload, payload, payload_length) == 0,
           "decoded payload differs");
}

int main(void)
{
    static const uint8_t heartbeat_payload[] = {0x78U, 0x56U, 0x34U, 0x12U};
    static const uint8_t heartbeat_wire[] = {
        0x06U, 0x52U, 0x42U, 0x02U, 0x01U, 0x01U, 0x02U, 0x04U,
        0x07U, 0x78U, 0x56U, 0x34U, 0x12U, 0x44U, 0x28U, 0x00U};
    static const uint8_t neutral_payload[] = {0x01U, 0x00U};
    static const uint8_t neutral_wire[] = {
        0x06U, 0x52U, 0x42U, 0x02U, 0x14U, 0x05U, 0x02U, 0x02U,
        0x02U, 0x01U, 0x03U, 0xC2U, 0xA4U, 0x00U};
    static const uint8_t angle_min_payload[] = {0x01U, 0x00U, 0xD8U, 0xDCU};
    static const uint8_t angle_min_wire[] = {
        0x06U, 0x52U, 0x42U, 0x02U, 0x13U, 0x06U, 0x02U, 0x04U,
        0x02U, 0x01U, 0x05U, 0xD8U, 0xDCU, 0xF4U, 0x4AU, 0x00U};
    static const uint8_t angle_zero_payload[] = {0x01U, 0x00U, 0x00U, 0x00U};
    static const uint8_t angle_zero_wire[] = {
        0x06U, 0x52U, 0x42U, 0x02U, 0x13U, 0x07U, 0x02U, 0x04U,
        0x02U, 0x01U, 0x01U, 0x01U, 0x03U, 0x58U, 0x9BU, 0x00U};
    static const uint8_t angle_max_payload[] = {0x01U, 0x00U, 0x28U, 0x23U};
    static const uint8_t angle_max_wire[] = {
        0x06U, 0x52U, 0x42U, 0x02U, 0x13U, 0x08U, 0x02U, 0x04U,
        0x02U, 0x01U, 0x05U, 0x28U, 0x23U, 0xD4U, 0xD9U, 0x00U};

    expect(rbp2_crc16_ccitt_false((const uint8_t *)"123456789", 9U) == 0x29B1U,
           "CRC-16/CCITT-FALSE reference value differs");
    expect_golden_vector(RBP2_MSG_HEARTBEAT, 1U, heartbeat_payload,
                         sizeof(heartbeat_payload), heartbeat_wire, sizeof(heartbeat_wire));
    expect_golden_vector(RBP2_MSG_NEUTRAL, 5U, neutral_payload,
                         sizeof(neutral_payload), neutral_wire, sizeof(neutral_wire));
    expect_golden_vector(RBP2_MSG_SET_SERVO_ANGLE, 6U, angle_min_payload,
                         sizeof(angle_min_payload), angle_min_wire, sizeof(angle_min_wire));
    expect_golden_vector(RBP2_MSG_SET_SERVO_ANGLE, 7U, angle_zero_payload,
                         sizeof(angle_zero_payload), angle_zero_wire, sizeof(angle_zero_wire));
    expect_golden_vector(RBP2_MSG_SET_SERVO_ANGLE, 8U, angle_max_payload,
                         sizeof(angle_max_payload), angle_max_wire, sizeof(angle_max_wire));

    if (failures == 0)
    {
        (void)puts("All firmware protocol golden-vector tests passed");
    }

    return failures == 0 ? 0 : 1;
}
