#ifndef RB_PROTOCOL_V2_H
#define RB_PROTOCOL_V2_H

#include <stddef.h>
#include <stdint.h>

#define RBP2_MAGIC0              0x52U
#define RBP2_MAGIC1              0x42U
#define RBP2_VERSION             0x02U

#define RBP2_MAX_PAYLOAD         64U
#define RBP2_HEADER_SIZE         8U
#define RBP2_CRC_SIZE            2U

#define RBP2_MAX_LOGICAL_SIZE \
    (RBP2_HEADER_SIZE + RBP2_MAX_PAYLOAD + RBP2_CRC_SIZE)

#define RBP2_MAX_COBS_SIZE \
    (RBP2_MAX_LOGICAL_SIZE + (RBP2_MAX_LOGICAL_SIZE / 254U) + 1U)

#define RBP2_MAX_WIRE_SIZE \
    (RBP2_MAX_COBS_SIZE + 1U)

typedef enum
{
    RBP2_MSG_HEARTBEAT = 0x01,
    RBP2_MSG_ACK       = 0x02,
    RBP2_MSG_ERROR     = 0x03,

    RBP2_MSG_SERVO_ENABLE  = 0x10,
    RBP2_MSG_SERVO_DISABLE = 0x11,
    RBP2_MSG_SET_SERVO_PWM = 0x12,
    RBP2_MSG_SET_SERVO_ANGLE = 0x13,
    RBP2_MSG_NEUTRAL = 0x14,
    RBP2_MSG_SET_MOTION_MODE = 0x15,

    /* Unacknowledged robot-status telemetry (not a command). */
    RBP2_MSG_LEAK_STATUS = 0x20,
    RBP2_MSG_IMU_SNAPSHOT = 0x21,
    RBP2_MSG_DEPTH_SNAPSHOT = 0x22

} rbp2_message_type_t;

typedef enum
{
    RBP2_RESULT_OK = 0,
    RBP2_RESULT_INVALID_PAYLOAD = 1,
    RBP2_RESULT_HOST_NOT_ALIVE = 2,
    RBP2_RESULT_UNSUPPORTED_SERVO = 3,
    RBP2_RESULT_SERVO_NOT_ENABLED = 4,
    RBP2_RESULT_OUT_OF_RANGE = 5,
    RBP2_RESULT_HARDWARE_FAILURE = 6,
    RBP2_RESULT_BUSY = 7

} rbp2_result_t;

typedef enum
{
    RBP2_OK = 0,

    RBP2_ERR_COBS,
    RBP2_ERR_TOO_SHORT,
    RBP2_ERR_MAGIC,
    RBP2_ERR_VERSION,
    RBP2_ERR_LENGTH,
    RBP2_ERR_CRC

} rbp2_status_t;

typedef struct
{
    uint8_t type;

    uint16_t sequence;
    uint16_t payload_length;

    uint8_t payload[RBP2_MAX_PAYLOAD];

} rbp2_frame_t;

uint16_t rbp2_crc16_ccitt_false(
    const uint8_t *data,
    size_t length);

rbp2_status_t rbp2_decode_wire(
    const uint8_t *wire,
    size_t wire_length,
    rbp2_frame_t *frame);

size_t rbp2_encode_wire(
    uint8_t type,
    uint16_t sequence,
    const uint8_t *payload,
    uint16_t payload_length,
    uint8_t *wire,
    size_t wire_capacity);

#endif
