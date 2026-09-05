#include "rb_protocol_v2.h"

#include <string.h>


static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t)data[0]
         | ((uint16_t)data[1] << 8U);
}

static void write_le16(
    uint8_t *data,
    uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

uint16_t rbp2_crc16_ccitt_false(
    const uint8_t *data,
    size_t length)
{
    uint16_t crc = 0xFFFFU;

    for (size_t i = 0; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8U;

        for (uint8_t bit = 0; bit < 8U; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)(
                    (crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }

    return crc;
}


static int cobs_decode(
    const uint8_t *input,
    size_t input_length,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length)
{
    size_t read_index = 0U;
    size_t write_index = 0U;

    while (read_index < input_length)
    {
        uint8_t code = input[read_index++];

        if (code == 0U)
        {
            return 0;
        }

        size_t copy_count =
            (size_t)code - 1U;

        if ((read_index + copy_count) >
            input_length)
        {
            return 0;
        }

        if ((write_index + copy_count) >
            output_capacity)
        {
            return 0;
        }

        for (size_t i = 0U;
             i < copy_count;
             ++i)
        {
            output[write_index++] =
                input[read_index++];
        }

        if ((code != 0xFFU) &&
            (read_index < input_length))
        {
            if (write_index >=
                output_capacity)
            {
                return 0;
            }

            output[write_index++] = 0U;
        }
    }

    *output_length = write_index;

    return 1;
}

static int cobs_encode(
    const uint8_t *input,
    size_t input_length,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length)
{
    if (output_capacity == 0U)
    {
        return 0;
    }

    size_t read_index = 0U;
    size_t write_index = 1U;
    size_t code_index = 0U;

    uint8_t code = 1U;

    while (read_index < input_length)
    {
        if (input[read_index] == 0U)
        {
            if (code_index >= output_capacity)
            {
                return 0;
            }

            output[code_index] = code;

            code = 1U;
            code_index = write_index;

            if (write_index >= output_capacity)
            {
                return 0;
            }

            ++write_index;
            ++read_index;
        }
        else
        {
            if (write_index >= output_capacity)
            {
                return 0;
            }

            output[write_index++] =
                input[read_index++];

            ++code;

            if (code == 0xFFU)
            {
                output[code_index] = code;

                code = 1U;
                code_index = write_index;

                if (write_index >= output_capacity)
                {
                    return 0;
                }

                ++write_index;
            }
        }
    }

    output[code_index] = code;

    *output_length = write_index;

    return 1;
}

rbp2_status_t rbp2_decode_wire(
    const uint8_t *wire,
    size_t wire_length,
    rbp2_frame_t *frame)
{
    uint8_t logical[RBP2_MAX_LOGICAL_SIZE];

    size_t logical_length = 0U;

    if (!cobs_decode(
            wire,
            wire_length,
            logical,
            sizeof(logical),
            &logical_length))
    {
        return RBP2_ERR_COBS;
    }

    if (logical_length <
        (RBP2_HEADER_SIZE + RBP2_CRC_SIZE))
    {
        return RBP2_ERR_TOO_SHORT;
    }

    if ((logical[0] != RBP2_MAGIC0) ||
        (logical[1] != RBP2_MAGIC1))
    {
        return RBP2_ERR_MAGIC;
    }

    if (logical[2] != RBP2_VERSION)
    {
        return RBP2_ERR_VERSION;
    }

    uint16_t payload_length =
        read_le16(&logical[6]);

    if (payload_length > RBP2_MAX_PAYLOAD)
    {
        return RBP2_ERR_LENGTH;
    }

    size_t crc_offset =
        RBP2_HEADER_SIZE +
        payload_length;

    if (logical_length !=
        (crc_offset + RBP2_CRC_SIZE))
    {
        return RBP2_ERR_LENGTH;
    }

    uint16_t received_crc =
        read_le16(&logical[crc_offset]);

    uint16_t calculated_crc =
        rbp2_crc16_ccitt_false(
            logical,
            crc_offset);

    if (received_crc != calculated_crc)
    {
        return RBP2_ERR_CRC;
    }

    frame->type = logical[3];

    frame->sequence =
        read_le16(&logical[4]);

    frame->payload_length =
        payload_length;

    if (payload_length > 0U)
    {
        memcpy(
            frame->payload,
            &logical[RBP2_HEADER_SIZE],
            payload_length);
    }

    return RBP2_OK;
}

size_t rbp2_encode_wire(
    uint8_t type,
    uint16_t sequence,
    const uint8_t *payload,
    uint16_t payload_length,
    uint8_t *wire,
    size_t wire_capacity)
{
    uint8_t logical[RBP2_MAX_LOGICAL_SIZE];

    if (payload_length > RBP2_MAX_PAYLOAD)
    {
        return 0U;
    }

    if (wire_capacity < RBP2_MAX_WIRE_SIZE)
    {
        return 0U;
    }

    logical[0] = RBP2_MAGIC0;
    logical[1] = RBP2_MAGIC1;
    logical[2] = RBP2_VERSION;
    logical[3] = type;

    write_le16(&logical[4], sequence);
    write_le16(&logical[6], payload_length);

    if ((payload_length > 0U) &&
        (payload != NULL))
    {
        memcpy(
            &logical[RBP2_HEADER_SIZE],
            payload,
            payload_length);
    }

    size_t crc_offset =
        RBP2_HEADER_SIZE + payload_length;

    uint16_t crc =
        rbp2_crc16_ccitt_false(
            logical,
            crc_offset);

    write_le16(
        &logical[crc_offset],
        crc);

    size_t logical_length =
        crc_offset + RBP2_CRC_SIZE;

    size_t encoded_length = 0U;

    if (!cobs_encode(
            logical,
            logical_length,
            wire,
            wire_capacity - 1U,
            &encoded_length))
    {
        return 0U;
    }

    wire[encoded_length++] = 0U;

    return encoded_length;
}
