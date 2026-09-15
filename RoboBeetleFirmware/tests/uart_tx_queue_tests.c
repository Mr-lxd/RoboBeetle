#include "uart_tx_queue.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures;

static void expect(bool condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

static void make_frame(
    uint8_t *bytes,
    uint16_t length,
    uint8_t seed)
{
    uint16_t index;

    for (index = 0U; index < length; ++index)
    {
        bytes[index] = (uint8_t)(seed + (uint8_t)index);
    }
}

static void test_stack_buffer_is_copied_before_return(void)
{
    uart_tx_queue_t queue;
    uart_tx_active_view_t view;
    uint8_t wire[5];
    const uint8_t expected[5] = {0xA1U, 0xA2U, 0xA3U, 0xA4U, 0xA5U};

    uart_tx_queue_init(&queue);
    (void)memcpy(wire, expected, sizeof wire);

    expect(
        uart_tx_queue_offer(
            &queue,
            wire,
            (uint16_t)sizeof wire,
            UART_TX_MESSAGE_ACK) == UART_TX_ENQUEUED,
        "ACK frame should be accepted into owned storage");

    (void)memset(wire, 0U, sizeof wire);
    expect(
        uart_tx_queue_begin_next(&queue, &view),
        "accepted frame should be promoted");
    expect(
        view.length == sizeof expected,
        "owned view should preserve frame length");
    expect(
        (view.data != NULL) &&
            (memcmp(view.data, expected, sizeof expected) == 0),
        "owned view should preserve caller bytes after overwrite");
}

static void test_ack_waits_behind_active_telemetry(void)
{
    uart_tx_queue_t queue;
    uart_tx_active_view_t telemetry_view;
    uart_tx_active_view_t ack_view;
    uint8_t telemetry[4];
    uint8_t ack[3];

    make_frame(telemetry, (uint16_t)sizeof telemetry, 0x10U);
    make_frame(ack, (uint16_t)sizeof ack, 0x80U);
    uart_tx_queue_init(&queue);

    expect(
        uart_tx_queue_offer(
            &queue,
            telemetry,
            (uint16_t)sizeof telemetry,
            UART_TX_MESSAGE_IMU) == UART_TX_ENQUEUED,
        "telemetry should be accepted");
    expect(
        uart_tx_queue_begin_next(&queue, &telemetry_view),
        "telemetry should become active");
    expect(
        uart_tx_queue_offer(
            &queue,
            ack,
            (uint16_t)sizeof ack,
            UART_TX_MESSAGE_ACK) == UART_TX_ENQUEUED,
        "ACK should wait in control storage while telemetry is active");
    expect(
        (telemetry_view.data != NULL) &&
            (telemetry_view.data[0] == 0x10U),
        "active telemetry bytes must remain unchanged by ACK enqueue");

    expect(
        uart_tx_queue_complete_active(&queue, telemetry_view.token),
        "active telemetry should complete with its token");
    expect(
        uart_tx_queue_begin_next(&queue, &ack_view),
        "waiting ACK should be promoted after telemetry completion");
    expect(
        ack_view.kind == UART_TX_MESSAGE_ACK,
        "control ACK should be selected after active telemetry");
}

static void test_control_capacity_is_bounded(void)
{
    uart_tx_queue_t queue;
    uart_tx_queue_stats_t stats;
    uint8_t frame[1] = {0x55U};
    uint32_t index;

    uart_tx_queue_init(&queue);
    for (index = 0U; index < UART_TX_CONTROL_QUEUE_CAPACITY; ++index)
    {
        expect(
            uart_tx_queue_offer(
                &queue,
                frame,
                (uint16_t)sizeof frame,
                UART_TX_MESSAGE_ACK) == UART_TX_ENQUEUED,
            "control slot should accept until capacity");
    }

    expect(
        uart_tx_queue_offer(
            &queue,
            frame,
            (uint16_t)sizeof frame,
            UART_TX_MESSAGE_ACK) == UART_TX_CONTROL_FULL,
        "fifth control frame should be rejected without waiting");
    uart_tx_queue_get_stats(&queue, &stats);
    expect(
        stats.control_queue_full_count == 1U,
        "control-full statistic should count the bounded rejection");
    expect(
        uart_tx_queue_pending_count(&queue) == UART_TX_CONTROL_QUEUE_CAPACITY,
        "control-full rejection must not overwrite an existing slot");
}

static void test_telemetry_capacity_and_coalescing_are_explicit(void)
{
    uart_tx_queue_t queue;
    uart_tx_queue_stats_t stats;
    uint8_t frame[2] = {0x11U, 0x22U};

    uart_tx_queue_init(&queue);
    expect(
        uart_tx_queue_offer(
            &queue,
            frame,
            (uint16_t)sizeof frame,
            UART_TX_MESSAGE_LEAK) == UART_TX_ENQUEUED,
        "first Leak frame should be accepted");
    frame[0] = 0x33U;
    expect(
        uart_tx_queue_offer(
            &queue,
            frame,
            (uint16_t)sizeof frame,
            UART_TX_MESSAGE_LEAK) == UART_TX_COALESCED,
        "pending same-kind Leak should coalesce");
    expect(
        uart_tx_queue_offer(
            &queue,
            frame,
            (uint16_t)sizeof frame,
            UART_TX_MESSAGE_IMU) == UART_TX_ENQUEUED,
        "second distinct telemetry kind should use the second slot");
    expect(
        uart_tx_queue_offer(
            &queue,
            frame,
            (uint16_t)sizeof frame,
            UART_TX_MESSAGE_DEPTH) == UART_TX_TELEMETRY_FULL,
        "third distinct telemetry kind should report bounded full");

    uart_tx_queue_get_stats(&queue, &stats);
    expect(
        stats.telemetry_queue_full_count == 1U,
        "telemetry-full statistic should count the bounded rejection");
    expect(
        uart_tx_queue_pending_count(&queue) == UART_TX_TELEMETRY_QUEUE_CAPACITY,
        "telemetry-full rejection must preserve both old slots");
}

static void test_each_telemetry_kind_replaces_only_itself(void)
{
    const uart_tx_message_kind_t kinds[] = {
        UART_TX_MESSAGE_LEAK,
        UART_TX_MESSAGE_IMU,
        UART_TX_MESSAGE_DEPTH
    };
    uint32_t index;

    for (index = 0U; index < (uint32_t)(sizeof kinds / sizeof kinds[0]); ++index)
    {
        uart_tx_queue_t queue;
        uart_tx_active_view_t view;
        uint8_t first[2];
        uint8_t replacement[2];

        make_frame(first, (uint16_t)sizeof first, (uint8_t)(0x10U + index));
        make_frame(
            replacement,
            (uint16_t)sizeof replacement,
            (uint8_t)(0xA0U + index));
        uart_tx_queue_init(&queue);
        expect(
            uart_tx_queue_offer(
                &queue,
                first,
                (uint16_t)sizeof first,
                kinds[index]) == UART_TX_ENQUEUED,
            "telemetry kind should accept its initial value");
        expect(
            uart_tx_queue_offer(
                &queue,
                replacement,
                (uint16_t)sizeof replacement,
                kinds[index]) == UART_TX_COALESCED,
            "telemetry kind should replace its pending value");
        expect(
            uart_tx_queue_begin_next(&queue, &view),
            "coalesced telemetry should remain pending");
        expect(
            (view.data != NULL) &&
                (view.data[0] == replacement[0]),
            "coalescing should expose only the latest same-kind bytes");
    }
}

static void test_control_precedes_telemetry_and_telemetry_round_robin(void)
{
    uart_tx_queue_t queue;
    uart_tx_active_view_t view;
    uint8_t frame[1] = {0x01U};

    uart_tx_queue_init(&queue);
    expect(
        uart_tx_queue_offer(
            &queue,
            frame,
            (uint16_t)sizeof frame,
            UART_TX_MESSAGE_LEAK) == UART_TX_ENQUEUED,
        "Leak should be queued");
    frame[0] = 0x02U;
    expect(
        uart_tx_queue_offer(
            &queue,
            frame,
            (uint16_t)sizeof frame,
            UART_TX_MESSAGE_IMU) == UART_TX_ENQUEUED,
        "IMU should be queued");
    frame[0] = 0x03U;
    expect(
        uart_tx_queue_offer(
            &queue,
            frame,
            (uint16_t)sizeof frame,
            UART_TX_MESSAGE_ACK) == UART_TX_ENQUEUED,
        "ACK should be queued");

    expect(
        uart_tx_queue_begin_next(&queue, &view) &&
            (view.kind == UART_TX_MESSAGE_ACK),
        "control frame should be selected before telemetry");
    expect(
        uart_tx_queue_complete_active(&queue, view.token),
        "ACK should complete");
    expect(
        uart_tx_queue_begin_next(&queue, &view) &&
            (view.kind == UART_TX_MESSAGE_LEAK),
        "first telemetry selection should use the initial cursor");
    expect(
        uart_tx_queue_complete_active(&queue, view.token),
        "Leak should complete");
    frame[0] = 0x04U;
    expect(
        uart_tx_queue_offer(
            &queue,
            frame,
            (uint16_t)sizeof frame,
            UART_TX_MESSAGE_LEAK) == UART_TX_ENQUEUED,
        "a completed telemetry kind should be queueable again");
    expect(
        uart_tx_queue_complete_active(&queue, view.token) == false,
        "a stale token must not complete a later active frame");
    expect(
        uart_tx_queue_begin_next(&queue, &view) &&
            (view.kind == UART_TX_MESSAGE_IMU),
        "telemetry selection should advance round-robin after Leak");
}

static void test_stale_completion_token_cannot_release_next_frame(void)
{
    uart_tx_queue_t queue;
    uart_tx_active_view_t first;
    uart_tx_active_view_t second;
    uint8_t first_bytes[1] = {0x11U};
    uint8_t second_bytes[1] = {0x22U};

    uart_tx_queue_init(&queue);
    (void)uart_tx_queue_offer(
        &queue,
        first_bytes,
        (uint16_t)sizeof first_bytes,
        UART_TX_MESSAGE_ACK);
    expect(
        uart_tx_queue_begin_next(&queue, &first),
        "first frame should become active");
    expect(
        uart_tx_queue_complete_active(&queue, first.token),
        "first frame should complete");
    (void)uart_tx_queue_offer(
        &queue,
        second_bytes,
        (uint16_t)sizeof second_bytes,
        UART_TX_MESSAGE_ACK);
    expect(
        uart_tx_queue_begin_next(&queue, &second),
        "second frame should become active");
    expect(
        !uart_tx_queue_complete_active(&queue, first.token),
        "stale completion token must be rejected");
    expect(
        uart_tx_queue_get_active_view(&queue, &first) &&
            (first.token == second.token) &&
            (first.data[0] == second_bytes[0]),
        "stale completion must not mutate the next active frame");
}

int main(void)
{
    test_stack_buffer_is_copied_before_return();
    test_ack_waits_behind_active_telemetry();
    test_control_capacity_is_bounded();
    test_telemetry_capacity_and_coalescing_are_explicit();
    test_each_telemetry_kind_replaces_only_itself();
    test_control_precedes_telemetry_and_telemetry_round_robin();
    test_stale_completion_token_cannot_release_next_frame();

    if (failures == 0)
    {
        (void)puts("All UART TX queue tests passed");
    }

    return failures == 0 ? 0 : 1;
}
