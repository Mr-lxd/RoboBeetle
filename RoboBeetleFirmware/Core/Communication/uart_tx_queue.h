#ifndef ROBOBEETLE_UART_TX_QUEUE_H
#define ROBOBEETLE_UART_TX_QUEUE_H

#include "rb_protocol_v2.h"

#include <stdbool.h>
#include <stdint.h>

#define UART_TX_CONTROL_QUEUE_CAPACITY 4U
#define UART_TX_TELEMETRY_QUEUE_CAPACITY 2U

typedef enum
{
    UART_TX_MESSAGE_ACK = 0,
    UART_TX_MESSAGE_LEAK,
    UART_TX_MESSAGE_IMU,
    UART_TX_MESSAGE_DEPTH,
    UART_TX_MESSAGE_MOTION,
    UART_TX_MESSAGE_CPG,
    UART_TX_MESSAGE_KIND_COUNT
} uart_tx_message_kind_t;

typedef enum
{
    UART_TX_ENQUEUED = 0,
    UART_TX_COALESCED,
    UART_TX_CONTROL_FULL,
    UART_TX_TELEMETRY_FULL,
    UART_TX_INVALID,
    UART_TX_TRANSPORT_ERROR,
    UART_TX_MOTION_FULL
} uart_tx_enqueue_result_t;

typedef enum
{
    UART_TX_ACTIVE_NONE = 0,
    UART_TX_ACTIVE_STARTING,
    UART_TX_ACTIVE,
    UART_TX_ACTIVE_START_DEFERRED
} uart_tx_active_state_t;

typedef uint32_t uart_tx_active_token_t;

typedef struct
{
    const uint8_t *data;
    uint16_t length;
    uart_tx_message_kind_t kind;
    uart_tx_active_token_t token;
} uart_tx_active_view_t;

typedef struct
{
    uint32_t control_queue_full_count;
    uint32_t telemetry_queue_full_count;
    uint32_t dropped_count[UART_TX_MESSAGE_KIND_COUNT];
} uart_tx_queue_stats_t;

typedef struct
{
    uint16_t length;
    uint8_t bytes[RBP2_MAX_WIRE_SIZE];
    uart_tx_message_kind_t kind;
    uart_tx_active_token_t token;
    bool occupied;
} uart_tx_frame_t;

typedef struct
{
    uart_tx_frame_t control[UART_TX_CONTROL_QUEUE_CAPACITY];
    uart_tx_frame_t telemetry[UART_TX_TELEMETRY_QUEUE_CAPACITY];
    uart_tx_frame_t active;
    uart_tx_frame_t motion;
    uart_tx_frame_t cpg;
    uint8_t control_head;
    uint8_t control_count;
    uint8_t telemetry_count;
    uint8_t telemetry_cursor;
    uart_tx_active_token_t next_token;
    uart_tx_active_state_t active_state;
    uart_tx_queue_stats_t stats;
} uart_tx_queue_t;

void uart_tx_queue_init(uart_tx_queue_t *queue);

uart_tx_enqueue_result_t uart_tx_queue_offer(
    uart_tx_queue_t *queue,
    const uint8_t *data,
    uint16_t length,
    uart_tx_message_kind_t kind);

bool uart_tx_queue_begin_next(
    uart_tx_queue_t *queue,
    uart_tx_active_view_t *view);

bool uart_tx_queue_get_active_view(
    const uart_tx_queue_t *queue,
    uart_tx_active_view_t *view);

bool uart_tx_queue_mark_start_active(
    uart_tx_queue_t *queue,
    uart_tx_active_token_t token);

bool uart_tx_queue_mark_start_deferred(
    uart_tx_queue_t *queue,
    uart_tx_active_token_t token);

bool uart_tx_queue_complete_active(
    uart_tx_queue_t *queue,
    uart_tx_active_token_t token);

bool uart_tx_queue_fail_active(
    uart_tx_queue_t *queue,
    uart_tx_active_token_t token);

void uart_tx_queue_drop_pending(
    uart_tx_queue_t *queue,
    uint32_t dropped_count[UART_TX_MESSAGE_KIND_COUNT]);

uint32_t uart_tx_queue_pending_count(
    const uart_tx_queue_t *queue);

void uart_tx_queue_get_stats(
    const uart_tx_queue_t *queue,
    uart_tx_queue_stats_t *stats);

#endif /* ROBOBEETLE_UART_TX_QUEUE_H */
