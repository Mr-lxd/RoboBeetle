#include "app_main.h"
#include "depth_telemetry.h"
#include "motion_state_codec.h"
#include "rb_protocol_v2.h"
#include "stm32f4xx_hal.h"
#include "uart_transport_stm32.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_MAX_TX_FRAMES 400U

static int failures = 0;
static uint32_t test_tick = 0U;
static uint32_t test_tick_read_count = 0U;
static bool advance_tick_on_second_read = false;
static uint8_t *uart1_receive_destination = NULL;
static uint8_t *uart3_receive_destination = NULL;
static uint8_t *uart6_receive_destination = NULL;
static uint8_t tx_wires[TEST_MAX_TX_FRAMES][RBP2_MAX_WIRE_SIZE];
static uint16_t tx_lengths[TEST_MAX_TX_FRAMES];
static size_t tx_frame_count = 0U;
static unsigned int blocking_tx_call_count = 0U;

static void expect(bool condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

static uint8_t **receive_destination_for(
    UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return NULL;
    }

    if (huart->Instance == USART1)
    {
        return &uart1_receive_destination;
    }

    if (huart->Instance == USART3)
    {
        return &uart3_receive_destination;
    }

    if (huart->Instance == USART6)
    {
        return &uart6_receive_destination;
    }

    return NULL;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size)
{
    uint8_t **destination = receive_destination_for(huart);

    if ((destination == NULL) || (data == NULL) || (size != 1U))
    {
        return HAL_ERROR;
    }

    *destination = data;
    huart->RxState = HAL_UART_STATE_BUSY_RX;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(
    UART_HandleTypeDef *huart,
    const uint8_t *data,
    uint16_t size)
{
    if ((huart == NULL) ||
        (huart->Instance != USART1) ||
        (data == NULL) ||
        (size > RBP2_MAX_WIRE_SIZE) ||
        (tx_frame_count >= TEST_MAX_TX_FRAMES))
    {
        return HAL_ERROR;
    }

    memcpy(tx_wires[tx_frame_count], data, size);
    tx_lengths[tx_frame_count] = size;
    ++tx_frame_count;
    huart->gState = HAL_UART_STATE_BUSY_TX;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit(
    UART_HandleTypeDef *huart,
    const uint8_t *data,
    uint16_t size,
    uint32_t timeout)
{
    (void)huart;
    (void)data;
    (void)size;
    (void)timeout;
    ++blocking_tx_call_count;
    return HAL_ERROR;
}

HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(
    UART_HandleTypeDef *huart)
{
    if (huart != NULL)
    {
        huart->gState = HAL_UART_STATE_READY;
    }
    return HAL_OK;
}

HAL_UART_StateTypeDef HAL_UART_GetState(
    const UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return HAL_UART_STATE_RESET;
    }
    return (HAL_UART_StateTypeDef)(huart->gState | huart->RxState);
}

uint32_t HAL_UART_GetError(
    const UART_HandleTypeDef *huart)
{
    return huart == NULL ? HAL_UART_ERROR_NONE : huart->ErrorCode;
}

uint32_t HAL_GetTick(void)
{
    ++test_tick_read_count;
    if (advance_tick_on_second_read && test_tick_read_count == 2U)
    {
        ++test_tick;
    }
    return test_tick;
}

GPIO_PinState HAL_GPIO_ReadPin(
    GPIO_TypeDef *GPIOx,
    uint16_t GPIO_Pin)
{
    (void)GPIOx;
    (void)GPIO_Pin;
    return GPIO_PIN_SET;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(
    TIM_HandleTypeDef *htim,
    uint32_t Channel)
{
    (void)htim;
    (void)Channel;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(
    TIM_HandleTypeDef *htim,
    uint32_t Channel)
{
    (void)htim;
    (void)Channel;
    return HAL_OK;
}

static void inject_byte(
    UART_HandleTypeDef *huart,
    uint8_t byte,
    void (*on_complete)(UART_HandleTypeDef *))
{
    uint8_t **destination = receive_destination_for(huart);

    expect((destination != NULL) && (*destination != NULL),
           "RX staging pointer is missing");
    if ((destination != NULL) && (*destination != NULL))
    {
        **destination = byte;
        huart->RxState = HAL_UART_STATE_READY;
        on_complete(huart);
    }
}

static void inject_depth_line(
    UART_HandleTypeDef *depth_uart,
    const char *line)
{
    for (size_t index = 0U; line[index] != '\0'; ++index)
    {
        inject_byte(
            depth_uart,
            (uint8_t)line[index],
            depth_transport_stm32_on_rx_complete);
    }
}

static void send_heartbeat(
    UART_HandleTypeDef *host_uart,
    uint16_t sequence)
{
    const uint8_t payload[4] = {0U, 0U, 0U, 0U};
    uint8_t wire[RBP2_MAX_WIRE_SIZE];
    size_t wire_length = rbp2_encode_wire(
        RBP2_MSG_HEARTBEAT,
        sequence,
        payload,
        sizeof payload,
        wire,
        sizeof wire);

    expect(wire_length > 0U, "Heartbeat test frame did not encode");
    for (size_t index = 0U; index < wire_length; ++index)
    {
        inject_byte(
            host_uart,
            wire[index],
            uart_transport_stm32_on_rx_complete);
    }

    app_main_process();
    while (uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_ACTIVE)
    {
        host_uart->gState = HAL_UART_STATE_READY;
        uart_transport_stm32_on_tx_complete(host_uart);
    }
}

static void send_servo_enable(
    UART_HandleTypeDef *host_uart,
    uint16_t sequence)
{
    const uint8_t payload[2] = {1U, 0U};
    uint8_t wire[RBP2_MAX_WIRE_SIZE];
    size_t wire_length = rbp2_encode_wire(
        RBP2_MSG_SERVO_ENABLE,
        sequence,
        payload,
        sizeof payload,
        wire,
        sizeof wire);

    expect(wire_length > 0U, "Servo Enable test frame did not encode");
    for (size_t index = 0U; index < wire_length; ++index)
    {
        inject_byte(
            host_uart,
            wire[index],
            uart_transport_stm32_on_rx_complete);
    }

    app_main_process();
    while (uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_ACTIVE)
    {
        host_uart->gState = HAL_UART_STATE_READY;
        uart_transport_stm32_on_tx_complete(host_uart);
    }
}

static void send_servo_pwm(
    UART_HandleTypeDef *host_uart,
    uint16_t sequence,
    uint16_t pulse_us)
{
    const uint8_t payload[4] = {
        1U,
        0U,
        (uint8_t)(pulse_us & 0xffU),
        (uint8_t)((pulse_us >> 8U) & 0xffU),
    };
    uint8_t wire[RBP2_MAX_WIRE_SIZE];
    size_t wire_length = rbp2_encode_wire(
        RBP2_MSG_SET_SERVO_PWM,
        sequence,
        payload,
        sizeof payload,
        wire,
        sizeof wire);

    expect(wire_length > 0U, "Set PWM test frame did not encode");
    for (size_t index = 0U; index < wire_length; ++index)
    {
        inject_byte(
            host_uart,
            wire[index],
            uart_transport_stm32_on_rx_complete);
    }

    app_main_process();
    while (uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_ACTIVE)
    {
        host_uart->gState = HAL_UART_STATE_READY;
        uart_transport_stm32_on_tx_complete(host_uart);
    }
}

static bool decode_tx_frame(
    size_t index,
    rbp2_frame_t *frame)
{
    if ((index >= tx_frame_count) || (tx_lengths[index] < 2U))
    {
        return false;
    }

    return rbp2_decode_wire(
               tx_wires[index],
               (size_t)tx_lengths[index] - 1U,
               frame) == RBP2_OK;
}

static void expect_tx_type(
    size_t index,
    uint8_t type,
    uint16_t payload_length)
{
    rbp2_frame_t frame;

    expect(decode_tx_frame(index, &frame),
           "application TX frame did not decode");
    if (decode_tx_frame(index, &frame))
    {
        expect(frame.type == type, "application TX type differs");
        expect(frame.payload_length == payload_length,
               "application TX payload length differs");
    }
}

static bool find_last_tx_type(
    uint8_t type,
    rbp2_frame_t *frame)
{
    for (size_t index = tx_frame_count; index > 0U; --index)
    {
        rbp2_frame_t candidate;

        if (decode_tx_frame(index - 1U, &candidate) &&
            (candidate.type == type))
        {
            if (frame != NULL)
            {
                *frame = candidate;
            }
            return true;
        }
    }

    return false;
}

static size_t count_tx_type_from(size_t first, size_t last, uint8_t type)
{
    size_t count = 0U;

    for (size_t index = first; index < last; ++index)
    {
        rbp2_frame_t frame;

        if (decode_tx_frame(index, &frame) && (frame.type == type))
        {
            ++count;
        }
    }

    return count;
}

/*
 * Drive 20 Heartbeats spaced by spacing_ms with a fresh depth line before each
 * one. Every accepted Heartbeat must publish a new depth frame, never send
 * more than two telemetry frames, and always send its ACK first.
 */
static void run_depth_rate_scenario(
    UART_HandleTypeDef *host_uart,
    UART_HandleTypeDef *depth_uart,
    uint32_t start_ms,
    uint32_t spacing_ms,
    uint16_t first_sequence,
    size_t *leak_frames,
    size_t *imu_frames)
{
    const size_t scenario_first_frame = tx_frame_count;
    size_t depth_frames;

    for (uint32_t beat = 0U; beat < 20U; ++beat)
    {
        const size_t before = tx_frame_count;
        rbp2_frame_t frame;

        test_tick = start_ms + (beat * spacing_ms);
        inject_depth_line(depth_uart, "Depth:0.50m Temp:20.00C\r\n");
        app_main_process();
        send_heartbeat(host_uart, (uint16_t)(first_sequence + beat));

        expect(tx_frame_count > before,
               "a Heartbeat must at least produce its ACK");
        expect(tx_frame_count - before <= 4U,
               "a Heartbeat must be followed by at most two telemetry frames");
        if (tx_frame_count > before)
        {
            expect(decode_tx_frame(before, &frame) && (frame.type == RBP2_MSG_ACK),
                   "the ACK must be transmitted before any telemetry");
        }
    }

    depth_frames = count_tx_type_from(
        scenario_first_frame, tx_frame_count, RBP2_MSG_DEPTH_SNAPSHOT);
    expect(depth_frames == 20U,
           "depth must be published once per accepted Heartbeat");
    *leak_frames = count_tx_type_from(
        scenario_first_frame, tx_frame_count, RBP2_MSG_LEAK_STATUS);
    *imu_frames = count_tx_type_from(
        scenario_first_frame, tx_frame_count, RBP2_MSG_IMU_SNAPSHOT);
}

static void test_depth_rate_follows_heartbeats(
    UART_HandleTypeDef *host_uart,
    UART_HandleTypeDef *depth_uart)
{
    size_t leak_frames = 0U;
    size_t imu_frames = 0U;

    /* 250 ms nominal heartbeats: 5 s. Leak refreshes every 500 ms, IMU 1 s. */
    run_depth_rate_scenario(
        host_uart, depth_uart, 20000U, 250U, 100U, &leak_frames, &imu_frames);
    expect(leak_frames >= 9U && leak_frames <= 11U,
           "Leak telemetry must keep about 2 Hz beside 4 Hz depth");
    expect(imu_frames >= 4U && imu_frames <= 6U,
           "IMU telemetry must keep about 1 Hz beside 4 Hz depth");

    /* Early heartbeats (205 ms) must not degrade depth to every other beat. */
    run_depth_rate_scenario(
        host_uart, depth_uart, 40000U, 205U, 200U, &leak_frames, &imu_frames);
    expect(imu_frames >= 3U, "IMU must not starve with early Heartbeats");
}

int main(void)
{
    UART_HandleTypeDef uart1 = {0};
    UART_HandleTypeDef uart3 = {0};
    UART_HandleTypeDef uart6 = {0};
    TIM_HandleTypeDef tim3 = {0};
    TIM_HandleTypeDef tim4 = {0};
    TIM_TypeDef fake_tim3 = {0};
    TIM_TypeDef fake_tim4 = {0};
    rbp2_frame_t depth_frame;
    depth_telemetry_source_t source;
    depth_telemetry_diagnostics_t diagnostics;

    uart1.Instance = USART1;
    uart3.Instance = USART3;
    uart6.Instance = USART6;
    tim3.Instance = &fake_tim3;
    tim4.Instance = &fake_tim4;
    test_tick = 1000U;

    app_main_init(
        &uart1,
        &uart3,
        &uart6,
        &tim3,
        &tim4,
        NULL,
        0U);

    expect(tx_frame_count == 0U,
           "application initialization transmitted unexpectedly");
    expect(blocking_tx_call_count == 0U,
           "application initialization must not use blocking TX");

    inject_depth_line(
        &uart6,
        "Depth:4.32m Temp:18.75C\r\n");
    app_main_process();
    while (uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_ACTIVE)
    {
        uart1.gState = HAL_UART_STATE_READY;
        uart_transport_stm32_on_tx_complete(&uart1);
    }

    send_heartbeat(&uart1, 1U);
    send_heartbeat(&uart1, 2U);
    send_heartbeat(&uart1, 3U);

    /* B': the first Heartbeat publishes the new depth sample and one
     * Leak/IMU frame, always after its ACK; later Heartbeats at the same tick
     * repeat nothing that is not due. */
    expect(tx_frame_count == 9U,
           "three Heartbeats should produce three ACKs, three old telemetry frames, and three empty motion batches");
    expect_tx_type(0U, RBP2_MSG_ACK, 4U);
    expect_tx_type(1U, RBP2_MSG_DEPTH_SNAPSHOT,
                   DEPTH_TELEMETRY_PAYLOAD_LENGTH);
    expect_tx_type(2U, RBP2_MSG_LEAK_STATUS, 1U);
    expect_tx_type(3U, RBP2_MSG_MOTION_STATE_BATCH, 16U);
    expect_tx_type(4U, RBP2_MSG_ACK, 4U);
    expect_tx_type(5U, RBP2_MSG_IMU_SNAPSHOT, 56U);
    expect_tx_type(6U, RBP2_MSG_MOTION_STATE_BATCH, 16U);
    expect_tx_type(7U, RBP2_MSG_ACK, 4U);

    expect_tx_type(8U, RBP2_MSG_MOTION_STATE_BATCH, 16U);

    expect(decode_tx_frame(1U, &depth_frame),
           "DepthSnapshot frame did not decode");
    if (decode_tx_frame(1U, &depth_frame))
    {
        expect(depth_telemetry_decode(
                   depth_frame.payload,
                   depth_frame.payload_length,
                   &source,
                   &diagnostics),
               "DepthSnapshot payload did not decode");
        expect(source.depth_valid, "DepthSnapshot depth is not valid");
        expect(source.temperature_valid,
               "DepthSnapshot temperature is not valid");
        expect(source.depth_mm == 4320, "DepthSnapshot depth differs");
        expect(source.temperature_centi_c == 1875,
               "DepthSnapshot temperature differs");
        expect(source.sample_age_ms == 0U,
               "DepthSnapshot age differs at same test tick");
        expect(diagnostics.rx_byte_count == 25U,
               "DepthSnapshot RX byte diagnostics differ");
        expect(diagnostics.valid_line_count == 1U,
               "DepthSnapshot valid-line diagnostics differ");
        expect(diagnostics.parse_error_count == 0U,
               "DepthSnapshot parse-error diagnostics differ");
        expect(diagnostics.overlong_line_count == 0U,
               "DepthSnapshot overlong diagnostics differ");
        expect(diagnostics.rx_buffer_overflow_count == 0U,
               "DepthSnapshot overflow diagnostics differ");
        expect(diagnostics.hard_rearm_failure_count == 0U,
               "DepthSnapshot re-arm diagnostics differ");
        expect(diagnostics.uart_error_count == 0U,
               "DepthSnapshot UART diagnostics differ");
    }

    test_tick = 4000U;
    send_heartbeat(&uart1, 4U);
    send_heartbeat(&uart1, 5U);
    send_heartbeat(&uart1, 6U);

    expect(find_last_tx_type(RBP2_MSG_DEPTH_SNAPSHOT, &depth_frame),
           "sensor-stop regression did not emit a DepthSnapshot heartbeat");
    if (find_last_tx_type(RBP2_MSG_DEPTH_SNAPSHOT, &depth_frame))
    {
        expect(depth_telemetry_decode(
                   depth_frame.payload,
                   depth_frame.payload_length,
                   &source,
                   &diagnostics),
               "stale DepthSnapshot payload did not decode");
        expect(!source.depth_valid && !source.temperature_valid,
               "stopped sensor must not keep old depth values valid");
        expect(source.depth_mm == 0 && source.temperature_centi_c == 0,
               "stopped sensor must scrub old depth values");
        expect(diagnostics.valid_line_count == 1U,
               "stale DepthSnapshot must retain parser diagnostics");
    }

    test_tick = 5000U;
    inject_depth_line(
        &uart6,
        "Depth:5.67m Temp:19.25C\r\n");
    app_main_process();
    while (uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_ACTIVE)
    {
        uart1.gState = HAL_UART_STATE_READY;
        uart_transport_stm32_on_tx_complete(&uart1);
    }
    send_heartbeat(&uart1, 7U);
    send_heartbeat(&uart1, 8U);
    send_heartbeat(&uart1, 9U);

    expect(find_last_tx_type(RBP2_MSG_DEPTH_SNAPSHOT, &depth_frame),
           "fresh sensor recovery did not emit a DepthSnapshot");
    if (find_last_tx_type(RBP2_MSG_DEPTH_SNAPSHOT, &depth_frame))
    {
        expect(depth_telemetry_decode(
                   depth_frame.payload,
                   depth_frame.payload_length,
                   &source,
                   &diagnostics),
               "recovered DepthSnapshot payload did not decode");
        expect(source.depth_valid && source.temperature_valid,
               "fresh sensor line must restore valid depth values");
        expect(source.depth_mm == 5670,
               "fresh sensor recovery depth differs");
        expect(source.temperature_centi_c == 1925,
               "fresh sensor recovery temperature differs");
    }

    /*
     * A heartbeat may complete after app_main_process() takes its first
     * timestamp. The safety check must use a current timestamp after input
     * dispatch, not the older pre-dispatch sample.
     */
    test_tick_read_count = 0U;
    advance_tick_on_second_read = true;
    send_heartbeat(&uart1, 10U);
    advance_tick_on_second_read = false;
    send_servo_enable(&uart1, 11U);
    expect(find_last_tx_type(RBP2_MSG_ACK, &depth_frame),
           "delayed-heartbeat regression should produce a Servo Enable ACK");
    if (find_last_tx_type(RBP2_MSG_ACK, &depth_frame))
    {
        expect(depth_frame.payload_length == 4U &&
                   depth_frame.payload[3] == RBP2_RESULT_OK,
               "heartbeat received during a process pass must not false-trip SafetySupervisor");
    }

    /* A stale actuator frame must not execute before the same-pass watchdog. */
    fake_tim3.CCR1 = 1234U;
    test_tick = 5602U;
    send_servo_pwm(&uart1, 12U, 1900U);
    expect(fake_tim3.CCR1 == 1234U,
           "a stale actuator frame must be rejected before SafetySupervisor disables outputs");

    test_depth_rate_follows_heartbeats(&uart1, &uart6);

    /* New gyro events retain their own samples; ANGLE alone creates none. */
    const uint8_t imu_types[] = {0x53U, 0x52U, 0x52U, 0x52U};
    for (size_t packet = 0; packet < sizeof imu_types; ++packet)
    {
        uint8_t imu_frame[11] = {0x55U, imu_types[packet], 0U, 0x10U};
        for (size_t byte = 0; byte < 10U; ++byte) imu_frame[10] += imu_frame[byte];
        test_tick += 5U;
        for (size_t byte = 0; byte < sizeof imu_frame; ++byte)
            inject_byte(&uart3, imu_frame[byte], jy901s_transport_stm32_on_rx_complete);
        app_main_process();
    }
    const size_t motion_first = tx_frame_count;
    send_heartbeat(&uart1, 100U);
    app_main_process();
    while (uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_ACTIVE)
    {
        uart1.gState = HAL_UART_STATE_READY;
        uart_transport_stm32_on_tx_complete(&uart1);
    }
    /* schema2 carries one sample per frame; service the third fragment. */
    app_main_process();
    while (uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_ACTIVE)
    {
        uart1.gState = HAL_UART_STATE_READY;
        uart_transport_stm32_on_tx_complete(&uart1);
    }
    unsigned motion_samples = 0, motion_fragments = 0;
    for (size_t i = motion_first; i < tx_frame_count; ++i)
    {
        rbp2_frame_t frame;
        rb_motion_state_batch_t batch;
        if (decode_tx_frame(i, &frame) && frame.type == RBP2_MSG_MOTION_STATE_BATCH)
        {
            expect(rb_motion_state_decode(frame.payload, frame.payload_length, &batch),
                   "motion fragment must decode");
            expect(batch.schema == 2U && batch.sample_count == 1U && batch.fragment_count == 3U &&
                       batch.fragment_index == motion_fragments,
                   "three gyros must form three schema2 ordered fragments");
            for (unsigned j = 0; j < batch.sample_count; ++j)
            {
                expect(batch.samples[j].gyro_valid && batch.samples[j].angle_valid,
                       "gyro samples must carry the preceding angle");
                expect(batch.samples[j].angle_age_ms == 5U * (motion_samples + 1U),
                       "angle age must follow real event timestamps");
                ++motion_samples;
            }
            ++motion_fragments;
        }
    }
    expect(motion_samples == 3U && motion_fragments == 3U,
           "only the three gyro events may generate motion samples");

    if (failures == 0)
    {
        (void)puts("All app_main DepthSnapshot integration tests passed");
    }

    return failures == 0 ? 0 : 1;
}
