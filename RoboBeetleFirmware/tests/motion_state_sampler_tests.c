#include "motion_state_sampler.h"
#include "uart_tx_queue.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    motion_state_sampler_t sampler;
    rb_motion_state_sample_t sample = {0};
    rb_motion_state_batch_t batch;
    motion_state_sampler_init(&sampler);
    for (unsigned i = 0; i < 64; ++i)
    {
        sample.mcu_ms = i;
        motion_state_sampler_push(&sampler, &sample);
    }
    motion_state_sampler_push(&sampler, &sample);
    assert(sampler.count == 64 && sampler.drop_total == 1);
    motion_state_sampler_begin_batch(&sampler);
    motion_state_sampler_begin_batch(&sampler);
    for (unsigned i = 0; i < 16; ++i)
    {
        assert(motion_state_sampler_fragment(&sampler, 123, &batch));
        assert(batch.batch_seq == 0 && batch.fragment_index == i);
        assert(batch.fragment_count == 16 && batch.sample_count == 2);
        assert(batch.samples[0].mcu_ms == i * 2);
        assert(batch.sampler_drop_total == 1 && batch.mcu_tx_ms == 123);
        assert(sampler.count == 64 - i * 2);
        motion_state_sampler_accept_fragment(&sampler);
    }
    assert(sampler.count == 32 && !sampler.batch_active);
    motion_state_sampler_begin_batch(&sampler);
    assert(motion_state_sampler_fragment(&sampler, 124, &batch));
    assert(batch.batch_seq == 1 && batch.samples[0].mcu_ms == 32);

    uart_tx_queue_t queue;
    uart_tx_active_view_t view;
    const uint8_t first[] = {1}, second[] = {2}, ack[] = {3}, old[] = {4};
    uart_tx_queue_init(&queue);
    assert(uart_tx_queue_offer(&queue, first, 1, UART_TX_MESSAGE_MOTION) == UART_TX_ENQUEUED);
    assert(uart_tx_queue_offer(&queue, second, 1, UART_TX_MESSAGE_MOTION) == UART_TX_MOTION_FULL);
    assert(uart_tx_queue_begin_next(&queue, &view) && view.data[0] == 1);
    assert(uart_tx_queue_offer(&queue, second, 1, UART_TX_MESSAGE_MOTION) == UART_TX_ENQUEUED);
    assert(uart_tx_queue_offer(&queue, ack, 1, UART_TX_MESSAGE_ACK) == UART_TX_ENQUEUED);
    assert(uart_tx_queue_offer(&queue, old, 1, UART_TX_MESSAGE_IMU) == UART_TX_ENQUEUED);
    assert(uart_tx_queue_complete_active(&queue, view.token));
    assert(uart_tx_queue_begin_next(&queue, &view) && view.kind == UART_TX_MESSAGE_ACK);
    assert(uart_tx_queue_complete_active(&queue, view.token));
    assert(uart_tx_queue_begin_next(&queue, &view) && view.kind == UART_TX_MESSAGE_IMU);
    assert(uart_tx_queue_complete_active(&queue, view.token));
    assert(uart_tx_queue_begin_next(&queue, &view) && view.data[0] == 2);
    puts("motion_state_sampler_tests passed");
    return 0;
}
