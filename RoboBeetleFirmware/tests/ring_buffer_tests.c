#include "ring_buffer.h"

#include <stdint.h>
#include <stdio.h>

static int failures = 0;

static void expect(int condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

static void expect_pop(ring_buffer_t *buffer, uint8_t expected)
{
    uint8_t actual = 0U;

    expect(ring_buffer_pop(buffer, &actual), "expected a queued byte");
    expect(actual == expected, "ring-buffer ordering differs");
}

static void test_empty_pop(void)
{
    uint8_t storage[RING_BUFFER_STORAGE_SIZE];
    ring_buffer_t buffer;
    uint8_t byte = 0xA5U;

    ring_buffer_init(
        &buffer,
        storage,
        (uint16_t)sizeof storage);

    expect(!ring_buffer_pop(&buffer, &byte), "empty pop should fail");
    expect(byte == 0xA5U, "empty pop should not modify output");
}

static void test_normal_ordering(void)
{
    uint8_t storage[RING_BUFFER_STORAGE_SIZE];
    ring_buffer_t buffer;

    ring_buffer_init(
        &buffer,
        storage,
        (uint16_t)sizeof storage);

    expect(ring_buffer_push(&buffer, 0x11U), "first push should succeed");
    expect(ring_buffer_push(&buffer, 0x22U), "second push should succeed");
    expect_pop(&buffer, 0x11U);
    expect_pop(&buffer, 0x22U);
    expect(!ring_buffer_pop(&buffer, &(uint8_t){0U}),
           "queue should be empty after normal pops");
}

static void test_wraparound_ordering(void)
{
    uint8_t storage[RING_BUFFER_STORAGE_SIZE];
    ring_buffer_t buffer;

    ring_buffer_init(
        &buffer,
        storage,
        (uint16_t)sizeof storage);

    for (uint8_t value = 0U; value < 100U; ++value)
    {
        expect(ring_buffer_push(&buffer, value), "initial wraparound push failed");
        expect_pop(&buffer, value);
    }

    for (uint8_t value = 0U; value < 127U; ++value)
    {
        expect(ring_buffer_push(&buffer, value), "post-wrap push failed");
    }

    for (uint8_t value = 0U; value < 127U; ++value)
    {
        expect_pop(&buffer, value);
    }
}

static void test_full_capacity_and_drop(
    uint8_t *storage,
    uint16_t storage_size)
{
    ring_buffer_t buffer;

    ring_buffer_init(&buffer, storage, storage_size);

    for (uint16_t value = 0U;
         value < (uint16_t)(storage_size - 1U);
         ++value)
    {
        expect(
            ring_buffer_push(&buffer, (uint8_t)value),
            "effective-capacity push failed");
    }

    expect(!ring_buffer_push(&buffer, 0xEEU),
           "push while full should be rejected");

    for (uint16_t value = 0U;
         value < (uint16_t)(storage_size - 1U);
         ++value)
    {
        expect_pop(&buffer, (uint8_t)value);
    }

    expect(!ring_buffer_pop(&buffer, &(uint8_t){0U}),
           "queue should be empty after full drain");
}

static void test_configured_capacities(void)
{
    uint8_t storage_128[128U];
    uint8_t storage_256[256U];

    test_full_capacity_and_drop(
        storage_128,
        (uint16_t)sizeof storage_128);
    test_full_capacity_and_drop(
        storage_256,
        (uint16_t)sizeof storage_256);
}

int main(void)
{
    test_empty_pop();
    test_normal_ordering();
    test_wraparound_ordering();
    test_configured_capacities();

    if (failures == 0)
    {
        (void)puts("All firmware ring-buffer tests passed");
    }

    return failures == 0 ? 0 : 1;
}
