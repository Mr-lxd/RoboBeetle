#include "imu_telemetry_policy.h"
#include "telemetry_scheduler.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static int failures = 0;

static void expect(bool condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

static void test_interval_and_wraparound(void)
{
    imu_telemetry_policy_t policy;

    imu_telemetry_policy_init(&policy);
    expect(imu_telemetry_policy_should_publish(
               &policy, 0U, JY901S_IMU_TELEMETRY_INTERVAL_MS),
           "unpublished IMU telemetry should be due immediately");

    imu_telemetry_policy_mark_published(&policy, 100U);
    expect(!imu_telemetry_policy_should_publish(
               &policy, 1099U, JY901S_IMU_TELEMETRY_INTERVAL_MS),
           "IMU telemetry should not publish before one second");
    expect(imu_telemetry_policy_should_publish(
               &policy, 1100U, JY901S_IMU_TELEMETRY_INTERVAL_MS),
           "IMU telemetry should publish at the one-second boundary");

    imu_telemetry_policy_mark_published(&policy, 0xFFFFFF00U);
    expect(imu_telemetry_policy_should_publish(
               &policy, 0x000002E8U, JY901S_IMU_TELEMETRY_INTERVAL_MS),
           "IMU interval should be safe across tick wraparound");
}

static void test_one_optional_slot_and_leak_priority(void)
{
    expect(telemetry_scheduler_select(false, false) ==
               TELEMETRY_SLOT_NONE,
           "no due telemetry should select no slot");
    expect(telemetry_scheduler_select(true, false) ==
               TELEMETRY_SLOT_LEAK_STATUS,
           "due LeakStatus should select LeakStatus");
    expect(telemetry_scheduler_select(false, true) ==
               TELEMETRY_SLOT_IMU_SNAPSHOT,
           "due IMU should select the IMU slot");
    expect(telemetry_scheduler_select(true, true) ==
               TELEMETRY_SLOT_LEAK_STATUS,
           "LeakStatus must win when both telemetry types are due");
}

static void test_due_imu_survives_leak_opportunity(void)
{
    expect(telemetry_scheduler_select(true, true) ==
               TELEMETRY_SLOT_LEAK_STATUS,
           "the first shared opportunity must send LeakStatus");
    expect(telemetry_scheduler_select(false, true) ==
               TELEMETRY_SLOT_IMU_SNAPSHOT,
           "a still-due IMU must use the next available opportunity");
}

int main(void)
{
    test_interval_and_wraparound();
    test_one_optional_slot_and_leak_priority();
    test_due_imu_survives_leak_opportunity();

    if (failures == 0)
    {
        (void)puts("All telemetry scheduler tests passed");
    }

    return failures == 0 ? 0 : 1;
}
