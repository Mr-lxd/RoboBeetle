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
    telemetry_scheduler_t scheduler;

    telemetry_scheduler_init(&scheduler);
    expect(telemetry_scheduler_select(&scheduler, false, false, false) ==
               TELEMETRY_SLOT_NONE,
           "no due telemetry should select no slot");
    expect(telemetry_scheduler_select(&scheduler, true, false, false) ==
               TELEMETRY_SLOT_LEAK_STATUS,
           "due LeakStatus should select LeakStatus");
    expect(telemetry_scheduler_select(&scheduler, false, true, false) ==
               TELEMETRY_SLOT_IMU_SNAPSHOT,
           "due IMU should select the IMU slot");
    expect(telemetry_scheduler_select(&scheduler, true, true, false) ==
               TELEMETRY_SLOT_LEAK_STATUS,
           "LeakStatus must win when both telemetry types are due");
}

static void test_due_imu_survives_leak_opportunity(void)
{
    telemetry_scheduler_t scheduler;

    telemetry_scheduler_init(&scheduler);
    expect(telemetry_scheduler_select(&scheduler, true, true, false) ==
               TELEMETRY_SLOT_LEAK_STATUS,
           "the first shared opportunity must send LeakStatus");
    telemetry_scheduler_mark_success(
        &scheduler,
        TELEMETRY_SLOT_LEAK_STATUS);
    expect(telemetry_scheduler_select(&scheduler, false, true, false) ==
               TELEMETRY_SLOT_IMU_SNAPSHOT,
           "a still-due IMU must use the next available opportunity");
}

static void test_dry_to_wet_leak_change_preempts_periodic_slots(void)
{
    telemetry_scheduler_t scheduler;

    telemetry_scheduler_init(&scheduler);
    expect(telemetry_scheduler_select(&scheduler, false, true, true) ==
               TELEMETRY_SLOT_IMU_SNAPSHOT,
           "periodic IMU should publish while LeakStatus is not due");
    telemetry_scheduler_mark_success(
        &scheduler,
        TELEMETRY_SLOT_IMU_SNAPSHOT);

    /* A DRY-to-WET transition is reported to the scheduler as leak_due. */
    expect(telemetry_scheduler_select(&scheduler, true, true, true) ==
               TELEMETRY_SLOT_LEAK_STATUS,
           "a DRY-to-WET LeakStatus change must preempt both due periodic slots");
    telemetry_scheduler_mark_success(
        &scheduler,
        TELEMETRY_SLOT_LEAK_STATUS);
    expect(telemetry_scheduler_select(&scheduler, true, true, true) ==
               TELEMETRY_SLOT_LEAK_STATUS,
           "an ordinary LeakStatus refresh must preempt both due periodic slots");
}

static void test_failed_leak_send_remains_pending(void)
{
    telemetry_scheduler_t scheduler;

    telemetry_scheduler_init(&scheduler);
    telemetry_scheduler_mark_success(
        &scheduler,
        TELEMETRY_SLOT_IMU_SNAPSHOT);
    expect(telemetry_scheduler_select(&scheduler, true, true, true) ==
               TELEMETRY_SLOT_LEAK_STATUS,
           "a due LeakStatus should be selected before its send attempt");
    /* No mark_success call models a failed optional send. */
    expect(telemetry_scheduler_select(&scheduler, true, true, true) ==
               TELEMETRY_SLOT_LEAK_STATUS,
           "a failed LeakStatus send must leave LeakStatus pending");
    expect(telemetry_scheduler_select(&scheduler, false, true, true) ==
               TELEMETRY_SLOT_DEPTH_SNAPSHOT,
           "a failed LeakStatus send must not advance past the last successful IMU");
}

static void test_all_slots_initially_due(void)
{
    telemetry_scheduler_t scheduler;

    telemetry_scheduler_init(&scheduler);
    expect(telemetry_scheduler_select(&scheduler, true, true, true) ==
               TELEMETRY_SLOT_LEAK_STATUS,
           "LeakStatus must win when all optional telemetry is initially due");
}

static void test_periodic_slots_are_fair_without_leak_due(void)
{
    telemetry_scheduler_t scheduler;

    telemetry_scheduler_init(&scheduler);

    expect(telemetry_scheduler_select(&scheduler, false, true, true) ==
               TELEMETRY_SLOT_IMU_SNAPSHOT,
           "IMU must get the first periodic opportunity when LeakStatus is not due");
    telemetry_scheduler_mark_success(
        &scheduler,
        TELEMETRY_SLOT_IMU_SNAPSHOT);

    expect(telemetry_scheduler_select(&scheduler, false, true, true) ==
               TELEMETRY_SLOT_DEPTH_SNAPSHOT,
           "Depth must get the next periodic opportunity after IMU");
    telemetry_scheduler_mark_success(
        &scheduler,
        TELEMETRY_SLOT_DEPTH_SNAPSHOT);

    expect(telemetry_scheduler_select(&scheduler, false, true, true) ==
               TELEMETRY_SLOT_IMU_SNAPSHOT,
           "IMU must regain the periodic opportunity after Depth");
    telemetry_scheduler_mark_success(
        &scheduler,
        TELEMETRY_SLOT_IMU_SNAPSHOT);

    expect(telemetry_scheduler_select(&scheduler, false, true, true) ==
               TELEMETRY_SLOT_DEPTH_SNAPSHOT,
           "Depth must not starve while both periodic slots remain due");

    expect(telemetry_scheduler_select(&scheduler, false, false, false) ==
               TELEMETRY_SLOT_NONE,
           "one Heartbeat opportunity must select at most one telemetry slot");
}

int main(void)
{
    test_interval_and_wraparound();
    test_one_optional_slot_and_leak_priority();
    test_due_imu_survives_leak_opportunity();
    test_dry_to_wet_leak_change_preempts_periodic_slots();
    test_failed_leak_send_remains_pending();
    test_all_slots_initially_due();
    test_periodic_slots_are_fair_without_leak_due();

    if (failures == 0)
    {
        (void)puts("All telemetry scheduler tests passed");
    }

    return failures == 0 ? 0 : 1;
}
