#include "safety_supervisor.h"

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

static void test_initial_state_and_heartbeat(void)
{
    safety_supervisor_t supervisor;

    safety_supervisor_init(&supervisor);

    expect(!safety_supervisor_is_host_alive(&supervisor),
           "supervisor must start not alive");

    safety_supervisor_on_heartbeat(&supervisor, 1000U);

    expect(safety_supervisor_is_host_alive(&supervisor),
           "heartbeat must mark host alive");
}

static void test_strict_timeout_and_one_shot_event(void)
{
    safety_supervisor_t supervisor;

    safety_supervisor_init(&supervisor);
    safety_supervisor_on_heartbeat(&supervisor, 1000U);

    expect(!safety_supervisor_process(&supervisor, 1499U),
           "499 ms must not report timeout");
    expect(safety_supervisor_is_host_alive(&supervisor),
           "host must remain alive at 499 ms");
    expect(!safety_supervisor_process(&supervisor, 1500U),
           "500 ms must not report timeout");
    expect(safety_supervisor_is_host_alive(&supervisor),
           "host must remain alive at 500 ms");
    expect(safety_supervisor_process(&supervisor, 1501U),
           "501 ms must report timeout");
    expect(!safety_supervisor_is_host_alive(&supervisor),
           "timeout must mark host not alive");
    expect(!safety_supervisor_process(&supervisor, 1502U),
           "same timeout must be reported only once");
}

static void test_heartbeat_refresh_and_recovery(void)
{
    safety_supervisor_t supervisor;

    safety_supervisor_init(&supervisor);
    safety_supervisor_on_heartbeat(&supervisor, 1000U);
    expect(!safety_supervisor_process(&supervisor, 1499U),
           "initial deadline should still be alive");

    safety_supervisor_on_heartbeat(&supervisor, 1400U);
    expect(!safety_supervisor_process(&supervisor, 1900U),
           "second heartbeat must refresh the deadline");
    expect(safety_supervisor_process(&supervisor, 1901U),
           "refreshed deadline must timeout at 501 ms");
    expect(!safety_supervisor_is_host_alive(&supervisor),
           "refreshed timeout must mark host not alive");

    safety_supervisor_on_heartbeat(&supervisor, 2000U);
    expect(safety_supervisor_is_host_alive(&supervisor),
           "new heartbeat must recover host alive state");
    expect(!safety_supervisor_process(&supervisor, 2500U),
           "recovered host must remain alive at 500 ms");
}

static void test_tick_rollover(void)
{
    safety_supervisor_t supervisor;
    const uint32_t heartbeat_ms = UINT32_MAX - 200U;

    safety_supervisor_init(&supervisor);
    safety_supervisor_on_heartbeat(&supervisor, heartbeat_ms);

    expect(!safety_supervisor_process(&supervisor, 100U),
           "rollover delta below timeout must remain alive");
    expect(!safety_supervisor_process(&supervisor, 299U),
           "rollover delta of 500 ms must remain alive");
    expect(safety_supervisor_process(&supervisor, 300U),
           "rollover delta of 501 ms must timeout");
}

int main(void)
{
    test_initial_state_and_heartbeat();
    test_strict_timeout_and_one_shot_event();
    test_heartbeat_refresh_and_recovery();
    test_tick_rollover();

    if (failures == 0)
    {
        (void)puts("All firmware Safety Supervisor tests passed");
    }

    return failures == 0 ? 0 : 1;
}
