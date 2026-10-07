#include "jy901s_oneshot_config.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    static const uint8_t expected[20] = {
        0xFF, 0xAA, 0x69, 0x88, 0xB5,
        0xFF, 0xAA, 0x02, 0x0E, 0x00,
        0xFF, 0xAA, 0x03, 0x07, 0x00,
        0xFF, 0xAA, 0x00, 0x00, 0x00
    };

    if (sizeof(jy901s_oneshot_config_commands) != sizeof(expected))
    {
        (void)fprintf(stderr, "FAIL: one-shot commands must contain 20 bytes\n");
        return 1;
    }
    if (memcmp(jy901s_oneshot_config_commands, expected, sizeof(expected)) != 0)
    {
        (void)fprintf(stderr, "FAIL: KEY/RSW/RRATE/SAVE byte sequence differs\n");
        return 1;
    }

    (void)puts("All JY901S one-shot configuration tests passed");
    return 0;
}
