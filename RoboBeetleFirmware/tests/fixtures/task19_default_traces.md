# Frozen production CPG traces

Captured before Task19 production changes from commit `89037208af6798f2aa360b9d80a077c8cf9b6289`.
Compiler: GCC 16.1.0 (MinGW-W64 x86_64-ucrt-posix-seh, Brecht Sanders r1).
Flags: `-std=c11 -Wall -Wextra -Werror`, no optimization, libm.
Reproducer: `tests/task19_default_trace.c` compiled without TASK19_APPLY_DEFAULT.
The original harness predates all production changes; baseline executable and provenance
are retained in `D:/RoboBeetle-results/task19-cpg-proportional-2026-10-10/baseline`.

Each 10ms sample is explicit little-endian int32 FL/FR/RL/RR/front-axis,
then u8 state/active/target, then little-endian u16 write_mask (25 bytes).
No struct padding or floating-point tolerance is compared.
Sequence: STOPPED20ms; Forward6750ms (750ms start + 6s steady);
TurnLeft/TurnRight/Ascend/Descend/Forward each2000ms; STOP780ms
(750ms completion plus two extra samples); Forward2750ms; STOP780ms.
Each trace has 2108 samples (52700 bytes).

SHA256 Same: `17FF8F3677EADABFF724178996C91B9758606D67A3A705D2C312C4C40F4F3550`.
SHA256 Opposite: `E10D2B580387B239208BEC954A7C856B7693674CA6FFA11BFF18E90EB281D9CA`.
