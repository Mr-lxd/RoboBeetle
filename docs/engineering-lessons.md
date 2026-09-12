# RoboBeetle Engineering Lessons

This note records the evidence and boundaries behind the Console PR #7 scheduler adaptation, the PR #8 five-servo semantic bring-up, the PR #9 leak-sensor telemetry bring-up, and the PR #10/PR #11 JY901S bring-up. The current host-link bench path is Qt Console → Windows COM13 → DAP UART/USB serial bridge → STM32 USART1 → Protocol V2. The Apc220HalfDuplex name is retained for a conservative software policy; APC220 is an earlier/legacy transport record and was not used in the recent runs. PR #9's PA11 leak path and the PR #10/PR #11 JY901S physical receive, Acc/Gyro/Angle decode, ImuSnapshot, and Qt monitoring paths are **[Hardware Verified]** in their recorded boundaries. The JY901S re-arm diagnostic follow-up is closed; USART3 UART/checksum physical-link quality, final body-frame mapping, and magnetic/yaw calibration remain **[Pending]**. For this servo-calibration feature, FrontAxis/Depth actuator mechanical and software verification—including ARM Build, Program Verify, and Hardware Verified—remains **[Pending]** even though its measurements are **[Bench Hardware Calibrated]**; separately, ROVMAKER absolute-depth sensor calibration/endpoints remain **[Pending]**. Neither the actuator verification status nor the ROVMAKER calibration/endpoints status is hardware-verified by this feature.

## Keep programming and runtime links separate

The DAP/ST-LINK and OpenOCD path is the programming and debug boundary. It should be validated independently by detecting the target, halting/resetting it, and confirming that the intended image is loaded. The run-time Protocol V2 stream is a separate boundary: a successful OpenOCD session does not prove that the DAP UART/COM13 bridge, UART pins, or application dispatcher are exchanging bytes. The stable bring-up sequence recorded for this project is `SWD clock 100 kHz → SYSRESETREQ → halt → program → verify`; if UART behaves abnormally after flashing, perform a complete power cycle before considering software changes. APC220 remains a separate historical/future transport.

If OpenOCD reports a target-side Flash algorithm failure, treat that as a programming-path failure even when DAP target detection succeeds. Check target power, reset/boot state, flash protection, adapter speed, and the selected device algorithm; a slower programming fallback can be useful for recovery, but a slow fallback success still does not validate the run-time DAP UART/COM13/USART1 path. Record the programming result and the run-time result as separate evidence.

## Host verification cannot replace the ARM target build

Host compiler success does not guarantee target translation-unit portability or hardware verification. Host-side pure-C tests and syntax checks can pass while the STM32CubeIDE `arm-none-eabi-gcc` build fails, and neither host result proves that an image was programmed or exercised. Transitive includes differ between toolchains and can hide a missing direct standard-header dependency; each translation unit must include the standard header that defines the symbols it uses. The STM32CubeIDE target build captured the missing `<stddef.h>` dependency for `NULL` in `servo_calibration.c`, which host checks had not exposed.

Treat this as an implementation and review checklist item: do not rely on
transitive includes for standard-library symbols or types. Every C translation
unit and public/internal header must directly include the defining header: `NULL`
and `size_t` → `<stddef.h>`, `bool` → `<stdbool.h>`, fixed-width integers →
`<stdint.h>`, memory/string APIs → `<string.h>`, stdio → `<stdio.h>`, stdlib →
`<stdlib.h>`, and math APIs → `<math.h>`. Host compiler success does not
guarantee ARM translation-unit portability.

## Do not use HAL enum values as invalid sentinels

Third-party HAL enum/raw constant values are part of the valid domain and must not double as invalid markers. STM32 HAL defines `TIM_CHANNEL_1` as `0x00000000U`; channel validity is therefore represented independently by the STM32 driver mapping result (`channel_valid`), and every write/start/stop operation checks that explicit validity. The FrontRight and RearRight CH1 bindings must remain valid even though their HAL channel value is zero; an unmapped symbolic channel must fail closed.

## Treat APC220 power and logic levels as an explicit legacy interface

An APC220 installation must document its supply rail, UART I/O voltage, common ground, and direction-control assumptions before it is used again. This is legacy/earlier hardware guidance; APC220 is not enabled or currently used in the present DAP UART/COM13 closeout. A module advertised or wired around 5 V power is not automatically proof that its UART pins are 5 V tolerant. Measure the actual rails and idle/high levels, use level translation where the selected module requires it, and keep the MCU's 3.3 V GPIO limits authoritative. Do not use a servo-power rail as an implicit logic reference.

## Isolate faults with segmented loopback tests

When an end-to-end command fails, split the path into reversible checks:

1. PC/USB-UART TX↔RX loopback, including the exact baud and framing.
2. STM32 UART pin loopback or a known-good wired peer, with the Protocol V2 decoder/CRC.
3. Each APC220 UART side checked locally, then an APC220-to-APC220 air/link test.
4. Only after those segments pass, run the complete laptop → APC220 → APC220/receiver → STM32 path.

Inject one fault at a time (wrong baud, disconnected ground, reversed TX/RX, framing/CRC corruption, and link loss) and record which boundary detects it. This prevents treating a DAP/OpenOCD result or a GUI status as proof of a wireless serial path.

## Historical APC220 RTT measurements and scheduler budgets

The normal desktop request/ACK round-trip time was approximately 160–170 ms (approximately 160–173 ms across the recorded observations). That is longer than the original 100 ms heartbeat cadence and leaves little margin under the original 200 ms ACK timeout. With independent heartbeat and command sends, a half-duplex link can therefore accumulate overlapping ACK-requiring traffic or time out a valid exchange during module turnaround. Deliberately injected Retry/Timeout events are expected fault-injection behavior and must not be counted as normal-link timeout statistics.

The Console keeps the original DirectUart 100 ms / 200 ms multi-pending behavior for regression compatibility, while the legacy-named Apc220HalfDuplex profile supplies a 250 ms heartbeat and 250 ms ACK stop-and-wait policy. The recent COM13/DAP/USART1 run exercised this policy as a conservative host-link load limit, not as an APC220 radio test. One ACK-requiring frame is active, heartbeat ticks collapse into one due intent, user commands use a bounded queue, heartbeat due work precedes an ordinary command retry, and retries preserve the original sequence and encoded frame. The latest matching ACK RTT is shown by the protocol monitor so future measurements can replace provisional values with evidence.

## [Historical Reference] PR #7 APC220 desktop-bench hardware verification

The APC220 Half-Duplex Scheduler is **[Hardware Verified - Bench]** only for the earlier tested 440 MHz two-module desktop setup. It is not current DAP UART/COM13/USART1 evidence and does not make APC220 part of the currently enabled path. User regression passed the complete Qt → APC220 → STM32 → ACK → APC220 → Qt path, 60 s idle Heartbeat, Servo1 Enable/ACK, Neutral, 0°/±10°/±45°/±90°, rapid queued Set Angle traffic, Disable/Disable All priority, robot-side disconnect, Heartbeat retry and Firmware watchdog safe-disable, recovery without automatic re-arm, and fresh Enable + matching ACK recovery. CRC errors were 0 during the normal run.

The 250 ms Heartbeat target, 250 ms ACK timeout, and 490 ms Console host-side/local safety admission budget remain **[Provisional]** until lab/poolside distance, antenna-orientation, and outdoor RF characterization are complete. The 490 ms value is a host-side/local admission policy, not a Windows-plus-RF hard-real-time guarantee; RF jitter and host scheduling still require characterization.

## Soft targets are not hard safety deadlines

The 250 ms heartbeat value is a soft target for the conservative host-link policy. A separate 490 ms **Console host-side/local safety admission budget** is anchored when each heartbeat is dispatched, not when its ACK arrives; an ACK confirms liveness and measures RTT but cannot buy another interval. The scheduler admits an ordinary command or retry only when its configured ACK timeout plus one retry-timer polling interval fits before that local boundary. This reserves an explicit 10 ms below the Firmware watchdog boundary of greater than 500 ms. The budget is a local policy for the current DAP UART/COM13/USART1 path, not a Windows-plus-RF hard-real-time guarantee; future APC220 RF jitter requires separate hardware regression. The historical 167–173 ms RTT makes `250 + 170 ≈ 420 ms` a useful illustrative observation, not a current DAP measurement or formal guarantee. Sustained-load and near-timeout tests measure the actual wire gap; widening the Firmware watchdog would only hide a Console scheduler defect.

## Distributed state must converge fail-closed

The Console and Firmware each keep enabled/liveness state. Firmware can clear its enabled bit at the watchdog boundary before a Console's retry budget is exhausted. The Console therefore fail-closes actuator state on the first missed host-link Heartbeat ACK: it clears logical enabled and Disable-pending state, queued actuator commands, and deferred retries while allowing heartbeat retry bookkeeping to continue. Heartbeat recovery only restores transport liveness; it never replays outage-era Enable/PWM/Angle/Neutral work. A new user Enable and matching ACK is required before motion commands are accepted.

## Safety commands need explicit priority

Disable and Disable All are safety actions, not ordinary FIFO work. When a Disable request is accepted, unsent Enable/PWM/Angle/Neutral commands for its affected servo are removed, and the Disable is placed ahead of ordinary retry/queue work after any uncancellable exchange and due heartbeat. This prevents stale motion from executing after the user has requested a stop while preserving the one-flight rule.

## Timing budget and safety boundary

Budget the complete exchange, not just MCU handler time: host serialization, DAP UART/COM13 buffering, STM32 receive/dispatch/ACK transmission, and host scheduling jitter all contribute to the current path. The legacy-named profile's 250 ms ACK timeout is a Console host-link budget; it does not change the Firmware watchdog. The Firmware watchdog remains greater than 500 ms after the last valid heartbeat, and a disconnect/error/reconnect clears Console in-flight work, queued commands, heartbeat intent, and logical enable state. Reconnect requires a fresh heartbeat and an explicit Enable ACK. APC220 air latency/turnaround is outside the current evidence and requires a separate future verification.

## Keep descriptor tables independent at a C/C++ boundary

The five-servo bring-up keeps a pure-C Firmware `servo_descriptor` table and an independent Qt/C++ table. Keep both tables in parity: compare IDs, supported mask, capability, calibration, neutral, software PWM limits, software angle limits, and Console `calibrationPending` state. This avoids coupling HAL headers to Qt and makes descriptor drift a visible test failure. HAL timer/channel constants belong only in `servo_driver_stm32`, which maps abstract selectors to `TIM_HandleTypeDef *` and HAL channels.

## Multi-servo Enable must be transactional

When a multi-bit Enable request is accepted, every newly requested channel must start or the operation must roll back. A requested channel that was already enabled is an idempotent no-op: rewriting its pulse to center would alter physical state even if the logical mask were restored later. If one fake/real driver start fails, only channels newly started by that call are stopped, the enabled mask returns to its pre-call value, and already-running channels receive no write/start/stop event. Partial arm is unsafe and must not be reported as success. Disable and Disable All intentionally remain fail-closed/best-effort stop operations.

## Treat pending Disable as a motion-command barrier

Once Disable is accepted and awaiting ACK, new PWM, Neutral, and Set Angle commands for the affected servo must be rejected at the Controller boundary—not merely disabled in the UI. This prevents host-link writes and command-queue entries from being created behind a safety command, so a later Disable Error or timeout cannot release stale post-disable motion.

## Separate electrical capability from the command exploration window

The pre-2026-09-12 observation of approximately `1100–2500 us` and angle-pending status is historical only; it is superseded by the current contract below. The FrontAxis/Depth actuator is a bench actuator calibration: `1060 us = -90 degrees face down`, `1745 us = 0 degrees vertical paddling`, and `2430 us = +90 degrees face up`, with measured 180 degree sweep. Software PWM limits are `1060–2430 us`, software angle limits are `-90 to +90 degrees`, Neutral is `1745 us`, and Console `calibrationPending=false`. This actuator contract is separate from the ROVMAKER depth sensor and does not establish hydrodynamic optimization, installed trim, autonomous depth-control calibration, magnetic/yaw calibration, or final body-frame calibration. RearRight/RearLeft retain electrical calibration `520/1520/2520 us` and software angle range `±45 degrees`, while `820–2220 us` is only a temporary PWM exploration window; those temporary values are not final `±45 degrees` endpoint calibration and final rear calibration remains Pending.

## Treat FrontAxis/Depth bench calibration as evidence-bounded

The user-provided/bench measurements are **[Bench Hardware Calibrated]** only. The new feature Firmware image remains **[Pending]** for ARM Build, Program Verify, and Hardware Verified until independently built, programmed, and exercised; prior PR or old-image PASS does not transfer. Do not turn this actuator calibration into a hydrodynamic, installed-trim, autonomous-depth-control, magnetic/yaw, or final body-frame claim. Waterproof capability remains **[Unverified]** and direct immersion is prohibited until reliable IP/sealing evidence is available.

## Start sensor bring-up with the smallest digital path

The first sensor phase is a leak module's digital output: power from 3.3 V,
common ground, `D0` to STM32 `PA11`, and `A0` unused. The implementation keeps
the boundary narrow: the existing CubeMX `MX_GPIO_Init()` configures PA11 as
`GPIO_MODE_INPUT` with `GPIO_NOPULL`, a thin HAL reader samples the pin, and a
HAL-independent pure-C mapper stores `UNKNOWN`, `DRY`, or `WET`. The explicit
resource scan against the active `.ioc`, `main.c`, HAL MSP, USART1, TIM3/TIM4,
SWD, and GPIO assignments confirms PA11 is free at the software configuration
level. The initial polarity is HIGH → Dry and LOW → Wet, but the output-stage
type is not
fully confirmed, so `GPIO_NOPULL` is a bring-up assumption that must be checked
on the bench rather than presented as an electrical fact.

The mapper is polled in `app_main` and has no EXTI, debounce, latch, alarm,
Safety action, or Servo action. PR #9 adds a deliberately narrow monitoring
path: after an accepted Heartbeat's normal ACK has finished transmitting,
`app_main` may emit unacknowledged Protocol V2 `LeakStatus` (`0x20`) telemetry
for the first valid sample, a state change, or a 500 ms refresh. It uses an
independent telemetry sequence and does not enter command pending/ACK matching.
The Qt controller displays Unknown/Dry/Wet and returns to Unknown on disconnect,
host-link liveness loss, invalid payload, or a provisional 1500 ms stale interval
(three 500 ms Firmware refresh opportunities). It is monitoring-only and must
not be wired to automatic stop behavior in this phase. Host tests can therefore
establish polarity, wire compatibility, and stale-state handling without
pretending to verify PA11 voltage or water response. The recorded PR #9 evidence
now separates the levels explicitly: pure-C/Qt checks are **[Host Test: PASS]**,
the STM32 target compile is **[ARM Build: PASS]**, the rebuilt current ELF was
programmed and checked as **[Program Verify: PASS]**, and PA11 detection,
Protocol V2 LeakStatus real-link exchange, the Qt indicator, and end-to-end Leak
monitoring are **[Hardware Verified]**. Leak-triggered Safety action remains
**[Pending / Not Implemented]**. The remaining order is physical leak evidence
(now complete) → JY901S IMU (now Hardware Verified in the recorded boundary) →
depth/sensor board; Depth work remains pending.

## Firmware image provenance is part of hardware verification

The earlier persistent `Leak: Unknown` result was caused by build/programming
artifact provenance rather than a new Protocol V2 or sensor-path defect. Rebuild
the correct current source into the matching ELF, program that image, verify the
programmed image, and only then interpret the real-link result. Once that
provenance was corrected, the complete PA11 → Firmware → LeakStatus → Qt path
worked and passed end-to-end hardware acceptance. Do not invent or infer numeric
voltage or response-time values when they were not recorded. Keep **Host Test**,
**ARM Build**, **Program Verify**, **Hardware Verified**, and **Pending** as
separate evidence levels; a host test or target compile alone cannot establish
physical sensor behavior.

## Keep sensor telemetry inside an existing liveness window

For a low-rate sensor, an independent transmit timer would compete with
Protocol V2 command/ACK traffic and create an unbounded source of frames. The
PR #9 policy therefore publishes LeakStatus only in the already controlled
Heartbeat opportunity, after the normal Heartbeat ACK has fully transmitted.
State changes are sent immediately at that opportunity and an unchanged state
is refreshed at most every 500 ms. The frame is unacknowledged and has its own
sequence space, so it cannot satisfy or reorder a command ACK. This keeps the
telemetry path observable without coupling leak indication to Servo or Safety
actions.

## Cross-swap actuator faults before changing firmware

During single-channel bring-up, a known-good RearLeft actuator rotated on the RearRight A12/PWM channel, and a new same-type replacement actuator also rotated there, while the original RearRight actuator did not. This cross-swap pattern shows that the STM32 timer/GPIO/PWM path is **[Hardware Verified]** and the original actuator/lead is a hardware fault to replace; an isolated actuator failure must not trigger speculative UART, timer, GPIO, or Protocol changes.

## Keep validation levels explicit

Record each result as one of: `Host Test` (desktop unit/regression test), `ARM Build` (target compiler build), `Program Verify` (DAP/ST-LINK image verification), `Hardware Verified` (physical behavior explicitly observed), `[Bench Hardware Calibrated]` (user-provided/bench actuator measurement), or `Pending` (not yet evidenced). For the current FrontAxis/Depth feature, the user-provided actuator measurements are **[Bench Hardware Calibrated]**; ARM Build, Program Verify, and Hardware Verified remain **[Pending]** until independently built, programmed, and exercised. The pre-2026-09-12 PR #8 Depth window expansion record is historical: it observed approximately 1100–2500 μs travel and below-1100 μs ACK timeouts, and its angle-pending/provisional-window wording is superseded by the current FrontAxis/Depth contract above. Do not collapse these levels into a single “tested” label.

## Record layout compatibility breaks explicitly

Semantic renaming can also change the physical wiring contract. The historical v0.4 Servo1/PA6 bring-up object was RearLeft; the five-servo layout assigns PA6/ID0 to FrontRight and PD13/TIM4_CH2 to RearLeft. Old and new Console/Firmware binaries must not be mixed with the rewired harness. A compatibility alias can preserve source builds, but it must be deprecated and never used as the current UI or log identity.

## Keep user-facing servo labels plain and semantic

The Qt descriptor keeps `FrontAxis` as the internal compatibility identifier, but its user-facing label is `Depth`; the other panels display `FrontRight`, `FrontLeft`, `RearRight`, and `RearLeft`. Exact ASCII labels avoid mojibake in the servo panel and keep logs/docs aligned with the frozen semantic IDs. Do not “fix” a display issue by renaming the internal identifier or by reintroducing a historical `Servo2` alias.

## JY901S listen-only lessons

- The STM32F4 HAL completes a standard one-byte interrupt receive, restores `RxState` to `HAL_UART_STATE_READY`, and then invokes `HAL_UART_RxCpltCallback`. `FE`/`NE`/`PE` are non-blocking in this path and can leave an active receive; `ORE`/DMA errors block or abort reception. Treat those states according to the HAL contract rather than unconditionally clearing an active receive.
- One-byte UART reception needs a recoverable ownership state: the RX/error callbacks only record the byte or error and mark `needs_rearm`; foreground maintenance retries once per poll. The old counter's `166240` could not identify its return statuses because it collapsed every non-`HAL_OK` result; in the repository's STM32F4 HAL, `HAL_UART_Receive_IT()` returns `HAL_BUSY` whenever `RxState` is not `HAL_UART_STATE_READY`, although the normal one-byte completion path sets `RxState` to `READY` before the completion callback. Therefore do not infer that a normal callback itself proves a completion-time `HAL_BUSY`. `HAL_BUSY` is deferred work and must not inflate the hard re-arm-failure counter; `HAL_ERROR` and other non-success statuses remain hard failures. A generation re-check is required around the foreground success transition so a callback/error event that races with HAL receive startup cannot be cleared. Keep separate deferred, aggregate, and per-flag UART diagnostics observable.
- A legal frame type that is intentionally not decoded is different from an unknown frame type. JY901S `0x54` Mag is known-but-ignored and needs its own counter so default output does not look like an unsupported-protocol fault.
- A 256-byte storage ring has 255 bytes of usable capacity under the empty-slot convention; the observed RX byte rate and overflow counter determine whether that is sufficient for a persistent output mask/rate.
- A single Qt invalid-length observation with CRC and timeout counters still clear is not evidence for a Protocol V2 redesign. Keep split and sticky/concatenated-frame regressions in the gate and investigate only a reproducible decoder failure.
- Host Test, ARM Build, Program Verify, Hardware Verified, and Pending are evidence categories; one cannot be inferred from another.
- A successful target build and DAP/OpenOCD `Verified OK` establish ARM Build and Program Verify for the intended ELF, but they do not establish that a JY901S is physically transmitting valid frames. Record the ELF path, artifact timestamp, programming/verify result, and sensor-runtime evidence separately so image provenance is auditable.

## Low-rate telemetry must fit an existing exchange window

The PR #11 ImuSnapshot uses one fixed 56-byte payload and a separate telemetry
sequence, but it is published only after an accepted Heartbeat ACK completes.
The selector allows at most one optional non-ACK frame per opportunity. A due
LeakStatus always preempts the optional IMU/Depth slots; when LeakStatus is not
due, the scheduler fairly rotates due IMU and Depth. Failed optional transmits
are not marked published, so a failed send remains retryable without changing
the priority rule.
This keeps low-rate monitoring from competing with the conservative host-link
command/ACK slot or creating an independent burst source.
Under the nominal accepted Heartbeat cadence, the effective IMU refresh is up
to approximately 1 Hz; delayed ACK opportunities or pending LeakStatus refreshes
can make it lower. This is a policy ceiling, not a fixed independent timer rate.
The 68-byte maximum IMU wire frame and the combined nominal budget are recorded
as DAP/USART1 host-link estimates; they are not APC220 physical RF throughput
evidence.

## Sensor freshness is separate from telemetry publication freshness

Publishing a sensor packet every second does not make its contained sample
fresh. Firmware therefore uses an explicit provisional 3000 ms depth-sample
freshness bound, separate from the publication interval and the Qt host-packet
stale timeout. At expiry it clears the depth/temperature validity and zeroes the
numeric fields, while keeping transport/parser diagnostics visible; a newly
accepted sensor line restores live validity. This prevents a stopped decoder
from appearing live merely because heartbeat telemetry continues.

## Vendor format evidence must be exact

The ROVMAKER decoder manual documents the canonical
`Depth:XX.XXm Temp:XX.XXC\r\n` line and the exact example
`Depth:1.21m Temp=25.27C`. The parser supports those two complete forms only;
it does not promote an inferred `T=...D=...` compact form into the contract.
The manual's surface-power/air-zero instruction is recorded as vendor guidance,
while installation, electrical levels, density, cadence, and physical response
remain pending verification. The local `ms5837.py` is a direct-I2C sensor-level
reference and does not establish the decoder-board UART boundary.

## Fixed-point schemas are an interoperability boundary

The ImuSnapshot payload documents every offset, width, endianness, scale, range,
rounding rule, validity rule, and schema version. Explicit little-endian fixed
point plus golden vectors avoids ABI/packing and floating-point differences
between pure-C Firmware and Qt/C++. Invalid or stale domains are cleared at the
monitor boundary, so diagnostics can remain visible without presenting old
sensor values as live.

## Telemetry isolation needs regression evidence

The Console controller must treat ImuSnapshot like monitoring telemetry: it must
not satisfy an ACK, release a pending Servo command, alter LeakStatus, or enter
the host-link command queue. A dedicated controller regression and a Qt lifecycle
test make those negative guarantees executable while preserving the existing
USART1 host-link and LeakStatus tests.

## Name the physical transport separately from scheduler policy

The current recent Servo, LeakStatus, and JY901S evidence used
`Qt Console → Windows COM13 → DAP UART/USB serial bridge → STM32 USART1`.
The source-level `Apc220HalfDuplex` name identifies a conservative stop-and-wait
host-link policy; it is not evidence that an APC220 radio was present or enabled.
APC220 remains an earlier/legacy transport record and requires a separate future
hardware verification run.

## Depth sensor bring-up lessons

- Keep decoder-board bring-up listen-only until the physical link and vendor configuration are verified; do not add speculative configuration commands.
- Match parser compatibility to exact vendor-recorded grammars. Canonical and explicitly documented alternate formats get separate tests; guessed separators and substring parsing do not.
- Treat fixed-point payload offsets, schema, validity flags, endianness, and saturation as an interoperability boundary. Encode/decode fields explicitly rather than copying a packed struct.
- Keep `Host Test`, `ARM Build`, `Program Verify`, `Hardware Verified`, and `Pending` separate; a host pass or target compile does not establish physical decoder behavior.
- A parser can receive syntactically valid data while the upstream sensor connection is unreliable. A loose MS5837-to-decoder connector can produce plausible or implausible values, temporary sample loss, and downstream `Stale`; software parser success does not prove connector integrity. Record stable-connection functional verification separately from connector retention, strain relief, wiring inspection, sealing, and post-assembly continuity/stability testing.
