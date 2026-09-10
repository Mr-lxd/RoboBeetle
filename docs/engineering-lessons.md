# RoboBeetle Engineering Lessons

This note records the evidence and boundaries behind the Console PR #7 scheduler adaptation, the PR #8 five-servo semantic bring-up, and the PR #9 leak-sensor telemetry bring-up. The scheduler is **[Hardware Verified - Bench]** on the tested desktop setup. PR #8 retains explicit partial-bench **[Hardware Verified]** findings for the installed servo paths and **[Pending Hardware Verification]** for the remaining Depth endpoints. PR #9's PA11 leak input, Protocol V2 LeakStatus path, Qt indicator, and end-to-end monitoring are now **[Hardware Verified]** after the correct current ELF was rebuilt, programmed, and verified.

## Keep programming and runtime links separate

The DAP/ST-LINK and OpenOCD path is the programming and debug boundary. It should be validated independently by detecting the target, halting/resetting it, and confirming that the intended image is loaded. The run-time Protocol V2 stream is a separate boundary: a successful OpenOCD session does not prove that the USB serial adapter, APC220 pair, UART pins, or application dispatcher are exchanging bytes. The stable bring-up sequence recorded for this project is `SWD clock 100 kHz → SYSRESETREQ → halt → program → verify`; if UART behaves abnormally after flashing, perform a complete power cycle before considering software changes.

If OpenOCD reports a target-side Flash algorithm failure, treat that as a programming-path failure even when DAP target detection succeeds. Check target power, reset/boot state, flash protection, adapter speed, and the selected device algorithm; a slower programming fallback can be useful for recovery, but a slow fallback success still does not validate the run-time UART/APC220 path. Record the programming result and the run-time result as separate evidence.

## Host verification cannot replace the ARM target build

Host-side pure-C tests and syntax checks can pass while the STM32CubeIDE `arm-none-eabi-gcc` build fails. Transitive includes differ between toolchains and can hide a missing direct standard-header dependency; each translation unit must include the standard header that defines the symbols it uses. The STM32CubeIDE target build captured the missing `<stddef.h>` dependency for `NULL` in `servo_calibration.c`, which host checks had not exposed.

## Do not use HAL enum values as invalid sentinels

Third-party HAL enum/raw constant values are part of the valid domain and must not double as invalid markers. STM32 HAL defines `TIM_CHANNEL_1` as `0x00000000U`; channel validity is therefore represented independently by the STM32 driver mapping result (`channel_valid`), and every write/start/stop operation checks that explicit validity. The FrontRight and RearRight CH1 bindings must remain valid even though their HAL channel value is zero; an unmapped symbolic channel must fail closed.

## Treat APC220 power and logic levels as an explicit interface

An APC220 installation must document its supply rail, UART I/O voltage, common ground, and direction-control assumptions. A module advertised or wired around 5 V power is not automatically proof that its UART pins are 5 V tolerant. Measure the actual rails and idle/high levels, use level translation where the selected module requires it, and keep the MCU's 3.3 V GPIO limits authoritative. Do not use a servo-power rail as an implicit logic reference.

## Isolate faults with segmented loopback tests

When an end-to-end command fails, split the path into reversible checks:

1. PC/USB-UART TX↔RX loopback, including the exact baud and framing.
2. STM32 UART pin loopback or a known-good wired peer, with the Protocol V2 decoder/CRC.
3. Each APC220 UART side checked locally, then an APC220-to-APC220 air/link test.
4. Only after those segments pass, run the complete laptop → APC220 → APC220/receiver → STM32 path.

Inject one fault at a time (wrong baud, disconnected ground, reversed TX/RX, framing/CRC corruption, and link loss) and record which boundary detects it. This prevents treating a DAP/OpenOCD result or a GUI status as proof of a wireless serial path.

## RTT measurements must drive scheduler budgets

The normal desktop request/ACK round-trip time was approximately 160–170 ms (approximately 160–173 ms across the recorded observations). That is longer than the original 100 ms heartbeat cadence and leaves little margin under the original 200 ms ACK timeout. With independent heartbeat and command sends, a half-duplex link can therefore accumulate overlapping ACK-requiring traffic or time out a valid exchange during module turnaround. Deliberately injected Retry/Timeout events are expected fault-injection behavior and must not be counted as normal-link timeout statistics.

The Console keeps the original DirectUart 100 ms / 200 ms multi-pending behavior for regression compatibility, while the APC220 profile uses 250 ms heartbeat and 250 ms ACK timeout. APC220 scheduling is stop-and-wait: one ACK-requiring frame is active, heartbeat ticks collapse into one due intent, user commands use a bounded queue, heartbeat due work precedes an ordinary command retry, and retries preserve the original sequence and encoded frame. The latest matching ACK RTT is shown by the protocol monitor so future measurements can replace provisional values with evidence.

## PR #7 desktop-bench hardware verification

The APC220 Half-Duplex Scheduler is **[Hardware Verified - Bench]** for the tested 440 MHz two-module desktop setup. User regression passed the complete Qt → APC220 → STM32 → ACK → APC220 → Qt path, 60 s idle Heartbeat, Servo1 Enable/ACK, Neutral, 0°/±10°/±45°/±90°, rapid queued Set Angle traffic, Disable/Disable All priority, robot-side disconnect, Heartbeat retry and Firmware watchdog safe-disable, recovery without automatic re-arm, and fresh Enable + matching ACK recovery. CRC errors were 0 during the normal run.

The 250 ms Heartbeat target, 250 ms ACK timeout, and 490 ms Console host-side/local safety admission budget remain **[Provisional]** until lab/poolside distance, antenna-orientation, and outdoor RF characterization are complete. The 490 ms value is a host-side/local admission policy, not a Windows-plus-RF hard-real-time guarantee; RF jitter and host scheduling still require characterization.

## Soft targets are not hard safety deadlines

The APC220 250 ms heartbeat value is a soft target for a high-latency link. A separate 490 ms **Console host-side/local safety admission budget** is anchored when each heartbeat is dispatched, not when its ACK arrives; an ACK confirms liveness and measures RTT but cannot buy another interval. The scheduler admits an ordinary command or retry only when its configured ACK timeout plus one retry-timer polling interval fits before that local boundary. This reserves an explicit 10 ms below the Firmware watchdog boundary of greater than 500 ms. The budget is a local admission policy, not a Windows-plus-RF hard-real-time guarantee; RF jitter and host scheduling still require hardware regression. The observed 167–173 ms RTT makes `250 + 170 ≈ 420 ms` a useful illustrative nominal observation, not a formal guarantee. Sustained-load and near-timeout tests measure the actual wire gap; widening the Firmware watchdog would only hide a Console scheduler defect.

## Distributed state must converge fail-closed

The Console and Firmware each keep enabled/liveness state. Firmware can clear its enabled bit at the watchdog boundary before a Console's retry budget is exhausted. The Console therefore fail-closes actuator state on the first missed APC220 Heartbeat ACK: it clears logical enabled and Disable-pending state, queued actuator commands, and deferred retries while allowing heartbeat retry bookkeeping to continue. Heartbeat recovery only restores transport liveness; it never replays outage-era Enable/PWM/Angle/Neutral work. A new user Enable and matching ACK is required before motion commands are accepted.

## Safety commands need explicit priority

Disable and Disable All are safety actions, not ordinary FIFO work. When a Disable request is accepted, unsent Enable/PWM/Angle/Neutral commands for its affected servo are removed, and the Disable is placed ahead of ordinary retry/queue work after any uncancellable exchange and due heartbeat. This prevents stale motion from executing after the user has requested a stop while preserving the one-flight half-duplex rule.

## Timing budget and safety boundary

Budget the complete exchange, not just MCU handler time: host serialization, APC220 buffering, half-duplex direction/turnaround, air/link latency, STM32 receive/dispatch/ACK transmission, and host scheduling jitter all contribute. The APC220 profile's 250 ms ACK timeout is a Console link budget; it does not change the Firmware watchdog. The Firmware watchdog remains greater than 500 ms after the last valid heartbeat, and a disconnect/error/reconnect clears Console in-flight work, queued commands, heartbeat intent, and logical enable state. Reconnect requires a fresh heartbeat and an explicit Enable ACK.

## Keep descriptor tables independent at a C/C++ boundary

The five-servo bring-up keeps a pure-C Firmware `servo_descriptor` table and an independent Qt/C++ table. Both freeze `FrontRight=0`, `FrontLeft=1`, `FrontAxis=2`, `RearRight=3`, `RearLeft=4`, and supported mask `0x001F`, while separate tests compare the capability and calibration contract. This avoids coupling HAL headers to Qt and makes descriptor drift a visible test failure. HAL timer/channel constants belong only in `servo_driver_stm32`, which maps abstract selectors to `TIM_HandleTypeDef *` and HAL channels.

## Multi-servo Enable must be transactional

When a multi-bit Enable request is accepted, every newly requested channel must start or the operation must roll back. A requested channel that was already enabled is an idempotent no-op: rewriting its pulse to center would alter physical state even if the logical mask were restored later. If one fake/real driver start fails, only channels newly started by that call are stopped, the enabled mask returns to its pre-call value, and already-running channels receive no write/start/stop event. Partial arm is unsafe and must not be reported as success. Disable and Disable All intentionally remain fail-closed/best-effort stop operations.

## Treat pending Disable as a motion-command barrier

Once Disable is accepted and awaiting ACK, new PWM, Neutral, and Set Angle commands for the affected servo must be rejected at the Controller boundary—not merely disabled in the UI. This prevents DirectUart writes and APC220 queue entries from being created behind a safety command, so a later Disable Error or timeout cannot release stale post-disable motion.

## Separate electrical capability from the command exploration window

The HDKJ S3150D seller values `500/1500/2500 μs` describe electrical/absolute capability metadata. They do not by themselves establish a final mechanical-safe user command range. The PR #8 follow-up keeps Firmware and Qt descriptors aligned at a provisional `500–2500 μs` PWM-only endpoint-exploration window. This window is **[Pending Hardware Verification]**, not the final mechanically safe endpoint range. The current bench observation was approximately `1100–2500 μs` for approximately 180 degrees of mechanism travel; commands below approximately `1100 μs` tended to cause ACK timeouts, so further probing below approximately `1100 μs` is paused. Final mechanical safe min/max, practical center, and angle mapping remain deferred until the complete mechanical assembly is installed. `1500 μs` remains a provisional bring-up center candidate, not a true mechanical center or calibrated Neutral; `angle_supported=false` and Set Angle rejection remain unchanged. Only the supplied `1480/1500/1520 μs` direction observation is **[Hardware Verified]**. Full travel, safe endpoints, practical center, and angle mapping remain **[Pending Hardware Verification]** and must be checked with the assembled mechanism and explicit safety margin.

## Treat FrontAxis center as a bring-up candidate, not calibration

The HDKJ S3150D FrontAxis descriptor separates seller-provided electrical capability (500/1500/2500 μs, 4.8–7.4 V, 0–270° travel, 4 μs dead band) from the provisional 500–2500 μs bring-up command envelope. It remains SetAngle-disabled, and 1500 μs is only a provisional startup/center candidate. The supplied 1480/1500/1520 direction observation is Hardware Verified; the approximately 1100–2500 μs bench travel observation, expanded window, and endpoint calibration are not. Commands below approximately 1100 μs tended to cause ACK timeouts and should not be probed further for now. Final safe endpoints and practical center require the complete mechanical assembly and explicit margin. The seller page also conflicts with the product shell/photo on waterproofing, so waterproof capability remains **[Unverified]** and direct immersion is prohibited until reliable IP/sealing evidence is available.

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
APC liveness loss, invalid payload, or a provisional 1500 ms stale interval
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
(now complete) → JY901S IMU → depth/sensor board; JY901S and the Depth sensor
work have not started.

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

Record each result as one of: `Host Test` (desktop unit/regression test), `ARM Build` (target compiler build), `Program Verify` (DAP/ST-LINK image verification), `Hardware Verified` (physical behavior explicitly observed), or `Pending` (not yet evidenced). The PR #8 Depth window expansion has a bench observation of approximately 1100–2500 μs travel and below-1100 μs ACK timeouts, but the command window, safe endpoints, practical center, and angle behavior remain Pending until the complete mechanical assembly is checked. Do not collapse these levels into a single “tested” label.

## Record layout compatibility breaks explicitly

Semantic renaming can also change the physical wiring contract. The historical v0.4 Servo1/PA6 bring-up object was RearLeft; the five-servo layout assigns PA6/ID0 to FrontRight and PD13/TIM4_CH2 to RearLeft. Old and new Console/Firmware binaries must not be mixed with the rewired harness. A compatibility alias can preserve source builds, but it must be deprecated and never used as the current UI or log identity.

## Keep user-facing servo labels plain and semantic

The Qt descriptor keeps `FrontAxis` as the internal compatibility identifier, but its user-facing label is `Depth`; the other panels display `FrontRight`, `FrontLeft`, `RearRight`, and `RearLeft`. Exact ASCII labels avoid mojibake in the servo panel and keep logs/docs aligned with the frozen semantic IDs. Do not “fix” a display issue by renaming the internal identifier or by reintroducing a historical `Servo2` alias.

## JY901S listen-only lessons

- One-byte UART reception needs a recoverable ownership state: a failed HAL re-arm cannot be left as a counter-only event. The ISR marks needs-rearm and the foreground retries once per poll, while UART error callbacks record flags and enter the same path.
- A legal frame type that is intentionally not decoded is different from an unknown frame type. JY901S `0x54` Mag is known-but-ignored and needs its own counter so default output does not look like an unsupported-protocol fault.
- A 256-byte storage ring has 255 bytes of usable capacity under the empty-slot convention; the observed RX byte rate and overflow counter determine whether that is sufficient for a persistent output mask/rate.
- Host Test, ARM Build, Program Verify, Hardware Verified, and Pending are evidence categories; one cannot be inferred from another.
- A successful target build and DAP/OpenOCD `Verified OK` establish ARM Build and Program Verify for the intended ELF, but they do not establish that a JY901S is physically transmitting valid frames. Record the ELF path, artifact timestamp, programming/verify result, and sensor-runtime evidence separately so image provenance is auditable.

## Low-rate telemetry must fit an existing exchange window

The PR #11 ImuSnapshot uses one fixed 56-byte payload and a separate telemetry
sequence, but it is published only after an accepted Heartbeat ACK completes.
The selector allows at most one optional non-ACK frame per opportunity. LeakStatus
wins the first shared due opportunity, then a still-due IMU gets the next shared
opportunity after a successful LeakStatus publication; LeakStatus regains
priority after successful IMU publication. Failed optional transmits are not
marked published, so repeated LeakStatus due events cannot starve a pending IMU.
This keeps low-rate monitoring from competing with the APC220 stop-and-wait
command/ACK slot or creating an independent burst source.
The 68-byte maximum IMU wire frame and the combined nominal budget are recorded
as estimates; they are not physical RF throughput evidence.

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
the APC220 command queue. A dedicated controller regression and a Qt lifecycle
test make those negative guarantees executable while preserving the existing
USART1/APC220 and LeakStatus tests.
