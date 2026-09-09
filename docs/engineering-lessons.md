# RoboBeetle Engineering Lessons

This note records the evidence and boundaries behind the Console PR #7 scheduler adaptation and the PR #8 five-servo semantic bring-up. The scheduler is **[Hardware Verified - Bench]** on the tested desktop setup, while the PR #8 five-servo layout remains pending target hardware regression.

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

The HDKJ S3150D seller values `500/1500/2500 μs` describe electrical/absolute capability metadata. They do not authorize those pulses as current user commands. The PR #8 follow-up keeps Firmware and Qt descriptors aligned at a provisional `1000–2000 μs` PWM-only endpoint-exploration window. This window is **[Pending Hardware Verification]**, not the final mechanically safe endpoint range. `1500 μs` remains a provisional bring-up center candidate, not a true mechanical center or calibrated Neutral; `angle_supported=false` and Set Angle rejection remain unchanged. Only the supplied `1480/1500/1520 μs` direction observation is **[Hardware Verified]**. Full travel, safe endpoints, practical center, and angle mapping remain **[Pending Hardware Verification]** and must be checked unloaded with small steps near resistance.

## Treat FrontAxis center as a bring-up candidate, not calibration

The HDKJ S3150D FrontAxis descriptor separates seller-provided electrical capability (500/1500/2500 μs, 4.8–7.4 V, 0–270° travel, 4 μs dead band) from the provisional 1000–2000 μs bring-up command envelope. It remains SetAngle-disabled, and 1500 μs is only a provisional startup/center candidate. The supplied 1480/1500/1520 direction observation is Hardware Verified; the expanded window and endpoint calibration are not. The next unloaded sequence is `1500,1400,1300,1200,1100,1000`, return to 1500, then `1500,1600,1700,1800,1900,2000`; only measured mechanical behavior can promote a practical center or endpoint. The seller page also conflicts with the product shell/photo on waterproofing, so waterproof capability remains **[Unverified]** and direct immersion is prohibited until reliable IP/sealing evidence is available.

## Cross-swap actuator faults before changing firmware

During single-channel bring-up, a known-good RearLeft actuator rotated on the RearRight A12/PWM channel, and a new same-type replacement actuator also rotated there, while the original RearRight actuator did not. This cross-swap pattern shows that the STM32 timer/GPIO/PWM path is **[Hardware Verified]** and the original actuator/lead is a hardware fault to replace; an isolated actuator failure must not trigger speculative UART, timer, GPIO, or Protocol changes.

## Keep validation levels explicit

Record each result as one of: `Host Test` (desktop unit/regression test), `ARM Build` (target compiler build), `Program Verify` (DAP/ST-LINK image verification), `Hardware Verified` (physical behavior explicitly observed), or `Pending` (not yet evidenced). The PR #8 Depth window expansion is Host-Tested only in this session; its 1000–2000 μs travel and endpoint behavior remain Pending until the unloaded hardware plan is completed. Do not collapse these levels into a single “tested” label.

## Record layout compatibility breaks explicitly

Semantic renaming can also change the physical wiring contract. The historical v0.4 Servo1/PA6 bring-up object was RearLeft; the five-servo layout assigns PA6/ID0 to FrontRight and PD13/TIM4_CH2 to RearLeft. Old and new Console/Firmware binaries must not be mixed with the rewired harness. A compatibility alias can preserve source builds, but it must be deprecated and never used as the current UI or log identity.
