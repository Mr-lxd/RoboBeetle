# RoboBeetle Engineering Lessons

This note records the evidence and boundaries behind the Console PR #7 scheduler adaptation and the PR #8 five-servo semantic bring-up. The scheduler is **[Hardware Verified - Bench]** on the tested desktop setup, while the PR #8 five-servo layout remains pending target hardware regression.

## Keep programming and runtime links separate

The DAP/ST-LINK and OpenOCD path is the programming and debug boundary. It should be validated independently by detecting the target, halting/resetting it, and confirming that the intended image is loaded. The run-time Protocol V2 stream is a separate boundary: a successful OpenOCD session does not prove that the USB serial adapter, APC220 pair, UART pins, or application dispatcher are exchanging bytes.

If OpenOCD reports a target-side Flash algorithm failure, treat that as a programming-path failure even when DAP target detection succeeds. Check target power, reset/boot state, flash protection, adapter speed, and the selected device algorithm; a slower programming fallback can be useful for recovery, but a slow fallback success still does not validate the run-time UART/APC220 path. Record the programming result and the run-time result as separate evidence.

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

When a multi-bit Enable request is accepted, every requested channel must start or the operation must roll back. If one fake/real driver start fails, channels already started by that call are stopped and the enabled mask returns to its pre-call value; partial arm is unsafe and must not be reported as success. Disable and Disable All intentionally remain fail-closed/best-effort stop operations.

## Treat FrontAxis center as a bring-up candidate, not calibration

The HDKJ S3150D FrontAxis channel uses a deliberately narrow 1450–1550 μs PWM clamp and remains SetAngle-disabled. Its 1500 μs value is only a provisional startup/center candidate. First hardware verification must be unloaded with the horn or linkage detached, using `1500 → 1450 → 1500 → 1550 → 1500`; only measured mechanical behavior can promote a neutral calibration.

## Record layout compatibility breaks explicitly

Semantic renaming can also change the physical wiring contract. The historical v0.4 Servo1/PA6 bring-up object was RearLeft; the five-servo layout assigns PA6/ID0 to FrontRight and PD13/TIM4_CH2 to RearLeft. Old and new Console/Firmware binaries must not be mixed with the rewired harness. A compatibility alias can preserve source builds, but it must be deprecated and never used as the current UI or log identity.
