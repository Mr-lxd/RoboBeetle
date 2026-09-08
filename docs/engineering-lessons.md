# APC220 Bring-up Engineering Lessons

This note records the evidence and boundaries behind the Console PR #7 scheduler adaptation. It is an engineering record, not a claim that the wireless path is fully hardware-validated in every installation.

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

The observed request/ACK round-trip time was approximately 167–173 ms. That is longer than the original 100 ms heartbeat cadence and leaves little margin under the original 200 ms ACK timeout. With independent heartbeat and command sends, a half-duplex link can therefore accumulate overlapping ACK-requiring traffic or time out a valid exchange during module turnaround.

The Console keeps the original DirectUart 100 ms / 200 ms multi-pending behavior for regression compatibility, while the APC220 profile uses 250 ms heartbeat and 250 ms ACK timeout. APC220 scheduling is stop-and-wait: one ACK-requiring frame is active, heartbeat ticks collapse into one due intent, user commands use a bounded queue, heartbeat due work precedes an ordinary command retry, and retries preserve the original sequence and encoded frame. The latest matching ACK RTT is shown by the protocol monitor so future measurements can replace provisional values with evidence.

## Soft targets are not hard safety deadlines

The APC220 250 ms heartbeat value is a soft target for a high-latency link. The hard deadline is anchored when each heartbeat is dispatched, not when its ACK arrives; an ACK confirms liveness and measures RTT but cannot buy another interval. With the observed 167–173 ms RTT, the nominal budget is 250 + 170 ≈ 420 ms against the Firmware watchdog boundary of greater than 500 ms, leaving an approximately 80 ms nominal safety margin. Sustained-load tests must measure the actual wire gap and preserve that margin; widening the Firmware watchdog would only hide a Console scheduler defect.

## Distributed state must converge fail-closed

The Console and Firmware each keep enabled/liveness state. A terminal APC220 heartbeat timeout can therefore invalidate the Console's prior Enable ACK at the same time that Firmware's watchdog clears its own enabled bit. The Console now clears logical enabled and Disable-pending state, queued actuator commands, and deferred retries at that boundary. Heartbeat recovery only restores transport liveness; it never replays outage-era Enable/PWM/Angle/Neutral work. A new user Enable and matching ACK is required before motion commands are accepted.

## Safety commands need explicit priority

Disable and Disable All are safety actions, not ordinary FIFO work. When a Disable request is accepted, unsent Enable/PWM/Angle/Neutral commands for its affected servo are removed, and the Disable is placed ahead of ordinary retry/queue work after any uncancellable exchange and due heartbeat. This prevents stale motion from executing after the user has requested a stop while preserving the one-flight half-duplex rule.

## Timing budget and safety boundary

Budget the complete exchange, not just MCU handler time: host serialization, APC220 buffering, half-duplex direction/turnaround, air/link latency, STM32 receive/dispatch/ACK transmission, and host scheduling jitter all contribute. The APC220 profile's 250 ms ACK timeout is a Console link budget; it does not change the Firmware watchdog. The Firmware watchdog remains greater than 500 ms after the last valid heartbeat, and a disconnect/error/reconnect clears Console in-flight work, queued commands, heartbeat intent, and logical enable state. Reconnect requires a fresh heartbeat and an explicit Enable ACK.
