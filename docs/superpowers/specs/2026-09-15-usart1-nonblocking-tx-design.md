# USART1 Non-blocking TX / Motion Cadence Stabilization

Status: design only. This branch contains no implementation. The design is
based on `70690af40b760bbc945b993643892eb941ba9573`, the merged `main` after
PR #18.

## Decision summary

V1 replaces the blocking USART1 transmit call with a fixed-size,
transport-owned queue and `HAL_UART_Transmit_IT()`:

```text
Protocol/App
  -> encode a complete RBP2 frame in the existing stack buffer
  -> copy the complete frame into an owned bounded slot
  -> return immediately

USART1 TX engine
  -> give one owned frame to HAL_UART_Transmit_IT()
  -> wait for HAL_UART_TxCpltCallback()
  -> release that frame and start the next owned frame
```

The queue API is independent of the HAL start mechanism. DMA is deliberately
not selected for v1. A later DMA adapter can replace the one-frame
start/completion adapter without rewriting Protocol V2 or App.

The design is generic with respect to physical link and baud rate. It does
not assume APC220 RF behavior. Non-blocking TX remains architecturally useful
for a future Raspberry Pi link at 115200 or higher because foreground code
still does not wait for physical serialization.

## Evidence and frozen scope

The merged investigation recorded:

```text
NORMAL CPG:                worst Motion gap about 96 ms; >30 ms = 39
NORMAL SimpleGait:         worst Motion gap about 97 ms; >30 ms = 43
REDUCED telemetry / CPG:   worst Motion gap about 26 ms; >30 ms = 0
```

Normal worst-gap context contained approximately ACK TX 16.8 ms and IMU TX
70.9 ms. The approved interpretation is that optional blocking USART1
telemetry TX is a confirmed major contributor and blocking ACK TX is a
confirmed residual contributor. CPG compute and gait backend are not the
primary cause in that evidence.

The delayed-call finding remains unchanged and remains a hypothesis, not a
proven root cause:

```text
delayed foreground call with elapsed_ms = 30
  -> generator state advances for 30 ms
  -> only one actuator-facing target application occurs
```

This feature does not change `MOTION_GAIT_TICK_MS`, `elapsed_ms`, catch-up
behavior, foreground order, RX drain policy, CPG/SimpleGait mathematics,
Servo target generation, PWM, calibration, Clock/RCC, baud, heartbeat/safety
values, or any stutter/jitter behavior. It does not implement a stutter fix.

Post-fix target measurements are later work and must use identical ELFs in
each pair:

```text
NORMAL:  runtime CPG = A, runtime SimpleGait = B
REDUCED: runtime CPG = C, runtime SimpleGait = D
```

The runtime selector remains STOPPED-only. Physical PWM, mechanical behavior,
and Water remain separate evidence domains.

## 1. Current data flow and ownership

The actual current path is:

```text
USART1 IRQ
  -> HAL_UART_IRQHandler(&huart1)
  -> HAL_UART_RxCpltCallback()
  -> uart_transport_stm32_on_rx_complete()
  -> fixed RX ring

app_main_process()
  -> host RX ring drain / protocol_feed_byte()
  -> protocol dispatcher
  -> protocol_send_ack()                 [stack payload + stack wire]
  -> uart_transport_stm32_transmit()
  -> HAL_UART_Transmit(..., 100U)        [foreground blocks]
  -> optional telemetry selection after ACK
  -> protocol_send_leak_status()/IMU/depth [stack wire]
  -> the same blocking transmit path
  -> SafetySupervisor
  -> MotionManager
```

`Core/App/app_main.c` encodes ACK, Leak, IMU, and Depth into automatic
`wire[RBP2_MAX_WIRE_SIZE]` buffers and currently treats `HAL_OK` as synchronous
completion. `Core/Communication/uart_transport_stm32.c` stores the UART
handle, owns the RX byte/ring, and currently calls blocking HAL TX.

`Core/Src/main.c` centrally routes RX completion to USART1, USART3, and
USART6. It has no TX-completion callback and its error callback currently
routes only JY901S and Depth. `stm32f4xx_it.c` already routes all three UART
IRQs through `HAL_UART_IRQHandler()`.

The current HAL IT implementation retains the caller pointer in
`huart->pTxBuffPtr`; therefore a caller stack buffer cannot be handed to HAL.
An accepted frame must be copied into a static owned slot before the enqueue
returns. That slot is immutable until TX completion or explicit reinitialization
abort.

The `.ioc` has USART1 at 9600 baud and an USART1 IRQ, but no USART1 DMA stream,
channel, handle/link, or DMA IRQ route. The generated MSP file likewise has no
`hdma_usart1_tx`. `NVIC.ForceEnableDMAVector` alone is not a USART1 DMA setup.

## 2. IT versus DMA

| Concern | Interrupt TX | DMA TX |
| --- | --- | --- |
| Current fit | Existing USART1 IRQ/HAL IT; no generated setup | No USART1 `hdmatx` or stream/link/IRQ configuration |
| CubeMX impact | None for v1 | Requires STM32F407 mapping and regenerated `.ioc`/MSP/IRQ code |
| 9600 CPU cost | One TXE service per byte plus one TC callback; 76-byte max means 76 byte services | Much lower per-byte ISR cost |
| Future 115200+ | Same byte count, much shorter wall time; 76 bytes is about 6.6 ms at 115200 | Better CPU scaling for frequent/larger traffic |
| Failure surface | Existing UART state and bounded IT callbacks | DMA stream errors, abort callbacks, and generated ownership |
| Migration | Replace only the start/completion adapter | Reuse this queue and ownership contract |

With 8-N-1, the conservative 76-byte wire bound is
`76 * 10 / 9600 = 79.17 ms`. The current 56-byte IMU payload gives a 66-byte
logical frame and at most 68 wire bytes, about 70.83 ms. A four-byte ACK is at
most 16 wire bytes, about 16.67 ms. These are physical serialization bounds,
not foreground time after the migration.

The current Qt APC220 source profile is also explicit: `heartbeatIntervalMs`
is 250 ms, `ackTimeoutMs` is 250 ms, and `heartbeatSafetyBudgetMs` is 490 ms.
At 9600 8-N-1, one maximum 76-byte active frame followed by one maximum
16-byte ACK has a serializer bound of about `79.17 + 16.67 = 95.84 ms`.
Therefore the current 250 ms ACK timeout has arithmetic margin for that one
non-preemptible frame followed by its ACK. This is a UART serialization bound,
not a measured APC220 RF turnaround or end-to-end latency result; module/RF
turnaround remains outside this calculation. This design does not change the
baud rate or any Qt timeout/budget value.

An ACK arriving immediately after a telemetry frame begins cannot preempt
that frame. It waits for one active frame to finish, then is selected ahead of
waiting telemetry. Optional telemetry coalescing reduces future backlog but
cannot shorten a frame already transmitting. This finite residual is
compatible with the current response semantics when the existing host timeout
has margin; ACK TX is never used as host-liveness evidence.

## 3. Bounded queue and frame ownership

The fixed storage contract is:

```text
RBP2_MAX_WIRE_SIZE       76 bytes per frame
CONTROL_QUEUE_CAPACITY   4 pending FIFO frames
TELEMETRY_QUEUE_CAPACITY 2 pending frames
ACTIVE_FRAME_CAPACITY    1 frame
```

Nominal byte storage is 7 * 76 = 532 bytes plus fixed metadata and counters.
There is no heap and no stored caller pointer. Conceptually each owned slot
contains:

```c
typedef struct {
    uint16_t length;
    uint8_t bytes[RBP2_MAX_WIRE_SIZE];
    uart_tx_message_kind_t kind;
    uint8_t occupied;
} uart_tx_frame_t;
```

The implementation may use private states, but must enforce:

1. Reject null, zero, oversize, or invalid-kind input before queue mutation.
2. Copy the complete encoded bytes and length before returning an accepted
   result.
3. Never overwrite or replace an active frame.
4. Never interleave bytes from two RBP2 frames.
5. Allow the caller to overwrite/destroy its stack buffer immediately after
   `UART_TX_ENQUEUED` or `UART_TX_COALESCED`.

Message kind derives class, so callers cannot smuggle in an arbitrary
priority:

```text
CONTROL/HIGH:   ACK
TELEMETRY/LOW:  Leak status, IMU snapshot, Depth snapshot
```

The application result is an enqueue result, not a physical-completion
status:

```c
typedef enum {
    UART_TX_ENQUEUED = 0,
    UART_TX_COALESCED,
    UART_TX_CONTROL_FULL,
    UART_TX_TELEMETRY_FULL,
    UART_TX_INVALID,
    UART_TX_TRANSPORT_ERROR
} uart_tx_enqueue_result_t;
```

`ENQUEUED` and `COALESCED` mean owned storage accepted the bytes; neither
means that wire TX completed. The four-slot control FIFO never coalesces. A
full control queue returns `UART_TX_CONTROL_FULL`, increments rejection/drop
diagnostics, and never waits. This is explicit bounded ACK failure, not a
silent overwrite.

The two-slot telemetry queue uses latest-value replacement:

* A pending same-kind Leak, IMU, or Depth frame is replaced and returns
  `UART_TX_COALESCED`.
* The active frame is never replaced.
* If two distinct kinds are pending and a third distinct kind arrives, return
  `UART_TX_TELEMETRY_FULL`, preserve both old slots, and increment the drop
  counter.
* A queue-full telemetry result does not mark the policy successful, so a
  later eligible publication may retry.

When free, selection is deterministic: oldest control first; otherwise a
fixed round-robin cursor scans the two telemetry slots and skips empty slots.
Control can delay low-priority telemetry, but memory remains bounded.

## 4. Application API and policy meaning

The STM32 header will expose an explicit enqueue API such as:

```c
uart_tx_enqueue_result_t uart_transport_stm32_enqueue(
    const uint8_t *data,
    uint16_t length,
    uart_tx_message_kind_t kind);
```

The old `HAL_StatusTypeDef uart_transport_stm32_transmit()` contract is removed
from App use. The four `protocol_send_*` functions keep their current
encoding/stack-buffer ownership and use the new result.

For the current heartbeat flow:

* `ack_sent` means ACK was accepted into owned control storage. Optional
  telemetry is not selected after an ACK queue-full/invalid result in that
  pass.
* Existing `*_mark_published()` and
  `telemetry_scheduler_mark_success()` names remain source-compatible, but
  their call-site meaning is “accepted or coalesced into the owned queue”, not
  physical completion. Queue failure does not mark success.
* No application completion callback is needed to advance telemetry cadence.
  Physical completion is recorded by transport diagnostics.
* Received/accepted heartbeat processing remains authoritative for host
  liveness. Outbound ACK failure never changes SafetySupervisor liveness.

Protocol V2 IDs, payloads, CRC/COBS encoding, ACK correlation, and command
execution are unchanged.

## 5. State machine, HAL behavior, and recovery

The transport states are:

```text
UNINITIALIZED -> IDLE -> STARTING -> ACTIVE
                         |             |
                         v             v
                   START_DEFERRED   STARTING for one next frame

Any operational state -> REINITIALIZING -> IDLE, with no replay
```

`IDLE` may retain pending frames after a failed start. `START_DEFERRED`
retains one promoted owned frame that HAL has not accepted yet.

Exact handling:

* Promote one frame before calling `HAL_UART_Transmit_IT()` and call HAL in a
  short masked critical section. `HAL_OK` establishes active ownership.
* `HAL_BUSY` leaves the promoted bytes owned as `START_DEFERRED`.
  `uart_transport_stm32_process()` makes at most one retry per foreground
  call; no caller waits or spins.
* `HAL_ERROR` releases only the failed promoted frame, increments start/drop
  diagnostics, and preserves remaining pending frames for a later attempt. An
  enqueue that immediately fails to start returns `UART_TX_TRANSPORT_ERROR`.
* A matching USART1 `HAL_UART_TxCpltCallback()` releases the active frame once,
  records physical completion, and starts at most one next owned frame. A
  duplicate/state-mismatched callback increments `unexpected_callback_count`
  and cannot free another frame.
* The current HAL treats non-ORE IT errors as non-blocking and lets TX
  continue; its ORE path ends RX but does not own/release TX. Thus
  `HAL_UART_ErrorCallback()` records the USART1 error and leaves active TX
  ownership intact. It does not encode, parse, drain, or call blocking abort.
* USART1 RX ownership is explicit. The transport keeps `rx_armed` and
  `rx_needs_rearm`; only a successful `HAL_UART_Receive_IT()` sets
  `rx_armed`. PE/FE/NE while HAL keeps RX active are recorded without
  unnecessarily tearing down/rearming the working transfer. ORE, or any HAL
  error callback observed after HAL has ended the RX transfer, clears
  `rx_armed` and sets `rx_needs_rearm`. The error callback does not rearm from
  ISR context. One bounded foreground `uart_transport_stm32_process()` pass
  attempts at most one `HAL_UART_Receive_IT()` rearm: `HAL_BUSY` leaves the
  request pending for a later pass, while `HAL_ERROR` leaves it pending and
  increments a rearm-error diagnostic. TX active ownership is unchanged.
* `HAL_UART_AbortTransmitCpltCallback()` is used only by explicit foreground
  reinitialization. It releases the aborted active or `START_DEFERRED`
  promoted slot, drops all ordinary pending slots with per-kind counts, resets
  to IDLE, and never replays old bytes.

The transport never calls blocking `HAL_UART_AbortTransmit()` from an ISR.
`HAL_UART_AbortTransmit_IT()` is only a foreground reinitialization action.

## 6. Reinitialization and concurrency

`uart_transport_stm32_init()` remains the boot-time attach/rearm operation.
An explicit `uart_transport_stm32_reinitialize()` is the only runtime queue
reset:

1. Foreground enters `REINITIALIZING`; new enqueue calls return
   `UART_TX_TRANSPORT_ERROR` without changing protocol, Motion, or safety
   state. A `START_DEFERRED` promoted slot is transport-owned but not
   physically active; it is not an ordinary pending frame and is included in
   the reinitialization cleanup.
2. Before calling `HAL_UART_AbortTransmit_IT()`, the transport prepares the
   complete `REINITIALIZING` ownership state under the critical section:
   `reinit_completion_handled` is false, the reinitialization epoch is
   advanced, and all active/promoted and pending ownership is still visible to
   the one cleanup path. The HAL call is made outside the critical section.
3. In the current IT-only HAL path, `HAL_UART_AbortTransmit_IT()` may invoke
   `HAL_UART_AbortTransmitCpltCallback()` synchronously before it returns
   `HAL_OK`. The callback therefore calls an idempotent
   `finish_reinitialize_once()` path. It marks completion handled before
   releasing the active/`START_DEFERRED` slot and pending slots, increments
   each drop diagnostic once, preserves cumulative counters, clears the
   reinitialization markers, and enters `IDLE` with no replay. If the callback
   has already run inline, code after the HAL call performs no second cleanup.
4. A later or duplicate abort callback sees that the transport is no longer
   awaiting this reinitialization (or that completion is already handled), is
   counted as unexpected, and cannot release or mutate a newly enqueued frame.
   This remains true if a test double invokes the callback twice inline before
   returning and invokes it again after a fresh enqueue.
5. If there is no active/promoted ownership, pending frames are finalized by
   the same once-only cleanup path without a HAL abort. If an abort call
   returns an error before a callback, the path finalizes local ownership once;
   a `HAL_BUSY` abort is retried from bounded foreground processing no more
   than `UART_TX_MAX_REINITIALIZE_ABORT_BUSY_RETRIES` times, then performs the
   same no-replay local cleanup and records recovery failure. No callback or
   return path may release a slot twice.
6. Persistent `HAL_BUSY` while starting a `START_DEFERRED` frame cannot wedge
   it forever. After exactly
   `UART_TX_MAX_CONSECUTIVE_START_BUSY` one-at-a-time start attempts (the v1
   policy value is 8), the transport increments `busy_recovery_count`, enters
   the same explicit reinitialization path, drops/releases the deferred and
   pending ownership exactly once, and returns to `IDLE`. A later fresh
   enqueue may start normally; the old frame is never replayed.
7. USART3/USART6 and the existing USART1 RX ring policy are untouched by TX
   reinitialization. RX rearm state is independent and is serviced by the
   bounded foreground process path.

The foreground is the sole producer; the USART1 TX callback is the consumer.
`volatile` does not replace synchronization. Compound operations use
save/disable/restore-PRIMASK critical sections:

* Copy at most one 76-byte frame, update metadata, and promote at most one
  slot while masked. Never encode, parse, sample sensors, or run arbitrary
  queue loops while masked.
* Restore the saved mask; never unconditionally enable IRQs.
* Fixed scans are limited to four control or two telemetry slots.
* TX completion does only fixed state/counter work and at most one nonblocking
  HAL start. RX recovery does at most one nonblocking receive-arm attempt per
  foreground process call; neither path generates telemetry or parses RBP2.

The current HAL has independent `gState` (TX) and `RxState` (RX). Tests must
prove the existing one-byte USART1 RX can rearm and deliver bytes while IT TX
is active, including ORE recovery. A PE/FE/NE callback while HAL still
reports an active RX transfer must not cause an unnecessary rearm.

## 7. Callback routing and safety boundary

`Core/Src/main.c` remains the central callback owner:

```text
HAL_UART_RxCpltCallback
  -> existing USART1 host RX, USART3 JY901S RX, USART6 Depth RX
HAL_UART_TxCpltCallback
  -> uart_transport_stm32_on_tx_complete()
HAL_UART_AbortTransmitCpltCallback
  -> uart_transport_stm32_on_abort_transmit_complete()
HAL_UART_ErrorCallback
  -> uart_transport_stm32_on_error()
  -> existing JY901S and Depth error handlers
```

Each handler filters the USART1 instance/handle and cannot mutate USART3/6
state. `app_main_process()` keeps all existing work in place and calls one
bounded `uart_transport_stm32_process()` retry only after the existing
MotionManager call. SafetySupervisor and Motion never wait for TX capacity or
wire completion.

Safety timeout/fault does not flush TX merely as a communication optimization;
heartbeat receive remains authoritative and immediate actuator fail-safe is
local. Only explicit UART reinitialization flushes with the no-replay rule.

## 8. Diagnostics and debugger ABI

The merged timing report is ABI version 3 and 1300 bytes. The async change
must distinguish foreground enqueue time from physical TX. V1 will append a
fixed transport block, bump ABI to 4, and set report size to 1432:

```c
typedef struct {
    uint32_t enqueued_count[4];
    uint32_t coalesced_count[4];
    uint32_t completed_count[4];
    uint32_t rejected_count[4];
    uint32_t dropped_count[4];
    uint32_t control_queue_full_count;
    uint32_t telemetry_queue_full_count;
    uint32_t start_busy_count;
    uint32_t start_error_count;
    uint32_t uart_error_count;
    uint32_t unexpected_callback_count;
    uint32_t high_water_mark;
    uint32_t reinitialization_count;
    uint32_t busy_recovery_count;
    uint32_t rx_error_count;
    uint32_t rx_rearm_attempt_count;
    uint32_t rx_rearm_busy_count;
    uint32_t rx_rearm_error_count;
} motion_timing_uart_transport_report_t;
```

This block is 132 bytes at offset 1300. C11 static assertions must cover its
size, the critical offset 1300, and the new total size 1432, while retaining
all old critical offsets. The fixed-width fields are ordered exactly as shown;
there is no packing pragma or decoder offset guessing. `uart_error_count`
counts all USART1 error callbacks, `rx_error_count` counts those that require
RX rearm, and the three `rx_rearm_*` fields count the bounded recovery
attempts/results. `busy_recovery_count` counts finite persistent-start-BUSY
escalations. `tools/read-motion-timing.ps1` must validate magic, version, and
declared size before decoding it and fail loudly on mismatch.

The existing per-kind `tx[]` timing is explicitly labelled as the foreground
enqueue/copy span and no longer claims physical serialization. The appended
block supplies accepted/coalesced/rejected/dropped/start/error/completion
counters. Existing worst-gap context records work that really occurred in
the foreground interval; physical serializer time is not falsely attributed
to it. Diagnostics hooks are fixed-cost RAM-only and are no-ops when
diagnostics are compiled off. A frozen report cannot be modified by later
loops, callbacks, ACKs, or reconnect traffic.

For the fixed `tx[]` block, the v4 decoder labels the existing fields with
these exact meanings: `call_count` is enqueue attempts, `byte_count` is bytes
offered by those attempts, `last_status` is the latest enqueue result,
`ok_count` counts both `ENQUEUED` and `COALESCED` results, `error_count` counts
all rejected results, `timeout_count` is reserved and remains zero in v1, and
`duration` is the enqueue/copy span. The appended per-kind
`completed_count` is the only completion count; it is not inferred from an
enqueue `OK` result.

The existing readout lifecycle remains safe:

```text
finish exercise -> accepted Motion STOP or safety/fault termination
-> actuator-safe stop path -> halt/read frozen report -> reset/run stopped
```

No diagnostic UART traffic is added, and active-motion halt is not the normal
measurement-ending action.

The diagnostic readout helper must be used only after the fixed exercise has
ended and a normal Motion STOP has been accepted, or after a safety/fault stop
has frozen the report. The target is then allowed to complete its actuator-safe
stop path before halt/read. After the frozen report is dumped, the helper
resets/runs the target into the normal stopped startup state rather than
silently leaving the MCU halted. It never uses an active-motion halt as the
exercise-ending mechanism and never emits diagnostic UART traffic.

## 9. Tests and post-fix acceptance

Host tests must cover:

```text
stack buffer overwrite after enqueue; bytes remain intact
idle control -> exactly one IT start
active telemetry + ACK -> telemetry intact, ACK starts next
waiting telemetry + ACK -> ACK first
same-kind pending telemetry -> bounded coalesce
distinct telemetry when two slots full -> explicit TELEMETRY_FULL
control when four slots full -> explicit CONTROL_FULL
completion -> release once and start next once
HAL_BUSY -> preserve ownership and retry once per process call
HAL_ERROR -> explicit drop; remaining frames progress
UART error -> active IT TX remains valid and can complete
RX remains operational while TX is active
USART1 ORE -> RX rearm is deferred to bounded foreground processing
PE/FE/NE with active HAL RX -> no unnecessary RX rearm
persistent start HAL_BUSY -> finite recovery, no deferred-frame replay
inline/double/later abort completion -> exactly-once cleanup
no byte interleaving or partial-frame handoff
reinitialize -> no replay; fresh later enqueue works
Safety liveness and Protocol ACK correlation remain passing
```

The adapter regression must use a HAL abort double that calls
`HAL_UART_AbortTransmitCpltCallback()` twice inline before returning
`HAL_OK`, then calls a later duplicate after a fresh frame has been enqueued.
It must prove that the active/deferred frame, pending frames, and drop
diagnostics are each released/countable exactly once, that the fresh frame is
not corrupted, and that no stale frame is replayed. The same regression must
cover a `START_DEFERRED` promoted frame.

The RX regression must keep an IT TX frame active, inject USART1 ORE with the
fake HAL RX state ended, invoke the error callback, and verify that one
bounded foreground process call re-arms RX; a later byte must reach the host
RX ring and the TX frame must complete intact. Separate PE/FE/NE cases must
leave an active HAL RX transfer armed. The finite-BUSY regression must return
`HAL_BUSY` for exactly `UART_TX_MAX_CONSECUTIVE_START_BUSY` start attempts,
verify one recovery escalation and one drop/no-replay cleanup, then enqueue a
fresh frame and verify it can start after HAL recovers. Every process call
must make no retry loop beyond the one TX attempt and one RX-rearm attempt.

After implementation review, target evidence uses one NORMAL and one REDUCED
diagnostic image, with runtime CPG/SimpleGait selection on identical ELFs.
Read only after STOP/safety termination freezes the report. Require measured
communication-induced `>30 ms` gaps to disappear, and compare enqueue spans
for short ACK versus long telemetry to show foreground latency is not wire
serialization time. Do not predict the final max gap or claim physical PWM or
Water verification.

## 10. Explicit non-goals

```text
UART baud or APC220 RF changes; Raspberry Pi software; DMA/CubeMX v1 setup;
UART RX conversion; RX queue budgets; foreground reordering; scheduler or
catch-up Servo writes; heartbeat/safety changes; PWM/calibration changes;
CPG/SimpleGait math or tuning; Servo stutter/jitter fix; Protocol V2 wire
redesign; Qt trajectory streaming; RTOS.
```

Future DMA stream/channel mapping and future Pi baud are intentionally deferred
questions, not v1 blockers.
