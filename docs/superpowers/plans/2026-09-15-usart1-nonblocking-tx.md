# USART1 Non-blocking TX / Motion Cadence Stabilization Implementation Plan

> **For agentic workers:** Execute this plan only after external review approves the design. Every implementation step is test-first and must preserve the frozen scope in the design specification.

**Goal:** Replace blocking USART1 TX with a fixed-size, transport-owned
asynchronous TX engine without changing Protocol V2, RX policy, Motion
scheduling, SafetySupervisor semantics, Servo output, or gait mathematics.

**Architecture:** Add a HAL-independent queue in
`RoboBeetleFirmware/Core/Communication/uart_tx_queue.[ch]`. Keep the STM32
handle, IRQ callbacks, and `HAL_UART_Transmit_IT` adapter in
`RoboBeetleFirmware/Core/Communication/uart_transport_stm32.[ch]`. Use a
four-slot control FIFO, two-slot same-kind-coalescing telemetry queue, and one
active owned frame. Add explicit USART1 RX arm/rearm state, a finite
`HAL_BUSY` start-recovery policy, and an exactly-once nonblocking abort cleanup
path. Append fixed transport and RX-recovery counters to the motion-timing
diagnostic report as ABI version 4.

**Tech Stack:** C11, STM32F407 HAL, existing USART1 RX interrupt, GCC host
tests, PowerShell host runner, and the existing Qt6/C++20 host test suite.

---

## Phase 1: Add red tests for the pure bounded queue

**Files:**

* Add `RoboBeetleFirmware/tests/uart_tx_queue_tests.c`.
* Add a `uart_tx_queue_tests` case to
  `RoboBeetleFirmware/tests/run_host_tests.ps1`.
* Add `Core/Communication/uart_tx_queue.c` to
  `RoboBeetleFirmware/CMakeLists.txt` when the implementation step is
  reached. The first test-first run should fail because the new API/source is
  not yet implemented.

**Required red cases:**

1. Offer a complete ACK wire frame, overwrite the caller buffer immediately,
   take the active view, and assert every owned byte and the length remain
   unchanged.
2. Keep telemetry active, offer an ACK, and assert the ACK is pending while
   the active telemetry bytes are unchanged.
3. Fill four control slots; assert the fifth offer returns
   `UART_TX_CONTROL_FULL`, does not wait or overwrite a slot, and increments
   the bounded statistic.
4. Fill two telemetry slots with distinct kinds; assert a third distinct kind
   returns `UART_TX_TELEMETRY_FULL` and preserves both old slots.
5. Offer a second pending Leak, IMU, and Depth frame; assert only the matching
   pending kind is replaced and the result is `UART_TX_COALESCED`.
6. Assert oldest control selection before telemetry and fixed round-robin
   telemetry selection.
7. Complete an active frame twice; assert the second completion cannot release
   or mutate the next active frame.

The queue/transport contract uses no dynamic allocation and declares the
finite recovery constants in the adapter seam:

```c
UART_TX_MAX_CONSECUTIVE_START_BUSY       8
UART_TX_MAX_REINITIALIZE_ABORT_BUSY_RETRIES 2
```

The first bounds repeated `HAL_UART_Transmit_IT()` `HAL_BUSY` results while a
promoted `START_DEFERRED` frame has no known physical transfer. The second
bounds foreground retries of a nonblocking abort that itself returns
`HAL_BUSY`; both values are fixed and host-testable.

Run only the new case and capture the expected failure. Do not add production
code before these ownership and priority assertions exist.

## Phase 2: Implement the HAL-independent queue

**Files:**

* Add `RoboBeetleFirmware/Core/Communication/uart_tx_queue.h`.
* Add `RoboBeetleFirmware/Core/Communication/uart_tx_queue.c`.
* Add the source to `RoboBeetleFirmware/CMakeLists.txt`.
* Add the source to every app/transport case in
  `RoboBeetleFirmware/tests/run_host_tests.ps1`.

The existing cases that link the full app and therefore need the queue source
are `app_main_timing_diagnostics_tests`,
`app_main_reduced_telemetry_tests`, `app_main_depth_telemetry_tests`, and the
three generated backend variants (`app_main_backend_default_tests`,
`app_main_backend_simple_override_tests`, and
`app_main_backend_cpg_override_tests`). The compile-contract-only cases do
not link the queue.

**Implementation steps:**

* Define `uart_tx_message_kind_t` for ACK, Leak, IMU, and Depth. Derive the
  control/telemetry class from kind.
* Define `uart_tx_enqueue_result_t` with
  `UART_TX_ENQUEUED`, `UART_TX_COALESCED`, `UART_TX_CONTROL_FULL`,
  `UART_TX_TELEMETRY_FULL`, `UART_TX_INVALID`, and
  `UART_TX_TRANSPORT_ERROR`.
* Use private static storage: four control slots, two telemetry slots, and
  one active slot, each holding up to `RBP2_MAX_WIRE_SIZE` bytes.
* Validate pointer, length, and kind before queue mutation. Copy the complete
  wire frame; never store the caller pointer.
* Implement same-kind telemetry replacement, distinct-kind full behavior,
  control FIFO ordering, fixed round-robin telemetry ordering, and one active
  ownership token.
* Expose only a transient active-frame view for the STM32 adapter.
* Implement `begin_next`, `mark_start_deferred`, `complete_active`, and
  `fail_active` transitions. A failed HAL start must not strand later pending
  frames.
* Keep fixed-width statistics and a test snapshot API. Queue code performs no
  UART I/O, protocol parsing, telemetry generation, heap allocation, or
  diagnostic encoding.

Run the queue tests to green, then run
`RoboBeetleFirmware/tests/run_host_tests.ps1`.

## Phase 3: Add red STM32 adapter tests

**Files:**

* Add `RoboBeetleFirmware/tests/uart_transport_stm32_tests.c`.
* Add a `uart_transport_stm32_tests` case to
  `RoboBeetleFirmware/tests/run_host_tests.ps1` linking queue and transport
  sources.

**Test seam:**

Provide host doubles for `HAL_UART_Receive_IT`,
`HAL_UART_Transmit_IT`, and `HAL_UART_AbortTransmit_IT`. The transmit double
records the pointer, length, start count, and bytes observed at each start.
Use distinct fake USART1/USART3/USART6 instances.

**Required red cases:**

1. Init rearms one-byte RX and does not start TX.
2. Idle ACK enqueue invokes IT start exactly once with an owned pointer and
   returns without calling blocking `HAL_UART_Transmit`.
3. Enqueue while telemetry is active returns immediately and does not start a
   second frame before completion.
4. Matching USART1 TX completion releases the active frame and starts the ACK
   exactly once; a duplicate completion is ignored and counted.
5. USART3/USART6 callback inputs cannot mutate USART1 TX state.
6. USART1 RX completion while TX is active still pushes and rearms the RX byte.
7. HAL `BUSY` preserves the active owned frame and `process()` makes one retry
   per call without a wait loop.
8. HAL `ERROR` drops only the failed frame, increments diagnostics, preserves
   later pending frames, and permits a later start.
9. USART1 UART error while TX is active records the error but leaves the frame
   intact; later completion and next-frame start succeed.
10. Unexpected TX and abort-completion callbacks cannot release another frame.
11. Reinitialization aborts nonblocking, drops active/pending frames once,
    never replays stale frames, preserves cumulative counters, and permits a
    fresh later enqueue.
12. A HAL abort double invokes
    `HAL_UART_AbortTransmitCpltCallback()` twice inline before returning
    `HAL_OK`; cleanup releases active ownership, including a `START_DEFERRED`
    promoted frame, and drops pending ownership exactly once. A later duplicate
    callback after a fresh enqueue cannot corrupt or release the new frame.
13. USART1 ORE with the fake HAL RX state ended marks `rx_needs_rearm`; one
    bounded foreground process call re-arms RX while active IT TX remains
    intact, and a later host byte reaches the RX ring. PE/FE/NE with HAL RX
    still active records the error without an unnecessary rearm.
14. Persistent `HAL_BUSY` for exactly
    `UART_TX_MAX_CONSECUTIVE_START_BUSY` start attempts increments one visible
    recovery escalation, drops the deferred/pending ownership once, performs
    no replay, and allows a fresh frame to start after HAL recovery. A
    reinitialization abort that returns `HAL_BUSY` is retried no more than
    `UART_TX_MAX_REINITIALIZE_ABORT_BUSY_RETRIES` times.

Run this case and retain the expected red result until the adapter is
implemented.

## Phase 4: Implement the USART1 IT adapter

**Files:**

* Modify `RoboBeetleFirmware/Core/Communication/uart_transport_stm32.h`.
* Modify `RoboBeetleFirmware/Core/Communication/uart_transport_stm32.c`.

**Implementation steps:**

* Preserve the current RX byte, RX ring, `pop`, and `HAL_UART_Receive_IT`
  behavior.
* Replace `uart_transport_stm32_transmit()` in App use with
  `uart_transport_stm32_enqueue()` and the explicit enqueue result.
* Keep the active owned bytes stable until matching TX completion or explicit
  abort completion.
* Protect compound transitions with save/disable/restore-PRIMASK critical
  sections; never unconditionally enable interrupts.
* Copy at most one 76-byte frame and promote at most one slot per operation.
  No encoding, parsing, sensor work, or arbitrary queue scan is masked.
* Promote before calling `HAL_UART_Transmit_IT` while masked so a pending TXE
  interrupt cannot see a partially established active state.
* Resolve HAL `OK`, `BUSY`, and `ERROR` exactly as the design specifies.
* Add `uart_transport_stm32_process()` with at most one deferred-start attempt
  and at most one RX-rearm attempt per foreground call; there are no wait or
  retry loops.
* Add instance-checked TX-complete, error, abort-complete, and explicit
  reinitialization entry points. Do not call blocking abort from an ISR.
* Track `rx_armed` and `rx_needs_rearm`. On RX completion, copy the byte and
  rearm immediately when possible; defer `HAL_BUSY`/`HAL_ERROR` outcomes to
  foreground processing. On USART1 error, keep RX armed for PE/FE/NE while
  HAL still reports `BUSY_RX`; mark ORE or HAL-ended RX for foreground rearm.
  A rearm `HAL_ERROR` remains pending and increments its diagnostic.
* Keep callback work fixed and bounded: state/counter updates, a scan of four
  control or two telemetry slots, and at most one nonblocking HAL start.
* Implement finite `HAL_BUSY` recovery. Count consecutive deferred-start
  attempts; at the fixed limit of 8, increment the escalation diagnostic and
  enter explicit reinitialization. A `START_DEFERRED` frame is transport-owned
  but not physically active, so reinitialization drops/releases it exactly once
  with all ordinary pending frames and never replays it.
* Prepare the full `REINITIALIZING` state, including an epoch and
  `reinit_completion_handled` guard, before calling
  `HAL_UART_AbortTransmit_IT()` outside the critical section. The abort
  completion handler marks the guard before cleanup, so the current HAL's
  synchronous inline callback, an inline duplicate, a later duplicate, and a
  fresh enqueue after recovery are all safe. Abort `HAL_BUSY` is retried from
  bounded foreground processing no more than twice; an abort error or exhausted
  policy finalizes local ownership once without replay.

Run adapter tests to green, then rerun the pure queue tests.

## Phase 5: Route HAL callbacks while preserving all UARTs

**Files:**

* Modify `RoboBeetleFirmware/Core/Src/main.c`.
* Leave `RoboBeetleFirmware/Core/Src/stm32f4xx_it.c` unchanged unless the
  existing HAL IRQ route cannot deliver the declared callbacks.

**Changes:**

* Add `uart_transport_stm32_on_tx_complete(huart)` to
  `HAL_UART_TxCpltCallback`.
* Add `uart_transport_stm32_on_abort_transmit_complete(huart)` to
  `HAL_UART_AbortTransmitCpltCallback`.
* Add `uart_transport_stm32_on_error(huart)` to the existing error callback
  while retaining JY901S and Depth error handlers.
* Keep RX callback routing to USART1, USART3, and USART6 exactly as-is.

Extend adapter tests to assert instance filtering and JY901S/Depth RX
operation during USART1 TX.

## Phase 6: Migrate App send semantics with tests first

**Files:**

* Modify `RoboBeetleFirmware/tests/app_main_timing_diagnostics_tests.c`.
* Modify `RoboBeetleFirmware/tests/app_main_reduced_telemetry_tests.c`.
* Modify `RoboBeetleFirmware/tests/app_main_depth_telemetry_tests.c`.
* Modify their HAL/test doubles to drive explicit TX completion where needed.
* Modify `RoboBeetleFirmware/Core/App/app_main.c` only after tests are red.

**Red assertions:**

1. ACK/Leak/IMU/Depth bytes remain intact after the local stack wire buffer is
   overwritten.
2. `ack_sent` is true only for accepted control enqueue; an ACK queue-full
   result does not select optional telemetry in that pass.
3. Telemetry policy and scheduler marks occur on accepted/coalesced enqueue,
   not physical completion.
4. Physical TX completion causes no application telemetry-policy mutation.
5. Outbound ACK failure does not alter heartbeat count, host liveness,
   SafetySupervisor state, or Motion execution.
6. Existing Protocol V2 ACK bytes, correlation, duplicate behavior, and wire
   format remain unchanged.

**Implementation steps:**

* Replace each `HAL_StatusTypeDef transmit_status` use with
  `uart_tx_enqueue_result_t`.
* Keep current encoding and validation, then enqueue the complete wire frame
  with its fixed message kind.
* Record the bounded enqueue/copy span and explicit result in diagnostics;
  remove the assumption that `HAL_OK` means physical completion.
* Preserve heartbeat and optional telemetry order except for the accepted
  enqueue meaning.
* Call `uart_transport_stm32_process()` once after the existing
  `motion_manager_process()` call; do not move or budget existing foreground
  work.

Run the three app cases and the existing Protocol dispatcher tests, then run
the full Firmware host gate.

## Phase 7: Add diagnostic ABI v4

**Files:**

* Modify `RoboBeetleFirmware/Core/Diagnostics/motion_timing_diagnostics.h`.
* Modify `RoboBeetleFirmware/Core/Diagnostics/motion_timing_diagnostics.c`.
* Modify `RoboBeetleFirmware/tests/motion_timing_diagnostics_tests.c`.
* Modify `RoboBeetleFirmware/tests/motion_timing_diagnostics_hooks_tests.c`.
* Modify `RoboBeetleFirmware/tests/motion_timing_diagnostics_lifecycle_tests.c`.

**Implementation steps:**

* Bump ABI version from 3 to 4.
* Append the 132-byte `motion_timing_uart_transport_report_t` block at offset
  1300 and set total report size to 1432. Keep the exact field order from the
  design: existing enqueue/completion counters, finite-BUSY recovery, all
  USART1 error callbacks, errors requiring RX rearm, and RX rearm
  attempt/BUSY/ERROR counters.
* Add static assertions for the new block size, offset, and total size while
  retaining every existing critical offset assertion.
* Add fixed-cost hooks for enqueue, coalesce, reject, drop, start-busy,
  start-error, finite-BUSY recovery, UART error, RX-rearm-required error,
  RX-rearm attempt/BUSY/ERROR, unexpected callback, high-water,
  reinitialization, and physical completion.
* Make every hook a no-op when diagnostics are compiled OFF.
* Clear the new block on `begin_run`, advance the run marker, and block all
  later event mutation after `freeze`.
* Label existing per-kind TX timing as foreground enqueue/copy timing: call
  count is attempts, byte count is offered bytes, `last_status` is the
  enqueue result, `ok_count` includes ENQUEUED/COALESCED, `error_count` is
  rejected results, `timeout_count` is reserved zero, and duration is the
  enqueue span. Do not attribute physical serializer time to a foreground
  span.

Add a lifecycle test that records events, freezes, attempts later events, and
asserts frozen bytes are unchanged. Add diagnostics-off and reduced-telemetry
compile-contract coverage.

## Phase 8: Update fixed debugger readout

**Files:**

* Modify `tools/read-motion-timing.ps1`.
* Modify `tools/tests/read-motion-timing-path-tests.ps1` only for new decoder
  assertions.

**Implementation steps:**

* Validate ABI version 4 and report size 1432 after validating magic.
* Decode the appended block from fixed offset 1300 only after the header
  checks succeed; fail loudly on mismatch.
* Preserve frozen-state checking and the safe lifecycle: fixed exercise,
  accepted STOP/safety termination, actuator-safe path, halt/read frozen
  report, and reset/run to the normal stopped startup state.
* Preserve OpenOCD forward-slash path normalization and add no UART traffic.

Run the Windows-path regression and a synthetic ABI mismatch failure test.

## Phase 9: Run all Firmware, Qt, and script gates

**Files:**

* No Qt production source change is planned.
* Run existing `RoboBeetleConsole` Protocol and RobotController tests through
  their Qt6 CMake/CTest configuration.

**Commands:**

```powershell
powershell -ExecutionPolicy Bypass -File RoboBeetleFirmware/tests/run_host_tests.ps1
cmake -S RoboBeetleConsole -B build/console-tests -DCMAKE_BUILD_TYPE=Debug
cmake --build build/console-tests --parallel
ctest --test-dir build/console-tests --output-on-failure
powershell -ExecutionPolicy Bypass -File tools/tests/read-motion-timing-path-tests.ps1
git diff --check
```

If Qt6 is unavailable, report Qt as `NOT RUN`; never claim PASS. Keep all
existing servo, gait, Safety, Protocol ACK, and host-liveness tests in the
runner.

## Phase 10: Target measurements only after implementation review

Do not start target A/B/C/D measurements during implementation. After the
implementation is externally reviewed and the user starts measurement:

1. Build one NORMAL and one REDUCED diagnostics image; use the same ELF for
   CPG and SimpleGait within each pair.
2. Runtime-select only while STOPPED and validate report magic/version/size,
   backend, run marker, and diagnostic condition.
3. Run the fixed exercise, accept STOP or safety termination, freeze, complete
   the actuator-safe path, read the report, and reset/run to stopped startup.
4. Report max interval, `>10/12/15/20/30 ms` counts, enqueue/completion
   counters, queue/drop counters, and exact trial metadata.
5. Require measured communication-induced `>30 ms` gaps to disappear and
   compare enqueue spans for short ACK versus long telemetry frames.
6. Keep PWM, Water, mechanical Servo stutter, and any scheduler conclusion
   outside this software-timing claim.

No UART baud, DMA configuration, CPG, Servo, Clock, PWM, safety, scheduler,
foreground-order, RX-budget, or stutter fix may be bundled into this PR.
