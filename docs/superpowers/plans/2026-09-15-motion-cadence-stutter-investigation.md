# Motion Cadence / Servo Stutter Investigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add compile-gated, RAM-only timing diagnostics and a reproducible target measurement procedure that can distinguish foreground communication delay, generator cost, logical Motion cadence, and physical servo/PWM behavior without changing production scheduling or actuator semantics.

**Architecture:** Keep app_main as the only cooperative scheduler. Add a disabled-by-default STM32F407 DWT counter module under the neutral Core/Diagnostics layer, place no-op hooks around existing app-loop, queue-drain, host-TX, and accepted Motion-tick boundaries, and expose one debugger-readable volatile report with a versioned fixed ABI. Begin and freeze one report per accepted Motion trial, retain interval-local causal context, and use host tests for counter arithmetic, ABI contracts, lifecycle, and compile contracts. Then run identical-ELF runtime-selector CPG/SimpleGait target trials under normal and reduced optional-telemetry diagnostic conditions. Treat scope and electrical measurements as separate evidence.

**Tech Stack:** C11, host GCC, existing Firmware host PowerShell runner, STM32F407 CMSIS DWT/CYCCNT, CMake/Ninja, arm-none-eabi toolchain, debugger/OpenOCD symbol readout, Markdown.

**Implementation status (2026-09-15):** Tasks 1-5 are implemented on this
branch: the neutral diagnostics module, ABI v3 fixed report, compile-gated
reduced-telemetry condition, per-trial begin/freeze lifecycle, interval-local
worst-gap context, observational app/Motion hooks, host regressions, and
fixed-offset readout helper are present. The Firmware host gate passes with 34
executables and 13 diagnostics/app/backend/benchmark compile contracts. The
ARM/OpenOCD toolchain is unavailable in this environment, so Task 6 target
images and A/B measurements remain NOT RUN; physical waveform, HAL tick,
post-clock gait, and Water evidence remain Pending.

---

## Task 1: Freeze the investigation baseline and source contract

**Files:**

- Read: RoboBeetleFirmware/Core/Src/main.c
- Read: RoboBeetleFirmware/Core/App/app_main.c
- Read: RoboBeetleFirmware/Core/Communication/uart_transport_stm32.c
- Read: RoboBeetleFirmware/Core/Communication/jy901s_transport_stm32.c
- Read: RoboBeetleFirmware/Core/Communication/depth_transport_stm32.c
- Read: RoboBeetleFirmware/Core/Communication/telemetry_scheduler.c
- Read: RoboBeetleFirmware/Core/Motion/motion_manager.c
- Read: RoboBeetleFirmware/Core/Motion/simple_gait_generator.c
- Read: RoboBeetleFirmware/Core/Motion/cpg_gait_generator.c
- Read: RoboBeetleFirmware/Core/Motion/cpg_core.c
- Read: RoboBeetleFirmware/Core/App/cpg_target_benchmark.c

- [ ] Verify the worktree is codex/motion-cadence-stutter-investigation and that
  HEAD and origin/main are ac36092ca3011a538420273d721794ac7a7d0a5a. Stop if
  the worktree contains unrelated changes.
- [ ] Record the source anchors in the design specification: app loop order,
  unbounded foreground drains, the four host transmit call sites, the 100 ms
  blocking HAL transmit, the 10 ms elapsed gate, and the one-update behavior
  after a delayed call.
- [ ] Run the existing host gate before adding any diagnostics:

~~~powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1
~~~

Expected result: 29 executable tests and 9 app/backend/benchmark
compile-contract objects pass. This is a software baseline only; it is not
target timing evidence.

## Task 2: Write counter arithmetic and ABI tests before the diagnostic module

**Files:**

- Create: RoboBeetleFirmware/tests/motion_timing_diagnostics_tests.c
- Modify: RoboBeetleFirmware/tests/run_host_tests.ps1
- No CMake registration is required for this manually compiled host test;
  production CMake changes belong to Task 3

- [ ] Add a red host test for unsigned DWT delta across 32-bit wrap, using a
  finish value below the start value and expecting finish minus start modulo
  2^32.
- [ ] Add a red host test for minimum, maximum, total, and sample-count
  accumulation, including the empty-report state.
- [ ] Add a red host test for the requested Motion gap buckets: greater than
  10, 12, 15, 20, and 30 ms. Boundary values equal to a threshold must not
  enter that strict greater-than bucket.
- [ ] Add a red host test for TX class accounting for ACK, LEAK, IMU, and
  DEPTH, including bytes, result/status, and duration accumulation.
- [ ] Add a red host test that the report has the documented fixed ABI:
  `magic`, `abi_version`, `report_size`, `system_core_clock_hz`, flags,
  runtime backend, run marker, reset marker, and fixed-width counters occur at
  the documented offsets and `sizeof` value. Use C11 `_Static_assert` in the
  module/header and assert the same values from the host test.
- [ ] Add a red host test that the report readout validation rejects wrong
  magic, ABI version, or report size before decoding counters.
- [ ] Add a red compile-contract test that the diagnostic-disabled API is a
  compile-time no-op and does not require a DWT or UART symbol. Add a second
  compile-contract test proving reduced telemetry is OFF by default and
  cannot become effective unless diagnostics is ON.
- [ ] Add the test case to the existing C11, Wall, Wextra, Werror host runner
  with an explicit expected executable name:

~~~powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1
~~~

Expected result while the implementation is absent: the new test is RED. Do
not weaken the test or call the baseline green until Task 3 implements the
specified API.

## Task 3: Implement the disabled-by-default RAM-only DWT report with a fixed ABI

**Files:**

- Create: RoboBeetleFirmware/Core/Diagnostics/motion_timing_diagnostics.h
- Create: RoboBeetleFirmware/Core/Diagnostics/motion_timing_diagnostics.c
- Modify: RoboBeetleFirmware/CMakeLists.txt
- Modify: RoboBeetleFirmware/tests/run_host_tests.ps1

- [ ] Add the CMake option
  ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS with default OFF. Add the diagnostic
  source and compile definition only when it is ON. Add
  ROBOBEETLE_MOTION_TIMING_REDUCED_TELEMETRY with default OFF; make it
  effective only inside the diagnostics-ON branch. Keep
  ROBOBEETLE_CPG_TARGET_BENCHMARK independent and OFF by default.
- [ ] Define the public fixed ABI in the neutral header. Start the report with
  these 32-bit words in this exact order:

~~~c
uint32_t magic;
uint32_t abi_version;
uint32_t report_size;
uint32_t system_core_clock_hz;
uint32_t diagnostic_flags;
uint32_t runtime_backend;
uint32_t run_marker;
uint32_t reset_marker;
~~~

  Follow them with fixed-width counter groups using 32-bit words and explicit
  `{ uint32_t lo; uint32_t hi; }` totals. Do not use pointers, `size_t`,
  `bool`, bit-fields, variable-length members, or compiler-dependent padding
  as part of the ABI. Include loop, three RX-drain, four TX-class, requested
  elapsed, accepted Motion-tick interval, Motion span, optional meaningful
  millisecond histogram, saturation, and diagnostic-wrap fields. Define the
  histogram bucket order and cycle-distribution no-histogram rule in the
  header.
- [ ] Add C11 `_Static_assert` checks for the expected `sizeof` and critical
  `offsetof` values: every header word, the first counter group, Motion gap
  counters, and the final report size. Increment the ABI version whenever the
  layout changes.
  ABI v3 is 1300 bytes with top-level offsets
  `app_loop_body=32`, `app_loop_interval=92`, `rx_drain=152`, `tx=368`,
  `motion=704`, saturation=1160, counter-wrap=1164, and invalid=1168;
  `run_state=1172`, `termination_reason=1176`, and
  `worst_gap_context=1180`. Millisecond-valued distributions use the fixed
  inclusive buckets `<=10`, `11..20`, `21..50`, `51..100`, `101..500`,
  `501..1000`, `1001..5000`, `>5000` in milliseconds. Cycle-valued
  distributions retain count/min/max/total/worst only; their histogram entries
  remain zero and no cycle median/percentile is claimed. A zero
  `SystemCoreClock` increments the invalid counter, retains the raw cycle
  interval, and does not classify millisecond gap thresholds.
- [ ] Implement initialization only in the ON path. Enable
  CoreDebug->DEMCR.TRCENA, clear DWT->CYCCNT, enable
  DWT_CTRL_CYCCNTENA_Msk, and use barriers matching the existing
  cpg_target_benchmark.c implementation.
- [ ] Use 32-bit unsigned subtraction for each short span and explicit
  lo/hi accumulators for totals. Do not keep a span open for 25.56 seconds at
  168 MHz, and expose any saturating/invalid condition in the report.
- [ ] Implement fixed distributions as count/min/max/total plus a meaningful
  fixed histogram only for millisecond-valued fields and, for interval
  measurements, a worst-interval field. Cycle-valued fields retain no
  histogram and expose an average derived from total/count. Do not calculate
  or store an exact streaming median. Record Motion actual interval evidence
  as max gap and strict `>10`, `>12`, `>15`, `>20`, and `>30` ms counters;
  threshold equality does not increment the bucket.
- [ ] Keep all report updates in foreground code. Do not allocate, print,
  transmit, enqueue, dequeue, disable Safety, or inspect a queue from the
  diagnostic module.
- [ ] Implement compile-time no-op macros or inline functions for OFF so that
  normal firmware has no DWT setup, no report traffic, and no extra UART
  traffic.
- [ ] Add compile contracts for all four option combinations. The default
  `ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS=OFF` and
  `ROBOBEETLE_MOTION_TIMING_REDUCED_TELEMETRY=OFF` build must not reference
  DWT or diagnostic symbols. A reduced-telemetry-only definition without
  diagnostics must compile as normal telemetry behavior, not as reduced mode.
- [ ] Turn the Task 2 tests GREEN and run the complete host gate. Expected
  result: the new arithmetic test and all existing tests pass, with the final
  runner summary still reporting all expected cases.

## Task 4: Add hooks and the diagnostic-only reduced-telemetry condition

**Files:**

- Modify: RoboBeetleFirmware/Core/App/app_main.c
- Modify: RoboBeetleFirmware/Core/Motion/motion_manager.c
- Modify: RoboBeetleFirmware/Core/Communication/uart_transport_stm32.c only
  if a transport-level span is required; do not change its HAL API or
  blocking behavior
- Create or modify: RoboBeetleFirmware/tests/app_main_timing_diagnostics_tests.c
- Modify: RoboBeetleFirmware/tests/run_host_tests.ps1

- [ ] Write integration regressions first for the hook contract:
  - an app pass records one loop sample;
  - host, JY901S, and Depth pop loops record exactly the bytes they already
    pop;
  - an empty queue records zero popped bytes without changing parser behavior;
  - ACK, LEAK, IMU, and DEPTH transmissions are classified at their existing
    app_main call sites;
  - a Motion tick is counted only when the existing elapsed gate accepts it;
  - a below-10 ms process call is not counted as a Motion tick;
  - a Safety/liveness abort does not invoke generator or Servo timing hooks.
- [ ] Write a RED compile/runtime regression for
  ROBOBEETLE_MOTION_TIMING_REDUCED_TELEMETRY: when diagnostics and reduced
  mode are ON, only optional Leak/IMU/Depth telemetry is suppressed; Heartbeat
  receive, Heartbeat ACK, SafetySupervisor, host-liveness timeout, actuator
  fail-safe behavior, and Protocol command dispatch remain active. When either
  diagnostics or reduced mode is OFF, the existing normal telemetry path is
  unchanged.
- [ ] Add ON-only app_main entry/exit span around the current
  app_main_process() body. Preserve the current order:
  JY901S poll, Depth poll, leak sample, USART1 drain, USART3 drain, USART6
  drain, HAL_GetTick, Safety, Motion.
- [ ] Add ON-only spans around each existing while-pop drain. Increment the
  byte counter in the same loop iteration in which the existing pop succeeds.
  Do not add a byte/time budget or alter the loop condition.
- [ ] Add ON-only start/finish calls around the existing
  uart_transport_stm32_transmit() calls in protocol_send_ack(),
  protocol_send_leak_status(), protocol_send_imu_snapshot(), and
  protocol_send_depth_snapshot(). Record the returned HAL status and length.
  Do not classify by parsing COBS bytes and do not change the wire frame.
- [ ] Add ON-only hooks in motion_manager_process() immediately around the
  existing motion_manager_tick(manager, elapsed_ms) call after
  last_tick_ms = now_ms. Do not move that assignment or alter the elapsed
  gate. Capture both elapsed_ms and the tick-to-tick DWT interval.
- [ ] If nested spans are added, place them only around the existing generator
  advance, sample, and target-application calls. Do not add a second advance,
  sample, Servo write, or scheduler pass.
- [ ] Add the diagnostic-only trial lifecycle: initialize an IDLE report at app
  startup; call `begin_run(current_backend)` only after a successful Motion
  START accepted from STOPPED; clear counters and marks, increment run/reset
  markers, and begin recording without resetting DWT or starting Motion. Freeze
  at accepted normal STOP or immediate Safety/fault termination, retain the
  termination reason, and make all later hooks no-ops until the next trial.
- [ ] Add a fixed RAM-only accumulator between accepted Motion ticks for the
  three RX byte/cycle classes and four TX call/byte/cycle classes. Snapshot it
  only when a new worst gap is observed, together with interval cycles/ms,
  requested elapsed_ms, backend, and previous tick duration, then reset it for
  the next interval. Do not allocate, print, transmit, or alter scheduling.
- [ ] Confirm the OFF compile path removes all hook work and that the original
  host tests still pass unchanged. The new integration test must pass in both
  OFF and ON host compile-contract configurations.

Expected result: instrumentation observes the existing path. It cannot alter
Motion state, last_tick_ms, generator state, Servo output, queue contents,
Safety action, UART status, or telemetry policy.

## Task 5: Add fixed-ABI target symbol readout without a telemetry surface

**Files:**

- Create: tools/read-motion-timing.ps1
- Modify: RoboBeetleFirmware/README.md only if a concise diagnostic runbook
  link is needed after review

- [ ] Write a script that accepts an ELF path, report symbol name, and a
  debugger/OpenOCD command configuration. Resolve the report with
  arm-none-eabi-nm and fail if the symbol or tool is absent.
- [ ] Keep target connection/read commands explicit and visible. Read the
  fixed report memory, validate `magic`, `abi_version`, and `report_size`
  before decoding any other field, require `run_state=FROZEN`, then decode the fixed offsets from the
  public ABI and write raw counters to a local file with branch SHA and trial
  metadata. The helper accepts caller-supplied trial id, workload, diagnostic
  condition, toolchain, CMake generator, build type, linker script, and
  image-size metadata so each raw report remains tied to its exact target
  cell. ABI mismatch must fail loudly; PowerShell must never infer C offsets
  from the symbol address.
- [ ] Refuse to fall back to UART, Protocol V2, printf, or a host executable
  when target tools are missing. A debugger Watch/Expressions read of the
  same volatile symbol is the documented fallback.
- [ ] Require the live helper caller to finish the exercise, send normal Motion
  STOP (or use an already-recorded Safety/fault termination), and allow the
  actuator-safe stop path before halting for readout. After dumping, issue
  `reset run` so the target returns to the normal stopped startup state. The
  helper must not send a diagnostic UART command or use active-motion halt as
  the measurement-ending mechanism.
- [ ] Add a dry-run or argument-validation check that proves missing
  arm-none-eabi-nm, OpenOCD, ELF, and symbol conditions fail loudly.

Expected result: the script is a readout helper only. It does not change
firmware scheduling or add a new command/telemetry message.

## Task 6: Build two diagnostic images and run the four-cell runtime-selector matrix

**Files/configuration:**

- Build from: RoboBeetleFirmware/CMakeLists.txt
- Record raw output in an external build/results directory, not in the
  repository source tree

- [ ] Verify the target tools first:

~~~powershell
Get-Command arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue | Select-Object Source
Get-Command arm-none-eabi-size.exe -ErrorAction SilentlyContinue | Select-Object Source
Get-Command openocd.exe -ErrorAction SilentlyContinue | Select-Object Source
~~~

If any required tool is unavailable, record the exact NOT_FOUND result and
leave target measurements NOT RUN. Do not substitute host timing.

- [ ] Build one NORMAL diagnostic ELF with
  ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS=ON,
  ROBOBEETLE_MOTION_TIMING_REDUCED_TELEMETRY=OFF, and
  ROBOBEETLE_CPG_TARGET_BENCHMARK=OFF. Program this one ELF once, then use
  the existing runtime `SetGaitBackend` selector while STOPPED for A (CPG)
  and B (SimpleGait). Do not define
  MOTION_DEFAULT_GAIT_BACKEND_CPG=0/1 for the experiment and do not override
  CMAKE_C_FLAGS.
- [ ] Build one REDUCED_OPTIONAL_TELEMETRY diagnostic ELF with the same
  optimization/linker/toolchain settings and
  ROBOBEETLE_MOTION_TIMING_REDUCED_TELEMETRY=ON. Program this one ELF once,
  then use the same runtime selector while STOPPED for C (CPG) and D
  (SimpleGait). Keep Heartbeat receive/ACK, Safety checks, host-liveness
  timeout, fail-safe actuator behavior, and Protocol command behavior active.
  If the external fixture cannot safely exercise reduced optional telemetry,
  leave C/D NOT RUN rather than changing production policy.
- [ ] Use separate build directories and record commit SHA, toolchain version,
  CMake generator, build type/optimization, linker script, image size,
  SystemCoreClock, diagnostic option values, and backend.

Example configuration shape:

~~~powershell
$toolchain = (Resolve-Path 'RoboBeetleFirmware\cmake\gcc-arm-none-eabi.cmake').Path
cmake -S RoboBeetleFirmware -B $diagBuild -G Ninja "-DCMAKE_BUILD_TYPE=Debug" "-DCMAKE_TOOLCHAIN_FILE=$toolchain" "-DROBOBEETLE_MOTION_TIMING_DIAGNOSTICS=ON" "-DROBOBEETLE_MOTION_TIMING_REDUCED_TELEMETRY=$reduced" "-DROBOBEETLE_CPG_TARGET_BENCHMARK=OFF"
cmake --build $diagBuild
arm-none-eabi-size $diagBuild\RoboBeetleFirmware.elf
~~~

  `$reduced` is `OFF` for the single NORMAL ELF and `ON` for the single
  REDUCED_OPTIONAL_TELEMETRY ELF. Backend selection is sent at runtime after
  the image is programmed; no backend-specific rebuild is permitted.

- [ ] For each runnable cell, perform three fixed-duration trials of 60
  seconds unless a target constraint is recorded. Use the same board,
  actuator power, sensor streams, host command pattern, and normal
  Heartbeat cadence. Complete each trial with normal Motion STOP, wait for the
  safe stop path, then halt/read the frozen report and reset/run the target;
  preserve raw values.
- [ ] Never disable the Heartbeat or Safety path to reduce load. Do not change
  MOTION_GAIT_TICK_MS, UART baud, queue behavior, foreground order, or
  optional telemetry policy in the production image.

Expected result: each completed cell has a raw debugger report, a workload
label, and trial metadata. A host build or isolated CPG benchmark cannot be
substituted for this result.

## Task 7: Correlate logical cadence with physical waveform and hardware

**Files:**

- Read: RoboBeetleFirmware/Core/Servo/servo_driver_stm32.c
- Record results in a future evidence report reviewed separately from this plan

- [ ] During a representative software timing trial, observe at least one
  active PWM channel with a scope or logic analyzer.
- [ ] Record frame period, pulse width, missing/runt pulses, and any safe-stop
  edge behavior. Do not infer waveform state from readable CNT/CCR alone.
- [ ] If software and PWM timing are regular, separately check servo supply
  voltage under load, current/transients, common ground/noise, linkage/load,
  backlash, servo deadband, and servo substitution.
- [ ] Keep PWM physical waveform, HAL tick physical measurement, post-clock gait
  exercise, and Water marked Pending until real measurements are supplied.

Expected result: a physical observation is classified independently from
logical target cadence and from water behavior.

## Task 8: Analyze, document, and verify scope before external review

**Files:**

- Modify or create a future evidence report only after target/physical data
  exists
- No production source changes are authorized by this documentation phase

- [ ] Compare A against B under normal load, then C against D under the same
  reduced-telemetry condition. Report raw count/min/max/total/worst interval
  and millisecond histograms for requested elapsed and Motion interval values.
  Report cycle distributions using count/min/max/total/worst and an average
  derived from total/count; do not claim a cycle median/percentile from the
  millisecond histogram. Include the fixed worst-gap context when available.
- [ ] Use the approved interpretation rules: both backends jitter implies a
  communication/foreground candidate; normal-load gaps that disappear under
  reduced optional telemetry implicate TX/telemetry; stable logical cadence
  plus irregular scope output implicates PWM/electrical/servo/mechanical
  behavior; no correlation means the hypothesis remains unresolved.
- [ ] Do not claim a root cause from source inspection or the isolated CPG
  benchmark alone.
- [ ] Run all required checks:

~~~powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1
git diff --check
git diff --stat
git diff --name-only
~~~

Expected result for this diagnostics implementation commit: the diagnostic
module, observational hooks, host regressions, and readout helper are present;
the Firmware host gate is PASS; target A/B measurements remain NOT RUN when
the ARM/OpenOCD tools are unavailable; and no production Clock/RCC, PWM, Servo
calibration, CPG math, UART architecture, foreground scheduling, queue budget,
or stutter/jitter fix is present.

## Final review gate

- [ ] Confirm the normal production configuration retains
  ROBOBEETLE_CPG_TARGET_BENCHMARK=OFF and
  ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS=OFF and
  ROBOBEETLE_MOTION_TIMING_REDUCED_TELEMETRY=OFF.
- [ ] Confirm the two target images use the same ELF within A/B and C/D, and
  every backend transition was made by the existing runtime selector while
  STOPPED with ACK confirmation.
- [ ] Confirm no diagnostic report is sent over UART and no Protocol V2
  message, Qt surface, telemetry query, or runtime gait behavior is added by
  this investigation.
- [ ] Confirm no safety heartbeat is disabled or bypassed in any runnable
  diagnostic cell.
- [ ] Confirm every report is a single frozen trial: run 2 counters do not
  include run 1, frozen reports are unchanged by later loops/ACKs/reconnects,
  and the worst-gap context contains only the immediately preceding interval.
- [ ] Confirm the branch remains independent from the merged selector branch
  and that no pull request is merged as part of this investigation plan.
