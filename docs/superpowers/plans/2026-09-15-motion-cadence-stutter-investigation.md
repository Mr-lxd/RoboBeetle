# Motion Cadence / Servo Stutter Investigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add compile-gated, RAM-only timing diagnostics and a reproducible target measurement procedure that can distinguish foreground communication delay, generator cost, logical Motion cadence, and physical servo/PWM behavior without changing production scheduling or actuator semantics.

**Architecture:** Keep app_main as the only cooperative scheduler. Add a disabled-by-default STM32F407 DWT counter module under Core/App, place no-op hooks around existing app-loop, queue-drain, host-TX, and accepted Motion-tick boundaries, and expose one debugger-readable volatile report. Use host tests for counter arithmetic and compile contracts, then run identical CPG/SimpleGait target trials under normal and reduced optional-telemetry diagnostic conditions. Treat scope and electrical measurements as separate evidence.

**Tech Stack:** C11, host GCC, existing Firmware host PowerShell runner, STM32F407 CMSIS DWT/CYCCNT, CMake/Ninja, arm-none-eabi toolchain, debugger/OpenOCD symbol readout, Markdown.

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

## Task 2: Write counter arithmetic tests before the diagnostic module

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
- [ ] Add a red host test that the diagnostic-disabled API is a compile-time
  no-op and does not require a DWT or UART symbol.
- [ ] Add the test case to the existing C11, Wall, Wextra, Werror host runner
  with an explicit expected executable name:

~~~powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1
~~~

Expected result while the implementation is absent: the new test is RED. Do
not weaken the test or call the baseline green until Task 3 implements the
specified API.

## Task 3: Implement the disabled-by-default RAM-only DWT report

**Files:**

- Create: RoboBeetleFirmware/Core/App/motion_timing_diagnostics.h
- Create: RoboBeetleFirmware/Core/App/motion_timing_diagnostics.c
- Modify: RoboBeetleFirmware/CMakeLists.txt
- Modify: RoboBeetleFirmware/tests/run_host_tests.ps1

- [ ] Add the CMake option
  ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS with default OFF. Add the diagnostic
  source and compile definition only when it is ON. Keep
  ROBOBEETLE_CPG_TARGET_BENCHMARK independent and OFF by default.
- [ ] Define a fixed-size report containing:
  - report/version/reset marker and SystemCoreClock;
  - loop body and loop-interval statistics;
  - per-transport drain calls, nonempty calls, bytes, and duration totals;
  - per-TX-class calls, bytes, statuses, durations, and timeout/error counts;
  - accepted Motion-tick count, requested elapsed_ms statistics, actual
    tick-to-tick interval statistics, and the five strict gap buckets;
  - Motion tick total duration and nested generator/sample/apply durations when
    their boundaries can be measured without changing control flow;
  - saturation and diagnostic counter-wrap indicators.
- [ ] Implement initialization only in the ON path. Enable
  CoreDebug->DEMCR.TRCENA, clear DWT->CYCCNT, enable
  DWT_CTRL_CYCCNTENA_Msk, and use barriers matching the existing
  cpg_target_benchmark.c implementation.
- [ ] Use 32-bit unsigned subtraction for each short span and wider
  accumulators for totals. Do not keep a span open for 25.56 seconds at
  168 MHz, and expose any saturating/invalid condition in the report.
- [ ] Keep all report updates in foreground code. Do not allocate, print,
  transmit, enqueue, dequeue, disable Safety, or inspect a queue from the
  diagnostic module.
- [ ] Implement compile-time no-op macros or inline functions for OFF so that
  normal firmware has no DWT setup, no report traffic, and no extra UART
  traffic.
- [ ] Turn the Task 2 tests GREEN and run the complete host gate. Expected
  result: the new arithmetic test and all existing tests pass, with the final
  runner summary still reporting all expected cases.

## Task 4: Add hooks at existing boundaries without changing behavior

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
- [ ] Confirm the OFF compile path removes all hook work and that the original
  host tests still pass unchanged. The new integration test must pass in both
  OFF and ON host compile-contract configurations.

Expected result: instrumentation observes the existing path. It cannot alter
Motion state, last_tick_ms, generator state, Servo output, queue contents,
Safety action, UART status, or telemetry policy.

## Task 5: Add target symbol readout without a telemetry surface

**Files:**

- Create: tools/read-motion-timing.ps1
- Modify: RoboBeetleFirmware/README.md only if a concise diagnostic runbook
  link is needed after review

- [ ] Write a script that accepts an ELF path, report symbol name, and a
  debugger/OpenOCD command configuration. Resolve the report with
  arm-none-eabi-nm and fail if the symbol or tool is absent.
- [ ] Keep target connection/read commands explicit and visible. Read the
  fixed report memory, decode the version/clock/backend/workload fields, and
  write raw counters to a local file with branch SHA and trial metadata.
- [ ] Refuse to fall back to UART, Protocol V2, printf, or a host executable
  when target tools are missing. A debugger Watch/Expressions read of the
  same volatile symbol is the documented fallback.
- [ ] Add a dry-run or argument-validation check that proves missing
  arm-none-eabi-nm, OpenOCD, ELF, and symbol conditions fail loudly.

Expected result: the script is a readout helper only. It does not change
firmware scheduling or add a new command/telemetry message.

## Task 6: Build four diagnostic images and run the A/B matrix

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

- [ ] Build A and B with
  ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS=ON,
  ROBOBEETLE_CPG_TARGET_BENCHMARK=OFF, and identical normal communication
  load. Use MOTION_DEFAULT_GAIT_BACKEND_CPG=1 for A and =0 for B.
- [ ] Build C and D with the same two backend settings and the reviewed
  reduced-optional-telemetry diagnostic condition. Keep Heartbeat receive/ACK,
  Safety checks, host-liveness timeout, and fail-safe actuator behavior active.
  If the scope restriction forbids diagnostic telemetry suppression, leave C/D
  NOT RUN and use an externally reviewed load fixture instead.
- [ ] Use separate build directories and record commit SHA, toolchain version,
  CMake generator, build type/optimization, linker script, image size,
  SystemCoreClock, diagnostic option values, and backend.

Example configuration shape:

~~~powershell
$toolchain = (Resolve-Path 'RoboBeetleFirmware\cmake\gcc-arm-none-eabi.cmake').Path
cmake -S RoboBeetleFirmware -B $diagBuild -G Ninja "-DCMAKE_BUILD_TYPE=Debug" "-DCMAKE_TOOLCHAIN_FILE=$toolchain" "-DROBOBEETLE_MOTION_TIMING_DIAGNOSTICS=ON" "-DROBOBEETLE_CPG_TARGET_BENCHMARK=OFF" "-DCMAKE_C_FLAGS=-DMOTION_DEFAULT_GAIT_BACKEND_CPG=$backend"
cmake --build $diagBuild
arm-none-eabi-size $diagBuild\RoboBeetleFirmware.elf
~~~

- [ ] For each runnable cell, perform three fixed-duration trials of 60
  seconds unless a target constraint is recorded. Use the same board,
  actuator power, sensor streams, host command pattern, and normal
  Heartbeat cadence. Halt/read the report after each trial and preserve raw
  values.
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
  reduced-telemetry condition. Report raw min/median/max or fixed-window
  percentile values for loop intervals, each drain, TX class, Motion tick
  intervals, requested elapsed_ms, and Motion spans.
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

Expected result for this design-only commit: only the investigation spec and
plan are changed, Firmware host gate is PASS, and no production source,
Clock/RCC, PWM, Servo calibration, CPG math, UART architecture, foreground
scheduling, queue budget, or stutter/jitter fix is present.

## Final review gate

- [ ] Confirm the normal production configuration retains
  ROBOBEETLE_CPG_TARGET_BENCHMARK=OFF and
  ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS=OFF.
- [ ] Confirm no diagnostic report is sent over UART and no Protocol V2
  message, Qt surface, telemetry query, or runtime gait behavior is added by
  this investigation.
- [ ] Confirm no safety heartbeat is disabled or bypassed in any runnable
  diagnostic cell.
- [ ] Confirm the branch remains independent from the merged selector branch
  and that no pull request is merged as part of this investigation plan.
