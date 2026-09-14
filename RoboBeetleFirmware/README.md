# RoboBeetleFirmware

RoboBeetleFirmware is the current STM32F407VET6 Phase 1 firmware for the Qt Console → Windows COM13 → DAP UART/USB serial bridge → STM32 USART1 → Protocol V2 host-link, the five-servo semantic descriptor path, the PR #9 leak-status telemetry path, the PR #11 low-rate JY901S telemetry path, and the Motion / CPG foundation with a completed SimpleGait mechanical baseline. This README records the merged hardware-verified modularization baseline, the PR #8 wiring baseline, the PR #9 leak-status hardware acceptance, the PR #10/PR #11 JY901S evidence boundaries, the PR #13 final four-paddle calibration contract, the SimpleGait Front/Rear anti-phase hardware result, and the bench-provisional STOP contract. The JY901S physical receive and end-to-end monitoring path and the SimpleGait mechanical baseline are hardware verified within their stated boundaries; this feature image's ARM Build, Program Verify, and physical CPG gait statuses remain **[Pending]**. Separate ROVMAKER depth-sensor calibration, USART3 UART/checksum physical-link quality, body-frame mapping, water propulsion, and final magnetic/yaw calibration remain pending.

The recent Servo, LeakStatus, and JY901S hardware runs used the wired DAP UART/COM13 host path above. APC220 is an earlier/legacy transport record, was not enabled in those runs, and is not current JY901S or PR #11 hardware evidence.

## Status labels

- **[Implemented]** Confirmed in current source or active `.ioc`.
- **[Implemented / Software Verified]** Confirmed in current source and host-side software checks; this label does not claim target hardware execution.
- **[Hardware Verified]** Reported in the current development record; source alone cannot prove physical execution.
- **[Bench Hardware Calibrated]** User-provided or bench actuator measurements; this does not prove the current Firmware image was built, programmed, or exercised.
- **[Provisional]** Bring-up values or incomplete calibration.
- **[Planned]** Recommended future work, not current behavior.
- **[Historical Reference]** Old F407ZE, STM32, Simulink, CPG, paper, slide, or resource-tree material that is not the current firmware.

The current recent Servo, LeakStatus, and JY901S hardware runs used the wired host path Qt Console → Windows COM13 → DAP UART/USB serial bridge → STM32 USART1 at 9600 8-N-1. APC220 is an earlier/legacy transport profile and was not enabled or used in those runs; it is not current JY901S or PR #10/PR #11 hardware evidence.

## Current five-servo semantic descriptor contract (PR #13 values; 2026-09-14 assembly remap)

The current implementation freezes five semantic IDs and the supported mask at `0x001F` (bits 0–4). Firmware and Qt maintain independent descriptor tables; host tests and pure-C tests assert the same IDs, masks, capabilities, and calibration envelopes so descriptor drift is detected without crossing the C/C++ boundary.

| ID / mask | Semantic actuator | Hardware / timer channel | Capability and command envelope |
|---:|---|---|---|
| `0` / `0x0001` | `FrontRight` | SAVOX SW-0250MG+, TIM3_CH2 / PA7 | command PWM 1140–1860 μs; calibration −45…+45° = 1140/1580/2020 μs; Neutral 1580 μs |
| `1` / `0x0002` | `FrontLeft` | SAVOX SW-0250MG+, TIM3_CH1 / PA6 | command PWM 1160–1900 μs; calibration −45…+45° = 1900/1450/1000 μs; Neutral 1450 μs; logical-angle-reversed PWM |
| `2` / `0x0004` | `Depth` (`FrontAxis` internal ID) | HDKJ S3150D, TIM3_CH3 / PB0 | Calibration 2430/1745/1060 μs; software PWM 1060–2430 μs; software angle −90…+90°; Neutral 1745 μs; logical-angle-reversed PWM; Console `calibrationPending=false` |
| `3` / `0x0008` | `RearRight` | GDW IPX896HV, TIM4_CH1 / PD12 | PWM 1110–2030 μs; angle −45…+45°; calibration 1110/1570/2030 μs; Neutral 1570 μs |
| `4` / `0x0010` | `RearLeft` | GDW IPX896HV, TIM4_CH2 / PD13 | PWM 960–1940 μs; angle −45…+45°; calibration 1940/1450/960 μs; Neutral 1450 μs; angle-inverted PWM |

TIM3 and TIM4 run at approximately 333 Hz with a 1 μs tick (PSC=15, ARR=3002). `servo_descriptor` is pure C and HAL-independent: it stores abstract timer/channel selectors, never `TIM_CHANNEL_x` constants. `servo_driver_stm32` is the only layer that maps those selectors to `TIM_HandleTypeDef *` and HAL channel values.

The four paddle servos share one logical convention: `0 degrees` is mechanical neutral, `+45 degrees` is the backward paddle stroke that produces forward propulsion, and `-45 degrees` is the opposite direction. FrontRight is `-4500/0/+4500 cdeg → 1140/1580/2020 us`; FrontLeft is `-4500/0/+4500 cdeg → 1900/1450/1000 us`; RearRight is `-4500/0/+4500 cdeg → 1110/1570/2030 us`; RearLeft is `-4500/0/+4500 cdeg → 1940/1450/960 us`. FrontLeft and RearLeft mappings intentionally decrease PWM as logical angle increases; raw PWM validation still uses the independent ascending numeric bounds shown in the table. The calibration angle envelopes remain `-4500..+4500 cdeg`; MotionManager separately enforces the installed operational envelope `-4500..+2800 cdeg` for both front channels and `-3000..+4500 cdeg` for both rear channels.

`FrontAxis/Depth` retains its raw envelope `1060–2430 us` and Neutral `1745 us`, with logical calibration `2430 us = -90 degrees`, `1745 us = 0 degrees`, and `1060 us = +90 degrees`. The desk-only mechanical direction `+10 degrees` downward / `-10 degrees` upward is **[Bench Mechanical Verified]**; post-fix end-to-end re-verification remains **[Pending Hardware Verification]**. This is the bench actuator, not the separate ROVMAKER depth sensor. It does not establish hydrodynamic optimization, installed trim, autonomous depth-control calibration, magnetic/yaw calibration, or final body-frame calibration. Waterproof capability is **[Unverified]**: the seller parameter page says “not waterproof,” while the product photo/shell says “Water proof Robot Servo.” Do not claim or test direct immersion without reliable IP/sealing evidence.

PR #13 evidence is deliberately retained as a pre-remap historical baseline: former logical FrontRight `1450/1900 us` are **[Bench Measured]** and `1000 us` is **[Symmetry-Derived / User Accepted]**; former logical FrontLeft `1580/1140 us` are **[Bench Measured]** and `2020 us` is **[Symmetry-Derived / User Accepted]**; RearRight `1110/1570/2030 us` and RearLeft `1940/1450/960 us` are **[Bench Hardware Verified]** user bench results. The final front calibration contract and raw shell command bounds are **[Software Verified]** by host tests, the SimpleGait Front/Rear physical anti-phase is **[Hardware Verified]** as the mechanical baseline, and the prior same-direction anomaly is **[Closed for SimpleGait]**. Explicit logical-to-physical side identity, Turn effectiveness, true forward propulsion, water behavior, physical CPG gait, ARM Build, and Program Verify remain **[Pending]** in their respective evidence categories. Do not copy prior PR or old-image PASS into this feature status.

The current Motion/gait foundation is documented in
[`../docs/motion-simple-gait.md`](../docs/motion-simple-gait.md). It emits
logical joint targets and passes them through `MotionManager →
CPG or SimpleGait generator → ServoService Motion-owned angle API → Servo
Calibration → PWM`. Neutral differences, mechanical remap, and PWM conversion
stay in the Servo descriptor/calibration layer. Closed-loop feedback and
water-tested gait calibration remain outside this feature. See
[`../docs/cpg-gait-core.md`](../docs/cpg-gait-core.md) for the production CPG
contract.

## Current Motion / CPG foundation — [Implemented / Software Verified]

Current normal bench image configuration:

```text
Motion backend: CPG (next target validation)
SimpleGait diagnostic baseline: Front/Rear physical anti-phase [Hardware Verified]
```

`Core/App/app_main.c` defaults `MOTION_DEFAULT_GAIT_BACKEND_CPG` to `1`, so
the normal Debug/bench build passes only
`cpg_gait_generator_interface(&cpg_gait_generator)` to `MotionManager`.
The explicit `-DMOTION_DEFAULT_GAIT_BACKEND_CPG=0` override reproduces the
completed SimpleGait mechanical baseline. The CPG sources remain linked and
all CPG and SimpleGait tests remain required; each build registers exactly one
generator. The separate `ROBOBEETLE_CPG_TARGET_BENCHMARK` option remains OFF
by default. See
[`../docs/simple-gait-bench-remap-2026-09-14.md`](../docs/simple-gait-bench-remap-2026-09-14.md).

Protocol V2 `SetMotionMode` (`0x15`) uses the exact three-byte payload
`schema=1, mode, action`. The stable mode order is `STOP`, `FORWARD`,
`BACKWARD`, `TURN_LEFT`, `TURN_RIGHT`, `ASCEND`, `DESCEND`; `STOP` uses the
STOP mode plus STOP action. The wire enum keeps `BACKWARD` for compatibility,
but the current bench SimpleGait implementation accepts `FORWARD`,
`TURN_LEFT`, `TURN_RIGHT`, `ASCEND`, and `DESCEND` only. `BACKWARD` is reserved
pending bench/water verification and is rejected by the generator and Qt
Console.

Ordinary STOP is graceful: the dispatcher returns its successful ACK when the
stop request is accepted, `MotionManager` enters `MOTION_STOPPING`, and the
cooperative foreground processing interpolates the retained logical targets to
neutral over the centralized `MOTION_TRANSITION_DURATION_MS=750U` provisional
duration using the actual wrap-safe elapsed time from the STOP acceptance
timestamp. Time before STOP acceptance is not consumed by the ramp.
Motion ownership remains held throughout the ramp, so manual
Enable/SetPWM/SetAngle/Neutral is `BUSY`; the final zero write releases Motion
ownership and enters `MOTION_STOPPED`. Disable All, an explicit Disable whose
validated mask intersects active Motion ownership, and SafetySupervisor
host-liveness failure abort immediately without waiting for the ramp; a
non-intersecting single-channel Disable remains allowed without preempting
Motion.

ServoService tracks the logical angle of each enabled channel. Enable starts a
new channel at logical neutral, SetAngle/Neutral/Motion writes update the
tracker, and raw SetPWM marks that channel's pose unknown. Motion START refuses
an unknown required pose through the existing internal
`MOTION_MANAGER_RESULT_HARDWARE_FAILURE` mapping, so Protocol V2 keeps its
existing result values. A known non-neutral pose is cross-faded to the gait
target over the same 750 ms transition. The selected generator emits logical
targets only; MotionManager applies the common front operational guard
(`-4500…+2800 cdeg`) and rear operational guard (`-3000…+4500 cdeg`) before
Servo calibration and owns its diagnostic count.

The PA11 leak path remains monitoring-only in the current source and has no
leak-to-Safety trip. If a future leak safety trip is added, it must call the
same immediate Motion abort plus actuator-disable path; this feature does not
invent a new leak safety policy. Interrupted Motion never auto-resumes after
heartbeat recovery or reconnect; explicit Servo re-enable and a new START are
required. See the full contract and host commands in
[`../docs/motion-simple-gait.md`](../docs/motion-simple-gait.md).

### PWM Disable safe-stop — [Implemented / Host-Tested]

`Neutral` continues legal calibrated PWM output and records logical `0 degrees`.
`Disable` and `Disable All` stop PWM drive without writing Neutral; logical
ownership and Motion/Safety ownership are cleared immediately. On TIM3/TIM4,
PWM mode 1 is active-high and up-counting, with HAL `OCxPE` preload enabled.
For any logically active channel with a running timer, the driver never uses
readable `CNT`/`CCR` ordering as proof that the current output is LOW: it
clears stale `CCxIF`, marks only that channel `stop_pending`, enables its
`CCxIE`, retains `CCxE`, and waits for the next real compare event. It re-reads
`CNT`, `CCR`, and `CCxIF` after arming for race diagnostics, but only `CCxIF`
can authorize same-edge finalization. The existing HAL compare callback then
performs the final `HAL_TIM_PWM_Stop()` after the falling edge, preserving HAL
channel state. A timer that is not running, or an already inactive channel,
may finalize immediately. The HAL's `__HAL_TIM_DISABLE()` keeps a shared timer
counter running while any other `CCxE/CCxNE` output remains enabled, so
stopping one channel does not stop its siblings.

Pending-stop channels reject SetAngle, ApplyPWM, Motion writes, and Enable with
`BUSY`; repeated Disable is idempotent. At the current `PSC=15`, `ARR=3002`,
1 microsecond-tick configuration, the physical shutdown wait is at most one
complete approximately 3 ms PWM frame plus compare-ISR latency, not the
separate 750 ms graceful Motion STOP. Safety events interrupt graceful Motion
immediately and may use only this frame-level safe edge for physical shutdown.
Host tests prove the decision/state policy, not waveform behavior. The current
hardware evidence remains `Physical no-jump Disable: Pending Hardware
Re-verification`; the dedicated host regression models a preload/shadow
mismatch (readable preload 1000 μs while the current shadow pulse is 1900 μs)
and requires deferral. An optional logic-analyzer check must find a complete
final pulse or a stop already in the LOW window, never a runt pulse.

Enable accepts a multi-bit mask only with all-or-nothing semantics. Requested channels already present in the pre-call enabled mask are idempotent and receive no pulse write, start, or stop. If any newly requested channel fails to start, only channels newly started by that call are stopped and the pre-call enabled state—including the physical pulse of an already-running channel—is preserved. Disable and Disable All retain fail-closed/best-effort stop behavior.

This is a hardware-layout compatibility break. The 2026-09-14 front assembly remap keeps logical IDs stable but binds logical FrontRight to the former FrontLeft physical actuator/channel and logical FrontLeft to the former FrontRight channel; FrontAxis keeps TIM3_CH3 while reversing logical-angle calibration. Historical v0.4 `Servo1`/PA6 bring-up referred to `RearLeft`; PR #8 formally assigns PA6/ID 0 to `FrontRight` and assigns `RearLeft` to PD13/TIM4_CH2. Do not mix a pre-PR8 or pre-remap Console/Firmware binary with the current harness. The Qt `Servo1` name is only a deprecated source-compatibility alias for `FrontRight`; new firmware code uses semantic names. See `../docs/front-assembly-remap-2026-09-14.md` for the exact current table and hardware checklist.

PR #13 software descriptor, calibration, service, driver, dispatcher, and Console parity regressions are the implementation gate. For this feature, ARM Build: **[Pending]**; Program Verify: **[Pending]**; Hardware Verified: **[Pending]**. The earlier Servo1-only hardware milestones remain historical evidence for the old PR #8 wiring/layout.

## PR #9 leak-status telemetry: leak D0 on PA11 — [Hardware Verified]

The first sensor phase adds a polled digital leak input and a monitoring-only
Protocol V2 telemetry path. The module is
powered from 3.3 V with common GND; its digital output `D0` is wired to
STM32 `PA11`, while analog `A0` is intentionally unused. The current path is:

```text
leak D0 → PA11 GPIO input → leak_sensor_stm32 raw reader
  → leak_sensor pure-C mapper → app_main internal state
  → LeakStatus `0x20` telemetry after an accepted Heartbeat ACK
  → Qt leak indicator
```

`PA11` is configured in the existing `MX_GPIO_Init()` path as
`GPIO_MODE_INPUT` with `GPIO_NOPULL`; there is no EXTI, debounce, alarm, or
Safety action. The module's output-stage type is not fully established by the
available documentation, so `GPIO_NOPULL` is a bring-up assumption rather than
a verified electrical conclusion. A source-level resource scan checked PA11
against the active `.ioc`, `main.c`, HAL MSP, USART1, TIM3/TIM4, SWD, and
existing GPIO assignments and found it free at the software resource level. The
initial polarity is PA11 HIGH → `LEAK_SENSOR_STATE_DRY` and PA11 LOW →
`LEAK_SENSOR_STATE_WET`.

Firmware emits `LeakStatus` (`0x20`) as one unacknowledged byte (`0=UNKNOWN`,
`1=DRY`, `2=WET`) only after an accepted Heartbeat has had its normal ACK
fully transmitted. The telemetry has an independent sequence space and is
published on the first valid sample, on state change, or at most once per
500 ms refresh interval; it has no independent transmit timer and never
changes Servo or Safety state. The Console returns to `Unknown` on disconnect,
host-link liveness loss, invalid payload, or a stale telemetry interval of 1500 ms
(three 500 ms Firmware refresh opportunities; provisional).

For the PR #9 LeakStatus change, the pure-C mapper/policy, Protocol V2 vectors,
and Qt/controller regressions are **[Host Test: PASS]**. Its recorded current
STM32CubeIDE Debug ARM configure/build is **[ARM Build: PASS]**, and that PR #9
ELF was programmed and verified with **[Program Verify: PASS]**. These rows are
PR #9 evidence, not PR #11 image evidence. Physical acceptance is now recorded
as:

| Acceptance item | Status |
|---|---|
| PA11 leak detection path | **[Hardware Verified]** |
| Protocol V2 `LeakStatus (0x20)` real-link exchange | **[Hardware Verified]** |
| Qt Leak indicator | **[Hardware Verified]** |
| End-to-end Leak monitoring | **[Hardware Verified]** |
| Leak Safety response | **[Pending / Not Implemented]** |

The earlier persistent `Leak: Unknown` observation was caused by programming/build
artifact provenance. After rebuilding the correct current ELF and programming and
verifying that image, the complete PA11 → Firmware → Protocol V2 → Qt path worked
as intended. No numeric voltage or response-time values are asserted here because
they were not part of the recorded acceptance result.

At the PR #9 closeout, the remaining sensor sequence was JY901S IMU →
depth/sensor board; the current JY901S phase is documented below. LeakStatus
remains monitoring-only and is not connected to Servo or Safety actions.

## JY901S listen-only bring-up — Hardware Verified / UART quality follow-up open

This phase adds only the receive and parser path:

```text
JY901S TX
  → PB11 / USART3_RX
  → one-byte interrupt receive
  → independent 256-byte ring buffer (255-byte effective capacity)
  → pure-C 11-byte parser
  → internal Acc / Gyro / Angle state
```

JY901S RX is connected to PB10 / USART3_TX in the hardware design. PB10 is
configured for the UART path, but this application does not send baud-rate,
output-rate, output-mask, save, restart, calibration, or any other JY901S
command. STM32 USART3 is locally configured for 9600 baud, 8-N-1, TX/RX, and
no hardware flow control so the firmware can listen to the sensor's current
persistent/default configuration. The bench expectation is approximately
10 Hz with Acc, Gyro, Angle, and possibly Mag frames.

The parser decodes WIT standard types `0x51` Acc, `0x52` Gyro, and `0x53`
Angle using signed little-endian values and the documented engineering-unit
scalings. A checksum-valid `0x54` Mag frame is known-but-ignored: it increments
`mag_frame_count`, preserves all supported state, and does not increment
`unsupported_frame_count`. A checksum-valid genuinely unknown type increments
`unsupported_frame_count` and also preserves supported state. The frame
contract follows the [official WIT standard protocol](https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol);
the product page lists 9600 baud and the default output expectations.

USART3 diagnostics are available through the bring-up accessors and include
`rx_byte_count`, successful ring pushes, foreground pops, ring overflow/drop,
hard re-arm failures, deferred `HAL_BUSY` re-arms, aggregate and per-flag UART
errors, parser header starts, valid checksum frames, checksum errors,
Mag-known-ignore frames, unsupported types, per-domain frame counts, and the
last valid-frame tick. The RX-complete and error callbacks only record bytes,
errors, and ownership state; `app_main_process` performs at most one
non-blocking re-arm attempt per poll. `HAL_BUSY` remains deferred and does not
increment the hard-failure counter; `HAL_ERROR` and other non-success statuses
do. FE/NE/PE preserve an active HAL receive, while ORE/DMA/unknown error states
enter the foreground recovery path. A generation check protects the foreground
state transition from a callback or error event that arrives while HAL is
starting the next receive, so a newer pending event cannot be cleared as stale.
No parser work or retry loop runs in the ISR.

The following is the PR #10 listen-only evidence matrix. Its ARM Build and
Program Verify rows apply to the PR #10 bring-up ELF; they do not silently
serve as independent PR #11 image evidence. The matching PR #11 Firmware + Qt
run supplies the physical JY901S and end-to-end rows below.

| Gate | Status |
|---|---|
| Host Test | **PASS**: parser, transport mock, ring-buffer, and all current Firmware regressions |
| ARM Build | **PASS**: PR #10 STM32 target build; 0 errors, 0 warnings; RAM 2680 B / 128 KB, FLASH 23260 B / 512 KB |
| Program Verify | **PASS**: PR #10 DAP/OpenOCD programming flow completed and reported `Verified OK` |
| Hardware Verified | **PASS**: matching PR #11 Firmware + Qt run received live JY901S data end to end |
| USART3 physical RX | **Hardware Verified**: PB11 / USART3 receive path and RX byte/ring diagnostics |
| JY901S valid real frames | **Hardware Verified**: valid frames continued with zero overflow |
| Acc/Gyro/Angle real data | **Hardware Verified**: live plausible Acc, stationary near-zero Gyro, responsive Angle |
| Re-arm diagnostics | **Hardware Verified**: matching post-fix runs kept hard re-arm failures at 0; deferred `HAL_BUSY` is separate |
| USART3 UART/checksum physical quality | **Pending / non-blocking**: aggregate UART and checksum errors remain observable; physical source not assigned |
| Pending | Final body-frame mapping, magnetic/yaw calibration, and USART3 physical-link quality follow-up |

On 2026-09-10, the hardware-verification checkout built
`RoboBeetleFirmware.elf` at
`D:\RoboBeetle\RoboBeetleFirmware\build\Debug\RoboBeetleFirmware.elf`
with 0 errors and 0 warnings. The recorded artifact had a last-write time of
2026-09-10 17:07:11. The DAP/OpenOCD flow used SWD 100 kHz, SYSRESETREQ,
halt, program, verify, and reset-run, and reported `Programming Finished`,
`Verify Started`, and `Verified OK`. These facts establish the ARM Build and
Program Verify gates for that recorded image. The subsequent matching PR #11
Firmware + Qt run supplies the separate physical JY901S evidence recorded below.

If no legal frame appears on the bench, first inspect RX bytes, ring-buffer
activity, `0x55` headers, valid/checksum/error counters, and state updates in
that order. Do not add automatic JY901S configuration in response; a separate
configuration/init phase requires evidence that the physical UART is working
but the sensor's current persistent settings are not the expected ones.

Post-fix short hardware regression evidence supplied for the matching PR #11
Firmware and Qt build:

- Run A: RX bytes 43295, headers 4006, valid frames 3831, checksum failures
  175, overflow 0, hard re-arm failures 0, UART errors 180, Mag frames 958,
  unsupported 0.
- Run B: RX bytes 73444, headers 6815, valid frames 6448, checksum failures
  366, overflow 0, hard re-arm failures 0, UART errors 376, Mag frames 1618,
  unsupported 0.

The runs kept RX bytes and valid frames increasing with zero overflow and zero
hard re-arm failures. UART aggregate/subtype and checksum counters remain
observable; this evidence does not assign their physical source or claim a
clean USART3 link.

## PR #11 JY901S low-rate telemetry — Hardware Verified / re-arm follow-up closed

PR #11 consumes the read-only Acc/Gyro/Angle state and diagnostics established
by the listen-only bring-up above. It publishes one fixed `ImuSnapshot`
telemetry frame (`0x21`) through the existing Protocol V2 → STM32 USART1 → DAP
UART/USB serial bridge → Windows COM13 host path
and adds no JY901S configuration, raw UART passthrough, servo/safety linkage,
body-frame transform, EKF, or depth work.

The payload is exactly 56 bytes: schema `0x01`, Acc/Gyro/Angle validity flags,
three groups of signed little-endian fixed-point `int16` values, and the
USART3/parser diagnostics counters. Acc uses mg, Gyro uses 0.1 dps, and Angle
uses 0.01 degrees. A domain whose validity flag is clear is encoded as zero.
The frame uses the existing independent telemetry sequence and is never sent
as an ACK-requiring request.

Firmware evaluates the one-second IMU publication policy only after a normal
Heartbeat ACK has completed. At most one optional telemetry frame is selected
per accepted Heartbeat opportunity. A due LeakStatus `0x20` always preempts
IMU/Depth; when LeakStatus is not due, the scheduler fairly rotates due IMU and
Depth slots. Failed optional transmits are not marked published, so the pending
policy remains retryable. At 9600 8-N-1, the maximum 68-byte IMU wire frame is
within the documented low-rate budget. The current physical host link is DAP
UART/COM13; APC220 remains a legacy/future-separate transport.
With the nominal accepted Heartbeat cadence, the effective ImuSnapshot refresh
is up to approximately 1 Hz; delayed ACK opportunities or pending LeakStatus
refreshes may reduce it, and no independent IMU transmit timer is used.

PR #11 software evidence is recorded separately from PR #10's target evidence:

| Gate | Status |
|---|---|
| Host Test | **PASS**: all current Firmware regressions, telemetry codec/scheduler tests, and Console tests |
| ARM Build | **PASS**: matching PR #11 Firmware build used for the reported hardware run |
| Program Verify | **Pending**: no standalone PR #11 programming/verify record is included in this closeout |
| Hardware Verified | **PASS**: JY901S → USART3/PB11 → ring/parser → Acc/Gyro/Angle → ImuSnapshot → STM32 USART1 → DAP UART/COM13 → Qt |
| Re-arm diagnostics follow-up | **PASS / resolved**: post-fix Run A and Run B both reported hard re-arm failures 0 |
| USART3 UART/checksum physical quality | **Pending / non-blocking**: aggregate/subtype UART and checksum errors remain observable |
| Pending | Final body-frame mapping and magnetic/yaw calibration |

The initial pre-fix hardware snapshot was RX bytes `131663`, headers `11967`,
valid frames `11957`, checksum errors `10`, overflow `0`, UART errors `20`, Mag
frames `2989`, unsupported frames `0`, and displayed `rx_rearm_failure_count`
`166240`. The root cause was diagnostic semantics, not proof that all of those
attempts were hard failures: the old implementation counted every return other
than `HAL_OK` and did not retain the HAL status. In this repository's STM32F4
HAL, `HAL_UART_Receive_IT()` returns `HAL_BUSY` whenever `RxState` is not
`HAL_UART_STATE_READY`, while the normal one-byte `UART_Receive_IT()` path sets
`RxState` to `READY` before invoking `HAL_UART_RxCpltCallback`. Thus a normal
completion callback is not, by itself, evidence of a busy transition, and the
pre-fix aggregate cannot be retrospectively decomposed; a busy result can still
occur when an arm overlaps another active receive or an error/foreground state
transition. PR #11 now lets callbacks only mark pending work, performs one
foreground re-arm attempt per poll, counts `HAL_BUSY` as deferred, and reserves
the hard-failure counter for `HAL_ERROR` and other non-success statuses. A
generation re-check prevents a newer callback/error event from being cleared by
a stale success path.

The post-fix Run A/Run B values are recorded in the listen-only section above.
UART aggregate and per-flag error counters remain observable and are not
silently reset or treated as a parser or Protocol V2 failure. The single Qt
`Invalid length` event remains an observation only: existing Console tests
cover split and sticky/concatenated (back-to-back) frames, while CRC errors and
timeouts remained zero. No wire-format or Qt redesign is justified by that one
event.

The existing fixed 56-byte ImuSnapshot wire format is unchanged. The transport's
deferred `HAL_BUSY` counter is internal; the existing payload field continues to
mean hard re-arm failures only. Final robot body-frame mapping and magnetic/yaw
calibration remain **[Pending]**.

PR #10's STM32 ARM Build and Program Verify remain PASS for the listen-only
bring-up ELF documented in the section above; that evidence does not silently
close the new PR #11 telemetry target gate.

## Current Depth Sensor / ROVMAKER decoder bring-up — Hardware Verified with stable connection; connector/calibration pending

This phase adds a listen-only ROVMAKER decoder-board path without sending any
decoder configuration command:

```text
ROVMAKER decoder board → USART6 / PC7 RX → one-byte interrupt receive
  → dedicated 512-byte ring buffer (511-byte effective capacity)
  → bounded pure-C ASCII parser → internal depth/temperature state
  → DepthSnapshot 0x22 → existing USART1 / DAP UART / COM13 → Qt read-only monitor
```

USART6 is configured as 115200 8-N-1 TX/RX with PC6 TX and PC7 RX. The
application does not send decoder commands, change output rate, or configure
the board. The intended physical topology is the wet pressure face/probe →
sealed hull penetration/threaded installation → pressure hull → cable → dry
ROVMAKER decoder board → PC7/USART6. The stable physical receive and
end-to-end telemetry path is **[Hardware Verified]**. The exact seal/thread
design, production connector retention, strain relief, final installation,
zeroing, density setting, cadence characterization, and absolute accuracy
remain **[Pending]**. The official
[ROVMAKER decoder-board manual](https://docs.rovmaker.cn/产品手册/水深传感器产品手册/深度传感器解算板V1.0.html)
instructs that the board and sensor be powered at the water surface so the
ambient air pressure establishes the zero output. This is vendor guidance, not
final calibration of this robot's installation.

During the stable-connection bench run, Depth status was `Receiving`, values
updated continuously, temperature was approximately 24 °C and plausible,
sample age refreshed, RX bytes and valid lines increased, parse errors stayed
approximately zero/very low, and overflow and hard re-arm failures were zero.
Disturbing the sensor-to-decoder cable/connector caused invalid or lost samples
until the connection was reseated; this is **[Pending mechanical/electrical
integration follow-up]**, not a proven Firmware defect or a production-ready
connector assessment. Final assembly requires connector retention, strain
relief, wiring inspection, sealing as applicable, and a post-assembly
continuity/stability test.

The local `ms5837.py` reference was also inspected. It implements direct
Raspberry Pi I2C access to the MS5837 (PROM/CRC, ADC conversion, compensation,
and density-based depth calculation); it does not establish the decoder UART
grammar, cadence, or electrical levels, and is not imported into Firmware. A
future laptop → network/tether → onboard Raspberry Pi → local serial → STM32
architecture may host ROS 2/high-level functions, but no such path is in this
bring-up.

The parser accepts only these two complete CRLF records:

```text
Depth:<signed two-decimal>m Temp:<signed two-decimal>C\r\n
Depth:<signed two-decimal>m Temp=<signed two-decimal>C\r\n
```

The canonical line is also the exact shape used by the vendor's example, which
uses `Temp=25.27C`; `Temp=` is therefore an explicit documented compatibility
form, not a guessed separator. There is no substring, arbitrary-separator,
bare-LF, or trailing-data fallback. Malformed and overlong lines have separate
counters; a valid line updates both fixed-point fields atomically. The Qt
monitor is read-only and uses local telemetry arrival for Unknown/Receiving/
Stale/Error lifecycle; it does not treat `sample_age_ms` as its only liveness
signal.

Firmware uses the provisional
`DEPTH_TELEMETRY_SENSOR_FRESHNESS_TIMEOUT_MS = 3000` ms sensor-sample bound,
separate from the one-second publication policy and Qt's host-packet stale
timeout. Once that bound expires, depth and temperature validity are cleared and
their wire values are zeroed; transport/parser diagnostics remain observable.
An accepted fresh line restores live validity.

`DepthSnapshot` is message `0x22`, unacknowledged, little-endian, fixed
payload length 38, schema version `1`. The frozen payload is:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | schema `u8 = 1` |
| 1 | 1 | flags: bit 0 depth valid, bit 1 temperature valid; bits 2–7 reserved zero |
| 2 | 4 | `depth_mm` signed `int32` LE; zero when invalid |
| 6 | 2 | `temperature_centi_c` signed `int16` LE; zero when invalid |
| 8 | 2 | `sample_age_ms` `uint16` LE; `0xFFFF` means unknown or saturated |
| 10 | 4 | `rx_byte_count` `uint32` LE |
| 14 | 4 | `valid_line_count` `uint32` LE |
| 18 | 4 | `parse_error_count` `uint32` LE |
| 22 | 4 | `overlong_line_count` `uint32` LE |
| 26 | 4 | `rx_buffer_overflow_count` `uint32` LE |
| 30 | 4 | `hard_rearm_failure_count` `uint32` LE |
| 34 | 4 | `uart_error_count` `uint32` LE |

Firmware evaluates the provisional one-second DepthSnapshot policy only after
an accepted Heartbeat ACK has completed and selects at most one optional
telemetry frame per opportunity. A due LeakStatus always preempts IMU/Depth;
when LeakStatus is not due, the scheduler fairly rotates the due IMU and Depth
slots. Failed optional sends do not mark a policy successful. The frame never
enters command ACK matching, Servo, Safety, or decoder control.

| Gate | Status |
|---|---|
| Host Test | **PASS**: all 17 Firmware executable regressions, `app_main` API syntax, and Console CTest |
| ARM Build | **PASS**: STM32CubeIDE/CMake Debug target build completed with 0 errors / 0 warnings |
| Program Verify | **PASS**: known-good DAP/OpenOCD flow reported `Programming Finished`, `Verify Started`, and `Verified OK` |
| Hardware Verified | **PASS**: stable decoder-board → USART6/PC7 → DepthSnapshot → DAP/COM13 → Qt path; connector/harness robustness remains pending |
| External GitHub Review | **Resolved for PR #12 closeout** |
| Pending | connector retention/strain relief/wiring stability, final installed zero/reference point, freshwater/seawater density calibration, body installation offset, and water-pool accuracy |

## Active target and CubeMX configuration

The active configuration file is `RoboBeetleFirmware/RoboBeetleFirmware.ioc`. A separately referenced `D:\RoboBeetle\RoboBeetle.ioc` was not present during this audit.

| Item | Current configuration |
|---|---|
| MCU | **STM32F407VET6**, LQFP100; build define `STM32F407xx` |
| CubeMX / HAL | STM32CubeMX 6.18.1; STM32Cube FW_F4 1.28.3 |
| Debug | Serial Wire on PA13/SWDIO and PA14/SWCLK |
| System clock | HSI 16 MHz, PLL off; SYSCLK/HCLK/PCLK1/PCLK2 all 16 MHz |
| USART1 | PA9 TX, PA10 RX; 9600 baud, 8 data bits, no parity, 1 stop bit, no flow control, oversampling 16 |
| USART1 NVIC | Enabled; preemption/subpriority 0/0 |
| TIM3 PWM | TIM3_CH1/PA6, TIM3_CH2/PA7, TIM3_CH3/PB0, AF2, PWM mode 1, active high |
| TIM3 timing | PSC=15, ARR=3002, CCR1/CCR2/CCR3 initial=1500 |
| TIM4 PWM | TIM4_CH1 on PD12 and TIM4_CH2 on PD13, AF2, PWM mode 1, active high |
| TIM4 timing | PSC=15, ARR=3002, CCR1/CCR2 initial=1520 |
| PWM result | 16 MHz / (15+1) = 1 MHz counter (1 μs/tick); 1 MHz / (3002+1) ≈ **333.0 Hz** |
| Debug GPIO | PB2 push-pull output labelled `DBG_LED`; initialized low, no runtime toggling in current code |

Do not apply the old STM32F407ZE configuration to this project. The current MCU and pinout are established by the active `.ioc`, generated HAL code, linker/startup files, and CMake define.

## Current runtime architecture

```text
USART1 RX byte
  ↓ USART1_IRQHandler → HAL_UART_IRQHandler
HAL_UART_RxCpltCallback
  ↓ push byte; immediately re-arm 1-byte HAL_UART_Receive_IT
single-producer/single-consumer ring buffer
  ↓ main-loop uart_transport_stm32_pop
Protocol V2 delimiter accumulator
  ↓ COBS + header/length + CRC decode
protocol_dispatcher_handle
  ├─ Heartbeat / host liveness outcome
  ├─ semantic five-servo enable / disable
  ├─ Set Servo PWM
  ├─ Set Servo Angle calibration mapping
  ├─ Neutral semantic command
  ├─ LeakStatus telemetry (after accepted Heartbeat ACK)
  └─ one-entry duplicate suppression / result replay
        ↓
main-loop ACK generation / UART TX
        ↓
servo_service → servo_driver_stm32
        ├─ TIM3_CH1 / PA6 → FrontLeft
        ├─ TIM3_CH2 / PA7 → FrontRight
        ├─ TIM3_CH3 / PB0 → FrontAxis/Depth (bench-calibrated PWM/angle; angle-supported)
        ├─ TIM4_CH1 / PD12 → RearRight
        └─ TIM4_CH2 / PD13 → RearLeft
```

In parallel, the sensor path is:

```text
JY901S TX → PB11 / USART3_RX → one-byte interrupt receive
  → independent 256-byte ring → 11-byte parser
  → Acc/Gyro/Angle state + diagnostics
  → ImuSnapshot telemetry policy → existing USART1 host link
```

The interrupt handler delegates to the HAL. The HAL completion and error callbacks perform only the byte/error accounting and mark the USART3 receive as needing re-arm; foreground transport maintenance makes at most one non-blocking re-arm attempt per poll. Protocol parsing, command dispatch, ACK encoding, blocking UART transmit, and PWM control occur in the main-loop context, not in the UART ISR.

The current communication split is:

- **[Implemented]** `Core/Communication/ring_buffer.c/.h` owns the fixed 128-byte single-producer/single-consumer ring. It reserves one slot (127-byte effective capacity) and silently rejects a push while full, preserving the original behavior.
- **[Implemented]** `Core/Communication/uart_transport_stm32.c/.h` owns the one-byte RX staging byte, USART1 receive interrupt arm/re-arm, ring interaction, main-loop byte retrieval, and the blocking `HAL_UART_Transmit(..., 100U)` wrapper.
- **[Hardware Verified]** `Core/Communication/jy901s_transport_stm32.c/.h` owns the independent USART3/PB11 receive staging/ring path and diagnostics; `Core/Sensors/jy901s_parser.c/.h` owns the pure-C 11-byte decode and Acc/Gyro/Angle state. PR #11 adds the low-rate ImuSnapshot producer without changing the listen-only sensor input.
- **[Hardware Verified]** `Core/App/app_main.c/.h` owns the application orchestration: Protocol V2 wire accumulation and decode integration, ACK/result transmission, diagnostics, module instances, initialization order, RX draining, and post-drain Safety timeout action. It calls existing Protocol, UART, Safety, Servo, and dispatcher modules without implementing their policies or touching TIM registers directly.
- **[Implemented]** `Core/Servo/servo_descriptor.c/.h` owns the pure-C semantic ID, capability, calibration-envelope, and abstract timer/channel table.
- **[Implemented]** `Core/Servo/servo_calibration.c/.h` owns per-descriptor integer angle-to-pulse mapping.
- **[Implemented]** `Core/Servo/servo_service.c/.h` owns supported-mask validation, enabled-state policy, command range checks, Neutral semantics, multi-bit Enable rollback, and driver-independent Servo results.
- **[Implemented]** `Core/Servo/servo_driver_stm32.c/.h` owns the HAL/TIM3/TIM4 channel adapter. It maps abstract descriptor selectors to timer handles and HAL channels and has no Protocol or heartbeat knowledge.
- **[Implemented / Software Verified]** `Core/Motion/motion_manager.c/.h` and `simple_gait_generator.c/.h` own the bench-provisional Motion state machine, logical target generation, cooperative foreground scheduling with wrap-safe wall-time deltas from a 10 ms minimum cadence, Motion Servo ownership, acceptance-time graceful STOP, centralized 750 ms neutral ramp, generator-independent operational limiting, and immediate abort hooks. The modules have no interrupt-driven gait path and no autonomous restart behavior.
- **[Implemented]** `Core/Sensors/leak_sensor.c/.h` owns the HAL-independent UNKNOWN/DRY/WET mapping; `leak_sensor_stm32.c/.h` only reads the configured PA11 GPIO.
- **[Implemented]** `Core/Sensors/leak_telemetry_policy.c/.h` limits LeakStatus publication to first sample/state changes/500 ms refreshes. `Core/App/app_main.c` sends one-byte `0x20` telemetry only after a successful Heartbeat ACK; it does not connect leak state to Safety or Servo behavior.
- **[Implemented / Software Verified]** `Core/Sensors/jy901s_telemetry.c/.h` encodes the fixed 56-byte `ImuSnapshot` payload with explicit little-endian fixed-point fields and bring-up diagnostics. `Core/Communication/imu_telemetry_policy.c/.h` and `telemetry_scheduler.c/.h` keep IMU publication at one second, after completed Heartbeat ACK, with immediate priority for due LeakStatus, fair rotation against a due DepthSnapshot when LeakStatus is not due, and at most one optional frame per opportunity. The IMU path does not enter command/ACK matching or alter USART1 behavior.
- **[Hardware Verified]** `Core/Safety/safety_supervisor.c/.h` owns host liveness, the last valid Heartbeat timestamp, strict timeout evaluation, and one-shot timeout transition reporting. It has no HAL, Protocol, UART, or Servo dependency.
- **[Hardware Verified]** `Core/Communication/protocol_dispatcher.c/.h` owns decoded command payload validation, HostAlive gating, Servo service invocation/result mapping, Heartbeat semantics, and the one-entry successful-command cache. It has no HAL, UART, TIM3, or Console dependency.
- **[Hardware Verified]** `main.c` keeps the CubeMX entry/configuration, `app_main_init`/`app_main_process` calls, and a small UART callback transport delegate. Protocol, Safety, Servo, ACK, diagnostics, and RX-drain orchestration live in `Core/App/app_main.c`.

The Servo service/calibration/driver extraction was **[Historical Hardware Verified]** for the PR #3 old image only. STM32CubeIDE target build passed, ST-LINK download completed with “Download verified successfully”, and the physical Servo regression passed for Connect + Heartbeat, Enable + ACK, Neutral, 0°, ±10°, ±45°, ±90°, Disable, Disable All, Disconnect, reconnect without automatic Enable, and manual Enable + ACK recovery. This confirms the behavior-preserving extraction on that old image; it does not verify the current `feature/servo-calibration-depth-limits` branch/image. For the current feature, ARM Build, Program Verify, and Hardware Verified remain **[Pending]** as stated in the current five-servo section above.

The App/Main extraction was **[Historical Hardware Verified]** for the PR #6 old image only. STM32CubeIDE build PASS, ST-LINK download PASS, and full physical regression PASS covered cold boot/reset without automatic Enable, Connect + Heartbeat, Enable + ACK, Neutral, Set Angle 0/+10/-10/+45/-45/+90/-90 degrees, Set PWM 1520 us, Disable/Disable All, re-enable, Disconnect, the >500 ms safe-disable transition, reconnect without automatic Enable, manual Enable + ACK recovery, and repeated disconnect/reconnect. This evidence does not verify the current `feature/servo-calibration-depth-limits` branch/image; for the current feature, ARM Build, Program Verify, and Hardware Verified remain **[Pending]** as stated in the current five-servo section above.

## UART receive and transmit audit

- **[Implemented]** The current USART1 host-link `ring_buffer` storage is 128 bytes with `uint16_t` head/tail indices. JY901S uses a separate 256-byte storage instance. APC220 is a legacy transport record, not the current host hardware.
- The empty/full distinction reserves one slot, so usable capacity is **127 bytes**.
- On the current USART1 host-link full buffer, `ring_buffer_push()` silently drops the new byte. That existing transport has no overflow flag/counter and no host-visible error; the JY901S transport has independent overflow diagnostics.
- Head and tail remain volatile, with the same one-byte ISR producer / main-loop consumer model as the original implementation.
- The current USART1 `uart_transport_stm32` calls `HAL_UART_Receive_IT()` at startup and re-arms it in the callback; no blocking receive remains. The recent host hardware for this path is DAP UART/COM13; the APC220 profile is historical.
- Return values from the existing USART1 initial and callback receive-arm calls are ignored. USART3/JY901S separates deferred `HAL_BUSY` from hard re-arm failures and recovers from foreground maintenance.
- The transport calls `HAL_UART_Transmit(..., 100U)` only while main-loop dispatch sends an ACK. It is blocking but not ISR-blocking. At 9600 8-N-1 a short ACK frame normally takes milliseconds, yet a stalled transmit can block the loop for up to 100 ms.

## Protocol V2

`rb_protocol_v2.c/.h` is already a HAL-independent C codec containing little-endian helpers, CRC-16/CCITT-FALSE, COBS encode/decode, logical header validation, and wire encoding. It should remain a pure protocol library.

Current constants:

- Magic `52 42`, version `02`
- Header 8 bytes, CRC 2 bytes, payload ≤64 bytes
- `WireFrame = COBS(LogicalFrame) + 00`
- Message IDs: Heartbeat `01`, ACK `02`, Error `03`, Servo Enable `10`, Servo Disable `11`, Set Servo PWM `12`, Set Servo Angle `13`, Neutral `14`, SetMotionMode `15`, LeakStatus `20`, ImuSnapshot `21`, DepthSnapshot `22`

See `../RoboBeetleConsole/docs/protocol.md` for the detailed Console ↔ Firmware matrix. Important current behavior is:

- Heartbeat, ACK, semantic five-servo Enable/Disable, and per-descriptor Set PWM agree with the Console.
- Error is declared but never sent by Firmware.
- Neutral validates liveness/mask/enabled state, writes each descriptor's neutral pulse, and leaves the selected channels enabled.
- Set Angle is available for all five angle-supported descriptors, including the four final logical paddle ranges `-45 to +45 degrees` and FrontAxis/Depth range `-90 to +90 degrees`; each angle is range-checked and mapped with `int32_t` intermediates. FrontRight, FrontAxis, and RearLeft logical inversion comes from signed calibration deltas, not Servo ID special cases.
- LeakStatus `0x20` is a one-byte, unacknowledged monitoring frame (`UNKNOWN=0`, `DRY=1`, `WET=2`). Firmware sends it only after an accepted Heartbeat and completed ACK transmission, on first sample/state change or a 500 ms refresh; it has an independent telemetry sequence and does not trigger Safety or Servo actions.
- ImuSnapshot `0x21` is a fixed 56-byte, unacknowledged monitoring frame. Firmware sends it only after an accepted Heartbeat ACK has completed, at most once per one-second policy interval, with due LeakStatus priority and fair rotation against a due DepthSnapshot when LeakStatus is not due. It uses the independent telemetry sequence and carries explicit little-endian fixed-point Acc/Gyro/Angle values plus JY901S diagnostics; it does not trigger Safety, Servo, or JY901S configuration actions.
- SetMotionMode `0x15` uses `schema=1, mode, action`; successful STOP ACK means request acceptance and `MOTION_STOPPING`, not completed neutral. Ordinary STOP ramps logical targets to zero over `MOTION_TRANSITION_DURATION_MS=750U` while retaining Motion ownership; Safety/Disable All abort immediately.
- ACK result values are frozen as `OK=0`, `InvalidPayload=1`, `HostNotAlive=2`, `UnsupportedServo=3`, `ServoNotEnabled=4`, `OutOfRange=5`, `HardwareFailure=6`, and `Busy=7`.
- Supported mask is exactly `0x001F`. Zero mask is invalid; any unknown bit fails with `UnsupportedServo`. Multi-bit Enable is all-or-nothing with rollback on a channel-start failure.
- The most recent successful non-Heartbeat request is cached by sequence and type. Its retry replays the ACK without executing the Servo action again. Heartbeats refresh liveness but do not evict this cache.

## Heartbeat and safety state

### [Implemented]

- The Safety Supervisor marks the host alive only after a valid four-byte Heartbeat.
- The supervisor stores `last_heartbeat_rx_ms`; `app_main` injects local `HAL_GetTick()`, not the host timestamp.
- Servo Enable, Set PWM, and Set Angle reject commands while the supervisor reports the host not alive.
- If more than 500 ms elapse after the last Heartbeat, the supervisor reports one timeout transition; the main loop stops all enabled PWM channels and clears the enabled mask.
- Reconnection/recovery requires a new valid Heartbeat followed by a new Servo Enable.
- Boot leaves PWM stopped. TIM3/TIM4 are configured with descriptor-specific initial CCR values, but `HAL_TIM_PWM_Start()` is called only on accepted Enable.
- Each accepted Enable writes its descriptor's neutral pulse before starting the requested channels.
- Watchdog timeout also invalidates the duplicate cache, preventing an old successful action ACK from bypassing re-enable after recovery.

### Limitations

- Watchdog processing shares the main loop with blocking ACK transmission and all frame dispatch.
- There is no independent hardware watchdog, fault state, persisted reset reason, leak safety response, battery/current input, or emergency-stop message in this Phase 1 source. Leak D0 is polled into an internal state and exposed through monitoring-only LeakStatus telemetry; it does not change Servo behavior.
- Duplicate suppression intentionally retains one successful non-Heartbeat request rather than a multi-entry replay window. A later distinct successful actuator request replaces it.
- Disconnect safety relies on the host's best-effort Disable All plus the 500 ms Firmware heartbeat timeout.

## Legacy Servo1-only state (Historical Reference)

The following Servo1-only notes describe the pre-PR #8 PA6/RearLeft layout and are retained for traceability. They do not describe the current five-servo ID map above; use the PR #8 section and active `.ioc` for current behavior.

### [Implemented]

- Logical identity: Servo1 ID 0, mask bit `0x0001`.
- Hardware mapping: TIM3_CH1 / PA6.
- PWM carrier: approximately 333 Hz.
- Set PWM requires a live host, exactly one payload item, Servo ID 0, enabled state, and 520–2520 μs inclusive.
- Set Angle requires a live host, exactly one payload item, Servo ID 0, enabled state, and −9000…+9000 cdeg inclusive; out-of-range values are rejected rather than clamped.
- Neutral requires a live host, valid Servo1 mask, and enabled state; it writes 1520 μs without stopping PWM or clearing enable state.
- Disable and heartbeat timeout call `HAL_TIM_PWM_Stop()`.

### [Provisional]

- Minimum ≈520 μs ≈−90°.
- Neutral 1520 μs =0°.
- Maximum ≈2520 μs ≈+90°.
- Servo model in the development record: GDW IPX896HV; merchant specification is PWM, 333 Hz, approximately 500/1500/2500 μs, 180°±5°, 4.8–8.4 V.
- The horn was mechanically re-centered near 1520 μs.

### [Hardware Verified]

The current development record states that Qt → Set Servo PWM → TIM3 CCR was observed, TIM3_CH1 drove Servo1, the GDW IPX896HV produced real motion, and the horn was mechanically centered. These are development-record claims, not conclusions produced by static code inspection.

The UART transport/ring-buffer extraction in this refactor was **[Historical Hardware Verified]** for the PR #2 old image only. STM32CubeIDE target build passed, ST-LINK download completed with “Download verified successfully”, and physical UART/Servo regression passed for Connect + Heartbeat, Enable + ACK, Neutral, +10°, 0°, −10°, Disable, Disconnect, reconnect without automatic Enable, and manual Enable + ACK recovery. The existing Servo1 and Set Angle hardware verification remains valid for that old image; this evidence does not verify the current `feature/servo-calibration-depth-limits` branch/image. For the current feature, ARM Build, Program Verify, and Hardware Verified remain **[Pending]** as stated in the current five-servo section above.

## PR #8 software verification status

- Pure-C descriptor, calibration, Servo service, ring-buffer, safety, and Protocol Dispatcher regressions pass with warnings treated as errors.
- The five-servo descriptor/service/dispatcher changes are software-verified; for the PR8 wiring, ARM Build: **[Pending]**; Program Verify: **[Pending]**; Hardware Verified: **[Pending]**.
- The target commands remain:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

## Historical target build snapshots (Historical Reference)

The following records refer to pre-PR8 or earlier modularization snapshots and are retained for traceability; they are not evidence that the current five-servo image has passed target build or hardware regression. The undefined angle calibration constants from the previous audit were replaced by one consistent Servo1 calibration set. A new-directory Debug configure/build using STM32CubeIDE's bundled CMake 4.3.1, Ninja 1.13.2, and GNU Tools for STM32 14.3.1 succeeded for that earlier snapshot.

Normal project commands remain:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

The project uses C11, Ninja, `arm-none-eabi-gcc`, and the generated STM32CubeMX CMake target. The generated CubeMX CMake remains untouched; the user-maintained top-level CMake lists the App, Communication, Motion, Servo, Safety, and Sensors modules and their include directories. The reproducible Firmware host gate is `powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1` from `RoboBeetleFirmware`; it compiles and runs all 27 executable test sources, including the SimpleGait, default/override app-main backend-selection, MotionManager, Motion-aware Protocol Dispatcher, CPG, and PWM safe-stop coverage, plus the four backend/benchmark app compile contracts and the CPG benchmark compile contract. The runner uses C11, `-Wall -Wextra -Werror`, host HAL stubs where required, and `-lm` for the deterministic sine gait. These host checks complement, but do not replace, the real ARM target build.

## App/Main maintainability audit（Historical Reference: PR #6 old image）

The App/Main boundary was **[Historical Hardware Verified]** for the PR #6 old image. This evidence applies only to that old image and does not verify the current `feature/servo-calibration-depth-limits` branch image; the current feature's ARM Build, Program Verify, and Hardware Verified status remains **[Pending]**. `main.c` is limited to the CubeMX-generated startup and peripheral initialization, `app_main_init`/`app_main_process` delegation, the thin UART completion callback, and the existing error/assert handlers. `Core/App/app_main.c` owns the cooperative application loop and glue code while delegating Protocol, UART, Safety, Servo, and dispatcher policy to their existing modules. STM32CubeIDE build, ST-LINK download, and full physical regression all passed for that PR #6 old image.

The split follows dependency direction instead of mechanically creating folders:

```text
Core/
├─ App/
│  ├─ app_main.c
│  └─ app_main.h
├─ Communication/
│  ├─ uart_transport_stm32.c/h
│  ├─ ring_buffer.c/h
│  └─ protocol_dispatcher.c/h
├─ Protocol/
│  └─ rb_protocol_v2.c/h
├─ Servo/
│  ├─ servo_driver_stm32.c/h
│  ├─ servo_service.c/h
│  └─ servo_calibration.c/h
├─ Motion/
│  ├─ motion_manager.c/.h
│  ├─ simple_gait_generator.c/.h
│  ├─ motion_types.h
│  └─ motion_config.h
├─ Sensors/
│  ├─ leak_sensor.c/h
│  └─ leak_sensor_stm32.c/h
├─ Safety/
│  └─ safety_supervisor.c/h
└─ Src/main.c
```

Recommended boundaries:

- Keep `main.c` limited to `HAL_Init`, clock/MX initialization, `app_main_init`, `app_main_process`, and CubeMX-safe callbacks that immediately delegate.
- `uart_transport_stm32` remains HAL-aware and owns `UART_HandleTypeDef`, RX re-arm, and eventually a nonblocking TX queue.
- `ring_buffer` is pure C and reusable; its silent full-buffer drop is preserved until a separately reviewed overflow policy is introduced.
- `rb_protocol_v2` remains pure C and HAL-independent.
- `protocol_dispatcher` parses command payloads, enforces the existing validation order, and calls service interfaces; it must not write TIM registers directly.
- `servo_driver_stm32` is the HAL-aware TIM3/TIM4 channel adapter: start, stop, and write descriptor pulse ticks.
- `servo_service` is pure C policy: supported IDs/masks, enabled state, bounds, and command semantics. It calls the driver through a narrow interface.
- `servo_calibration` is pure C data/mapping: per-servo min/neutral/max angle and pulse, direction, and later nonlinear points if required.
- `safety_supervisor` is pure C state/timing policy: host liveness, deadline evaluation, and one-shot timeout transition reporting. It has no HAL or Servo dependency; `app_main` requests safe actions through `servo_service`.
- `app_main` wires modules together and owns cooperative scheduling.

### Recommended order

1. Preserve this clean Protocol V2 / Servo1 baseline and perform a controlled hardware check of Neutral and Set Angle before expanding capability.
2. **[Implemented in this refactor]** Extract ring buffer and UART transport, preserving exact ISR behavior.
3. **[Historical Hardware Verified in PR #3 old image]** Extract the Servo HAL driver and pure Servo service/calibration, preserving the existing policy and calibration values; this does not verify the current feature/branch image.
4. **[Hardware Verified in this refactor]** Extract the Safety Supervisor with externally injected time and preserve the strict 500 ms timeout policy.
5. **[Hardware Verified in this refactor]** Extract the Protocol Dispatcher while preserving command validation order, Heartbeat semantics, and duplicate suppression.
6. **[Historical Hardware Verified in PR #6 old image]** Reduce `main.c` to initialization and `app_main_init`/`app_main_process` delegation. STM32CubeIDE build, ST-LINK download, and full physical regression passed for that old image; this does not verify the current feature/branch image.

Do not split the already isolated Protocol V2 codec further during Phase 1, add an RTOS, or introduce generic device frameworks before a second actuator/transport actually requires them.

## Remaining technical debt

- P1: the HAL-coupled App/Main layer has no dedicated host integration test; target build and physical regression are the verification gate.
- P1: the one-entry duplicate cache is deliberately minimal and is not a general replay window.
- P1: JY901S overflow, RX re-arm, and UART error diagnostics are volatile/debug-visible only; the corrected hard/deferred re-arm semantics are verified, while USART3 physical-link quality remains pending; these diagnostics are not exposed through Protocol V2 telemetry.
- P1: blocking UART ACK transmit shares the watchdog/parser loop.
- P2: Error `0x03` remains reserved; command failures currently use the frozen ACK result enum.
- P2: diagnostics are volatile counters only and are not exposed as telemetry.
- P2: debug LED is configured but unused.
- P2: no integrated Firmware-native codec/dispatcher/safety/calibration CTest target; current pure-C checks are manually compiled.

PR #3's old-image record adds only the three `Core/Servo` modules, their user-maintained top-level CMake source/include entries, pure-C Servo regression tests, the `main.c` service/driver delegation points, and this documentation. `.ioc`, generated CubeMX CMake, pins, clocks, USART settings, base frame format, CRC, COBS rules, message IDs, heartbeat policy, and Servo calibration values were not changed. The prior UART/ring-buffer extraction and Servo service/calibration/driver extraction were **[Historical Hardware Verified]** for the PR #3 old image: STM32CubeIDE build PASS, ST-LINK download PASS, and physical Servo regression PASS. This old-image evidence does not verify the current feature/branch image. No further code refactoring is included in that historical change.

PR #4 adds only the pure-C `Core/Safety` supervisor, its user-maintained top-level CMake source/include entries, the liveness regression test, the `main.c` time-injection/timeout delegation points, and this documentation. The strict `(now - last_heartbeat_rx_ms) > 500U` behavior, Protocol V2 parsing, ACKs, Servo behavior, UART, `.ioc`, and generated CubeMX CMake remain unchanged. The Safety Supervisor is now **[Hardware Verified]**: STM32CubeIDE build PASS, ST-LINK download PASS, and physical Safety regression PASS, covering Heartbeat loss, the strict timeout safe-disable transition, reconnect without automatic Enable, manual Enable + ACK recovery, and repeated disconnect/reconnect behavior.

PR #5 adds only the pure-C `Core/Communication/protocol_dispatcher` module, its host regression test, the user-maintained CMake source entry, the `main.c` decoded-frame/outcome integration, and this documentation. Protocol V2 wire framing, ACK encoding/transmit, UART transport, Safety policy, Servo behavior/calibration, `.ioc`, and generated CubeMX CMake remain unchanged. The Protocol Dispatcher extraction is now **[Hardware Verified]**: STM32CubeIDE build PASS, ST-LINK download PASS, and physical Protocol regression PASS covering Connect + Heartbeat, Enable + ACK, Neutral, Set Angle 0/+10/-10 degrees, Set PWM 1520 us near Neutral, Disable/re-enable, Disconnect, the >500 ms safe-disable transition, reconnect without automatic Enable, and manual Enable + ACK recovery.

PR #6's old-image record adds only the `Core/App/app_main.c/.h` orchestration layer, its user-maintained CMake source/include entries, the `main.c` delegation points, and this documentation. Protocol V2 wire framing, ACK encoding/transmit, duplicate cache, UART transport, Safety policy, Servo behavior/calibration, `.ioc`, generated CubeMX CMake, and peripheral initialization values remain unchanged. App/Main extraction was **[Historical Hardware Verified]** for the PR #6 old image: STM32CubeIDE Build PASS, ST-LINK Download PASS, and Full physical regression PASS. This old-image evidence does not verify the current feature/branch image.

PR #9 adds the pure-C leak-state telemetry policy, the Protocol V2 `LeakStatus (0x20)` frame, and a monitoring-only Qt indicator. Firmware samples PA11 and publishes after an accepted Heartbeat ACK on first/change/500 ms refresh opportunities; Console stale/disconnect/liveness loss returns the indicator to Unknown. Host Test, ARM Build, Program Verify, PA11 detection, real-link telemetry, Qt indication, and end-to-end Leak monitoring are now **[Hardware Verified]**. The acceptance also confirmed that rebuilding/programming/verifying the correct current ELF resolved the earlier persistent Unknown display; no numeric voltage or response-time claim is made. No leak state is connected to Servo or Safety actions.
