# Motion Cadence / Servo Stutter Investigation Design Specification

## Status and scope

This is the design for a new, independent investigation after the Runtime
SimpleGait / CPG selector closeout. The selector PR #17 is merged into main as:

~~~
ac36092ca3011a538420273d721794ac7a7d0a5a
~~~

The investigation branch is
codex/motion-cadence-stutter-investigation, based on that merge commit. This
document defines measurement and diagnosis only. It does not implement a
cadence fix, change the foreground order, or change any production behavior.

The question is whether observed servo stutter is caused by irregular
foreground Motion target updates, by the cost of communication and blocking
transmit work, or by a stable software cadence interacting with PWM,
electrical, servo-internal, or mechanical behavior.

The previous selector validation is context, not new cadence evidence:
SimpleGait and CPG selection is functionally available, dry-bench visual
difference was inconclusive, and water behavior remains pending. Water is not
required to begin this software timing investigation.

## Evidence boundary

The following boundaries remain frozen:

| Item | Status in this investigation |
| --- | --- |
| Clock source and 168 MHz configuration | Existing merged evidence; not re-opened here |
| Firmware host tests | Software evidence only |
| PWM physical waveform on a scope or logic analyzer | **Pending** |
| HAL tick physical/target measurement | **Pending** |
| Post-clock Forward/Turn/Ascend/Descend/CPG gait exercise | **Pending** unless separately supplied |
| Water verification | **Pending** |

The 168 MHz CPG DWT benchmark remains isolated compute evidence. It does not
prove foreground cadence, zero Motion jitter, PWM waveform regularity, or
servo health. No result in this investigation may promote host timing,
debugger-read counters, or a stable logical target cadence into physical PWM
or water evidence.

The investigation must not modify CPG equations, beta, Forward Euler dt,
theta_dot semantics, phase coupling, production double, Servo calibration,
Clock/RCC/PWM configuration, safety timeout values, UART baud, or the current
168 MHz production configuration.

## Investigation questions

1. How often does app_main_process() reach MotionManager under each
   communication load?
2. Which foreground region consumes the time before MotionManager is reached:
   RX queue draining, parser work, blocking TX, or another operation?
3. When MotionManager receives a delayed elapsed_ms, how many generator state
   steps and how many Servo target writes occur?
4. Does CPG isolated compute cost explain a measured whole-loop gap, or is the
   gap present for both generators?
5. If logical Motion updates are regular, does the physical PWM waveform still
   contain irregular periods or pulse widths?
6. If the waveform is regular, do supply, ground/noise, servo deadband, load,
   backlash, or linkage effects explain the visible symptom?

The design deliberately separates correlation from causation. A blocking call
is a confirmed timing hazard in source, not a confirmed physical root cause
until the target counters and physical measurements correlate with the symptom.

## Source-grounded current path

### Foreground order

RoboBeetleFirmware/Core/App/app_main.c:532-588 is the current cooperative
application loop. One pass performs the following operations in this order:

1. jy901s_transport_stm32_poll();
2. depth_transport_stm32_poll();
3. leak GPIO sampling through leak_sensor_stm32_read_level();
4. an unbounded while (uart_transport_stm32_pop(&byte)) host USART1
   extraction and Protocol V2 dispatch;
5. an unbounded while (jy901s_transport_stm32_pop(&byte)) USART3 parser
   drain;
6. an unbounded while (depth_transport_stm32_pop(&byte)) USART6 parser
   drain;
7. one HAL_GetTick() capture;
8. safety_supervisor_process();
9. motion_manager_process() when Safety has not requested the existing
   fail-safe stop.

The code does not impose a per-pass byte or time budget on any of the three
drains. The storage sizes are 128 bytes for the host ring buffer
(Core/Communication/ring_buffer.h:7), 256 bytes for JY901S
(jy901s_transport_stm32.c:9), and 512 bytes for Depth
(depth_transport_stm32.h:9). Because the ring buffer reserves one slot to
distinguish full from empty, the usable capacities are one less than those
storage sizes.

This ordering is a static timing hazard because work already present in a
queue is consumed before the MotionManager call. It is not proof that a queue
is full or that a particular physical stutter came from a queue.

### Blocking transmit path

The four current host-link transmit call sites are in
RoboBeetleFirmware/Core/App/app_main.c:

- ACK: protocol_send_ack() at approximately lines 285-316;
- LeakStatus: protocol_send_leak_status() at approximately lines 322-351;
- IMU snapshot: protocol_send_imu_snapshot() at approximately lines 354-402;
- Depth snapshot: protocol_send_depth_snapshot() at approximately lines
  405-470.

All four call uart_transport_stm32_transmit(). That function
(RoboBeetleFirmware/Core/Communication/uart_transport_stm32.c:42-50) calls:

~~~c
HAL_UART_Transmit(uart_handle, data, length, 100U);
~~~

The call is synchronous and has a 100 ms HAL timeout. At the configured 9600
baud, wire serialization is also nonzero. The exact occupied time depends on
HAL state, frame length, and link conditions, so source inspection alone does
not establish a 100 ms stall on every call.

For an accepted Heartbeat, protocol_feed_byte() sends the ACK first and may
then send one selected telemetry item according to the existing policy and
telemetry_scheduler_select() (app_main.c:167-239). Thus a single host RX byte
stream can cause multiple sequential blocking transmissions. The
investigation records ACK, LeakStatus, IMU, and Depth separately; it does not
change their policy.

### MotionManager schedule

RoboBeetleFirmware/Core/Motion/motion_manager.c:794-832 implements a
minimum-cadence elapsed-time gate:

~~~text
elapsed_ms = now_ms - last_tick_ms
elapsed_ms < MOTION_GAIT_TICK_MS  -> return without a tick
otherwise:
    last_tick_ms = now_ms
    motion_manager_tick(elapsed_ms)
~~~

MOTION_GAIT_TICK_MS is fixed at 10 ms by
Core/Motion/motion_config.h:4,20-21. The schedule is not a timer interrupt and
does not replay one Servo output for each nominal 10 ms slot.

Before generator advancement, motion_manager_process() checks SafetySupervisor
liveness. This safety-before-catch-up ordering remains unchanged. A
stale-liveness abort must not be interpreted as a normal-load cadence sample.

### Exact 30 ms delayed-call behavior

For a steady-state MOTION_STATE_RUNNING manager with no mode transition, a
call delayed by 30 ms has the following source-defined behavior:

| Backend | Generator advancement | Generator samples | Servo target application |
| --- | --- | ---: | --- |
| SimpleGait | One simple_gait_generator_advance(..., 30); phase advances by the 30 ms elapsed duration and is wrapped | 1 | One motion_manager_apply_targets() pass; one write per channel in write_mask |
| CPG | One cpg_gait_generator_advance(..., 30); cpg_core_advance_elapsed_ms() consumes three 10 ms source-equivalent steps | 1 | One motion_manager_apply_targets() pass; one write per channel in write_mask |

The SimpleGait behavior is implemented at
Core/Motion/simple_gait_generator.c:153-168, and its target is sampled at
:170-221. The CPG elapsed-step and cap are implemented at
Core/Motion/cpg_core.c:308-345; the adapter is advanced and sampled at
Core/Motion/cpg_gait_generator.c:129-185.

Therefore the CPG internal state can advance by three 10 ms substeps while the
actuator-facing logical target is emitted only once after the delayed
foreground call. SimpleGait advances its phase by the same elapsed duration
and also emits only one target. This is a candidate explanation for
discontinuous target-update cadence, not an authorization to add catch-up
Servo writes.

During a START transition, MotionManager samples once after its generator
advance and blends that target with the start pose. During a MODE transition,
it samples the old and new modes but still applies one blended target pass.
Those transition cases must be tagged or excluded when interpreting steady
state cadence.

The current CPG adapter also calls cpg_core_set_target_amplitudes() during
sample (cpg_gait_generator.c:166-172) so Motion-mode modulation remains a
separate mutable parameter concern. This investigation must observe that
existing behavior without changing it.

### Servo output boundary

MotionManager applies logical centidegree targets through
servo_service_set_angle_from_motion() (Core/Motion/motion_manager.c:301-338
and Core/Servo/servo_service.c:402-447). ServoService converts through the
existing calibration table before the STM32 driver writes timer compare
registers. The selector and this investigation never bypass that boundary.

servo_driver_stm32.c uses the existing timer compare/preload safe-stop policy.
A source counter showing regular logical writes cannot prove that the physical
PWM pulse widths are regular; a scope or logic analyzer remains required for
that distinction.

## Hypotheses and falsification

| ID | Hypothesis | Measurement that supports it | Measurement that weakens it |
| --- | --- | --- | --- |
| H1 | Unbounded RX drains delay MotionManager | Large drain byte counts/durations precede long Motion intervals; both backends show similar gaps | Long intervals occur with empty/short drains |
| H2 | Blocking host TX contributes to gaps | TX blocking duration, call count, or timeout/error events correlate with long Motion intervals | Long intervals remain when TX duration is small and stable |
| H3 | CPG isolated compute is the primary cause | CPG-only tick/generator span is materially larger and CPG has extra gaps under the same load | SimpleGait has similar gaps, or reduced-telemetry build removes gaps |
| H4 | Logical cadence is stable but physical output is irregular | Motion intervals and target writes are regular while scope shows irregular PWM period/width | Scope waveform is regular during the symptom |
| H5 | Electrical/servo/mechanical behavior is primary | Stable software and PWM timing, but supply/current/noise/load/deadband/backlash changes the symptom | Symptom tracks software gap counters instead |

These are falsifiable alternatives. The investigation must not label UART,
CPG, PWM, or mechanics as the root cause before the corresponding evidence
exists.

## Timing diagnostics architecture

### Compile-time boundary

The future instrumentation is gated by:

~~~
ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS
~~~

The default is OFF. The normal production image therefore has no DWT
initialization, no diagnostic RAM report, no diagnostic UART traffic, and no
diagnostic work in the loop. An ON image is a measurement image and must be
identified as such; it is not a production image.

The planned implementation module is:

~~~
RoboBeetleFirmware/Core/Diagnostics/motion_timing_diagnostics.h
RoboBeetleFirmware/Core/Diagnostics/motion_timing_diagnostics.c
~~~

The module is deliberately in a neutral layer because both Motion and
Communication call its observation hooks. It owns only counters and readout
data. It does not own scheduler decisions, Motion state, generator state,
Servo output, Safety state, or Protocol state, and it creates no dependency
from Motion to App.

A second compile-time option is:

~~~
ROBOBEETLE_MOTION_TIMING_REDUCED_TELEMETRY
~~~

It defaults to OFF and is effective only when
ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS is ON. In that diagnostic-only
condition, app_main may suppress optional Leak, IMU, and Depth telemetry so
the A/B result can be compared with a lower optional-TX load. It must still
receive Heartbeat, send Heartbeat ACK, run SafetySupervisor, enforce the
existing host-liveness timeout, preserve actuator fail-safe behavior, and
dispatch Protocol commands. The normal production image and its telemetry
policy are unchanged. The report records both option flags so a reduced-load
run cannot be mistaken for normal operation.

### DWT source and wrap handling

On STM32F407, the ON image enables CoreDebug->DEMCR.TRCENA, clears/enables
DWT->CYCCNT, and uses the same barrier pattern as the existing
cpg_target_benchmark.c:64-84. Each measured span uses unsigned 32-bit
subtraction:

~~~c
delta = finish - start;
~~~

This is wrap-safe for spans shorter than one 32-bit counter period. At
168 MHz, one full 32-bit cycle period is approximately 25.56 seconds, so the
implementation must not leave a single span open longer than that. Long-run
totals use a wider accumulator updated from short deltas. The report records
SystemCoreClock at initialization so conversion to microseconds is explicit
and not inferred from a host clock.

Diagnostics are updated in the cooperative foreground context. The design
does not add a diagnostic ISR, lock, printf, or transport. The volatile report
is halted/read by a debugger after a defined exercise window.

### Counter definitions

The future report must contain enough raw counts to separate queue work, TX
work, Motion work, and physical follow-up:

| Counter group | Required fields and definition |
| --- | --- |
| App loop | pass count; body duration count/min/max/total cycles; interval count/min/max/total cycles; worst interval; fixed histogram |
| Host USART1 RX drain | drain-call count; bytes popped; nonempty-drain count; duration count/min/max/total cycles; fixed histogram |
| JY901S USART3 drain | same fields, plus parser event/error counters copied only if already available without a new telemetry surface |
| Depth USART6 drain | same fields, plus existing parser/error counters copied only if already available without a new telemetry surface |
| Host TX | call count, bytes, status, duration count/min/max/total cycles by ACK, LEAK, IMU, and DEPTH; timeout/error count |
| Motion tick | tick count at the exact motion_manager_tick() invocation; requested elapsed_ms distribution; actual tick-to-tick interval count/min/max/total cycles; worst interval; fixed histogram; strict gap buckets above 10, 12, 15, 20, and 30 ms |
| Motion span | duration count/min/max/total cycles for motion_manager_tick(); generator advance span where separable; target sample/apply span where separable; returned result count |
| Run identity | diagnostics version, SystemCoreClock, backend label, build/configuration flags, and explicit run/reset markers |

The TX classification is attached at the existing protocol_send_ack,
protocol_send_leak_status, protocol_send_imu_snapshot, and
protocol_send_depth_snapshot call sites. It is not guessed by scanning COBS
bytes and it does not change the wire format.

The Motion tick timestamp is taken only when the existing elapsed gate accepts
a tick, not on every motion_manager_process() call. The report retains both
the actual tick interval and the elapsed_ms argument so a delayed call is not
confused with a normal 10 ms check that returned early.

Every distribution uses fixed-width fields: count, minimum, maximum, total,
and a fixed histogram. Interval distributions additionally expose their worst
interval. There is no exact streaming median requirement and no dynamic
allocation. Host tooling may derive an approximate median or percentile from
the fixed histogram, but raw samples are not implied by that estimate.

For Motion cadence, the required directly reportable evidence is the maximum
gap and counts for intervals strictly greater than 10 ms, 12 ms, 15 ms,
20 ms, and 30 ms. Values equal to a threshold remain outside that threshold's
bucket. Saturation and invalid/wrapped diagnostic conditions must be visible
rather than silently wrapping a statistic.

The fixed histogram bucket definitions and report field order are part of the
report ABI below; they are not implementation-private choices.

### Fixed debugger/OpenOCD report ABI

The diagnostic report is a versioned, fixed-layout object. `nm` resolves only
the address; no PowerShell tool may guess C struct offsets. The header is
encoded as fixed-width 32-bit words in this order:

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

All following fields use fixed-width `uint32_t` words or an explicitly
defined pair `{ uint32_t lo; uint32_t hi; }` for 64-bit totals. The ABI
defines the exact field order, histogram bucket order, and report size in the
public header. It does not use compiler-dependent pointers, `size_t`, `bool`,
bit-fields, or variable-length members. The implementation must include C11
`_Static_assert` checks for `sizeof` and critical `offsetof` values, including
the header fields, the first counter group, the Motion gap counters, and the
end of the report. A change to the layout requires a new ABI version.

The report initialization writes `magic`, `abi_version`, `report_size`,
`SystemCoreClock`, diagnostic flags, runtime backend, and a new reset/run
marker before counters are collected. The report is `volatile` for debugger
readout, but its contents are otherwise RAM-only.

ABI v1 currently fixes `report_size` at 1168 bytes. The fixed top-level
offsets are: `app_loop_body` 32, `app_loop_interval` 92, `rx_drain` 152,
`tx` 368, `motion` 704, `diagnostic_saturation_count` 1160, and
`diagnostic_counter_wrap_count` 1164. Each distribution is 60 bytes and its
histogram buckets are, in order, `<=10`, `11..20`, `21..50`, `51..100`,
`101..500`, `501..1000`, `1001..5000`, and `>5000` in the distribution's
recorded value units. These values are duplicated in the public header's
static ABI contract and in the readout decoder; changing them requires a new
ABI version.

The readout helper must read and validate `magic`, `abi_version`, and
`report_size` before decoding any remaining field. A mismatch fails loudly
and produces no interpreted timing result. The helper uses the public ABI
field offsets or a fixed byte decoder generated from the same documented
layout; it never infers offsets from `nm`, host compiler packing, or a guessed
struct definition.

### Instrumentation placement

The implementation plan places hooks at these boundaries without changing
control flow:

1. app_main_process() entry/exit;
2. each existing while (..._pop()) drain entry/exit and byte increment;
3. each existing protocol transmit call-site before/after
   uart_transport_stm32_transmit();
4. the accepted-tick path immediately around the existing
   motion_manager_tick() call;
5. optional nested spans around the existing generator advance, sample, and
   Servo application calls, using compile-time no-op hooks when disabled.

No hook may call HAL_UART_Transmit, alter a queue, change last_tick_ms,
advance a generator, write a Servo, or suppress a Safety action.

## Readout and target procedure

RAM-only readout is the preferred first implementation. The future report is a
debugger-readable volatile object, analogous to
cpg_target_benchmark_report; it is not Protocol V2 telemetry.

The preferred command-line path, if the target tools are available, is:

1. build an ON diagnostic ELF with the exact branch SHA and configuration;
2. use arm-none-eabi-nm to resolve the report symbol and
   arm-none-eabi-size to record image sizes;
3. halt the STM32F407 after the fixed exercise window;
4. read the report through an OpenOCD/debugger memory command or inspect the
   symbol in the debugger;
5. save raw counter values together with clock, backend, workload, trial, and
   board metadata.

A future tools/read-motion-timing.ps1 may automate symbol lookup and
debugger/OpenOCD readout, but it must fail loudly when
arm-none-eabi-nm, OpenOCD, the ELF symbol, or the target connection is missing.
It must never fall back to UART printing or claim a target result from a host
executable. If OpenOCD is unavailable, a debugger Watch/Expressions read of
the same volatile symbol is the approved fallback.

At this design baseline, arm-none-eabi-gcc.exe, arm-none-eabi-size.exe, and
openocd.exe are not available in the current Codex environment. No target
readout is claimed.

## A/B measurement matrix

Every cell uses the same board, power setup, firmware optimization/linker
configuration, exercise duration, host command pattern, and sensor stream.
The backend is selected through the already merged runtime selector while the
Motion state is STOPPED. A selector request must therefore obey the existing
STOPPED-only contract and be ACK-confirmed before the exercise starts.

There are exactly two diagnostic ELFs for this matrix: one NORMAL image and
one REDUCED_OPTIONAL_TELEMETRY image. A and B use the identical NORMAL ELF;
C and D use the identical REDUCED_OPTIONAL_TELEMETRY ELF. Do not build a
separate CPG or SimpleGait binary, pass
`MOTION_DEFAULT_GAIT_BACKEND_CPG=0/1`, or override `CMAKE_C_FLAGS` to choose
the experimental backend. The compile-time default remains the production
CPG contract; it is not the A/B experimental variable.

Each cell must include at least three fixed-duration trials and preserve raw
reports; the implementation runbook will use 60 seconds per trial unless a
reviewed target constraint requires a shorter, explicitly recorded window.

| Cell | Backend | Communication condition | Purpose |
| --- | --- | --- | --- |
| A | Runtime select CPG | NORMAL ELF and normal communication load/current optional telemetry behavior | Baseline current production behavior with timing counters |
| B | Runtime select SimpleGait | The same NORMAL ELF and identical load as A | Determine whether gaps are backend-independent |
| C | Runtime select CPG | REDUCED_OPTIONAL_TELEMETRY ELF; Heartbeat receive/ACK and Safety remain active | Isolate optional TX/telemetry contribution |
| D | Runtime select SimpleGait | The same REDUCED_OPTIONAL_TELEMETRY ELF and identical load as C | Separate backend cost from optional communication cost |

The runtime selector is used only between stopped exercises. No selector
request is sent during a trial, and no selector path may start motion, emit a
trajectory, add a Servo write, or bypass Safety.

The C/D reduced-telemetry condition is diagnostic-only and must never become
the production default. It may suppress only optional telemetry for the
measurement image; it must not disable or lengthen the Heartbeat path, Safety
Supervisor checks, host-liveness timeout, or actuator fail-safe behavior. The
implementation plan must make this condition an explicit build/test
configuration and record it in the report. If the no-telemetry-policy-change
scope is interpreted as forbidding even a diagnostic-only suppression, C/D
must remain marked NOT RUN until an externally reviewed load fixture can
reduce optional telemetry without changing firmware policy; A/B must not be
silently relabeled as C/D.

Heartbeat and Safety are never disabled to make a cell smoother. A stale
heartbeat must continue to prevent Motion catch-up and Servo writes according
to the existing Safety contract.

### Interpretation rules

- A and B both show the same long Motion intervals under normal load:
  generator mathematics is unlikely to be the primary explanation; inspect
  queue and TX spans.
- A/B show gaps but C/D become smooth with lower TX/telemetry work:
  communication/TX work is implicated, subject to correlation in raw counters.
- CPG differs from SimpleGait only when generator advance/sample spans differ
  materially: generator cost is implicated; the existing isolated CPG benchmark
  is supporting evidence only.
- Motion tick intervals and logical writes are regular in all cells, but the
  scope shows irregular PWM: software cadence is not the immediate cause;
  investigate timer waveform, supply, ground/noise, and driver/servo behavior.
- Software and PWM are both regular while the visible symptom changes with
  load, linkage, or servo replacement: investigate current sag, mechanical
  backlash, load, deadband, and servo internal control behavior.

No cell can establish water propulsion or hydrodynamic effectiveness.

## Physical follow-up kept separate

After logical timing is characterized, a physical check should observe at
least one active PWM channel with a scope or logic analyzer while recording
the same counter run. The check must distinguish:

- frame period regularity;
- pulse-width regularity;
- missing/runt pulses;
- timer output state during any safe-stop event.

Also record actuator supply voltage at the servo load, current/transient
behavior if available, common-ground integrity, and whether the symptom tracks
mechanical load, backlash, linkage, or a different servo. The existing OCxPE
safe-stop contract means readable CNT/CCR values are not by themselves
physical waveform evidence.

These checks remain Pending until actually performed.

## Acceptance criteria for the investigation phase

The phase is complete only when:

1. source facts above are preserved and cited in the implementation;
2. diagnostics compile out of the normal image and add no UART traffic when
   disabled;
3. host tests cover counter arithmetic, saturation/wrap handling, backend
   labels, fixed ABI size/offset contracts, strict gap buckets, and
   diagnostic/reduced-telemetry compile contracts;
4. target trials produce raw reports for the documented A/B matrix, or
   unavailable target tools are recorded as not run;
5. any claimed software correlation is supported by raw counters;
6. PWM physical and water statuses retain their Pending boundary;
7. no production scheduling, UART architecture, CPG math, Clock, PWM, Servo
   calibration, Safety, or stutter fix is included in this phase.

The output of this phase is an evidence-backed root-cause report or a bounded
list of remaining alternatives. A remediation such as queue budgeting,
nonblocking TX, scheduler reordering, PWM changes, or Servo replacement is a
separate reviewed task.

## Explicit non-goals

This design does not authorize:

- HAL_UART_Transmit() to DMA or IT;
- reordering Motion ahead of communication;
- RX byte/time budgets;
- changing baud from 9600;
- changing Heartbeat interval, Safety timeout, or existing telemetry policy in
  the production image;
- changing MOTION_GAIT_TICK_MS;
- adding catch-up Servo writes or scheduler changes;
- changing CPG mathematics, profile parameters, reset semantics, or numeric
  representation;
- changing Servo calibration, PWM frequency, timer registers, or Clock/RCC;
- changing JY901S, Depth, Leak, APC, or Safety semantics;
- implementing a Servo stutter, blocking UART, or cadence fix.
