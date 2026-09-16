# Motion Cadence / Servo Stutter Investigation Evidence

Date: 2026-09-15
Repository: `Mr-lxd/RoboBeetle`
Investigation branch: `codex/motion-cadence-stutter-investigation`
Investigation head at report preparation: `ae44d7f8303086cefbb9516e7391c31c07dd8682`
Base: `ac36092ca3011a538420273d721794ac7a7d0a5a`

## Provenance and evidence boundary

This report records the STM32F407 168 MHz target measurements supplied by the
user for the Motion Cadence / Servo Stutter Investigation. The values are
recorded as supplied; they are not reconstructed from host timing or inferred
from source inspection.

The supplied summary identifies the NORMAL cells by runtime gait backend and
the reduced-load cell by diagnostic condition and backend. The closeout data
does not include raw debugger dump files, trial IDs, trial duration/repetition
counts, toolchain version, CMake generator, linker script, image size, or a
separate binary hash. Those metadata fields are therefore not invented here.

The report remains software timing evidence. It does not promote logical
Motion timing into physical PWM evidence, and it does not establish an
electrical, servo-internal, mechanical, or water-performance conclusion.

## Supplied target results

All cells below report `SystemCoreClock = 168 MHz`.

### NORMAL diagnostic image, optional telemetry enabled

The NORMAL CPG and NORMAL SimpleGait results are recorded as separate runtime
backend trials. Their similar worst gaps, together with the supplied note that
SimpleGait generator compute is negligible compared with the gap, are the
backend comparison used below.

| Cell | Runtime backend | Worst Motion gap | Gaps strictly `>30 ms` | Optional USART1 telemetry TX in the supplied gap count | Worst-gap context |
| --- | --- | ---: | ---: | --- | --- |
| NORMAL / CPG | `CPG` | approximately `96 ms` | `39` | Leak `19`, IMU `10`, Depth `10`; total `39` | ACK approximately `16.8 ms`; IMU approximately `70.9 ms` |
| NORMAL / SimpleGait | `SimpleGait` | approximately `97 ms` | `43` | Leak `22`, IMU `11`, Depth `10`; total `43` | ACK approximately `16.7 ms`; IMU approximately `70.9 ms` |

The optional telemetry counts sum to the supplied `>30 ms` counts in both
NORMAL cells. This is a correlation in the target evidence; it is not a claim
that the aggregate count alone proves every individual gap's causal chain.

### REDUCED_OPTIONAL_TELEMETRY diagnostic image

The supplied reduced-load result is for CPG. Optional Leak, IMU, and Depth TX
were disabled by the diagnostic condition, while Heartbeat/ACK and the safety
path remained active.

| Cell | Runtime backend | Optional telemetry TX | ACK | Worst Motion gap | Gaps strictly `>30 ms` | Worst-gap context |
| --- | --- | --- | --- | ---: | ---: | --- |
| REDUCED_OPTIONAL_TELEMETRY / CPG | `CPG` | Leak `0`, IMU `0`, Depth `0` | Remained active | approximately `26 ms` | `0` | ACK approximately `16.8 ms` only |

No REDUCED_OPTIONAL_TELEMETRY / SimpleGait result is included in the supplied
closeout data.

## Evidence interpretation

The supplied target measurements support the following bounded conclusions:

| Question | Evidence status | Basis |
| --- | --- | --- |
| CPG compute as the primary cause of the NORMAL gaps | **NOT PRIMARY** | CPG and SimpleGait have comparable approximately 96/97 ms worst gaps; SimpleGait compute is supplied as negligible compared with the gap. |
| Gait backend as the primary cause | **NOT PRIMARY** | The large NORMAL gap remains for both runtime-selected backends. |
| Optional blocking USART1 telemetry TX | **CONFIRMED MAJOR CONTRIBUTOR** | Removing optional Leak/IMU/Depth TX reduces the supplied CPG worst gap from approximately 96 ms to approximately 26 ms and removes all `>30 ms` gaps; the NORMAL worst-gap contexts include approximately 70.9 ms IMU TX. |
| Blocking ACK TX | **CONFIRMED RESIDUAL CONTRIBUTOR** | ACK remains active in reduced telemetry, the reduced worst-gap context contains approximately 16.8 ms of ACK TX, and a residual approximately 26 ms worst gap remains. |
| Primary software timing root cause | **SYNCHRONOUS/BLOCKING USART1 TX IN THE COOPERATIVE FOREGROUND** | The target comparison is consistent with the existing synchronous host USART1 transmit path and the supplied worst-gap contexts. |

The last row identifies the primary software timing mechanism supported by this
investigation. It is not a claim that the complete visible servo symptom has a
single proven physical root cause.

## Preserved boundaries

The following remain outside this evidence report and must not be reported as
verified by it:

| Item | Status |
| --- | --- |
| Physical PWM waveform measured with oscilloscope/logic analyzer | **Pending** |
| HAL tick physical/target verification | **Pending** |
| Electrical cause excluded | **Not established / Pending physical evidence** |
| Servo-internal or mechanical cause excluded | **Not established / Pending physical evidence** |
| Water behavior/performance verified | **Pending** |

The existing delayed-foreground interpretation is preserved: when a Motion
call receives `elapsed_ms = 30`, generator state advances for 30 ms while only
one actuator-facing target application occurs. That remains a stutter
hypothesis and is not changed into a scheduler or catch-up-write fix by this
report.

## Historical investigation scope and next gate

The preceding investigation closeout changed evidence documentation only. It
did not implement the USART1 non-blocking TX design or fix the stutter, and it
did not change UART mode/baud, queue policy, foreground ordering, Motion
scheduling, CPG mathematics, Servo calibration, PWM/Clock configuration,
Safety behavior, or sensor behavior. The dedicated USART1 feature was
subsequently implemented on its own branch; its supplied target acceptance is
recorded below.

## USART1 non-blocking TX target short-trial acceptance (user-supplied)

The target results below were supplied from frozen normal STOP reports for the
USART1 non-blocking TX implementation at head
`3ffa19f36cd13e49064968a8c8db4b84db27d806`. Both trials used
`SystemCoreClock = 168 MHz`. They are short-trial evidence for the named
NORMAL runtime backends, not a claim that every possible workload has zero
jitter or that the complete physical stutter cause has been proven.

### NORMAL / CPG

| Item | Supplied result |
| --- | --- |
| Runtime backend / report state | `CPG` / frozen normal STOP report |
| Worst Motion interval | `1,710,195 cycles` approximately `10.18 ms` |
| Gaps strictly `>12 ms`, `>15 ms`, `>20 ms`, `>30 ms` | `0`, `0`, `0`, `0` |
| UART transport | ACK/Leak/IMU/Depth `enqueued == completed`; rejected `0`; dropped `0`; queue full `0`; start busy/error `0`; UART error `0`; unexpected callback `0` |
| Foreground enqueue maxima | ACK approximately `17.6 us`; IMU approximately `10.5 us` |

### NORMAL / SimpleGait

| Item | Supplied result |
| --- | --- |
| Runtime backend / report state | `SimpleGait` / frozen normal STOP report |
| Worst Motion interval | `1,686,244 cycles` approximately `10.04 ms` |
| Gaps strictly `>12 ms`, `>15 ms`, `>20 ms`, `>30 ms` | `0`, `0`, `0`, `0` |
| ACK | `45 enqueued / 45 completed` |
| Leak | `22 enqueued / 22 completed` |
| IMU | `11 enqueued / 11 completed` |
| Depth | `11 enqueued / 11 completed` |
| Transport/recovery counters | rejected `0`; dropped `0`; control queue full `0`; telemetry queue full `0`; start busy `0`; start error `0`; UART error `0`; unexpected callback `0`; busy recovery `0`; RX error `0` |
| Queue high-water mark | `2` |
| Worst-gap context | Contains no TX calls |

### Approved before/after comparison

| Trial | Before non-blocking TX | After non-blocking TX |
| --- | --- | --- |
| NORMAL / CPG | Worst approximately `96 ms`; `>30 ms = 39` | Worst approximately `10.18 ms`; `>30 ms = 0` |
| NORMAL / SimpleGait | Worst approximately `97 ms`; `>30 ms = 43` | Worst approximately `10.04 ms`; `>30 ms = 0` |

The supplied desktop mechanical observation is recorded narrowly as:
**visibly/audibly significantly smoother on the desktop bench**. It is not
water evidence, electrical evidence, PWM waveform evidence, or complete
physical-root-cause proof. The measured result also does not claim ideal
10.5x clock scaling or that the actual Servo stutter is fixed.

### Status and preserved evidence boundaries

| Item | Status |
| --- | --- |
| USART1 non-blocking TX remediation | **IMPLEMENTED / HOST-TESTED / ARM-BUILT / TARGET SHORT-TRIAL VERIFIED** |
| Communication-induced `>30 ms` Motion gaps | **Removed in the supplied NORMAL CPG and SimpleGait short trials** |
| Oscilloscope/logic-analyzer PWM evidence | **Pending** |
| Independent physical HAL tick verification | **Pending** |
| Electrical/mechanical exclusion | **Pending** |
| Water behavior | **Pending** |

`ARM-BUILT` and the target measurements in this section are user-supplied
target acceptance evidence. The pending physical evidence categories remain
independent and are not upgraded by the short trials.
