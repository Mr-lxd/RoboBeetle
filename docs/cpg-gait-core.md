# RoboBeetle CPG gait core

## Scope and status

This document describes the production Legacy Source-Compatible CPG v1 core and its RoboBeetle semantic adapter.

The approved runtime path is:

~~~text
Legacy Source-Compatible CPG v1
    -> RoboBeetle semantic adapter
    -> joint_targets_t / LogicalJointTargets
    -> MotionManager common guard
    -> ServoService
    -> ServoCalibration
~~~

SimpleGaitGenerator remains available as an alternate backend. Its Forward
mechanical-remap baseline is **Hardware Verified**: the front pair and rear
pair are each same-phase, and the installed front group is physically
anti-phase to the rear group. Backward remains pending/disabled. The normal
Debug/bench image now defaults to CPG with
`MOTION_DEFAULT_GAIT_BACKEND_CPG=1`; an explicit `=0` build reproduces the
completed SimpleGait diagnostic baseline. This does not add closed-loop CPG,
IMU feedback, depth feedback, ROS2, Protocol V2, Qt gait selection, or legacy
raw PWM/CCR mappings.

## Backend selection and debug boundary

`app_main_init()` initializes both generator objects but registers exactly one
`GaitGenerator` with MotionManager according to the compile-time backend
selection. The default `=1` path is

```text
Qt -> Protocol Motion -> MotionManager -> CPGGaitGenerator
    -> LogicalJointTargets -> Motion common guard -> ServoService
    -> ServoCalibration -> PWM
```

The explicit `MOTION_DEFAULT_GAIT_BACKEND_CPG=0` path selects
`SimpleGaitGenerator` through the same downstream pipeline and is retained as
the installed mechanical baseline. If CPG later fails the physical
Front/Rear anti-phase check while this SimpleGait result remains correct, the
debug boundary is CPG state, semantic adapter, backend selection, transition,
or actuator command path. Do not reopen Servo calibration or the front remap
from a CPG-only symptom. The latest CPG desktop physical gait is
**Hardware Verified** for the recorded Forward, synchrony, opposite-motion,
Turn Left/Right, Ascend/Descend mechanical-direction, Stop, and Disable All
checks. Water propulsion and hydrodynamic effectiveness remain pending.

## Latest CPG desktop verification

The normal CPG-default Firmware image completed a real desktop hardware
exercise on 2026-09-14. The observed Forward gait passed front-pair synchrony,
rear-pair synchrony, and physical opposite motion between the Front and Rear
groups. Turn Left, Turn Right, Ascend mechanical direction, Descend mechanical
direction, Stop, and Disable All also passed. This is **[Hardware Verified]**
desktop physical CPG gait evidence.

The steady-state CPG motion looked similar to the SimpleGait sinusoidal motion
by eye. That similarity is expected and non-blocking; it does not establish
numerical identity and does not justify changing the verified CPG. True water
propulsion, Turn hydrodynamic effectiveness, and Ascend/Descend hydrodynamics
remain **[Pending Water Verification]**. The DWT target-performance runbook is
in [`cpg-gait-performance.md`](cpg-gait-performance.md), with real STM32F407
measurements still pending.

## Historical source provenance

The historical generated source is the read-only oracle for this implementation.

| Evidence | Value |
| --- | --- |
| [SRC] generated source | D:\RoboBeetle\resource\CPG（高新义毕业论文 源代码）\CPG（高新义毕业论文 源代码）\stm32_demo\CPG_RoboBeetle_stm_ert_rtw\CPG_RoboBeetle_stm.c |
| [SRC] generated source SHA-256 | 879BD16C8853AA5191953E8598173EB37CE5104870640E9DF04A3ABE4E314BD3 |
| [SRC] paper PDF | D:\RoboBeetle\resource\高新义毕业论文.pdf |
| [SRC] paper PDF SHA-256 | 7DBD3D58AA5BADFD913DD11B34460903C8E68F2DE23E0004D526AFEF8839D265 |
| [SRC] real_T definition | rtwtypes.h:63 defines real_T as double |
| [SRC] generated step | CPG_RoboBeetle_stm.c:43 |
| [SRC] generated initialization | CPG_RoboBeetle_stm.c:853-861 |

The generated code is Simulink Coder output for an x86-64 Windows target, but its numerical state and update order are the source-compatible behavior being reproduced. The paper/source differences remain documented as unresolved paper/source discrepancies: the paper prints a different nonlinear denominator term, a beta example of 0.5, and RK4 prose, while this core follows the generated source's 2*beta*(1-beta), beta 0.75 defaults, and 0.01 s Forward Euler schedule.

## Frozen state mapping

The production core stores four source-order oscillator nodes. The node indices are numerical legacy slots only; they are not Servo IDs.

| Node | phase state | phase-rate memory | amplitude state | amplitude derivative | amplitude acceleration memory | offset state | offset derivative | offset acceleration memory | output memory | `theta_dot` expression | UnitDelay |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | DiscreteTimeIntegrator6_DSTATE | Memory_PreviousInput | DiscreteTimeIntegrator1_DSTATE | DiscreteTimeIntegrator_DSTATE | Memory4_PreviousInput | DiscreteTimeIntegrator3_DSTATE | DiscreteTimeIntegrator2_DSTATE | Memory5_PreviousInput | Memory19_PreviousInput | `rtb_TSamp - UD_DSTATE` | UD_DSTATE |
| 1 | DiscreteTimeIntegrator13_DSTATE | Memory2_PreviousInput | DiscreteTimeIntegrator8_DSTATE | DiscreteTimeIntegrator7_DSTATE | Memory8_PreviousInput | DiscreteTimeIntegrator10_DSTATE | DiscreteTimeIntegrator9_DSTATE | Memory9_PreviousInput | Memory7_PreviousInput | `rtb_TSamp_d - UD_DSTATE_o` | UD_DSTATE_o |
| 2 | DiscreteTimeIntegrator20_DSTATE | Memory1_PreviousInput | DiscreteTimeIntegrator15_DSTATE | DiscreteTimeIntegrator14_DSTATE | Memory12_PreviousInput | DiscreteTimeIntegrator17_DSTATE | DiscreteTimeIntegrator16_DSTATE | Memory13_PreviousInput | Memory11_PreviousInput | `rtb_TSamp_p - UD_DSTATE_b` | UD_DSTATE_b |
| 3 | DiscreteTimeIntegrator27_DSTATE | Memory3_PreviousInput | DiscreteTimeIntegrator22_DSTATE | DiscreteTimeIntegrator21_DSTATE | Memory16_PreviousInput | DiscreteTimeIntegrator24_DSTATE | DiscreteTimeIntegrator23_DSTATE | Memory17_PreviousInput | Memory15_PreviousInput | `rtb_TSamp_b - UD_DSTATE_a` | UD_DSTATE_a |

Each of the 12 directed source coupling slots has three states:

| Edge range | Source node | Target order | target phase state | target phase derivative | target acceleration memory |
| --- | --- | --- | --- | --- | --- |
| 0-2 | 0 | 1, 2, 3 | DiscreteTimeIntegrator5_DSTATE[0..2] | DiscreteTimeIntegrator4_DSTATE[0..2] | Memory6_PreviousInput[0..2] |
| 3-5 | 1 | 0, 2, 3 | DiscreteTimeIntegrator12_DSTATE[0..2] | DiscreteTimeIntegrator11_DSTATE[0..2] | Memory10_PreviousInput[0..2] |
| 6-8 | 2 | 0, 1, 3 | DiscreteTimeIntegrator19_DSTATE[0..2] | DiscreteTimeIntegrator18_DSTATE[0..2] | Memory14_PreviousInput[0..2] |
| 9-11 | 3 | 0, 1, 2 | DiscreteTimeIntegrator26_DSTATE[0..2] | DiscreteTimeIntegrator25_DSTATE[0..2] | Memory18_PreviousInput[0..2] |

All production mathematical fields in cpg_core_t and cpg_model_params_t are double.

## Exact theta_dot_i semantics

There is no dedicated theta_dot source symbol in CPG_RoboBeetle_stm.c. The exact source expression is built locally for each node:

~~~text
CPG_i[k]       = Memory_i_PreviousInput[k]
TSamp_i[k]     = CPG_i[k] * 100.0
theta_dot_i[k] = TSamp_i[k] - UD_i[k]
               = 100.0 * (Memory_i[k] - Memory_i[k-1])
~~~

The exact symbol chains are:

| Node | source output symbol | scaled sample expression | previous sample state | theta_dot expression |
| --- | --- | --- | --- | --- |
| 0 | CPG_1 = Memory19_PreviousInput | rtb_TSamp = CPG_1 * 100.0 | UD_DSTATE | rtb_TSamp - UD_DSTATE |
| 1 | CPG_2 = Memory7_PreviousInput | rtb_TSamp_d = CPG_2 * 100.0 | UD_DSTATE_o | rtb_TSamp_d - UD_DSTATE_o |
| 2 | CPG_3 = Memory11_PreviousInput | rtb_TSamp_p = CPG_3 * 100.0 | UD_DSTATE_b | rtb_TSamp_p - UD_DSTATE_b |
| 3 | CPG_4 = Memory15_PreviousInput | rtb_TSamp_b = CPG_4 * 100.0 | UD_DSTATE_a | rtb_TSamp_b - UD_DSTATE_a |

This is a scaled discrete difference. It is not an analytic derivative, not a separately integrated derivative state, and not a post-Euler difference.

When the source output is interpreted as logical degrees, CPG_i and output-memory values are logical degrees, while TSamp_i, UD_i, and theta_dot_i are logical degrees per second because the source scale is 100 samples per second. The source UnitDelay is the previous scaled sample used by the next step.

The generated source relies on C static-storage initialization for all DW state. CPG_RoboBeetle_stm_initialize initializes gait, prd, ampli, and Beta, but does not write the output-memory or UnitDelay state. Therefore the initial output memory, UnitDelay, and theta_dot vectors are all zero.

## Source equations and update schedule

The source-compatible defaults are:

~~~text
beta_i = 0.75
T_i = prd_i = 1.0 s for legacy golden vectors
k_v_i = 1.0
a_i = 20.0
b_i = 20.0
c_ij = {20,20,0,20,0,20,20,0,20,0,20,20}
w_ij = {2,2,0,2,0,2,2,0,2,0,2,2}
X_i = 0
desired phase target_ij = 0
step = 0.01 s
~~~

For each node:

~~~text
nu_i = [ (2*beta_i - 1)
         / (2*beta_i*(1 - beta_i)*(exp(-k_v_i*theta_dot_i) + 1))
         + 1/(2*beta_i) ] / T_i * 2*pi

r_ddot_i = ((R_i - r_i) * 5 - r_dot_i) * 20
x_ddot_i = ((X_i - x_i) * 5 - x_dot_i) * 20

phi_dot_i = nu_i + sum_j sin((phi_j - phi_i) - DeltaPhi_t_ij) * w_ij
theta_i_next = r_i * sin(phi_i) + x_i
~~~

For each directed edge:

~~~text
DeltaPhi_t_ddot_ij =
    ((c_ij / 4) * (desired_phase_ij - DeltaPhi_t_ij)
     - DeltaPhi_t_dot_ij) * c_ij
~~~

The generated source uses explicit one-step Memory blocks between each current derivative expression and the corresponding DiscreteIntegrator update. The equivalent core schedule is:

1. Select source forward target amplitudes and coupling arrays.
2. Read old output memory and UnitDelay values. Emit old output memory as raw output and form pre-Euler theta_dot.
3. Evaluate all phase-rate, amplitude-acceleration, offset-acceleration, and phase-target-acceleration locals from the step-entry state. Every nu_i uses the pre-Euler theta_dot_i.
4. Compute output memory from pre-Euler r, phi, and x. The source memory writes appear as Memory7, Memory11, Memory15, and Memory19.
5. In source node order 0, 1, 2, 3, write each UnitDelay with the pre-step TSamp value and update phase, amplitude, amplitude derivative, offset, and offset derivative. The phase update consumes the previous phase-rate memory; amplitude and offset derivatives consume their previous acceleration memories.
6. Update all 12 phase-target states from their pre-step derivatives and all 12 phase-target derivatives from their previous acceleration memories.
7. Store the current phase-rate, amplitude acceleration, offset acceleration, and phase-target acceleration locals into their source Memory-equivalent fields for the next step.

No derivative is recalculated after a state write. No output memory is recomputed from post-Euler oscillator states in the same step. The core does not phase-wrap because the generated source does not perform a phase wrap.

## Golden-vector contract

RoboBeetleFirmware/tests/test_cpg_core.c contains both:

- an independent test-only source oracle with the state/memory mapping above; and
- literal source checkpoints exported from the historical generated C.

The checkpoints include initial state, step 1/2 one-step-delay behavior, step 5 and step 10 raw delayed output, output memory, theta_dot, UnitDelay, phase, phase-rate memory, amplitude, amplitude derivative, amplitude acceleration memory, offset state and derivative memories, all 12 phase-target states/derivatives/acceleration memories, and the source-compatible parameters.

The test also seeds one directed phase-target state to prove the target integrator and derivative memory are one-step delayed. Exact zero assertions cover the initial theta_dot and UnitDelay vectors. Host comparison tolerance is documented in the test as 1e-11 relative to the literal double values; zero/order checks remain exact.

## Semantic adapter

cpg_gait_generator.c uses only logical targets and the existing gait_generator_t callback contract.

The fixed mapping is:

~~~text
core node 0 -> front_right_cdeg
core node 3 -> front_left_cdeg
core node 1 -> rear_right_cdeg
core node 2 -> rear_left_cdeg
~~~

The production Forward profile uses front and rear logical target amplitudes of 10 degrees, sets every core period parameter to `T=2.0 s` as a nominal period parameter, and records `nominal_period_s=2.0`. The signed source-compatible target convention is:

~~~text
R = {-front_amplitude, +rear_amplitude, +rear_amplitude, -front_amplitude}
~~~

This creates same-phase front and rear pairs and approximately pi-separated front versus rear output through the approved signed semantic mapping. The adapter converts logical degrees to centidegrees with double lround. For TURN_LEFT it installs the same signed vector with legacy nodes 3/2 (left front/rear) multiplied by the profile's 0.5 reduced-side scale; for TURN_RIGHT it multiplies nodes 0/1 (right front/rear). It does not post-scale the current raw output, so the next 10 ms advances move the core amplitude state toward the turn target. ASCEND/DESCEND add only the profile FrontAxis bias. FrontAxis is not a fifth oscillator.

Backward is rejected by cpg_gait_generator_is_mode_valid and sample; the adapter never fakes reverse motion by sign inversion. The adapter does not apply installed mechanical limits. MotionManager remains the sole owner of the operational front guard of -4500 to +2800 cdeg and rear guard of -3000 to +4500 cdeg.

## Elapsed-time and safety contract

cpg_core_advance_elapsed_ms accumulates uint64_t elapsed milliseconds, consumes complete 10 ms source-equivalent substeps, retains a remainder below 10 ms, and caps one call at 100 substeps. Excess complete substeps are discarded and counted diagnostically.

Safety-before-catch-up is enforced in two places:

1. app_main_process runs safety_supervisor_process before motion_manager_process and immediately applies the existing safety stop on timeout.
2. motion_manager_process checks safety_supervisor_is_host_alive before calculating elapsed time or invoking the generator advance callback.

Therefore a 700 ms foreground gap after a stale heartbeat cannot run 70 CPG steps and cannot emit a ServoService command. A stale abort leaves MotionManager faulted; a later heartbeat alone does not auto-resume it.

RoboBeetleFirmware/tests/test_cpg_safety_catchup.c covers the active-motion -> timeout gap -> safety abort -> no post-gap CPG step -> no post-gap Servo write -> no auto-resume sequence, plus live 20/70/100 ms catch-up, CPG output beyond the installed front/rear limits, and common MotionManager front/rear clamping before ServoService observes the target.

## Long-run period evidence

The production nominal-period profile was run for 5000 deterministic 10 ms steps with a 5000 ms transient exclusion. The test detects unwrapped phase[1] crossings at successive 2*pi thresholds.

The current host result is:

~~~text
nominal_period_s=2.000000000
measured_period_s=1.504827586
measured_frequency_hz=0.664527956
ratio=0.752413793
cycles=29
transient_exclusion_ms=5000
~~~

This result is intentionally not labeled as 0.5 Hz. The actual frequency is an emergent result of the source-compatible nu_i dependence on beta, theta_dot, k_v, and the coupled state. The `T=2.0 s` value remains a nominal period parameter until a target-representative long-run measurement says otherwise.
