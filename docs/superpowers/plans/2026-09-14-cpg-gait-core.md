# CPG Gait Core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement and integrate a Legacy Source-Compatible CPG v1 gait core with a RoboBeetle semantic adapter, preserving the approved logical-target pipeline and adding source, safety, timing, period, and target-resource evidence.

**Architecture:** The firmware will use a double-precision Legacy Source-Compatible CPG v1 core, followed by a RoboBeetle semantic adapter that emits logical joint targets. The existing MotionManager common guard, ServoService, ServoCalibration, and SimpleGaitGenerator remain the integration boundaries; Backward stays pending/disabled.

**Tech Stack:** STM32 C11 firmware, host GCC/CMake/PowerShell tests, existing MotionManager/SafetySupervisor/ServoService interfaces, optional STM32F407 DWT benchmark, no Qt or Protocol V2 changes.

---

The implementation must use the test-driven-development, verification-before-completion, requesting-code-review, and finishing-a-development-branch skills. The execution order below is intentionally RED -> GREEN for each behavior.

## Working rules and baseline

- Work only in C:\Users\laixindong\.config\superpowers\worktrees\RoboBeetle\feature-cpg-gait-core on branch feature/cpg-gait-core.
- Preserve the original checkout's detached state and preserve these original untracked files exactly:
  - D:\RoboBeetle\RoboBeetleFirmware\daplink.cfg
  - D:\RoboBeetle\docs\superpowers\plans\2026-09-09-leak-telemetry.md
- Do not reset, clean, force-push, merge, edit Qt, edit Protocol V2, edit calibration, or change the existing safety policy.
- Keep the historical source evidence read-only:
  - D:\RoboBeetle\resource\CPG（高新义毕业论文 源代码）\CPG（高新义毕业论文 源代码）\stm32_demo\CPG_RoboBeetle_stm_ert_rtw\CPG_RoboBeetle_stm.c
  - D:\RoboBeetle\resource\高新义毕业论文.pdf
- Record source SHA-256 values and source line anchors in implementation documentation. Do not copy old PWM/Servo IDs or raw CCR/timer code into production.

## File map

Create or modify only these implementation surfaces:

- RoboBeetleFirmware/Core/Motion/cpg_core.h
- RoboBeetleFirmware/Core/Motion/cpg_core.c
- RoboBeetleFirmware/Core/Motion/cpg_gait_generator.h
- RoboBeetleFirmware/Core/Motion/cpg_gait_generator.c
- RoboBeetleFirmware/Core/App/app_main.h
- RoboBeetleFirmware/Core/App/app_main.c
- RoboBeetleFirmware/Core/App/cpg_target_benchmark.h
- RoboBeetleFirmware/Core/App/cpg_target_benchmark.c
- RoboBeetleFirmware/CMakeLists.txt
- RoboBeetleFirmware/tests/run_host_tests.ps1
- RoboBeetleFirmware/tests/test_cpg_core.c
- RoboBeetleFirmware/tests/test_cpg_gait_generator.c
- RoboBeetleFirmware/tests/test_cpg_safety_catchup.c
- RoboBeetleFirmware/tests/test_cpg_period.c
- RoboBeetleFirmware/tests/app_main_jy901s_api_tests.c (extend the existing compile contract)
- docs/cpg-gait-core.md
- docs/cpg-gait-performance.md
- README.md only for a minimal link or status note if the existing README has an appropriate firmware architecture section

Do not create generated binaries, target build directories, or captured benchmark logs in the repository. Use a temp directory outside the worktree for generated test executables and target reports.

### Task 1: Establish the independent legacy source oracle and RED tests

**Files:**
- Create: `RoboBeetleFirmware/tests/test_cpg_core.c`
- Read-only reference: `D:\RoboBeetle\resource\CPG（高新义毕业论文 源代码）\CPG（高新义毕业论文 源代码）\stm32_demo\CPG_RoboBeetle_stm_ert_rtw\CPG_RoboBeetle_stm.c`
- Document after RED: `docs/cpg-gait-core.md`

- [ ] **Step 1: Add the compileable RED harness before adding production CPG files.** Start with an oracle-only fixture and a failing presence assertion so the first failure is an intentional missing-core checkpoint rather than a missing-header compiler error:

~~~c
static int cpg_core_is_present(void)
{
    return 0;
}

int main(void)
{
    assert(cpg_core_is_present() == 1);
    return 0;
}
~~~

- [ ] Add RoboBeetleFirmware/tests/test_cpg_core.c containing a deliberately failing first test before adding any production CPG implementation.
- [ ] **Step 2: Define the oracle fixture and exact source state names.** Use this fixed test-only shape so every golden vector exposes the derivative state required by the hardening contract:

~~~c
typedef struct
{
    double phase[4];
    double phase_rate_memory[4];
    double amplitude[4];
    double amplitude_dot[4];
    double amplitude_accel_memory[4];
    double offset[4];
    double offset_dot[4];
    double offset_accel_memory[4];
    double phase_target[12];
    double phase_target_dot[12];
    double phase_target_accel_memory[12];
    double output_memory[4];
    double unit_delay[4];
    double theta_dot[4];
    double emitted_output[4];
} legacy_oracle_t;
~~~

- [ ] Define a small oracle fixture in the test source with the exact legacy state names and source-order semantics:
  - node order 0..3;
  - phase state and phase-rate memory;
  - amplitude state, amplitude derivative state, and amplitude-acceleration memory;
  - offset state, offset derivative state, and offset-acceleration memory;
  - 12 directed phase-target states, derivatives, and acceleration memories;
  - output-memory state;
  - UnitDelay state corresponding to UD_DSTATE, UD_DSTATE_o, UD_DSTATE_b, UD_DSTATE_a;
  - theta_dot[4] as the externally inspectable scaled discrete difference.
- [ ] Encode the frozen source parameters in the oracle:
  - beta[4] = {0.75, 0.75, 0.75, 0.75};
  - k_v[4] = {1.0, 1.0, 1.0, 1.0};
  - prd[4] = {1.0, 1.0, 1.0, 1.0};
  - ampli[4] = {30.0, 30.0, 30.0, 30.0};
  - a = 20.0, b = 20.0;
  - edge weights in source storage order {2,2,0,2,0,2,2,0,2,0,2,2};
  - forward target assignment R = {-ampli[0], +ampli[1], +ampli[2], -ampli[3]};
  - integration step 0.01 seconds and source output scale 100.0.
- [ ] Implement the oracle one-step function only inside the test fixture. Evaluate all derivatives and nu_i before state writes, with theta_dot_i = (100.0 * output_memory_i) - unit_delay_i. Use the exact source nonlinear term 2 * beta * (1 - beta) and the exact source coupling/target equations established in the design spec.

The oracle function must follow this order, with no intervening post-Euler recomputation:

~~~c
static void legacy_oracle_step(legacy_oracle_t *state,
                               const double target_amplitude[4])
{
    double theta_dot[4];
    double emitted_output[4];
    double next_output_memory[4];
    double next_phase_rate[4];
    double next_amplitude_dot[4];
    double next_offset_dot[4];
    double next_phase_target_dot[12];

    for (size_t i = 0U; i < 4U; ++i)
    {
        emitted_output[i] = state->output_memory[i];
        theta_dot[i] = (100.0 * state->output_memory[i]) -
                       state->unit_delay[i];
    }

    /* Evaluate nu_i, all derivatives, and pre-Euler output here. */
    /* Write next output memory, UnitDelay, and Euler states in source
     * node order 0, 1, 2, 3. */
    (void)target_amplitude;
    (void)theta_dot;
    (void)emitted_output;
    (void)next_output_memory;
    (void)next_phase_rate;
    (void)next_amplitude_dot;
    (void)next_offset_dot;
    (void)next_phase_target_dot;
}
~~~

Replace the body only after the source equations and array indices have been transcribed from the generated C; the named temporaries make the pre-Euler versus post-Euler boundary reviewable.
- [ ] In the oracle step, emit the old source output-memory values as the current CPG_i, compute new output memory from pre-Euler r, phi, and x, then interleave each node's UnitDelay write and Euler state writes in source node order. Do not recompute theta_dot after the writes.
- [ ] Add literal golden-vector assertions for at least:
  - initial state and first step;
  - a state after multiple consecutive 10 ms steps;
  - a state after a nonzero output-memory transition where theta_dot is nonzero;
  - all four theta_dot values and all four UnitDelay values at the same checkpoints;
  - emitted delayed output and newly computed output-memory values;
  - phase, phase-rate memory, amplitude, amplitude derivative, amplitude-acceleration memory;
  - offset, offset derivative, offset-acceleration memory;
  - all 12 phase-target, phase-target derivative, and phase-target acceleration-memory values.
- [ ] **Step 3: Store literal golden values, including derivatives.** Each checkpoint must use literals in the test source, for example:

~~~c
static void assert_vector_close(const double *actual,
                                const double *expected,
                                size_t count,
                                double tolerance);

assert_vector_close(snapshot.theta_dot,
                    (const double[]){0.0, 0.0, 0.0, 0.0},
                    4U,
                    0.0);
assert_vector_close(snapshot.unit_delay,
                    (const double[]){0.0, 0.0, 0.0, 0.0},
                    4U,
                    0.0);
~~~

Use the generated literal values from the independent oracle for later checkpoints; do not assert only the four actuator outputs.
- [ ] Add an explicit assertion that initial theta_dot[4] and UnitDelay values are zero because the generated source relies on C static zero initialization; CPG_RoboBeetle_stm_initialize does not explicitly initialize those DW fields.
- [ ] Run the new test using the repository's existing host compiler flags before creating cpg_core.c. Capture the failing assertion as RED evidence. The test must fail for the intended missing-core reason, not because of a syntax or command error.

Run from the feature worktree, with output outside the repository:

~~~powershell
$testRoot = Join-Path $env:TEMP 'robobeetle-cpg-red'
New-Item -ItemType Directory -Force -Path $testRoot | Out-Null
$gcc = (Get-Command gcc -ErrorAction Stop).Source
$outputPath = Join-Path $testRoot 'test_cpg_core.exe'
& $gcc -std=c11 -Wall -Wextra -Werror -I RoboBeetleFirmware/Core/Inc -I RoboBeetleFirmware/Core/App -I RoboBeetleFirmware/Core/Motion -o $outputPath RoboBeetleFirmware/tests/test_cpg_core.c
if ($LASTEXITCODE -ne 0) { throw 'RED harness compile failed' }
& $outputPath
if ($LASTEXITCODE -eq 0) { throw 'Expected intentional RED assertion' }
~~~

Expected result: the executable compiles with `-Werror` and exits nonzero at `cpg_core_is_present() == 1`. After the production header exists, replace this presence assertion with the oracle-versus-core assertions and rerun the same command for GREEN.
- [ ] Add a temporary source-dump/oracle utility outside the repository if needed to generate literal values. Compare its output with the historical source by compiling the copied generated C in a temporary directory only; do not modify the source tree and do not treat the generated executable as a repository artifact.
- [ ] Document the exact historical source evidence in docs/cpg-gait-core.md only after the RED test exists. Include:
  - no dedicated theta_dot symbol exists;
  - node 0 Memory19_PreviousInput -> rtb_TSamp -> UD_DSTATE;
  - node 1 Memory7_PreviousInput -> rtb_TSamp_d -> UD_DSTATE_o;
  - node 2 Memory11_PreviousInput -> rtb_TSamp_p -> UD_DSTATE_b;
  - node 3 Memory15_PreviousInput -> rtb_TSamp_b -> UD_DSTATE_a;
  - source line anchors and source SHA-256;
  - real_T is double from rtwtypes.h.

### Task 2: Implement the double-precision source-compatible core after RED

**Files:**
- Create: `RoboBeetleFirmware/Core/Motion/cpg_core.h`
- Create: `RoboBeetleFirmware/Core/Motion/cpg_core.c`
- Modify: `RoboBeetleFirmware/tests/test_cpg_core.c`

- [ ] **Step 1: Declare the fixed-size double API.** The public header must expose these names and signatures so the adapter and tests use one stable contract:

~~~c
#include <stdint.h>

#define CPG_CORE_NODE_COUNT 4U
#define CPG_CORE_EDGE_COUNT 12U
#define CPG_CORE_STEP_MS 10U
#define CPG_CORE_MAX_CATCH_UP_STEPS 100U

typedef struct
{
    double beta[CPG_CORE_NODE_COUNT];
    double period_s[CPG_CORE_NODE_COUNT];
    double velocity_gain[CPG_CORE_NODE_COUNT];
    double amplitude_gain[CPG_CORE_NODE_COUNT];
    double offset_gain[CPG_CORE_NODE_COUNT];
    double phase_target_gain[CPG_CORE_EDGE_COUNT];
    double coupling_weight[CPG_CORE_EDGE_COUNT];
    double desired_phase[CPG_CORE_EDGE_COUNT];
    double target_amplitude[CPG_CORE_NODE_COUNT];
    double target_offset[CPG_CORE_NODE_COUNT];
    double step_s;
} cpg_model_params_t;

typedef struct
{
    cpg_model_params_t params;
    double phase[CPG_CORE_NODE_COUNT];
    double phase_rate_memory[CPG_CORE_NODE_COUNT];
    double amplitude[CPG_CORE_NODE_COUNT];
    double amplitude_dot[CPG_CORE_NODE_COUNT];
    double amplitude_accel_memory[CPG_CORE_NODE_COUNT];
    double offset[CPG_CORE_NODE_COUNT];
    double offset_dot[CPG_CORE_NODE_COUNT];
    double offset_accel_memory[CPG_CORE_NODE_COUNT];
    double phase_target[CPG_CORE_EDGE_COUNT];
    double phase_target_dot[CPG_CORE_EDGE_COUNT];
    double phase_target_accel_memory[CPG_CORE_EDGE_COUNT];
    double output_memory[CPG_CORE_NODE_COUNT];
    double raw_output[CPG_CORE_NODE_COUNT];
    double unit_delay[CPG_CORE_NODE_COUNT];
    double theta_dot[CPG_CORE_NODE_COUNT];
    uint64_t elapsed_remainder_ms;
    uint32_t executed_step_count;
    uint32_t discarded_catch_up_count;
} cpg_core_t;

typedef cpg_core_t cpg_core_snapshot_t;

void cpg_legacy_source_compatible_default_params(cpg_model_params_t *params);
void cpg_core_init(cpg_core_t *core, const cpg_model_params_t *params);
void cpg_core_reset(cpg_core_t *core);
void cpg_core_set_target_amplitudes(
    cpg_core_t *core,
    const double target_amplitude[CPG_CORE_NODE_COUNT]);
void cpg_core_set_periods(
    cpg_core_t *core,
    const double period_s[CPG_CORE_NODE_COUNT]);
void cpg_core_step(cpg_core_t *core);
uint32_t cpg_core_advance_elapsed_ms(cpg_core_t *core, uint32_t elapsed_ms);
void cpg_core_snapshot(const cpg_core_t *core, cpg_core_snapshot_t *snapshot);
uint32_t cpg_core_executed_step_count(const cpg_core_t *core);
uint32_t cpg_core_discarded_catch_up_count(const cpg_core_t *core);
~~~

- [ ] **Step 2: Implement one source-ordered 10 ms step.** Compute every local derivative from the state at function entry, assign raw output from old output memory, compute next output memory from pre-Euler phase/amplitude/offset, then write output memory and the four UnitDelay values before applying Euler state writes in node order 0 through 3. Keep the implementation entirely in `double` and link `libm` for `exp` and `sin`.

~~~c
cpg_core_t next = *core;

for (size_t i = 0U; i < CPG_CORE_NODE_COUNT; ++i)
{
    next.raw_output[i] = core->output_memory[i];
    next.theta_dot[i] = (100.0 * core->output_memory[i]) -
                        core->unit_delay[i];
    /* Use next.theta_dot[i] in exp(-k_v[i] * theta_dot[i]). */
    /* Compute all next_* derivative locals before writes. */
}
for (size_t i = 0U; i < CPG_CORE_NODE_COUNT; ++i)
{
    next.output_memory[i] = core->offset[i] +
        core->amplitude[i] * sin(core->phase[i]);
}
for (size_t i = 0U; i < CPG_CORE_NODE_COUNT; ++i)
{
    next.unit_delay[i] = 100.0 * core->output_memory[i];
    next.phase[i] = core->phase[i] +
                    core->phase_rate_memory[i] * core->params.step_s;
    next.amplitude[i] = core->amplitude[i] +
                       core->amplitude_dot[i] * core->params.step_s;
    next.offset[i] = core->offset[i] +
                    core->offset_dot[i] * core->params.step_s;
    next.phase_rate_memory[i] = next_phase_rate[i];
    next.amplitude_dot[i] = core->amplitude_dot[i] +
                           core->amplitude_accel_memory[i] *
                           core->params.step_s;
    next.offset_dot[i] = core->offset_dot[i] +
                        core->offset_accel_memory[i] *
                        core->params.step_s;
    next.amplitude_accel_memory[i] = next_amplitude_dot[i];
    next.offset_accel_memory[i] = next_offset_dot[i];
}
for (size_t e = 0U; e < CPG_CORE_EDGE_COUNT; ++e)
{
    next.phase_target[e] = core->phase_target[e] +
                           core->phase_target_dot[e] *
                           core->params.step_s;
    next.phase_target_dot[e] = core->phase_target_dot[e] +
                               core->phase_target_accel_memory[e] *
                               core->params.step_s;
    next.phase_target_accel_memory[e] = next_phase_target_dot[e];
}
*core = next;
~~~

Use these exact source expressions in the implementation, with one local next-state value per derivative before any write:

~~~c
const double nu = ((2.0 * beta[i] - 1.0) /
    (2.0 * beta[i] * (1.0 - beta[i]) *
     (exp(-velocity_gain[i] * theta_dot[i]) + 1.0)) +
    1.0 / (2.0 * beta[i])) / period_s[i] * 2.0 * pi;
next_phase_rate[i] = nu + coupling_sum[i];
next_amplitude_dot[i] =
    ((target_amplitude[i] - amplitude[i]) *
     (amplitude_gain[i] / 4.0) - amplitude_dot[i]) *
    amplitude_gain[i];
next_offset_dot[i] =
    ((target_offset[i] - offset[i]) *
     (offset_gain[i] / 4.0) - offset_dot[i]) *
    offset_gain[i];
for (size_t e = 0U; e < CPG_CORE_EDGE_COUNT; ++e)
{
    next_phase_target_dot[e] =
        ((phase_target_gain[e] / 4.0) *
         (desired_phase[e] - phase_target[e]) -
         phase_target_dot[e]) * phase_target_gain[e];
}
next.phase[i] = phase[i] + step_s * phase_rate[i];
next.amplitude[i] = amplitude[i] + step_s * amplitude_dot[i];
next.offset[i] = offset[i] + step_s * offset_dot[i];
next.phase_rate[i] = next_phase_rate[i];
next.amplitude_dot[i] = amplitude_dot[i] +
                       step_s * next_amplitude_dot[i];
next.offset_dot[i] = offset_dot[i] +
                    step_s * next_offset_dot[i];
~~~

Build `coupling_sum[i]` using these source edge pairs and weights in the same storage order:

~~~c
static const uint8_t edge_source[12] =
    {0U, 0U, 0U, 1U, 1U, 1U, 2U, 2U, 2U, 3U, 3U, 3U};
static const uint8_t edge_target[12] =
    {1U, 2U, 3U, 0U, 2U, 3U, 0U, 1U, 3U, 0U, 1U, 2U};
double coupling_sum[4] = {0.0, 0.0, 0.0, 0.0};
for (size_t e = 0U; e < 12U; ++e)
{
    coupling_sum[edge_source[e]] +=
        sin((phase[edge_target[e]] - phase[edge_source[e]]) -
            phase_target[e]) * coupling_weight[e];
}
~~~

The generated source uses `target_offset = 0`, `desired_phase = 0`, `a = 20`, `b = 20`, `c = 20`, and `pi = 3.141592653589793`; retain those values in the source-compatible default. The write boundaries and source node order must remain visible in the final code.

- [ ] Add cpg_core.h with a fixed-size, allocation-free API suitable for firmware:
  - CPG_CORE_NODE_COUNT 4U;
  - CPG_CORE_EDGE_COUNT 12U;
  - CPG_CORE_STEP_MS 10U;
  - CPG_CORE_MAX_CATCH_UP_STEPS 100U;
  - cpg_model_params_t;
  - cpg_core_t;
  - cpg_core_snapshot_t;
  - init/reset/default-parameter functions;
  - target-amplitude and period setters;
  - one 10 ms cpg_core_step;
  - elapsed-time advancement with bounded catch-up;
  - snapshot and diagnostic-count accessors.
- [ ] Make every production mathematical state and parameter double. Do not add a float production path, cast the core to float, or change the source-compatible baseline because STM32F407 has a single-precision FPU.
- [ ] Implement the frozen source equations and storage order in cpg_core.c:
  - phase-rate equations use the exact source 2 * beta * (1 - beta) term;
  - nu_i uses exp(-k_v_i * theta_dot_i) from the pre-Euler value;
  - theta_dot_i = 100.0 * output_memory_i - unit_delay_i;
  - phase, amplitude, offset, and phase-target integrators consume the
    previous source derivative-memory values before current accelerations are
    stored;
  - output is the delayed source memory value;
  - new output memory is computed from pre-Euler state and then state updates occur in source node order;
  - no analytic derivative, no post-Euler derivative recomputation, and no phase wrapping unless the oracle proves the source performs it;
  - preserve source phase/coupling edge order and forward target signs.
- [ ] Initialize all fields deterministically to the source-equivalent zero state, then apply source defaults. Make the relationship between zero output memory, zero UnitDelay, zero theta_dot, and first emitted output explicit in code comments and docs.
- [ ] Implement elapsed-time handling as a uint64_t millisecond accumulator:
  - consume complete 10 ms substeps;
  - retain a substep remainder;
  - cap any one foreground catch-up at 100 substeps;
  - discard excess stale time and increment a diagnostic counter;
  - expose executed-step and discarded-catch-up diagnostics for safety/performance tests.
- [ ] Run test_cpg_core.c after this implementation and replace the RED assertion with GREEN literal golden comparisons. Use a tight, documented tolerance only for host double arithmetic and preserve exact zero/order assertions where exact equality is expected.
- [ ] Add a regression that changes output memory between steps and proves the core theta_dot equals the source scaled difference at step start, not an analytic or post-Euler derivative.
- [ ] Add a regression that proves cpg_core_advance_elapsed_ms(700) never executes more than CPG_CORE_MAX_CATCH_UP_STEPS and reports the discarded time.

### Task 3: Implement the RoboBeetle semantic adapter

**Files:**
- Create: `RoboBeetleFirmware/Core/Motion/cpg_gait_generator.h`
- Create: `RoboBeetleFirmware/Core/Motion/cpg_gait_generator.c`
- Test: `RoboBeetleFirmware/tests/test_cpg_gait_generator.c`
- Read: `RoboBeetleFirmware/Core/Motion/gait_generator.h`, `joint_targets.h`, and `motion_types.h`

- [ ] **Step 1: Declare the adapter profile and interface.** Keep the existing callback ABI and make the production policy explicit:

~~~c
typedef struct
{
    double front_amplitude_deg;
    double rear_amplitude_deg;
    double nominal_period_s;
    double turn_reduced_side_scale;
    double front_axis_bias_cdeg[MOTION_COUNT];
} cpg_gait_profile_t;

typedef struct
{
    cpg_core_t core;
    cpg_gait_profile_t profile;
} cpg_gait_generator_t;

void cpg_gait_generator_init(cpg_gait_generator_t *generator);
gait_generator_t cpg_gait_generator_interface(cpg_gait_generator_t *generator);
bool cpg_gait_generator_sample(cpg_gait_generator_t *generator,
                               motion_mode_t mode,
                               float amplitude_scale,
                               float bias_scale,
                               joint_targets_t *targets);
~~~

- [ ] Add cpg_gait_generator.h exposing the existing gait_generator_ops_t interface through a cpg_gait_generator_t context and a named cpg_gait_profile_t.
- [ ] Keep the adapter semantic and logical:
  - map core node 0 -> front right;
  - map core node 3 -> front left;
  - map core node 1 -> rear right;
  - map core node 2 -> rear left;
  - produce joint_targets_t centidegrees;
  - never reference legacy servo IDs, timers, CCR registers, PWM channels, or calibration tables.
- [ ] Set the approved forward topology:
  - front pair same phase;
  - rear pair same phase;
  - front versus rear approximately pi through the approved source-compatible signed target/output convention;
  - no separate Backward implementation;
  - no fifth FrontAxis oscillator.
- [ ] Define the production profile with:
  - front and rear logical amplitudes of 10 degrees;
  - nominal_period_s = 2.0 as a nominal period parameter only;
  - a bounded, signed mode-specific target-amplitude vector: reduce legacy
    nodes 3/2 for TURN_LEFT and nodes 0/1 for TURN_RIGHT;
  - FrontAxis profile bias fields only;
  - no rear -30 degree clamp in this adapter.
- [ ] Keep source-compatible 30-degree / 1-second defaults available for golden/oracle tests so adapter tests can distinguish legacy numeric reproduction from production profile policy.
- [ ] Implement advance, sample, mode validation, and diagnostics through the existing interface. sample must install the mode-specific target-amplitude vector, convert the current raw logical degrees to centidegrees with a documented rounding policy, and add only the profile-level FrontAxis bias and global amplitude scale.

The adapter's core-to-joint mapping and centidegree conversion must be visible in one function:

~~~c
targets->front_right_cdeg = rounded_cdeg(100.0 * snapshot.raw_output[0]);
targets->front_left_cdeg = rounded_cdeg(100.0 * snapshot.raw_output[3]);
targets->rear_right_cdeg = rounded_cdeg(100.0 * snapshot.raw_output[1]);
targets->rear_left_cdeg = rounded_cdeg(100.0 * snapshot.raw_output[2]);
targets->front_axis_cdeg = rounded_cdeg(
    generator->profile.front_axis_bias_cdeg[mode] * (double)bias_scale);
~~~

Install turn scaling in the signed core target-amplitude vector before the next
advance; do not apply a second left/right scale to the current raw output. Use
`lround` on the double value for centidegree conversion. Do not invoke
MotionManager's rear sanitizer from this file.
- [ ] Add test_cpg_gait_generator.c RED/GREEN assertions for:
  - deterministic initialization;
  - logical mapping and sign/phase topology;
  - production amplitudes and nominal-period metadata;
  - mode-specific turn target vectors, no current-sample post-scale jump,
    and subsequent amplitude-state convergence;
  - a long-run dynamically advanced production profile with pair symmetry,
    approximate anti-phase, and a conservative output envelope;
  - FrontAxis as bias only;
  - Forward accepted;
  - Backward rejected or reported disabled without changing signs;
  - output contains no raw actuator identifiers;
  - adapter does not apply the rear operational limit.
- [ ] Run the adapter test with existing warning-as-error host flags.

### Task 4: Integrate the CPG backend and freeze safety-before-catch-up

**Files:**
- Modify: `RoboBeetleFirmware/Core/App/app_main.c`
- Modify: `RoboBeetleFirmware/CMakeLists.txt`
- Create: `RoboBeetleFirmware/tests/test_cpg_safety_catchup.c`
- Modify: `RoboBeetleFirmware/Core/Motion/motion_manager.c` only if the existing pre-tick liveness guard needs an explicit regression-preserving helper; otherwise leave it unchanged
- Test against: `RoboBeetleFirmware/Core/Safety/safety_supervisor.c` and `RoboBeetleFirmware/Core/Servo/servo_service.c`

- [ ] **Step 1: Add the compile-time backend selector without changing the generator ABI.** Put the default in `app_main.c` or its public configuration header exactly as follows, and select the interface passed to `motion_manager_init`:

~~~c
#ifndef MOTION_DEFAULT_GAIT_BACKEND_CPG
#define MOTION_DEFAULT_GAIT_BACKEND_CPG 1
#endif

#if MOTION_DEFAULT_GAIT_BACKEND_CPG
    motion_manager_init(
        &motion_manager,
        &servo_service,
        &safety_supervisor,
        cpg_gait_generator_interface(&cpg_gait_generator));
#else
    motion_manager_init(
        &motion_manager,
        &servo_service,
        &safety_supervisor,
        simple_gait_generator_interface(&simple_gait_generator));
#endif
~~~

- [ ] Initialize both SimpleGaitGenerator and CPG adapter in app_main.c, then select the default backend through a compile-time setting whose default is CPG and whose alternate value preserves SimpleGaitGenerator. Keep the existing generator interface, MotionManager guard, ServoService, and calibration path.
- [ ] Make the liveness/safety ordering explicit in the app entry point and preserve the existing MotionManager common guard as the final defense:
  - foreground obtains current time;
  - safety/liveness supervisor processes heartbeat and has final authority;
  - if stale beyond SAFETY_SUPERVISOR_HEARTBEAT_TIMEOUT_MS, abort/stop immediately;
  - only a live motion state can call elapsed-time CPG advancement;
  - only after that guard may MotionManager sanitize and emit ServoService commands.
- [ ] Do not let a stale foreground gap be converted into a CPG catch-up followed by actuator writes. No auto-resume after a stale abort.
- [ ] Add test_cpg_safety_catchup.c using an instrumented CPG context and a fake ServoService sink:
  - start Motion active;
  - advance the fake clock by more than the heartbeat timeout, including a 700 ms gap;
  - process the safety supervisor before MotionManager;
  - assert the supervisor aborts/faults;
  - assert CPG executed-step count does not change after the gap;
  - assert no post-gap CPG actuator command reaches the fake Servo sink;
  - assert a later heartbeat/reconnect does not auto-resume or write a command.

The integration assertion must have this observable sequence:

~~~c
motion_manager_start(&manager, MOTION_FORWARD);
const uint32_t steps_before_gap = cpg_core_executed_step_count(&generator.core);
fake_now_ms += SAFETY_SUPERVISOR_HEARTBEAT_TIMEOUT_MS + 200U;
assert(safety_supervisor_process(&supervisor, fake_now_ms));
assert(motion_manager_process(&manager, fake_now_ms) ==
       MOTION_MANAGER_RESULT_HOST_NOT_ALIVE);
assert(cpg_core_executed_step_count(&generator.core) == steps_before_gap);
assert(fake_servo.write_count == writes_before_gap);
assert(motion_manager_start(&manager, MOTION_FORWARD) ==
       MOTION_MANAGER_RESULT_HOST_NOT_ALIVE);
assert(fake_servo.write_count == writes_before_gap);
~~~

The test must call the safety supervisor before MotionManager at the stale timestamp and must also retain the MotionManager liveness guard as a second, common-path defense.
- [ ] Add an integration assertion for ordinary live 20/70/100 ms elapsed intervals, verifying that catch-up is bounded and commands occur only while liveness is valid.
- [ ] Keep existing common rear bounds (-3000 to +4500 cdeg) solely in MotionManager sanitizer and add a test proving CPG output reaches that common guard without duplicating the clamp in the adapter.
- [ ] Compile the app contract with both backend selections. The contract must prove the CPG implementation is linked and the SimpleGaitGenerator alternate remains buildable.

### Task 5: Add target performance evidence without changing precision

**Files:**
- Create: `docs/cpg-gait-performance.md`
- Modify: `RoboBeetleFirmware/CMakeLists.txt` only for the benchmark option and source registration
- Modify: `RoboBeetleFirmware/Core/App/app_main.c` only for the benchmark entry point when the option is enabled
- Create: `RoboBeetleFirmware/Core/App/cpg_target_benchmark.h`
- Create: `RoboBeetleFirmware/Core/App/cpg_target_benchmark.c`
- Modify: `RoboBeetleFirmware/tests/app_main_jy901s_api_tests.c` for compile-contract coverage

- [ ] **Step 1: Add the disabled-by-default target option.** Define the option without changing the normal firmware build:

~~~cmake
option(ROBOBEETLE_CPG_TARGET_BENCHMARK
       "Enable the STM32F407 CPG DWT benchmark"
       OFF)
~~~

When the option is ON, add the benchmark source/definition to the firmware target; when it is OFF, compile no DWT reporting code and keep normal behavior byte-for-byte outside the new CPG integration.

- [ ] **Step 2: Implement the target cycle measurement around the required cases.** Use the MCU DWT counter only in the target-gated file:

~~~c
static uint32_t cpg_benchmark_cycles(void (*operation)(void *),
                                     void *context)
{
    const uint32_t before = DWT->CYCCNT;
    __DSB();
    operation(context);
    __DSB();
    __ISB();
    return DWT->CYCCNT - before;
}
~~~

Enable `CoreDebug->DEMCR` and `DWT->CTRL` once, warm up the core, run a fixed repetition count, and report minimum/maximum plus a robust central statistic for 10 ms, 20 ms, 70 ms, 100 ms, and 100-step bounded catch-up. Convert cycles using the measured firmware clock, not a host clock.

- [ ] Add docs/cpg-gait-performance.md defining the target evidence table and exact calculation:
  - FLASH delta = (text + data)_CPG - (text + data)_baseline;
  - RAM delta = (data + bss)_CPG - (data + bss)_baseline;
  - report bytes and percentages when the denominator is nonzero;
  - identify compiler flags, link script, optimization, clock, and target revision.
- [ ] Add an optional ROBOBEETLE_CPG_TARGET_BENCHMARK firmware build variant, disabled by default, that uses STM32F407 DWT cycle counting around:
  - one nominal 10 ms CPG substep;
  - 20 ms catch-up;
  - 70 ms catch-up;
  - 100 ms catch-up;
  - maximum bounded catch-up of 100 substeps.
- [ ] Add `RoboBeetleFirmware/Core/App/cpg_target_benchmark.h` and
  `cpg_target_benchmark.c` with a debugger-readable volatile report containing
  SystemCoreClock, repetition count, min/median/max cycles, and converted
  microseconds for every required case. Keep the real `__DSB`/`__ISB` path
  target-only; the host compile contract may use a no-op barrier macro solely
  to check the C/header integration on x86.
- [ ] Ensure the target benchmark uses DWT->CYCCNT with the required counter enable, barriers, warm-up, and repeated measurements. Report clock frequency, repetition count, min/max/median or equivalent robust statistic, cycles, and converted microseconds. Keep benchmark output separate from Protocol V2 and do not alter runtime behavior when the option is off.
- [ ] Add a host compile contract for benchmark-disabled and benchmark-enabled preprocessor paths where MCU headers are available; do not pretend the host compiler is target evidence.
- [ ] Use the actual firmware toolchain to record baseline and CPG image sizes. If arm-none-eabi-gcc or the build tooling is absent, record the exact command and failure as [UNKNOWN]/not run; do not switch to float and do not claim target performance.
- [ ] Record one nominal 10 ms timing, representative 20/70/100 ms catch-up timing, and worst bounded catch-up timing in the performance document. State whether the measured double implementation meets the project's real-time budget. If it does not, stop at evidence and mark a separate future double-reference/float-production-parity design; do not implement that redesign in this task.

### Task 6: Add long-run oscillator period/frequency evidence

**Files:**
- Create: `RoboBeetleFirmware/tests/test_cpg_period.c`
- Modify: `RoboBeetleFirmware/tests/run_host_tests.ps1`
- Document: `docs/cpg-gait-core.md` and `docs/cpg-gait-performance.md` only for the measured result

- [ ] **Step 1: Define a reproducible event and report calculation.** After transient exclusion, detect a rising crossing of a selected node's raw output around its midpoint and record the first several crossing timestamps:

~~~c
const double midpoint = 0.0;
if ((previous_output < midpoint) &&
    (current_output >= midpoint))
{
    crossing_ms[crossing_count++] = elapsed_ms;
}
const double measured_period_s =
    ((double)(crossing_ms[last] - crossing_ms[first]) / 1000.0) /
    (double)(last - first);
const double measured_frequency_hz = 1.0 / measured_period_s;
const double ratio = measured_period_s / nominal_period_s;
printf("nominal_period_s=%.9f measured_period_s=%.9f "
       "measured_frequency_hz=%.9f ratio=%.9f cycles=%u\n",
       nominal_period_s, measured_period_s, measured_frequency_hz,
       ratio, (unsigned)(last - first));
~~~

The test must run the same initialized production profile twice, use a fixed transient-exclusion count, require at least three complete cycles, and compare the two reports within the stated millisecond resolution tolerance.

- [ ] Add test_cpg_period.c that runs the production CPG profile for a long, deterministic multi-cycle window after a documented transient exclusion.
- [ ] Detect a stated phase/output crossing or another reproducible oscillator event, record event timestamps in milliseconds, calculate measured period and frequency, and report:
  - nominal period parameter;
  - measured steady-state period;
  - measured frequency;
  - measured/nominal ratio;
  - number of cycles and transient-exclusion window.
- [ ] Do not assert that the measured result must equal exactly 2.0 s or 0.5 Hz. Assert only finite, positive, repeatable behavior and document any relationship or discrepancy caused by beta, theta_dot, and k_v.
- [ ] Run the test twice with the same build and deterministic initialization; require reported values to agree within the documented tolerance.

### Task 7: Update design and integration documentation

**Files:**
- Modify: `docs/superpowers/specs/2026-09-14-cpg-gait-core-design.md`
- Create: `docs/cpg-gait-core.md`
- Create: `docs/cpg-gait-performance.md`
- Modify: `README.md` only if a single architecture link is needed

- [ ] Update docs/superpowers/specs/2026-09-14-cpg-gait-core-design.md only where implementation evidence requires it:
  - keep exact theta_dot_i source semantics and source ordering;
  - keep T=2.0 s as a nominal period parameter;
  - keep safety-before-catch-up as a frozen contract;
  - keep approved architecture and all exclusions unchanged.
- [ ] Add docs/cpg-gait-core.md with source provenance, complete state table including theta_dot_i, source-compatible equations, state/output order, golden-vector field list, adapter topology, and explicit evidence labels [SRC], [MODEL], [DOC], [UNKNOWN].
- [ ] Add docs/cpg-gait-performance.md with target measurement procedure, output tables, and honest unavailable-toolchain boundary.
- [ ] Add only a minimal README link if needed; do not turn this task into a broad documentation rewrite.
- [ ] Keep paper/source disagreements labeled as unresolved paper/source discrepancies. Do not silently replace generated source 2*beta*(1-beta), source beta .75, or generated Forward Euler ordering with paper alternatives.

### Task 8: Verification and review

**Files:**
- Read-only verification: all changed files in this plan plus the original checkout status
- No source changes are allowed during this task except the fixes required by a failing verification command

- [ ] Run git diff --check.
- [ ] Run rg -n -i ("T" + "B" + "D|" + "T" + "O" + "D|" + "F" + "I" + "X" + "M" + "E") docs/cpg-gait-core.md docs/cpg-gait-performance.md docs/superpowers/specs/2026-09-14-cpg-gait-core-design.md and resolve every hit in changed documentation.
- [ ] Run the complete host test runner. The expected final count is existing 20 executables plus four new CPG core, adapter, safety/catch-up, and period executables, plus nine app/backend/benchmark compile-contract objects. Report actual count and every failure.
- [ ] Run app compile contract with both backend selections and any available CMake/Ninja firmware configure/build. If Qt6 is unavailable, report exact configure boundary and do not label it a pass.
- [ ] Run ARM target build/size/timing commands if toolchain is available. Otherwise include exact not-run evidence and leave performance status [UNKNOWN].
- [ ] Inspect git status --short, git diff --stat, and full diff for scope. Confirm no daplink.cfg, leak-telemetry plan, Qt, Protocol, calibration, or unrelated files changed.
- [ ] Confirm all numeric golden vectors include theta_dot and UnitDelay/relevant derivative state, not only actuator outputs.
- [ ] Before claiming completion, perform a fresh verification pass and report source compatibility, host results, target evidence, and unrun boundaries separately.

### Task 9: External review handoff

**Files:**
- Modify: `docs/cpg-gait-pr-body.md` only if a temporary PR body is needed, and delete it with `apply_patch` after use
- Git state: branch `feature/cpg-gait-core` and its configured remote

- [ ] Review final diff against approved design and this plan, focusing on source order, pre-Euler theta_dot, double precision, safety ordering, adapter-only semantics, and clamp ownership.
- [ ] If review finds an implementation defect, return to a failing test first, then fix and rerun affected verification.
- [ ] Commit implementation and documentation with focused commits; do not squash commits if a merge later occurs.
- [ ] Push feature/cpg-gait-core only after local verification and only to configured origin.
- [ ] Open or update a GitHub PR targeting main, clearly label target performance as measured or unavailable, and leave merge/merge strategy to the user or ChatGPT external review.
- [ ] Final handoff must say READY FOR EXTERNAL GITHUB REVIEW only when all required local work is complete; never claim hardware/ARM timing unless directly measured on target.
