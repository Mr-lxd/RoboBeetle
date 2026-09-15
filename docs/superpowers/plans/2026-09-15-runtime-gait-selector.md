# Runtime SimpleGait / CPG Selector Implementation Plan

> For agentic workers: use the subagent-driven-development or executing-plans workflow to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Add a STOPPED-only STM32-owned runtime selector between the existing SimpleGait and CPG generators, expose it as an ACK-confirmed Qt command/UI, and preserve all frozen motion, calibration, clock, UART, and scheduling boundaries.

**Architecture:** Additive multi-backend registration keeps the existing single-generator motion_manager_init() source-compatible for legacy/custom host fixtures while production uses motion_manager_init_with_backends() with an explicit initial backend. MotionManager owns backend validation, reset, selection, and state; Protocol V2 only decodes/maps; Qt tracks one outstanding selector request separately from its ACK-confirmed local state. A backend-level CPG reset reconstructs the core from its preserved profile so mode-mutated target amplitudes cannot survive a switch.

**Tech Stack:** Firmware C11, the existing PowerShell host runner, Qt 6 C++20, CMake/CTest, FakeTransport, Protocol V2 COBS/CRC framing, Git worktree, and GitHub CLI.

---

## File map and locked scope

Files to modify:

- RoboBeetleFirmware/Core/Motion/motion_types.h — valid backend enum and internal UNSPECIFIED sentinel.
- RoboBeetleFirmware/Core/Motion/gait_generator.h — backend-local reset callback.
- RoboBeetleFirmware/Core/Motion/simple_gait_generator.h/.c — deterministic phase reset callback.
- RoboBeetleFirmware/Core/Motion/cpg_gait_generator.h/.c — profile-preserving full generator reset callback.
- RoboBeetleFirmware/Core/Motion/motion_manager.h/.c — dual registration, current backend, selector result/API, and STOPPED-first switching.
- RoboBeetleFirmware/Core/App/app_main.c — register both generators while preserving MOTION_DEFAULT_GAIT_BACKEND_CPG.
- RoboBeetleFirmware/Core/Inc/rb_protocol_v2.h — 0x16 command ID only; wire result values remain unchanged.
- RoboBeetleFirmware/Core/Communication/protocol_dispatcher.c — exact payload gate, MotionManager call, and result mapping.
- RoboBeetleFirmware/tests/app_main_backend_tests.c and RoboBeetleFirmware/tests/run_host_tests.ps1 — update the compile-contract wrapper to the explicit multi-backend initializer.
- RoboBeetleFirmware/tests/test_cpg_gait_generator.c — CPG reset regressions.
- RoboBeetleFirmware/tests/motion_manager_tests.c — selector and reset lifecycle regressions.
- RoboBeetleFirmware/tests/protocol_dispatcher_tests.c — 0x16 decode, precedence, ACK/result, and no-side-effect regressions.
- RoboBeetleConsole/src/protocol/Packet.h — Qt message ID and known-type table.
- RoboBeetleConsole/src/robot/RobotCommand.h — Qt backend enum and value validation.
- RoboBeetleConsole/src/robot/RobotController.h/.cpp — command API, pending/confirmed state, sequence-first ACK/error correlation, and selector serialization.
- RoboBeetleConsole/src/ui/MainWindow.h/.cpp — backend combo and state/status refresh.
- RoboBeetleConsole/tests/protocol_tests.cpp — Qt 0x16 codec contract.
- RoboBeetleConsole/tests/robot_controller_tests.cpp — selector ACK, serialization, timeout/error, and reconnect tests.
- RoboBeetleConsole/tests/main_window_tests.cpp — combo lifecycle tests.
- RoboBeetleConsole/docs/protocol.md — wire and ACK-confirmed local-state documentation.

No other production surfaces are in scope. Do not edit CPG equations, Servo calibration, Clock/RCC, PWM, UART/APC scheduling, foreground order, safety timeout constants, Motion cadence, or stutter/jitter behavior.

The isolated worktree is C:\Users\laixindong\.config\superpowers\worktrees\RoboBeetle\codex-runtime-gait-selector on branch codex/runtime-gait-selector. Base is 13ea8aba453cb4bc49aa04c71748fb630563b15f. Firmware baseline passed. Qt baseline was not run because the environment lacks a Qt6 CMake package; any later Qt result must preserve that distinction.

## Task 1: Add failing generator reset tests

Files:
- Test: RoboBeetleFirmware/tests/test_cpg_gait_generator.c
- Test: RoboBeetleFirmware/tests/simple_gait_generator_tests.c

- [ ] Step 1: Write the failing SimpleGait reset test.

Add a test that initializes SimpleGait, advances it by 250 ms, calls the not-yet-existing simple_gait_generator_reset(), and asserts simple_gait_generator_phase() is exactly 0.0F. Register it in main(). It must fail at compile/link time before implementation.

    static void test_reset_returns_to_deterministic_initial_phase(void)
    {
        simple_gait_generator_t generator;

        simple_gait_generator_init(&generator);
        simple_gait_generator_advance(&generator, 250U);
        simple_gait_generator_reset(&generator);

        expect(simple_gait_generator_phase(&generator) == 0.0F,
               "SimpleGait reset must restore phase zero");
    }

- [ ] Step 2: Write the failing CPG profile-preserving reset test.

In test_cpg_gait_generator.c, construct a non-default profile, initialize fresh and exercised with it, sample/advance exercised in MOTION_TURN_LEFT, call cpg_gait_generator_reset(), and compare profile plus every public cpg_core_t field, especially params.target_amplitude, oscillator/runtime memories, elapsed_remainder_ms, executed_step_count, and discarded_catch_up_count, against fresh. Run the same helper with the production profile. Add string.h if needed for the existing test style.

    cpg_gait_generator_init_with_profile(&fresh, &profile);
    cpg_gait_generator_init_with_profile(&exercised, &profile);
    cpg_gait_generator_advance(&exercised, 37U);
    expect(cpg_gait_generator_sample(&exercised, MOTION_TURN_LEFT,
                                      1.0F, 1.0F, &targets),
           "custom CPG TURN sample should succeed");
    cpg_gait_generator_advance(&exercised, 113U);
    cpg_gait_generator_reset(&exercised);

    expect(memcmp(&exercised.profile, &fresh.profile,
                  sizeof(profile)) == 0,
           "CPG reset must preserve the configured profile");
    expect(memcmp(&exercised.core, &fresh.core,
                  sizeof(exercised.core)) == 0,
           "CPG reset must reconstruct the fresh profile-matched core");

- [ ] Step 3: Run the RED checkpoint.

    Set-Location RoboBeetleFirmware
    powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1

Expected: new reset references fail because the APIs do not exist. Do not add production implementation before this failure is observed.

## Task 2: Add failing MotionManager backend tests

Files:
- Test: RoboBeetleFirmware/tests/motion_manager_tests.c
- Later implementation: RoboBeetleFirmware/Core/Motion/motion_manager.h/.c

- [ ] Step 1: Add a dual-backend fixture.

Add a fixture containing real SimpleGait and CPG objects/interfaces and initialize it with motion_manager_init_with_backends(..., MOTION_GAIT_BACKEND_CPG). Leave existing custom fixtures on the legacy single-generator initializer.

- [ ] Step 2: Add stopped, moving, reset, and infrastructure assertions.

Assert default CPG, STOPPED CPG to SimpleGait, SimpleGait to CPG, same-backend OK/no-op, no Servo write, no Motion start, unchanged STOPPED state, and deterministic backend getter.

Start Motion and assert both same-backend and raw backend byte 0xff return MOTION_MANAGER_RESULT_BUSY before value validation. Capture backend, Motion state/mode, last targets, generator state, and driver write count and assert they remain unchanged. Request STOP, assert selector returns BUSY during STOPPING, complete the existing ramp, and assert selection is then allowed.

Keep a legacy fixture on motion_manager_init(). Assert its backend getter is MOTION_GAIT_BACKEND_UNSPECIFIED. A valid selector against an unregistered/missing reset backend must return MOTION_MANAGER_RESULT_HARDWARE_FAILURE with no active-generator, state, target, or Servo mutation.

- [ ] Step 3: Run the RED checkpoint.

    Set-Location RoboBeetleFirmware
    powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1

Expected: compilation fails on the new enum, initializer, getter, and setter references.

## Task 3: Add failing Protocol and default compile-contract tests

Files:
- Test: RoboBeetleFirmware/tests/protocol_dispatcher_tests.c
- Test/compile contract: RoboBeetleFirmware/tests/app_main_backend_tests.c and RoboBeetleFirmware/tests/run_host_tests.ps1
- Later implementation: RoboBeetleFirmware/Core/Inc/rb_protocol_v2.h and RoboBeetleFirmware/Core/Communication/protocol_dispatcher.c

- [ ] Step 1: Extend the Protocol fixture.

Add a CPG generator to the existing fixture and initialize production-style fixtures explicitly with CPG. Add a helper for a one-byte RBP2_MSG_SET_GAIT_BACKEND frame.

- [ ] Step 2: Add exact payload and precedence tests.

Assert exact one-byte SimpleGait returns RBP2_RESULT_OK, length zero or greater than one returns RBP2_RESULT_INVALID_PAYLOAD, STOPPED plus 0xff returns RBP2_RESULT_INVALID_PAYLOAD, RUNNING plus 0xff returns RBP2_RESULT_BUSY, and STOPPING plus either backend returns RBP2_RESULT_BUSY. Complete STOP, select the other backend, and assert the switch returns OK without Servo writes or Motion start. Assert same-backend STOPPED selection is OK/no-op.

- [ ] Step 3: Update the failing app compile-contract wrapper.

Change __wrap_motion_manager_init to __wrap_motion_manager_init_with_backends, capture both interfaces and initial_backend, and assert both interfaces are registered while initial_backend still follows MOTION_DEFAULT_GAIT_BACKEND_CPG. The normal/default and explicit =0/=1 runner variants remain required.

- [ ] Step 4: Run the RED checkpoint.

    Set-Location RoboBeetleFirmware
    powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1

Expected: the new protocol enum/case and wrapper references fail to compile or link.

## Task 4: Add failing Qt tests

Files:
- Test: RoboBeetleConsole/tests/protocol_tests.cpp
- Test: RoboBeetleConsole/tests/robot_controller_tests.cpp
- Test: RoboBeetleConsole/tests/main_window_tests.cpp

- [ ] Step 1: Add the Qt wire contract.

Add MessageType::SetGaitBackend = 0x16 to the codec round-trip tests, assert isKnownMessageType(0x16), and assert GaitBackend::SimpleGait is 0 and GaitBackend::CPG is 1.

- [ ] Step 2: Add Controller selector tests.

Using FakeTransport and the existing lastPacket()/acknowledge() helpers, assert the initial confirmed state is absent/UNKNOWN, the outgoing payload is exactly one byte, and a second selector is rejected while the first is queued, in flight, deferred, or retrying.

Add the three sequence/type regressions:

    selector request sequence S + ACK sequence S+1
        pending remains; confirmed remains unchanged

    selector request sequence S + ACK sequence S with wrong request type
        pending clears; confirmed remains unchanged

    selector request sequence S + matching SetGaitBackend ACK OK
        pending clears; requested backend becomes confirmed

Add matching BUSY, invalid/error, terminal timeout, and write-failure cases; each clears pending and preserves the prior confirmed backend. Add disconnect/reconnect after a confirmed backend and assert UNKNOWN with no automatic selector command.

- [ ] Step 3: Add MainWindow selector tests.

Find the Motion panel combo, assert an explicit Unknown display plus SimpleGait and CPG choices, assert disconnected disablement, choose SimpleGait after connection, verify the exact outgoing packet, verify it is not confirmed before ACK, inject BUSY and verify the prior state is preserved, then inject matching OK and verify confirmation. Verify a second selection is disabled/rejected during pending.

- [ ] Step 4: Run the Qt RED checkpoint when Qt6 is available.

    $ErrorActionPreference = 'Stop'
    cmake -S . -B ..\build\console-gait-selector -G Ninja -DBUILD_TESTING=ON
    if ($LASTEXITCODE -ne 0) { throw 'Qt configure failed' }
    cmake --build ..\build\console-gait-selector
    if ($LASTEXITCODE -ne 0) { throw 'Qt build failed' }
    ctest --test-dir ..\build\console-gait-selector --output-on-failure
    if ($LASTEXITCODE -ne 0) { throw 'Qt tests failed' }

Expected: new Qt tests fail before implementation. If Qt6 is unavailable, record NOT RUN — Qt6 development package/config unavailable.

## Task 5: Implement generator reset APIs

Files:
- Modify: RoboBeetleFirmware/Core/Motion/gait_generator.h
- Modify: RoboBeetleFirmware/Core/Motion/simple_gait_generator.h/.c
- Modify: RoboBeetleFirmware/Core/Motion/cpg_gait_generator.h/.c

- [ ] Step 1: Add reset to gait_generator_ops_t.

    void (*reset)(void *context);

Keep designated initializers source-compatible. A legacy/custom interface with a null reset remains valid for the legacy initializer but is not switchable as a runtime backend.

- [ ] Step 2: Implement SimpleGait reset.

Declare and implement simple_gait_generator_reset() by calling simple_gait_generator_init(). Add its context adapter and .reset ops entry. Do not change frequency, profile constants, signs, or sampling.

- [ ] Step 3: Implement full CPG reset.

Declare and implement cpg_gait_generator_reset() by copying generator->profile and passing that copy to cpg_gait_generator_init_with_profile(). This reconstructs all model parameters and runtime fields, restoring canonical profile-derived target amplitudes instead of retaining a TURN-mutated params.target_amplitude. Do not use cpg_core_reset() as the sole operation and do not edit cpg_core.c or any equation.

- [ ] Step 4: Run the generator tests.

    Set-Location RoboBeetleFirmware
    powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1

Expected: reset tests and all pre-existing generator/safety tests pass.

- [ ] Step 5: Commit the generator slice.

    git add RoboBeetleFirmware/Core/Motion/gait_generator.h RoboBeetleFirmware/Core/Motion/simple_gait_generator.h RoboBeetleFirmware/Core/Motion/simple_gait_generator.c RoboBeetleFirmware/Core/Motion/cpg_gait_generator.h RoboBeetleFirmware/Core/Motion/cpg_gait_generator.c RoboBeetleFirmware/tests/test_cpg_gait_generator.c RoboBeetleFirmware/tests/simple_gait_generator_tests.c
    git commit -m "feat: add deterministic gait generator resets"

## Task 6: Implement MotionManager ownership and selector

Files:
- Modify: RoboBeetleFirmware/Core/Motion/motion_types.h
- Modify: RoboBeetleFirmware/Core/Motion/motion_manager.h/.c
- Test: RoboBeetleFirmware/tests/motion_manager_tests.c

- [ ] Step 1: Add explicit backend values.

Define MOTION_GAIT_BACKEND_SIMPLE_GAIT=0, MOTION_GAIT_BACKEND_CPG=1, a count sentinel, and internal-only MOTION_GAIT_BACKEND_UNSPECIFIED=0xff. Add a validity helper accepting only 0 and 1 and compile-time assertions. Never infer identity from pointers/context.

- [ ] Step 2: Add explicit multi-backend initialization.

Keep motion_manager_init(..., gait_generator_t generator) unchanged for legacy fixtures. It retains the active supplied generator, reports UNSPECIFIED, and leaves selector unavailable. Add:

    void motion_manager_init_with_backends(
        motion_manager_t *manager,
        servo_service_t *servo_service,
        safety_supervisor_t *safety_supervisor,
        gait_generator_t simple,
        gait_generator_t cpg,
        motion_gait_backend_t initial_backend);

Store both registered interfaces, the active interface/backend, and selector availability. Production passes the explicit initial backend chosen from MOTION_DEFAULT_GAIT_BACKEND_CPG. Do not reset or advance during initialization.

- [ ] Step 3: Implement exact setter precedence.

Add motion_manager_set_gait_backend() and motion_manager_gait_backend(). Use this order:

    if (manager == NULL)
        return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
    if (manager->state != MOTION_STATE_STOPPED)
        return MOTION_MANAGER_RESULT_BUSY;
    if (!motion_gait_backend_is_valid(backend))
        return MOTION_MANAGER_RESULT_INVALID_BACKEND;
    if (selector unavailable or target ops/context/reset infrastructure is missing)
        return MOTION_MANAGER_RESULT_HARDWARE_FAILURE;
    if (backend == manager->gait_backend)
        return MOTION_MANAGER_RESULT_OK;
    target.ops->reset(target.context);
    manager->generator = target;
    manager->gait_backend = backend;
    return MOTION_MANAGER_RESULT_OK;

Add MOTION_MANAGER_RESULT_INVALID_BACKEND without changing existing wire-result numbers. The setter must not call Servo APIs, write targets, alter Motion state/mode/ownership, advance scheduler, or start/stop Motion. A missing target infrastructure must be detected before mutation.

- [ ] Step 4: Run MotionManager and full Firmware tests.

    Set-Location RoboBeetleFirmware
    powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1

Expected: all existing and selector tests pass.

- [ ] Step 5: Commit the MotionManager slice.

    git add RoboBeetleFirmware/Core/Motion/motion_types.h RoboBeetleFirmware/Core/Motion/motion_manager.h RoboBeetleFirmware/Core/Motion/motion_manager.c RoboBeetleFirmware/tests/motion_manager_tests.c
    git commit -m "feat: own runtime gait backend selection in MotionManager"

## Task 7: Integrate production default and Protocol V2

Files:
- Modify: RoboBeetleFirmware/Core/App/app_main.c
- Modify: RoboBeetleFirmware/Core/Inc/rb_protocol_v2.h
- Modify: RoboBeetleFirmware/Core/Communication/protocol_dispatcher.c
- Modify: RoboBeetleFirmware/tests/protocol_dispatcher_tests.c
- Modify: RoboBeetleFirmware/tests/app_main_backend_tests.c
- Modify: RoboBeetleFirmware/tests/run_host_tests.ps1

- [ ] Step 1: Switch app_main to dual registration.

Initialize both existing objects exactly as today. Replace the conditional single-interface motion_manager_init() calls with one motion_manager_init_with_backends() call passing both interfaces and an explicit CPG or SimpleGait initial value under MOTION_DEFAULT_GAIT_BACKEND_CPG. Leave benchmark behavior unchanged and keep ROBOBEETLE_CPG_TARGET_BENCHMARK=OFF for production.

- [ ] Step 2: Add the 0x16 dispatcher case.

Add RBP2_MSG_SET_GAIT_BACKEND = 0x16 after 0x15. Do not add a wire result. In the dispatcher, first reject payload_length != 1 with RBP2_RESULT_INVALID_PAYLOAD. For exactly one byte, cast the raw value to motion_gait_backend_t and call motion_manager_set_gait_backend(), then map the result. Add INVALID_BACKEND to INVALID_PAYLOAD and null-manager handling to existing HARDWARE_FAILURE. Do not pre-validate the byte or inspect state in the dispatcher, so RUNNING + 0xff remains BUSY. Preserve the existing successful duplicate cache.

- [ ] Step 3: Update app compile-contract wrapper.

Capture both generator interfaces and initial_backend in __wrap_motion_manager_init_with_backends(). Change the runner linker option to -Wl,--wrap=motion_manager_init_with_backends. Preserve all default, override, JY901S API, and benchmark compile variants.

- [ ] Step 4: Run Firmware tests and inspect scope.

    Set-Location RoboBeetleFirmware
    powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1
    if ($LASTEXITCODE -ne 0) { throw 'Firmware host gate failed' }
    git diff --check
    git diff --stat

Expected: all Firmware tests and compile contracts pass.

- [ ] Step 5: Commit the Firmware protocol slice.

    git add RoboBeetleFirmware/Core/App/app_main.c RoboBeetleFirmware/Core/Inc/rb_protocol_v2.h RoboBeetleFirmware/Core/Communication/protocol_dispatcher.c RoboBeetleFirmware/tests/protocol_dispatcher_tests.c RoboBeetleFirmware/tests/app_main_backend_tests.c RoboBeetleFirmware/tests/run_host_tests.ps1
    git commit -m "feat: add Protocol V2 runtime gait selector"

## Task 8: Implement Qt types, Controller state, and correlation

Files:
- Modify: RoboBeetleConsole/src/protocol/Packet.h
- Modify: RoboBeetleConsole/src/robot/RobotCommand.h
- Modify: RoboBeetleConsole/src/robot/RobotController.h/.cpp
- Test: RoboBeetleConsole/tests/protocol_tests.cpp
- Test: RoboBeetleConsole/tests/robot_controller_tests.cpp

- [ ] Step 1: Add Qt wire enums.

Add MessageType::SetGaitBackend = 0x16 and include it in isKnownMessageType(). Add GaitBackend values SimpleGait=0 and CPG=1 plus Count and a validity helper. Keep static assertions aligned with Firmware.

- [ ] Step 2: Add Controller state and command API.

Add setGaitBackend(), confirmedGaitBackend(), requestedGaitBackend(), and isGaitBackendChangePending(). Add optional backend metadata to PendingRequest and QueuedCommand, plus optional confirmedGaitBackend_ and pendingGaitBackend_ fields and one gaitBackendStateChanged() signal. setGaitBackend() rejects disconnected/invalid calls and rejects a second selector whenever the first is queued, deferred, in flight, or retrying. It sends exactly one payload byte and no Servo mask or MotionRequest.

- [ ] Step 3: Preserve sequence-first ACK correlation.

In handleAck(), locate the request by requestSequence first. If no request is found, log/handle the unmatched ACK and leave selector pending. Once a selector request is found, clear its pending state only for a matching sequence. A wrong request type clears selector pending but never confirms. A matching SetGaitBackend with result OK clears pending and updates confirmed. A matching non-OK result clears pending and preserves confirmed. Apply the same sequence-first rule to matching Error, timeout, and write-failure paths. A selector retry write failure terminates that selector request so no late frame can confirm it. Preserve non-selector request behavior.

- [ ] Step 4: Reset selector state on transport lifecycle.

Extend resetSchedulerState() to clear confirmed and pending optionals and emit gaitBackendStateChanged() when state changes. Both initial connection and reconnect expose UNKNOWN. Do not add query/telemetry or auto-commands.

- [ ] Step 5: Run Qt tests when available.

    $ErrorActionPreference = 'Stop'
    Set-Location RoboBeetleConsole
    cmake -S . -B ..\build\console-gait-selector -G Ninja -DBUILD_TESTING=ON
    if ($LASTEXITCODE -ne 0) { throw 'Qt configure failed' }
    cmake --build ..\build\console-gait-selector
    if ($LASTEXITCODE -ne 0) { throw 'Qt build failed' }
    ctest --test-dir ..\build\console-gait-selector --output-on-failure
    if ($LASTEXITCODE -ne 0) { throw 'Qt tests failed' }

If Qt6 remains unavailable, report it as not run.

- [ ] Step 6: Commit the Qt Controller slice.

    git add RoboBeetleConsole/src/protocol/Packet.h RoboBeetleConsole/src/robot/RobotCommand.h RoboBeetleConsole/src/robot/RobotController.h RoboBeetleConsole/src/robot/RobotController.cpp RoboBeetleConsole/tests/protocol_tests.cpp RoboBeetleConsole/tests/robot_controller_tests.cpp
    git commit -m "feat: add ACK-confirmed Qt gait selector"

## Task 9: Implement MainWindow and protocol documentation

Files:
- Modify: RoboBeetleConsole/src/ui/MainWindow.h/.cpp
- Test: RoboBeetleConsole/tests/main_window_tests.cpp
- Modify: RoboBeetleConsole/docs/protocol.md

- [ ] Step 1: Add the backend combo.

Create a QComboBox in the existing Motion/Gait panel with an explicit Unknown item and the two valid choices. Connect user changes only to RobotController::setGaitBackend(). Do not call generator, Motion, Servo, PWM, or trajectory APIs from this path.

- [ ] Step 2: Refresh UI from confirmed/pending state.

Use QSignalBlocker while reflecting Controller state. While pending, show the requested item only with an explicit pending indication and disable a second change. On rejection/timeout/error restore confirmed or Unknown. On matching OK show confirmed. Keep the combo disabled while disconnected and do not treat the local combo value as authoritative.

- [ ] Step 3: Document the contract.

Add a SetGaitBackend 0x16 section and matrix row to RoboBeetleConsole/docs/protocol.md covering exact payload values, STOPPED-only Firmware behavior, BUSY, invalid mapping, ACK-confirmed local state, sequence-first correlation, one outstanding selector, and no backend telemetry/query or Qt trajectory stream.

- [ ] Step 4: Run UI tests and commit.

Run the explicit Qt command from Task 8 when available, then:

    git diff --check
    git add RoboBeetleConsole/src/ui/MainWindow.h RoboBeetleConsole/src/ui/MainWindow.cpp RoboBeetleConsole/tests/main_window_tests.cpp RoboBeetleConsole/docs/protocol.md
    git commit -m "feat: expose runtime gait selector in Console"

## Task 10: Final verification, review, push, and PR

Files:
- Review all branch changes; no production surface outside the file map.

- [ ] Step 1: Run the complete Firmware host gate.

    Set-Location RoboBeetleFirmware
    powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1
    if ($LASTEXITCODE -ne 0) { throw 'Firmware host gate failed' }

Expected: all Firmware tests and app/backend/benchmark compile contracts pass.

- [ ] Step 2: Run Qt configure/build/CTest with explicit exit checks.

If Qt6 is installed, require configure, build, and CTest to pass. If it is not installed, report exactly: Qt tests: NOT RUN — Qt6 development package/config unavailable.

- [ ] Step 3: Verify scope and clean state.

    git diff --check
    git diff --stat origin/main...HEAD
    git diff origin/main...HEAD -- RoboBeetleFirmware/Core RoboBeetleConsole/src
    git status --short --branch

Confirm no CPG math/core equation, Servo calibration, Clock/RCC, PWM, UART/APC, scheduling, queue budget, Safety timeout, or stutter/jitter edits beyond selector plumbing. Confirm the detached root worktree user files remain untouched.

- [ ] Step 4: Perform implementation review.

Check state-first precedence including RUNNING/STOPPING same-backend and invalid-byte BUSY; profile-matched CPG reset; missing infrastructure no mutation; no Servo/Motion/scheduler side effects; sequence-first ACK correlation; one outstanding selector; reconnect UNKNOWN; preserved compile-time CPG default; and production benchmark OFF.

- [ ] Step 5: Push and create the PR without merging.

    git push -u origin codex/runtime-gait-selector
    gh pr create --base main --head codex/runtime-gait-selector --title "feat: add runtime SimpleGait/CPG selector" --body "Implements a STOPPED-only STM32-owned runtime gait backend selector with ACK-confirmed Qt state. No CPG math, Servo calibration, Clock, UART, scheduling, or stutter/jitter changes."

Do not merge. Report base SHA, final HEAD SHA, PR number, Firmware result, Qt result, git diff --check, and the frozen-scope summary.

