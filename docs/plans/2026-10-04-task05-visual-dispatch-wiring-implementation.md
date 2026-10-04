# Task05 PR3 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development for task-by-task implementation and spec/quality review. Steps use checkbox tracking.

**Goal:** Connect the approved visual dispatch policy to the remote console behind a session-only disabled-by-default switch, with complete fake/loopback coverage and CSVv3 evidence.
**Architecture:** Existing RemoteRobotController is the only RBRP owner. A small request-ID adapter and GUI-thread 50ms dispatch session connect the pure policy; MainWindow binds operator controls and complete diagnostic snapshots; VisualCsvLogger records send/outcome rows.
**Tech Stack:** C++20, Qt6.11.2/MinGW13.1, CMake/Ninja, offscreen Qt and loopback TCP simulation.

## Task1: Runtime evidence, RBRP adapter and session

Files: controller/IConsoleController.h; remote/RemoteRobotController.{h,cpp}; vision/VisualDispatchStateMachine.h (read-only status getters); create vision/VisualDispatchSession.{h,cpp}, small controller-port files/event header as needed; CMakeLists.txt; remote/controller/session test files. No MainWindow or CSV edits.

- [ ] Write RED tests first: accepted Enable/Neutral/Angle known, PWM/Disable unknown, repeated Enable after PWM preserves unknown; loss/session reconstruction clears masks, old ACK ignored. Check START result6 exposes possible pose mismatch.
- [ ] Add minimal API declarations/stubs solely to compile behavioral RED tests. Observe runtime assertion failures; save named RED logs. Suggested controller interface:
```cpp
virtual std::optional<quint32> submitVisualMotion(MotionMode mode) { return {}; }
virtual quint16 inferredPoseKnownMask() const { return 0; }
// Typed signal: request ID, Ok/Busy/Rejected/OutcomeUnknown, raw result, RTT.
```
- [ ] Implement evidence and terminal-result path inside the existing RemoteRobotController, matching servo_service.c and gateway wire kind/sequence validation. Automatic STOP bypasses manual stopped-state/dedup guards. Surface Unknown before cleanup when STOP is still possible.
- [ ] Write adapter RED loopback tests for ID correlation, StartMotion(mode)/StopMotion payloads, Ok/Busy7/Rejected/Unknown/Cancelled and terminalize_outstanding wire messages, stale ACKs, retries and synchronous callback mapping. Implement minimal adapter; no second TCP session.
- [ ] Write dispatch-session RED tests: off by default; explicit arm/sign; exact snapshot.command.ex_f; manual/dwell/pending ACK; feature-off conditional operator STOP and result UI state; 50ms no-frame timeout/retry/alert; AwaitingVideo400ms preserves confirmed Forward, snapshot.STALE500ms stops/disarms. Use a fake controller and injected monotonic clock for boundaries plus real timer wakeup tests.
- [ ] Implement a GUI-thread50ms timer, snapshot source, session-only sign/feature settings, one-shot operator STOP on off iff armed or automatic STOP awaiting. Expose status getters/signals and dispatch send/outcome events (policy ID, wire ID, command, result, RTT, monotonic time). No additional awaitingVideo STALE rule.
- [ ] Run new targets and existing policy/remote targets; spec compliance review then quality review; fix findings and commit.

## Task2: Operator UI and CSVv3 wiring

Files: ui/MainWindow.{h,cpp}; vision/VisualCsvLogger.{h,cpp}; vision/VisualPolicyConfig.h schema; existing CSV/UI tests; new MainWindow loopback integration tests/CMake target if needed.

- [ ] Write and run RED UI tests finding session-only default-off checkbox, separate Arm/Disarm, unconfirmed +/- selector and confirmation, armReasonName/current-confirmed-mode/latched alert/result display. Two window instances prove nonpersistence; direct backend never submits.
- [ ] Implement focused UI binding to runtime session; pass the complete diagnostic snapshot.command, preserve off-mode diagnostics, no automatic arm. Changing selected sign invalidates confirmation and safely disarms.
- [ ] Enumerate every movement/servo path before editing it: six motion-mode buttons including disabled Backward, STOP, global enable/disable, individual enable/neutral/PWM/Angle Apply, user PWM/angle spin and slider edits, gait and coordination combo activations, keyboard equivalents. Each test arms then invokes a single path, including dwell/pending ACK, and asserts manualInput plus zero further automatic sends. Existing disabled Emergency action stays disabled; any handler is hooked before actual movement. Do not add unrequested shortcuts.
- [ ] Add minimal manualInput(Motion/Stop) calls before those controller actions/user edits; programmatic refresh must remain signal-blocked. Feature-off after manual takeover with ongoing manual motion emits no STOP.
- [ ] Write CSVv3 RED using named columns and preserved old row content: append policy_request_id,request_id,dispatch_command,dispatch_result,ack_rtt_ms. Record every send/retry and terminal result; diagnostic rows leave them blank. Observe failures before logger changes.
- [ ] Implement logger event rows with existing escaping, size limit, flush and file lifecycle; bump schema v3; bind session dispatch events to logger. Test monotonic RTT, retries/unknown outcomes, columns and existing CSV behavior.
- [ ] Full loopback path: qualify enabled/known masks, direction select+confirm, gate+arm, tracking/ACK motion, stop frame stream, STALE/STOP/disarm, rearm then manual takeover. Verify no physical transport is accessed. Run targets; spec review then code review; fix and commit.

## Task3: Full validation, documentation and draft delivery

- [ ] Run complete Qt build/regression offscreen with existing real-clock CTest isolation; require all tests green. New tests expand baseline28. Fix any failure using TDD, retain all existing assertions.
- [ ] Independent final spec review against all ten user requirements plus latest pose/off/AwaitingVideo corrections, then quality review.
- [ ] Update existing ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md and create Task05PR3 validation document with RED/GREEN logs and explicit pending user-only unloaded/out-of-water checklist: direction two-step confirmation; unplug network legs soft/hold position (not zero); STALE STOP; manual takeover. No claimed hardware acceptance.
- [ ] Build application/test EXEs, calculate SHA256 and record exact source commit. Package new app with windeployqt in a distinct Task05PR3 portable directory; startup smoke only, no robot connections. Keep prior portable packages untouched.
- [ ] Commit/push branch normally, create and attach draft PR; record source/head and hashes in PR description and BUILD_INFO. Verify GitHub head/draft state and clean worktree. Do not merge PR3.

## Commands

```powershell
$env:PATH='D:/Qt/6.11.2/mingw_64/bin;D:/Qt/Tools/mingw1310_64/bin;' + $env:PATH
$env:QT_QPA_PLATFORM='offscreen'; $env:QT_QPA_FONTDIR='C:/Windows/Fonts'
& D:/Qt/Tools/CMake_64/bin/cmake.exe --build C:/Users/laixindong/.codex/worktrees/task05-arming-dry-run-build/qt --parallel4
& D:/Qt/Tools/CMake_64/bin/ctest.exe --test-dir C:/Users/laixindong/.codex/worktrees/task05-arming-dry-run-build/qt --output-on-failure -j4
```

Existing worktree: C:/Users/laixindong/.codex/worktrees/task05-arming-dry-run/RoboBeetle; current branch codex/task05-pr3-visual-dispatch-wiring created directly from main dfd7972. Preserve detached root and historical output folders. Implementers and reviewers receive full task context; one implementer at a time to avoid shared-file conflicts.
