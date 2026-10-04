# Task 05 PR2 — Arming / dispatch policy validation (application stays DRY_RUN)

2026-10-04; baseline `99515ce563ee143c2a2a0224352ee0a9b6321a5a` (PR #44).
PR #45 STOP cadence characterization is a separate branch/PR; this slice does
not require its firmware tests as a runtime dependency and changes no firmware.
Approved contract: [design / plan](plans/2026-10-04-task05-arming-dispatch-design.md).

## Result and isolation

Implemented `VisualDispatchStateMachine` as standalone C++20 logic with a fake
`VisualCommandSendPort`. Current application remains DRY_RUN. The policy is
built only under BUILD_TESTING and linked only to its test; no network/controller
adapter exists. `MainWindow`, RobotController, RemoteRobotController,
RbrpClientSession, transports and Firmware/Pi production inputs are unchanged.
No UI arming button or physical command sender is delivered by this PR.

The caller must supply current gateway authority, enabled/pose-known masks,
explicit direction confirmation and vision state. Default masks are zero and
confirmedTurnSign is absent; arm refusals have typed reasons and readable names.
Required mask defaults to 0x001b (the four paddles used by Forward/Turn), not
an assumption that any servo or all servos remain enabled after disconnect.

## Requirement-to-test matrix

All scenarios use injected monotonic time and record the actual commands emitted
by the policy into the FakePort; they do not connect sockets or hardware.

| Requirement | Test function / assertions |
| --- | --- |
| Default OFF; every arm gate / reason | `armingConditions`: disconnected, no authority, zero/incomplete enabled or known-pose mask, absent/invalid confirmed sign, each non-TRACKING state, invalid config/time; eligible arm. |
| STALE / OFF / link / control loss STOP immediately + disarm | `disarmingSafety`: each loss during dwell and pending motion ACK; recovery never rearms; cleared masks prevent rearm. Also servo/pose/direction loss fail closed. |
| NO_TARGET grace / LOST STOP | `noTargetHoldsConfirmedModeOnly`: confirmed Forward through 1499 ms NO_TARGET sends nothing; only LOST stops. NO_TARGET recovery unchanged does not resend. LOST recovery respects STOP OK+1000 ms. |
| Only manual motion incl manual STOP takes over | `manualTakeover`: explicit Motion and Stop kinds during dwell and pending ACK; clear retries/ACK associations and prohibit later sends. NonMotion preserves arming. |
| ACK OK only confirms mode | `ackModeDwellAndBusy`, `nonStopUnknownAndTimeout`, `additionalAckAndStopBoundaries`: submissions/Busy/STOP timeout never confirm; OK does. |
| Dwell=max(last non-STOP send+1000,last STOP accepted+1000) | `ackModeDwellAndBusy`: 999/1000; `targetLossRecoveryAndStopGate` and additional boundaries: TRACKING during return-to-zero blocked until first STOP OK+1000, then last non-STOP gate separately enforced. |
| Pending ACK blocks non-STOP | First test keeps motion ACK pending; recovery test keeps STOP pending beyond motion dwell and retries only STOP. |
| Busy uses current newest suggestion at next window | Old right request gets Busy; newest left sends at next dwell boundary. If newest suggestion equals confirmed Forward, no retry is sent. |
| Non-STOP timeout / OutcomeUnknown fail-safe | `nonStopUnknownAndTimeout`: +999 not expired, +1000 STOP/disarm; OutcomeUnknown/rejection same-call STOP. Late OK and Busy at deadline cannot evade timeout. |
| STOP episode once; configurable retry | `stopRetriesAnyOkAndStaleMotion`: 999/1000 timeout, new IDs, custom 200 ms boundary, no retry burst after long pause. STOP bypasses dwell and pending non-STOP. |
| Any OK in one STOP episode confirms; old motion cannot overwrite | Both first and retry IDs tested; duplicate OK cannot extend acceptance dwell. Earlier motion or preceding episode ACK ignored. |
| STOP retry terminates / three-timeout alert | OK, manual Stop, link/authority loss all tested. Three consecutive expiries expose alert state; successful OK ends retry. |
| Mapping both confirmed signs | `directionMappingAndHold`: +1/-1 × positive/negative ex × both turn-intent labels; no reuse of DRY_RUN default sign. HOLD sends no command only with a confirmed mode; invalid turn error fails closed. |
| Review edge cases | Decreasing-time arm while armed, rejected-arm time observation, disarmed pending STOP loss via arm rejection; stale ACK cannot revive cancelled episode. |

STOP acceptance uses the local first-OK receipt time, conservatively no earlier
than firmware acceptance. Duplicate episode OKs do not shift this deadline.
This policy cannot measure physical movement or infer it from an ACK.

## RED / GREEN and review evidence

- Initial compiled inert implementation: **195 checks / 153 assertion failures**,
  exit 1. This is real behavioral RED, not a missing-header/compiler failure.
- Initial implementation: **195 checks / 0 failures**.
- Added correlation/recovery edges: 217 checks / 0 failures.
- Late ACK review reproduction: 220 checks / 1 failure; repaired GREEN.
- Rejected-arm clock/loss cases separately reproduced RED then repaired.
- Explicit manual STOP event distinguished from other motion: 246 checks /
  5 failures before dispatch fix, then **246 checks / 0 failures**, exit 0.
- Final independent read-only review found no remaining important issue in
  arming/correlation gates. Reviewer did not claim an independent test rerun.
- GCC 13.1 / C++20, policy and test compile with `-Wall -Wextra -Werror`.

Logs preserved outside repo under
`C:/Users/laixindong/.codex/worktrees/`:
`task05-arming-red.log`, `task05-arming-late-ack-red.log`,
`task05-arming-review-red.log`, `task05-arming-arm-edge-red.log`,
`task05-arming-manual-stop-red.log`, `task05-arming-final-policy-ctest.log`.

## Full Qt regression and pre-existing limits

Baseline full CTest: **25/27 passed**, two existing UI test failures:
`main_window_tests` (IMU/Depth/Protocol/Leak clipping plus a capture close-guard
assertion in that run) and `main_window_layout_tests` (Motion/Gait clipping).

Earlier feature serial CTest: **26/28 passed**, only the same two UI suites fail on
clipping. Final delivery serial CTest: **25/28 passed**, those two UI failures plus
one main_window_visual_error_tests LOST/CSV timer assertion failure. An isolated
rerun passes that suite (11.11 s). Its test and UI implementation are unchanged.
The capture close-guard assertion did not recur in the serial runs.
One intermediate parallel run additionally failed `robot_controller_tests`
(scheduled 170 ms APC220 ACK budget). An isolated rerun and subsequent full serial
run pass it; this timing-sensitive failure is recorded, not erased.

Initial policy CTest passed independently with 246 assertions; revised policy passes 273 assertions. Full Qt suite is
**not all green**; no unrelated UI or controller fixes are included. Baseline /
parallel / serial logs: `task05-arming-baseline-ctest.log`,
`task05-arming-feature-ctest.log`, `task05-arming-controller-rerun.log`,
`task05-arming-final-ctest.log`, `task05-arming-delivery-ctest.log` and
`task05-arming-visual-rerun.log` in the external worktrees directory.

## Reproduction / runnable artifact

```powershell
$env:PATH='D:/Qt/6.11.2/mingw_64/bin;D:/Qt/Tools/mingw1310_64/bin;' + $env:PATH
& D:/Qt/Tools/CMake_64/bin/cmake.exe -S RoboBeetleConsole -B C:/Users/laixindong/.codex/worktrees/task05-arming-dry-run-build/qt -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DCMAKE_PREFIX_PATH=D:/Qt/6.11.2/mingw_64 -DCMAKE_MAKE_PROGRAM=D:/Qt/Tools/Ninja/ninja.exe -DCMAKE_CXX_COMPILER=D:/Qt/Tools/mingw1310_64/bin/g++.exe
& D:/Qt/Tools/CMake_64/bin/cmake.exe --build C:/Users/laixindong/.codex/worktrees/task05-arming-dry-run-build/qt --parallel 6
& D:/Qt/Tools/CMake_64/bin/ctest.exe --test-dir C:/Users/laixindong/.codex/worktrees/task05-arming-dry-run-build/qt -R '^visual_dispatch_state_machine_tests$' -V
$env:QT_QPA_PLATFORM='offscreen'; $env:QT_QPA_FONTDIR='C:/Windows/Fonts'
& D:/Qt/Tools/CMake_64/bin/ctest.exe --test-dir C:/Users/laixindong/.codex/worktrees/task05-arming-dry-run-build/qt --output-on-failure --parallel 1
```

Portable pure-logic test package:
`D:/RoboBeetle-results/task05-pr2-logic-20261004/visual_dispatch_state_machine_tests.exe`.
Contains matching MinGW runtime DLLs. Fresh reduced-PATH execution (Windows
system directories only) passes **246 checks / 0 failures**. EXE imports only
MinGW/runtime/system DLLs, no Qt Network or socket DLLs. CMake link graph also
shows no policy library in RoboBeetleConsole.exe.

EXE SHA-256 (build and package identical):
`922CE0ED148546AAF000B38F1FC9DC20CEEBACF1A3A6BB3E5A5DED36BF8F1201`.
Test source commit: `8da248f30b74b883b8d5bb51ff48882dedd7dc73`.
Later provenance changes are docs-only; build inputs remain unchanged. Final
delivery head is in the PR description and package BUILD_INFO.txt.

No hardware connected, firmware flashed, servo movement or water verification.
Physical direction remains unconfirmed. Later desktop hardware tests must use
unloaded servos out of water; only a later separately reviewed adapter may
connect this policy to a real command sender.
## 2026-10-04 PR46 review correction — NO_TARGET is confirmed HOLD

NO_TARGET is the single-frame-miss grace state, not a target-loss STOP state.
The dispatcher preserves the confirmed mode and arming and initiates no command
for 1499 ms. Only LOST issues the target-loss STOP. This matches PR42 HOLD and
the existing VisualTargetStateMachine's 1500 ms loss grace. NO_TARGET recovery
with unchanged confirmed suggestion never resends. A missing confirmed mode is
an explicit exception: NO_TARGET/HOLD immediately requests STOP; tests cover
fresh arm, manual STOP takeover/rearm and link loss/rearm (with fresh eligible
servo/pose evidence), all within dwell. STALE/OFF, motion ACK timeout and already
pending safety STOP retry retain their independent safety priority.

Revision RED: 268 checks / 8 failures on the previous implementation. Separate
LOST-with-HOLD precedence RED: 268 checks / 1 failure. Independent review added
pending-motion timeout during NO_TARGET: RED 273 checks / 1 failure.
Timeout checks now precede the visual-state send gate. Final GREEN:
**273 checks / 0 failures**, warning-clean CMake build and standalone CTest.
Tests also verify pending STOP invalidation through disconnection: reconnecting
and reacquiring control beyond the timeout creates no retry, and the old OK
cannot change current mode or rearm. No application or TCP wiring was added.

Logs: `C:/Users/laixindong/.codex/worktrees/task05-arming-no-target-red.log`,
`task05-arming-lost-hold-red.log`, `task05-arming-no-target-timeout-red.log`,
`task05-arming-no-target-final.log`.
CSV contiguous NO_TARGET duration analysis is a TODO in the design document,
not an analysis result or reason to change lost_ms in this revision.

Revised runnable test package (initial 246-check package remains preserved):
`D:/RoboBeetle-results/task05-pr2-no-target-review-20261004/visual_dispatch_state_machine_tests.exe`.
Revised EXE SHA-256:
`AA6425E2DAEA7F56A66387B348FAC9FC1B2932CE07B213EC8BB30E1575D69FF1`.
Revised tested source commit: `2e2ddddb0ee874821ab03209add7640ac5d47bc6`. Final head
is in the PR description / new package BUILD_INFO.txt. The earlier source/hash
above remain historical provenance, not the revised artifact.