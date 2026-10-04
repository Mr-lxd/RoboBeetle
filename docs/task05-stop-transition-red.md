# Task 05 — STOP transition investigation and RED tests

2026-10-04. Baseline: `99515ce563ee143c2a2a0224352ee0a9b6321a5a`
(merged Task 04 / PR #44). This draft changes tests and this investigation only.
No automatic command sender, arming, motion connection, firmware programming,
or hardware operation is introduced. GREEN is deliberately deferred.

## Background availability

The requested `docs/HANDOFF.md` does not exist in the root checkout or baseline
Git tree. The available background read for this investigation is
`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md`,
`docs/target-temporal-association-dry-run-validation.md`, and
`RoboBeetlePi/application/README.md`. This does not claim the missing handoff was
read. Task 04 remains a proposal-only DRY_RUN; its existing `turn_sign=+1` must
not become a default for a future armed sender.

## Source finding: STOP does not return Busy during a transition

Line anchors refer to the baseline production files, unchanged in this PR.

| Location | Evidence |
| --- | --- |
| `RoboBeetleFirmware/Core/Motion/motion_config.h:5` | Transition duration is 750 ms. |
| `RoboBeetleFirmware/Core/Communication/protocol_dispatcher.c:411` | START dispatches to `motion_manager_start`; STOP dispatches separately to `motion_manager_request_stop_at(manager, now_ms)`. |
| `RoboBeetleFirmware/Core/Motion/motion_manager.c:755` | A different non-STOP mode during an active transition returns BUSY; this is not the STOP route. |
| `RoboBeetleFirmware/Core/Motion/motion_manager.c:857` | STOP in STOPPED/FAULTED/STOPPING is idempotent OK. A running transition is accepted without a Busy check. |
| `RoboBeetleFirmware/Core/Motion/motion_manager.c:876` | Captures last targets, resets stop elapsed, restarts its clock at acceptance, immediately sets STOPPING and returns OK. |
| `RoboBeetleFirmware/Core/Motion/motion_manager.c:409` | STOPPING interpolates toward zero and returns before gait advance; original gait no longer advances. |
| `RoboBeetleFirmware/Core/Motion/motion_manager.c:430` | STOPPED, MOTION_STOP and ownership release occur after a separate 750 ms stop ramp. |
| `RoboBeetleFirmware/Core/Inc/rb_protocol_v2.h:55` | Wire Busy code is 7. |

Conclusion: STOP immediately **accepts and preempts** the original transition
with ACK OK, rather than Busy=7. It does not immediately complete the physical
stop or even the logical return-to-zero ramp. For example, a transition starting
at t=0 and STOP accepted at t=250 completes its normal stop ramp at t=1000,
later than the original t=750 deadline. ACK proves acceptance, not physical
completion. Gateway retries after this OK cannot accelerate the stop ramp;
duplicates replay ACK and fresh STOP while STOPPING is also idempotent.

## RED contract and reproduction

The phrase "effective before transition end" is ambiguous. The current firmware
already satisfies acceptance/preemption. Pending operator clarification, these
RED tests explicitly use the stricter interpretation: logical STOPPED,
MOTION_STOP and released motion ownership before the original transition ends.
This is a proposed completion deadline, not a claim that STOP is currently
rejected, and not a physical-motion measurement. If acceptance/preemption is
the intended requirement, these stricter RED assertions need revisiting.

Two tests in `RoboBeetleFirmware/tests/protocol_dispatcher_tests.c` send actual
typed Protocol V2 frames through the production dispatcher and MotionManager,
with only the servo hardware driver faked:

- Startup transition: Forward at t=0, STOP at t=250, observe at t=740.
- Mode transition: complete Forward startup, start TurnLeft at t=760, STOP at
  t=1010, observe at t=1500 (transition age 740 ms).

Both verify the transition is active before injection, ACK OK, immediate
STOPPING, and nonblocking acceptance without servo writes. They keep host
heartbeat alive at each observation and require process OK, so watchdog faults
cannot satisfy the deadline. Only the two final completion assertions fail.

```powershell
./RoboBeetleFirmware/tests/run_host_tests.ps1 -BuildRoot C:/Users/laixindong/.codex/worktrees/task05-stop-transition-red-build/red
```

Fresh baseline gate: **PASS**, 37 executables + 13 app/backend/benchmark/
diagnostics compile-contract objects + USART2 source/config contract.
After the new tests: compile succeeds with `-std=c11 -Wall -Wextra -Werror`;
the unchanged MotionManager suite passes; the standard host gate stops at
`protocol_dispatcher_tests` with exit 1. Direct execution confirms exactly:

```text
FAIL: Task05 RED: start-transition STOP must complete before original 750 ms deadline
FAIL: Task05 RED: mode-transition STOP must complete before original 750 ms deadline
```

Logs are preserved outside the repo:
`C:/Users/laixindong/.codex/worktrees/task05-stop-transition-red-baseline.log`
and `C:/Users/laixindong/.codex/worktrees/task05-stop-transition-red-red.log`.
This draft intentionally leaves the normal gate RED; do not treat it as ready
to merge or as delivery of Task 05 movement.

## Artifact provenance

New host test EXE (not a robot-control executable):
`C:/Users/laixindong/.codex/worktrees/task05-stop-transition-red-build/red/protocol_dispatcher_tests.exe`.
SHA-256: `CE9ABAA95BC739AC7D3D50A97CEE29CF74B694E4A1179FA2A17702FCCB83A5AE`.
Its source commit is recorded in the PR description after committing tests.

No new Qt EXE was built for this tests-only draft. The preserved Task 04 EXE
was freshly hashed:
`D:/RoboBeetleConsole-portable-target-temporal-association-dry-run-20261003/RoboBeetleConsole.exe`,
SHA-256 `C6A525E920FC1DB7624F60967FF161AAC50C07C4678816A13C73A1405F6E1C78`.
Its original build source is `ec5841f30ace8eadc551b5893a849087c553aa67`;
Git comparison confirms Console inputs unchanged through this draft.
It is a retained DRY_RUN artifact, not a Task 05 motion-enabled build.

## Required later PR coverage (all pending)

| Requirement | Required tests / acceptance |
| --- | --- |
| Arming defaults OFF | Startup/reconnect remain disarmed; Qt link loss and gateway link loss each disarm and request STOP. |
| Manual takeover | Every manual input immediately takes authority and disarms automatic mode, including during dwell and in-flight commands. |
| STALE / LOST / INFERENCE_OFF | Each triggers STOP in the same evaluation, including within dwell and during motion transitions. |
| Non-STOP dwell | At least 1000 ms between sends; boundary tests at 999/1000 ms; Busy=7 causes no immediate retry. STOP bypasses dwell. |
| `turn_sign` configuration | No default direction; unset/invalid/unconfirmed physical direction refuses arming; both explicitly confirmed signs tested. |
| STOP completion semantics | Resolve acceptance versus completion before GREEN; tests must match the approved semantics. ACK must never be labelled physical completion. |
| Desktop hardware validation | Only with servos unloaded and out of water. No hardware validation performed in this PR; physical direction remains unconfirmed. |

No later requirement is declared implemented or covered by this initial draft.
