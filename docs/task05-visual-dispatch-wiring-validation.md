# Task 05 PR3: visual dispatch wiring validation

Base: `dfd7972bf1c6102e83f406050c3f62730b046bca` (PR47 merged first, then rebased PR46). Branch `codex/task05-pr3-visual-dispatch-wiring` was created directly from that main commit. The design was pushed before implementation and approved by the operator, including the direct-snapshot AwaitingVideo clarification.

This record covers host tests and loopback simulation. Physical motion and water acceptance remain pending. The final source commit and application/test EXE SHA-256 are recorded in the draft PR and the portable package's BUILD_INFO.

## Activation and evidence contract

- Every application instance starts with visual dispatch OFF. Enabling the switch requires a separate Arm action; no settings persist the switch or direction confirmation. OFF retains DRY_RUN diagnostics. Direct maintenance remains DRY_RUN.
- Direction confirmation is two separate operator actions: choose +1 or -1, then Confirm. `VisualPolicyConfig::turn_sign` is diagnostic configuration, not arming evidence.
- A 50 ms GUI-thread timer evaluates the complete diagnostic snapshot, using its VisualState verbatim and its own policy result's `ex_f`. AwaitingVideo alone does not add STALE: confirmed Forward is retained at 400 ms; the upstream snapshot becomes STALE at 500 ms, causing STOP and disarm. Missing valid snapshot is STALE.
- Enabled and known pose evidence is inferred from correlated ACK OK. Newly enabled servos, Neutral and Angle establish known pose; PWM and Disable clear it. Re-enabling an already-enabled servo is a no-op and preserves its existing evidence. Link/authority loss or session reconstruction clears both masks; old ACKs cannot restore them.
- START rejection with raw result6 and inferred known pose displays the possible firmware mismatch explanation; it is not a diagnosis of the hardware failure. The message survives the subsequent safety STOP.
- NO_TARGET retains the confirmed mode and arming, without initiating motion. NO_TARGET/HOLD with no confirmed mode requests STOP. LOST requests STOP while retaining arming. STALE, INFERENCE_OFF, control/link loss and uncertain non-STOP outcomes disarm. Safety STOP bypasses dwell.
- Non-STOP changes wait for the previous ACK and both 1000 ms gates: previous non-STOP send and accepted STOP. Busy does not update the mode or immediately retry; the next window re-evaluates the latest suggestion.
- Manual motion/STOP takes over before the controller action or user actuator edit and cancels automatic commands and retries. Closing the visual gate only sends one operator STOP if armed or an automatic STOP is awaiting confirmation. That operator request does not retry. Closing after manual takeover sends no command.

## RED to GREEN evidence

Logs are retained under `C:/Users/laixindong/.codex/worktrees/task05-arming-dry-run-build/`. They include intentional assertion failures before each corresponding implementation change, rather than compilation failures alone.

| Runtime issue | RED evidence | GREEN evidence |
| --- | --- | --- |
| Automatic STOP requires a fresh RBRP request even if Qt believes it stopped or another STOP is pending | `task1-red-controller.log` | `task1-green-final-runtime.log` |
| Default OFF, explicit arm/sign, exact policy error, feature-off rules and timer behavior | `task1-red-session.log` | `task1-green-final-runtime.log` |
| Controller destruction with an active socket must not invoke callbacks after pending state destruction | `task1-red-destructor.log` (debugger crash evidence) | controller lifetime regression |
| Loss evaluates immediate safety handling; later feature-off retains a pending operator STOP result | `task1-red-loss.log`, `task1-red-operator-correlation.log` | `task1-green-final-runtime.log` |
| Known submission rejection is Rejected, not OutcomeUnknown; terminal RTT is monotonic | `task1-red-submission-rejection.log`, `task1-red-monotonic-rtt.log` | `task1-green-review-final.log` (2/2) |
| Any STOP in the same episode can confirm; unanswered duplicates must not block resumed START or cause later authority loss | `task1-red-stop-retirement.log` (10 assertions, original/retry OK) | `task1-green-stop-retirement-final.log` (2/2); independent focused run 2/2 |
| MainWindow default OFF, sign/arm controls, manual handlers and CSV v3 | `task2-red-tests.log` | `task2-green-focused-final-tests.log` (7/7, 45.57 s) |
| START result6 and possible pose mismatch remain visible after safety STOP | `task2-red-expanded-tests.log` | MainWindow visual dispatch target in focused GREEN |
| Direct maintenance presentation must remain DRY_RUN | `task2-red-direct-tests.log` | MainWindow visual dispatch target in focused GREEN |
| One-shot operator STOP exposes SENT while awaiting ACK, then its terminal result | `task2-red-operator-submission-tests.log` | `task2-green-final-full-tests.log` (30/30, 62.91 s) |
| Enabled presentation must show the operator-confirmed sign, not a conflicting diagnostic default | `task2-red-sign-presentation-tests.log` | `task2-green-sign-presentation-tests.log` (2/2, 14.58 s) |

STOP retirement coverage also waits through the controller timeout, delivers an old STOP ACK after resumed Forward, and submits a new STOP reentrantly during retirement. The old ACK cannot change the new mode; the new request retains its own correlation.

The new MainWindow target uses fake controller evidence for exhaustive handlers and an actual RemoteRobotController against a loopback RBRP peer for the integrated path. The loopback covers OFF with zero writes, servo Enable ACK/pose evidence, operator direction selection and confirmation, Arm, START/ACK, no-frame STALE/STOP/disarm, explicit rearm, manual STOP takeover and feature-off without an extra STOP. CSV reads use named columns to verify the real wire ID, command, matching OK and measured RTT.

The handler matrix exercises five available motion modes, motion STOP, global Enable/Disable, gait/coordination, and each of five servos' enable/release, Neutral, PWM/Angle Apply, PWM spin/slider and Angle spin. Each path is checked with pending ACK and during dwell; controller-call probes verify takeover happens before the manual command. Keyboard tests exercise button Space and spin/slider arrow inputs. Existing buttons do not acquire new Enter/Return accelerators. Disabled Backward and the unimplemented Emergency action remain disabled and emit nothing. Programmatic refresh remains signal-blocked.

UI tests also verify a second window cannot inherit activation, selection alone is insufficient confirmation, changing a confirmed sign disarms, all four operator STOP results remain visible after OFF, a STOP timeout alert remains latched, and manual takeover followed by OFF sends nothing. Retry CSV checks require distinct policy/wire IDs and an outcome row for each recorded terminal result. The original MainWindow CSV regression only changes its column-count/schema expectations to v3; existing catchup, frame ID, normalized error and transition assertions remain intact.

Final implementation commit: `00beeeded151a66c45dd6ad98d868adf1707f7e5`. The root agent rebuilt all targets and ran the complete Qt regression on this final code: **30/30 PASS, zero failures, 62.72 s** (`task3-final-build.log`, `task3-final-full-tests.log`). Baseline28 gains the dispatch-session and MainWindow-dispatch targets. Existing real-clock CTest isolation remains in place.

Independent final specification and code-quality reviews approved this implementation. Findings corrected during review were known submission rejection mapping, monotonic RTT, controller-level duplicate STOP retirement, operator STOP submission display and active turn-sign presentation. Subsequent closeout commits only update documentation; they do not change the tested application or test sources.

## RBRP outcomes and CSV v3

The existing remote controller owns the only RBRP connection. Policy IDs map to wire IDs; StartMotion carries the mode byte and StopMotion has an empty command payload. Correlated accepted result0 maps to OK, rejected result7 to Busy, other rejection to Rejected, and OutcomeUnknown/Cancelled (including gateway terminalization) to OutcomeUnknown. A validated negative CommandSubmitted status is a known submission rejection.

CSV v3 retains all v2 columns and appends `policy_request_id,request_id,dispatch_command,dispatch_result,ack_rtt_ms`. Diagnostic rows leave these fields blank. Actual sends and retries have separate dispatch rows and wire IDs; outcomes have outcome rows. RTT uses monotonic send-to-terminal timing. Unknown without a measured RTT leaves that field blank. Operator STOP uses policy ID0, distinct from positive automatic policy IDs.

The PR46 TODO remains open: analyze contiguous NO_TARGET durations in PR43/44 CSV, preserving session boundaries and censored runs, to justify `lost_ms`. This PR does not claim that analysis or change the 1500 ms grace.

## Operator desktop checklist: pending

All checks below require **unloaded servos, out of water**, and are performed by the operator. Record actual observations separately from host/loopback results.

- [ ] On startup, verify the visual switch is OFF. Verify enabling it alone cannot issue motion.
- [ ] Try Arm with no confirmed direction and record `TURN_SIGN_UNCONFIRMED`.
- [ ] Record the chosen sign (+1 or -1), then the separate Confirm action. Check actual left/right direction and retain the observed result before allowing normal operation.
- [ ] With remote control, enable/Neutral the required servos and confirm arming evidence. PWM must invalidate the corresponding pose evidence.
- [ ] Unplug Ethernet and record elapsed behavior: legs become soft and stay at their position. This is the expected disable-all safety behavior, **not a smooth return to zero**.
- [ ] Stop the frame stream and verify STALE causes STOP and disarm. ACK means acceptance; ordinary firmware STOP then ramps to zero with the documented tick-dependent upper bound.
- [ ] Use manual motion/STOP and actuator editors while armed, including dwell and a pending ACK. Verify immediate takeover and no further automatic sends or retries.
- [ ] Close the switch while armed and record the one operator STOP result. Repeat after manual takeover with manual motion in progress and verify it sends nothing.

No physical controller connection, firmware flashing, servo movement or water test is part of this host validation. Disconnect safety continues through gateway abort/UART teardown/heartbeat timeout/firmware disable-all; abort itself does not send a STOP frame. See [Task05 STOP investigation](task05-stop-transition-red.md) for the code anchors and detection/scheduling budget.
