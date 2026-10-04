# Task 05 PR3 visual dispatch wiring design

Baseline: merged PR47 then rebased/merged PR46. Main: dfd7972bf1c6102e83f406050c3f62730b046bca. Branch: codex/task05-pr3-visual-dispatch-wiring, newly created directly from that main commit (not from the merged PR46 branch). Only the worktree directory is reused after clean verification. Rebased Qt baseline:28/28 PASS (47.59s); source tree matches merged main.

## Selected architecture and alternatives

Use a GUI-thread VisualDispatchSession (50ms precise timer) and a small controller-backed VisualCommandSendPort adapter. RemoteRobotController continues to own the sole existing RBRP TCP session, command validation and wire IDs. MainWindow owns controls, snapshot delivery and manual hooks; VisualCsvLogger owns all CSV file writes. This preserves the already-tested pure policy and allows fake-port and loopback testing.

Putting all request correlation/timing directly in MainWindow would increase coupled test state; a second TCP client would duplicate authority and is excluded. Neither alternative is selected.

## Activation and operator control

The visual dispatch checkbox starts unchecked for every MainWindow instance. It is never loaded/saved in settings. Checking enables the feature gate, not automatic arming: a separate Arm button attempts policy.arm against the latest complete snapshot/controller evidence. Before turning the feature off, capture whether the policy is armed or an automatic STOP is still awaiting confirmation. Only if either is true, issue exactly one operator StopMotion request and expose its submission/terminal result in the UI; it is not automatically retried. Then clear automatic requests, correlations and retries, disarm, and return to DRY_RUN. If enabled but never armed, or already disarmed by manual takeover with no automatic STOP awaiting confirmation, switching off emits no command. In particular it must not stop an operator's ongoing manual motion. A pending non-STOP request in an armed policy also triggers the one operator STOP; its late OK cannot revive automation. Once off, snapshot/display/CSV diagnostic behavior follows existing DRY_RUN and the automatic layer sends nothing. Explicit Disarm uses the same conditional operator-stop rule while leaving the feature gate on. The operator STOP result correlation is separate from invalidated automatic correlations so the UI can display its eventual result after disabling.

A +1/-1 selector has an unconfirmed placeholder and a separate Confirm button. No VisualPolicyConfig.turn_sign value constitutes confirmation. Changing selection invalidates confirmation; if armed this first stops/disarms. Confirmation lives only in the application session and is not persisted. Missing confirmation displays TURN_SIGN_UNCONFIRMED for otherwise eligible arming. Safety/manual takeover requires explicit Arm again.

## Evidence, evaluation and ownership

At every 50ms tick the session samples controller link/authority/enabled and known-pose masks and the current complete VisualDiagnosticSnapshot; it passes suggestion and ex_f from that same snapshot.command. No separately cached raw target.ex can substitute. Awaiting-video/no valid fresh snapshot is STALE. Existing diagnostic watchdog still runs independently; session tick handles policy timeouts/retries even with no frame arrivals. Link/authority signals also trigger an immediate evaluation, preventing a delayed non-STOP after loss.

The firmware does not report logical_pose_known_mask. Qt infers it from correlated ACK OK according to RoboBeetleFirmware/Core/Servo/servo_service.c; UI values or enabled bits alone never establish evidence:

| Confirmed operation | Qt inferred pose mask update | Firmware anchor |
| --- | --- | --- |
| Enable a previously disabled servo, ACK OK | Set newly enabled bits known, because firmware writes calibrated Neutral and logical angle zero | servo_service_enable:134-177 |
| Neutral, ACK OK | Set affected bits known | servo_service_neutral:487-497 |
| Angle, ACK OK | Set corresponding bit known | servo_service_set_angle:395-397 |
| PWM, ACK OK | Clear corresponding bit, even if pulse equals calibrated Neutral | servo_service_set_pwm:348-349 |
| Disable, ACK OK | Clear affected bits (and enabled evidence) | servo_service_disable:209-210 |
| Link/authority loss or session reconstruction | Clear all pose bits and enabled evidence; stale responses cannot restore them | servo_service_disable_all:230-234 and existing gateway safety path |

Enable detail needed for exact firmware alignment: servo_service_enable skips bits already enabled (134-137), without rewriting Neutral. A repeated Enable ACK therefore preserves their previous pose evidence and must not turn an already-enabled PWM/unknown pose into known. Tests cover both genuine disabled-to-enabled Neutral initialization and this no-op case. Non-OK outcomes cannot set a bit known; uncertain pose-changing operations conservatively clear affected evidence. All required paddle bits must have enabled and known evidence before arm. Direct-maintenance backend exposes no automatic submission capability and stays DRY_RUN.

If Qt inferred all required poses known but START returns HARDWARE_FAILURE (ACK result6), retain the raw rejection and also show exactly: "可能是姿态未知（Qt 推断与固件不一致）". This is a possible mismatch explanation, not proof of the cause: motion_manager.c:775-779 rejects unknown logical pose with HARDWARE_FAILURE, while other hardware failures can share result6. Automatic rejection still follows PR46 STOP/disarm behavior; the message must survive the subsequent STOP result.

NO_TARGET/HOLD keeps confirmed mode only, LOST stops while armed, STALE/OFF disarms, and all PR46 dwell/ACK/timeout/retry contracts remain intact.

## Adapter and terminal outcomes

Controller exposes request-ID-returning automatic motion submission and a typed terminal-result signal, separate from existing bool manual methods. Forward/left/right map to StartMotion(mode byte); STOP maps to StopMotion(empty payload), including when Qt believes motion already stopped or another STOP is pending. New automatic STOP retries receive fresh RBRP IDs. A conditional operator STOP on feature-off is a separate one-shot request, without automatic retries, even if the automatic STOP it supersedes was already pending.

Adapter maps each uint64 policy ID to its actual uint32 RBRP ID and reverse. Submission rejection is Rejected; accepted gateway outcome with result0 is Ok; rejected result7 is Busy; other rejection is Rejected; OutcomeUnknown/Cancelled and gateway terminalize_outstanding messages are OutcomeUnknown. Only correlated kind/sequence/wire ID outcomes count. Any signal arriving synchronously during send is deferred until mapping is installed. No ID reuse across sessions.

Connection/authority loss and manual takeover invalidate adapter correlations. Late responses may be audited in CSV but cannot update policy mode. Non-STOP uncertainty emits policy OutcomeUnknown while control is still available for immediate STOP, before existing controller authority cleanup. Disconnected transport cannot deliver STOP; existing gateway abort/watchdog provides disable-all, not smooth return-to-zero.

## Manual inputs and UI

Every existing motion/servo actuation path calls manualInput(Motion/Stop) before the controller call: six motion modes (including disabled Backward path), motion STOP, global Disable All/emergency, Enable All, individual enable/disable, neutral, angle/PWM Apply, gait backend and front/rear coordination selectors. Servo input editors/sliders that only alter pending values also trigger Motion takeover when user-edited; programmatic refresh uses signal blocking. Existing keyboard accelerators for these actions share the same hooks; enumerate actual shortcuts and controls rather than adding movement shortcuts.

UI displays feature gate, armed/disarmed, armReasonName, current ACK-confirmed mode and latched STOP timeout alert. Manual takeover cancels automatic pending commands and all retries, including while dwell or motion ACK is pending.

## CSV v3

Retain v2 columns/row semantics and append policy_request_id, request_id (RBRP), dispatch_command, dispatch_result, ack_rtt_ms. Every actual send has a dispatch row; every terminal result has an outcome row. Retries are separate sends/IDs. Existing frame/transition rows leave dispatch columns blank. Use monotonic session time and send-to-outcome RTT; unresolved outcomes keep RTT empty unless measured. Size-limit/escaping/flush behavior remains covered. Schema identifier becomes v3.

## RED to GREEN and integration gates

1. Existing baseline28/28; write/observe failures for default-off, missing sign, paired ex_f, no-frame ticking/timeouts/STOP retries, and CSV v3.
2. Loopback RBRP peer covers StartMotion/STOP wire kinds, IDs both ways, OK/Busy/Rejected/OutcomeUnknown (same payload emitted by terminalize_outstanding), repeated STOP and late/superseded ACKs, every pose inference row above, repeated Enable after accepted PWM, non-OK outcomes, session loss/reconstruction and ignored old-session ACKs; START HARDWARE_FAILURE mismatch wording must remain visible after safety STOP.
3. Enumerated MainWindow actuation/edit/shortcut tests each arm first, invoke exactly that path, assert disarmed and no further automatic traffic; include pending ACK and dwell.
4. End-to-end loopback: qualify controller+pose, choose and confirm direction, enable/arm, feed tracking, accepted motion, stop frame arrivals, observe STALE+STOP+disarm, then rearm and manual takeover. Default-off fixture proves existing diagnostics produce zero automatic robot writes.
5. Feature-off matrix: armed => exactly one operator STOP; automatic STOP awaiting ACK while disarmed => exactly one operator STOP; enabled/not armed => zero sends; manual takeover then ongoing manual motion => zero sends. Verify no operator STOP retry beyond multiple timeout windows and its OK/Busy/Rejected/OutcomeUnknown result remains visible. Disable with pending motion ACK and late OK must remain disarmed.
6. Full Qt regression green; independent review; source commit and application/test EXE SHA256 recorded in draft PR. No real hardware actions.

## Operator desktop checklist (pending user execution)

All items require unloaded servos out of water. Record +1/-1 selection then explicit confirmation and observed physical turn direction; confirm unselected/unconfirmed rejects arm. Unplug network: legs become soft and remain at their position (expected safety disable, not return-to-zero). Test STALE sends STOP and manual input takes over. Record evidence separately from desktop loopback results. No physical acceptance is claimed here.

## Design review checkpoint

2026-10-04: operator approved the preceding architecture with the pose inference and conditional feature-off adjustments captured above. Push this design-only branch before any RED tests or implementation edits. Await operator review of this remotely published revision. No application, test, firmware or transport implementation is changed by this document commit.
