# Task 05 PR2 — Arming and dispatch policy design for review

2026-10-04. Baseline `99515ce563ee143c2a2a0224352ee0a9b6321a5a`.
Scope: a pure C++ policy, a fake send port, deterministic tests. Do not wire the
policy into MainWindow, RobotController, RemoteRobotController, RbrpClientSession
or real TCP. Existing application remains DRY_RUN. No physical verification.

## Approach

Recommended: standalone stateful C++ class with injected command port and
caller-supplied monotonic milliseconds. The port records command and unique
request ID; ACK is delivered separately with the same ID and result code.
Alternatives: pure reducer returning effects (more caller bookkeeping), or
reusing RobotController (couples safety policy to live transport and violates
this slice's isolation). Use the standalone class.

The class starts disarmed. An explicit arm operation evaluates current link,
gateway authority, enabled_mask and pose-known mask for the required paddle
mask 0x001b, confirmed turn_sign (+1 or -1) and TRACKING state. Reject ineligible
requests with a typed reason plus displayable reason name. Direction has no
default and is separate from VisualPolicyConfig's historical DRY_RUN +1.

## State and precedence

1. Manual-input notification immediately disarms and clears every pending
   automatic request/retry. It emits no new automatic command; late ACKs cannot
   rearm. Only a new eligible explicit operator arm request enables evaluation.
2. Armed link loss, authority loss, STALE or INFERENCE_OFF requests STOP in
   that evaluation and disarms. Safety STOP retry remains pending even when
   disarmed, unless manual takeover cancels it. No automatic recovery/rearming.
   Invalidated enabled/known posture also requires rearming with fresh evidence.
3. Armed NO_TARGET or LOST requests STOP but keeps arming. On TRACKING recovery,
   the prior non-STOP suggestion may be resent after STOP ACK and dwell permit.
4. Each continuous need-to-stop episode sends one initial STOP. Switching between
   NO_TARGET and LOST without resuming motion is still one episode. STOP can
   bypass a pending non-STOP request and invalidates its ACK association.
5. Unacknowledged STOP may retry on a configurable acknowledgement timeout,
   provisionally 1000 ms; use new request IDs and ignore stale IDs. Successful
   STOP ACK suppresses further retries within that episode. NO_TARGET/LOST
   recovery cannot send non-STOP until the latest STOP is confirmed.
6. Non-STOP sends only when the mapped effective command differs from the last
   successfully established command (or after a STOP interrupted it). Minimum
   dwell is 1000 ms from the last non-STOP send. First non-STOP may send at once.
   Any unacknowledged request blocks new non-STOP. No automatic non-STOP timeout
   retries: an unknown outcome remains blocked until ACK or safety takeover.
7. Busy=7 releases pending non-STOP and retains the desired candidate. It does
   not immediately retry; next attempt is at the next 1000 ms dwell boundary
   from that non-STOP send. Other non-STOP rejection disarms and requests STOP.
8. Positive sign(ex)*turn_sign maps right, negative maps left; both suggested
   TurnLeft/TurnRight are treated as turning intent and remapped from ex and
   explicitly confirmed direction, avoiding the old DRY_RUN default. Forward
   and STOP retain their meanings; HOLD produces no new non-STOP command.

Monotonic time is injected, not a timer/thread. Decreasing/invalid time and
nonfinite/zero turning ex fail closed. Production transport submission and ACK
semantics are intentionally not claimed by fake-port success.

## Test coverage / RED to GREEN plan

Files proposed: `src/vision/VisualDispatchStateMachine.h/.cpp`,
`tests/visual_dispatch_state_machine_tests.cpp`, CMake standalone test target,
this design and a validation/provenance document. Link no network/controller
classes to the new test. UI files remain unchanged.

- Write desired-API tests first; build/run against an inert initial policy and
  record actual assertion failures (not only compile errors), then implement.
- Arming table: default OFF; every missing condition separately (link,
  authority, mask incomplete/zero, posture unknown/zero, absent/invalid
  direction, each non-TRACKING state), specific rejection reasons; valid arm.
- Safety table: STALE, OFF, link/authority loss during dwell and pending ACK
  produce same-evaluation STOP + disarm; recovery alone cannot rearm.
- NO_TARGET and LOST: STOP + still armed; repeated evaluation one STOP,
  continuous transition between stop states one episode; TRACKING recovery
  sends again only after STOP ACK/dwell. Fresh episode emits another STOP.
- Manual takeover while dwell blocked and while ACK pending: immediate OFF,
  no subsequent sends/retries even after late ACK or later timeout.
- Non-STOP unchanged suppression, pending-ACK gating, dwell 999/1000 boundary,
  Busy immediate suppression and next-window retry; successful ACK suppression.
- STOP bypasses dwell and non-STOP ACK; ACK timeout boundary and repeat;
  repeated STOP has new ID; stale ACK ignored; STOP ACK ends retries.
- Mapping table: both signs (+1/-1) times positive/negative ex; no default,
  invalid sign rejected; nonfinite turn ex fails closed.
- Full existing Qt CTest suite, standalone warning-clean test build, source
  audit proving no MainWindow/transport wiring; record source commit and new
  test EXE SHA-256. Deliver draft PR, no merge, no robot motion.

## Review needed before implementation

Approve the described policy, especially provisional STOP ACK timeout=1000 ms,
Busy retry measured from last send, continuous stop-episode deduplication and
STOP retry surviving safety disarm but being cancelled by manual takeover.
## Approved review amendments (take precedence over the initial proposal)

The operator approved implementation with these exact refinements:
- Current mode updates only on ACK OK, never Busy or timeout. Busy retries compare
  the latest suggestion against that mode, never replay a saved stale command.
- Non-STOP deadline is max(last non-STOP send +1000, last STOP accepted +1000).
  STOP accepted time is the local receipt time of the first OK for that episode,
  conservatively later than firmware acceptance; duplicate OK never extends it.
- STOP invalidates all preceding non-STOP outcomes. Any OK for first/retry STOP
  in the same episode confirms it. Fresh episodes and manual/link loss invalidate
  prior episodes; their late ACKs cannot change current mode.
- Non-STOP ACK timeout or OutcomeUnknown immediately sends STOP and disarms.
- Only manual motion (including manual STOP) takes over. Manual non-motion input
  does not disarm. Motion takeover clears all automatic pending requests/retries.
- STOP retries end on OK, manual takeover, link/authority loss. Three consecutive
  STOP timeouts latch an operator-visible alert; timeout default configurable 1000.

## Implementation steps (approved inline execution)

1. Add the desired API and fake-port assertion tests for the arming rejection
   table, both turn signs, ACK-only mode, latest Busy suggestion, pending-ACK and
   999/1000 dwell boundaries, recovery gate after STOP OK, safety and manual tables,
   timeout/OutcomeUnknown, stop episode any-OK correlation and three-timeout alert.
2. Compile with g++ -std=c++20 -Wall -Wextra -Werror using an inert implementation;
   execute and record assertion RED, preserving log outside the repository.
3. Replace the inert implementation with the approved policy; rerun the same
   command for GREEN; register a standalone CMake library/test target with no
   Qt Network/transport dependencies and no runtime application instantiation.
4. Build full Qt tree and run complete CTest. Compare with baseline; preserve
   unrelated baseline UI failures rather than editing UI in this slice.
5. Commit tested source; hash test EXE and record source commit. Commit docs-only
   delivery provenance, normal push and create draft PR; do not merge either PR.

Baseline Qt full gate: 25/27 passed; main_window_tests and main_window_layout_tests
already fail on UI clipping, with one capture close-guard assertion in this run.
Logs: C:/Users/laixindong/.codex/worktrees/task05-arming-baseline-ctest.log.
These pre-existing failures are outside this pure-logic slice.