# Task 05 PR2 — Approved arming / dispatch policy and implementation plan

2026-10-04; baseline `99515ce563ee143c2a2a0224352ee0a9b6321a5a`.
The operator approved the standalone policy and subsequently specified the
ACK-only mode, two dwell gates, STOP episode correlation, non-STOP unknown/
timeout fail-safe, manual-motion-only takeover and stop-retry termination.
These final rules replace the earlier design proposals.

## Architecture / scope

`VisualDispatchStateMachine` is pure C++20. A `VisualCommandSendPort` records
`DispatchRequest {id, command}`; later outcomes arrive via `acknowledge()`.
Caller supplies monotonic milliseconds and verified gateway/servo/vision data.
The test uses a FakePort. CMake builds the policy only under BUILD_TESTING and
links it only to its new standalone test. There is no Qt/network dependency,
transport adapter or application instantiation. MainWindow and all controller/
transport code remain unchanged; existing application continues DRY_RUN.

A standalone class was chosen over a reducer with caller-managed pending
requests (more bookkeeping) or RobotController reuse (live-transport coupling).

## Final contract

- Default disarmed and mode unknown. Explicit arm needs connected link, gateway
  authority, required enabled and pose-known masks (default paddle mask 0x001b),
  explicit confirmed sign +1/-1 and TRACKING. Each refusal has a typed/displayable
  reason. No default direction; the old DRY_RUN turn_sign is not reused.
- STALE/OFF/link/authority loss: same-evaluation STOP attempt and disarm.
  Servo/pose/direction invalidation also fails closed. Operator must rearm.
  Link/authority loss cancels pending retry and ACK association after the attempt.
- NO_TARGET/LOST: STOP but stay armed. Each continuous stop episode emits one
  initial STOP. NO_TARGET→LOST does not start a fresh episode without intervening
  motion. TRACKING recovery may resume only after confirmed STOP and dwell.
- Only manual motion (including manual STOP) disarms/takes over. It cancels all
  automatic requests/retries, emits no automatic command and invalidates old ACKs.
  Non-motion input leaves arming unchanged.
- Current mode changes only on ACK OK. Submission, Busy and timeout do not update
  it. Busy retries compare the current newest suggestion against confirmed mode.
- Non-STOP gate is max(last non-STOP send +1000, last STOP accepted +1000).
  STOP accepted time is receipt of the first episode OK (a conservative bound on
  firmware acceptance); later episode OKs never extend it. No new non-STOP while
  any request is unconfirmed. First non-STOP sends immediately after valid arm.
- Non-STOP timeout or OutcomeUnknown/rejection immediately STOPs and disarms;
  late OK/Busy at or after deadline cannot bypass this even if the timer is late.
- STOP bypasses dwell and supersedes prior non-STOP. Any OK from first/retry STOP
  in that same episode confirms STOP. Old non-STOP or older episode ACKs cannot
  update mode. Contiguous STOP request IDs retain the whole episode in O(1) memory.
- STOP retry default ACK timeout=1000 ms is configurable (shared ACK deadline
  parameter for this isolated policy). Retry stops on OK, motion takeover or
  link/authority loss. Three consecutive expiries latch an operator alert;
  successful ACK resets the count but the alert remains until explicit eligible
  operator arming. At most one retry per evaluation, never a burst after a stall.
- Turn intent uses sign(ex)*confirmed turn_sign: positive→right, negative→left.
  Forward/STOP retain their meaning; HOLD sends no non-STOP. Invalid turn error
  fails closed. Confirmed ex supplied here is a logical input, not a new filter.
- Invalid/decreasing time fails closed while armed. Rejected arm calls also
  observe time and invalidate loss, including disarmed pending safety STOPs.

## File map and completed plan

- `RoboBeetleConsole/src/vision/VisualDispatchStateMachine.h/.cpp`: policy/port/types.
- `RoboBeetleConsole/tests/visual_dispatch_state_machine_tests.cpp`: deterministic
  fake-port scenarios with every operator requirement and review edge covered.
- `RoboBeetleConsole/CMakeLists.txt`: standalone test-only library/test, warnings
  -Wall -Wextra -Werror; no linkage into RoboBeetleConsole or RoboBeetleCore.
- `docs/task05-arming-dispatch-validation.md`: evidence and artifact provenance.

- [x] Inspect baseline and requirements; present/approve standalone design.
- [x] Write tests and inert API, compile and execute behavioral assertion RED.
- [x] Implement policy and execute same scenarios for GREEN.
- [x] Add STOP correlation, timing, manual and safety boundary cases.
- [x] Independent read-only review; reproduce late ACK and rejected-arm clock/loss
  findings as RED, repair, then GREEN; final review no important findings.
- [x] Build Qt tree and run full baseline and feature CTest, preserving unrelated
  UI failures; source/link audit confirms application remains DRY_RUN.
- [x] Commit tested source; package/hash and verify reduced-PATH execution; record provenance.
- [ ] Normal push and draft PR; final PR status is recorded in the PR description.
  No merge or hardware operation is authorized by this slice.