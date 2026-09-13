# Motion Simple Gait Review Fixes Implementation Plan

> **Execution note:** implement this plan in the isolated `feature/motion-simple-gait` worktree, keeping PR #14 as the only integration target. Follow the test-first loop for each behavior: add a focused failing test, run it, implement the smallest fix, then rerun the focused test before moving on.

**Goal:** close the external review findings without changing the existing Protocol V2 wire result contract, preserving immediate SafetySupervisor takeover and the current Motion ownership model.

**Design decisions:**

- Ordinary Motion STOP is accepted at request time, then enters `MOTION_STOPPING` and completes a centralized `MOTION_TRANSITION_DURATION_MS` (750 ms) wall-time ramp to neutral. STOP is idempotent when no Motion work is unresolved.
- STOP supersedes queued/deferred Motion work and cancels stale in-flight Motion completion handling. APC keeps one request in flight and schedules Motion STOP above ordinary work but below Disable All. DirectUart sends the canonical STOP immediately after marking stale Motion pending requests cancelled.
- Heartbeat/liveness loss, leak safety, Disable All, and the existing fail-safe path bypass the graceful ramp and clear Motion ownership immediately.
- ServoService records per-servo logical pose. Manual SetAngle, Neutral, Enable, and Motion SetAngle establish known pose; raw SetPWM marks pose unknown. Motion START rejects an unknown required pose through the existing internal `HARDWARE_FAILURE` to Protocol V2 mapping.
- Motion START cross-fades every required logical target from the recorded pose to the gait target over the same 750 ms transition, including mirrored left/right channels.
- The SimpleGaitGenerator emits logical targets only. Rear operational limits are enforced by a generator-independent MotionManager/common output guard, with its diagnostic counter owned by Motion/common code.
- `motion_manager_process(now_ms)` advances by wrap-safe unsigned elapsed wall time, accepting delayed 10/20/70/100 ms foreground calls without nominal-tick drift.

## Tasks

1. **Capture Qt STOP races in tests.** Add DirectUart and APC tests for START-in-flight followed by STOP, stale START ACK suppression, queued Motion supersession, deferred retry cancellation, STOP idempotence, and Disable All priority. Run the focused Qt tests and record RED failures.
2. **Implement Qt STOP arbitration.** Add pending/queued Motion-work detection, a dedicated APC Motion-STOP priority lane, stale-request cancellation, scheduler ordering `Disable All > Motion STOP > ordinary`, and correct ACK/state/timer handling. Keep Safety fail-closed paths clearing any graceful-stop work.
3. **Capture logical-pose and ownership behavior in firmware tests.** Add ServoService tests for known/unknown pose tracking, raw SetPWM rejection on Motion START, manual ownership BUSY, and mirrored pose handoff. Add MotionManager tests for cross-fade from non-neutral pose and existing STOP ownership behavior. Run focused firmware tests and record RED failures.
4. **Implement ServoService logical pose tracking.** Track pose and known bits transactionally across Enable/Disable/Disable All, manual angle/neutral/raw PWM, and Motion angle writes. Expose only generic ServoService pose APIs with self-sufficient headers; keep raw PWM behavior documented as pose-unknown.
5. **Capture and implement real-time MotionManager progression.** Add deterministic tests for 10/20/70/100 ms gaps, 750 ms STOP wall duration, convergence/monotonicity toward neutral, and uint32 wrap. Change processing/tick arithmetic to actual wrap-safe elapsed time and saturating transition counters.
6. **Move the operational envelope out of SimpleGaitGenerator.** Add an alternate/fake generator test that emits rear `-4000` cdeg and proves MotionManager clamps to `-3000` while counting the clamp. Remove physical rear clamping from SimpleGaitGenerator and update its tests/docs to describe logical output plus common-layer enforcement.
7. **Add safety interruption/reconnect coverage.** Test Disable All and heartbeat/liveness loss during STOPPING for immediate takeover, and verify reconnect never auto-resumes an interrupted stop. Confirm STOP when already STOPPED remains safe/idempotent.
8. **Update contract documentation and run validation.** Reconcile firmware/Qt/protocol docs with the final STOP, pose, wall-time, and common-envelope semantics. Run firmware host regressions, Qt clean build/CTest, header self-sufficiency, warning audit, and `git diff --check`; record ARM build as Pending if the toolchain remains unavailable and do not claim hardware evidence.
9. **Read-only internal review and PR handoff.** Review STOP stale-ACK races, APC one-flight ordering, elapsed-time math, pose handoff, envelope independence, and Safety takeover. Commit the fixes, push `feature/motion-simple-gait` to PR #14, verify the PR remains open/unmerged and points at the new head, then stop with `READY FOR EXTERNAL GITHUB RE-REVIEW`.
