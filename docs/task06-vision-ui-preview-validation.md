# Task 06 PR-A: Vision UI preview

This is a draft preview for operator and Claude review. Do not merge until the
operator has inspected the screenshots and tried the portable application.
Depth (PR-B), pitch (PR-C), firmware and gateway changes are out of scope.

## Changes and safety boundary

The Vision column contains Auto follow, with DRY RUN / NOT READY / READY / ARMED /
FAULT labels, independent control/servo-pose/tracking checks and one Arm/Disarm
button. ARMED adds a green video border and an ACK-confirmed command badge.
Faults appear in a red banner. Four Vision Controls use a 2x2 grid; the visible
status area has Video, Inference and Recording rows. Measurements remain in
tooltips and Vision Details. The exceptional inference Clear Error action remains.

The explicitly requested direction change defines +1 once in VisualPolicyConfig
and initializes the session's confirmed direction from it. There is no direction
selection/confirmation workflow. CSV still records the configured direction.
The dispatch state machine, remote controller and all other safety behavior are
unchanged: manual takeover, feature-off operator STOP, ACK correlation, dwell,
STOP retries and immediate transient STALE/INFERENCE_OFF/LOST handling.

## Host verification

RED: startup direction and missing Auto follow card assertions failed in the
unmodified implementation. GREEN includes mixed independent prerequisite checks,
button geometry, configured direction, fault banner and painted armed border/
badge lifecycle, while retaining existing safety tests.

Release build: Qt 6.11.2 / MinGW GCC 13.1 / CMake Ninja on Windows. Full Qt
regression and exact binary source/hash are recorded in the draft PR and the
external delivery BUILD_INFO. No physical robot operation was performed.

## Screenshot review

`operator_console_preview --suite auto-follow` exercises the real MainWindow and
dispatch session with a fake controller, real diagnostic inputs and fake ACKs.
It opens no robot connections. Fresh loopback HTTP and layout events are settled
before each scenario; a fresh detection frame is supplied before capture so a
late UI refresh cannot invalidate the depicted state.

Ten PNGs cover DRY RUN, NOT READY, READY, ARMED and FAULT at both 1420x880 and
1100x720. Output: `D:/RoboBeetle-results/task06-ui-preview-20261004/`. The manifest
records actual sizes, state, readiness masks, action geometry and simulated
requests. FAULT screenshots use the real hardware-failure/pose-mismatch path;
the latched STOP-timeout banner is covered by the UI test.

## Operator trial (pending)

- Inspect both sizes, especially checklist wrapping, fault banner, controls and
  ARMED border/confirmed command.
- Confirm each launch starts with Visual dispatch OFF and explicit Arm is needed.
- Check manual takeover, feature-off STOP rules and short STALE disarming with
  unloaded servos, out of water.
- Turn +1 was confirmed on the desktop in Task 05; underwater remains unconfirmed.

Preview approval and physical acceptance remain pending. No merge was requested.
