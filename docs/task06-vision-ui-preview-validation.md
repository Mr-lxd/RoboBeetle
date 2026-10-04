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

## Delivery provenance

Tested binary source: `93a640ff8b10119c2aafa284f2120c063e9fcab1`.
Full Qt regression: **30/30 PASS**, 67.52 seconds. RED and final GREEN logs are
retained in the screenshot output directory. Independent spec and quality reviews
found no remaining actionable issue.

Portable directory: `D:/RoboBeetleConsole-portable-task06-ui-preview-20261004/`.
Application EXE SHA-256:
`0A0756036B4D690E76479B9E5EEF1EF60159BF59ADFE1EC2423E054EC9479ACF`.
Portable and build EXEs are identical. windeployqt Release deployment excludes
optional OpenSSL, retains Schannel, and startup passed with a system-only PATH,
eight Qt/MinGW runtime modules from the package and empty stderr.

Test EXE SHA-256:

| EXE | SHA-256 |
| --- | --- |
| main_window_visual_dispatch_tests | 41F671ABD3ED769DF45BEECD07B398560215E115DB1A8280C12DB106248647CD |
| visual_dispatch_session_tests | 6F47FBC939A09B8D893844AB9217DD59AC5C9CA72CBF30F0C671FB8F69AC50AF |
| main_window_layout_tests | ACE3E3462B86C0FA2AEC40A15A8467F73BBEE987038875EBD23507DE7BAD1E6C |
| video_view_tests | B8B08949DDEB38E97E7A15767E8F3826E01861A5AAC153AE07B92D198A625461 |
| operator_console_preview | A2BCEA5D592B085B0264016D295361A94BF34A9E277B52A3D9C4FFC53AD4A06E |

The output `README.md` indexes all ten PNGs. `manifest.json` records captured
geometry/state and `BUILD_INFO.json` records source, EXE and screenshot hashes.
Subsequent documentation-only commits do not change the tested binaries.
