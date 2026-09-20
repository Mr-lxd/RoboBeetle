# Slice 4 Task 02 — Qt Manual Inference Control + Shared Pi Host Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use TDD and execute this plan
> task-by-task with explicit RED/GREEN checkpoints. Steps use checkbox syntax.

**Goal:** Consume the merged Task 01 HTTP contract in the Qt console, add manual
inference controls, and make one explicitly committed Pi Host authoritative for
Robot, RBVS, and HTTP control.

**Architecture:** `VisionControlClient` remains the only HTTP transport and owns
parsing, serialized request admission, generation/freshness, and ACK→GET
reconciliation. `InferenceUiState` is a timer/signal/socket-free projection;
`MainWindow` owns widgets, the committed Host transaction, and close guards.

**Tech Stack:** C++20, Qt 6.11.2 Core/Gui/Widgets/Network/SerialPort, CMake 4.3.2,
Ninja, Qt kit MinGW 13.1.0, existing no-QtTest expect/event-loop test style.

**Frozen source:** `Slice4_Codex_Task02_Frozen_Pack.md`, sections 0–12.
**Required base:** `c06dd44419c9b56dd97c6424d1b3b555f5697453`.
**Read-only backend:** fomo main `10e01bb4e2130c0a866f15bd6ee5ba5dd36ac20d`.

---

## Scope and files

Production changes are limited to:

- `RoboBeetleConsole/src/vision/VisionControlClient.h/.cpp`
- `RoboBeetleConsole/src/vision/InferenceUiState.h/.cpp` (new)
- `RoboBeetleConsole/src/vision/VisionClient.h/.cpp` (endpoint activity only)
- `RoboBeetleConsole/src/ui/MainWindow.h/.cpp`
- `RoboBeetleConsole/CMakeLists.txt`

Tests are limited to the existing Vision/MainWindow/client suites and new
`inference_ui_state_tests.cpp` and
`main_window_inference_controls_tests.cpp`. Documentation is limited to this
plan and the paired design record. No fomo, gateway, RBRP, firmware, robot
controller, deployment, or Task 03 layout file is in scope.

## Execution sequence

### Task 0 — Preflight and baseline

- [x] Identify the actual RoboBeetle root, origin, detached-root status,
  applicable AGENTS files, worktree list, and existing build caches.
- [x] Fetch origin and verify `origin/main` equals the frozen base.
- [x] Query fomo `refs/heads/main` read-only and verify the backend contract SHA.
- [x] Create the named isolated worktree at the exact base and verify a clean
  feature branch.
- [x] Configure/build/run original Console CTest in a new build directory with
  the matching Qt kit; record the initial environment mismatch separately from
  the corrected kit run.

### Task 1 — Contract documents

- [x] Create the paired design and plan records before feature tests.
- [x] Self-review both records against the frozen wire/status/UI/host/close
  tables; no implementation success or hardware acceptance is claimed here.

### Task 2 — Client RED tests: parsing, ACK, queue, freshness, endpoint

Modify `tests/vision_control_client_tests.cpp` with deterministic loopback
servers and add tests with these exact names:

- `configured_disabled_status_parses_capabilities_and_null_operation`
- `strict_capability_and_operation_impostors_fail_closed`
- `legacy_status_is_readable_but_manual_actions_are_unavailable`
- `capture_post_preserves_inference_metadata_and_freshness`
- `zero_metrics_and_full_sha_are_preserved`
- `manual_start_and_stop_use_exact_paths_body_and_injected_port`
- `retry_and_clear_error_send_one_post_each`
- `inference_ack_preserves_state_and_reconciles_once`
- `malformed_or_fake_inference_ack_cannot_change_state`
- `inference_error_preserves_code_message_and_capture_state`
- `second_mutation_is_rejected_while_one_is_active_or_queued`
- `failed_get_cancels_queued_action_and_revalidates_capability`
- `reply_finalization_has_no_reentrant_idle_gap`
- `ambiguous_inference_post_is_uncertain_without_auto_retry`
- `freshness_expires_and_capture_post_does_not_renew_it`
- `failed_get_disables_actions_until_refresh_recovers`
- `inference_actions_wait_for_reconciliation_get`
- `sent_post_blocks_low_level_endpoint_change`
- `endpoint_switch_cancels_get_and_stale_reply_cannot_cross_generation`
- `redirect_is_rejected_without_following_or_resending`

The 20 tests were added and the focused client target was run before adding
production members. The expected RED was feature/contract-missing compile
failure, not a malformed test or missing dependency.

### Task 3 — `VisionControlClient` implementation

- [x] Extend `VisionCaptureStatus` with typed presence/capability/operation data.
- [x] Add Start/Stop methods, typed acknowledgement/error signals, and the
  `actionBusy`, freshness, reconciliation, and polling accessors.
- [x] Keep one active request plus one queued mutation; preserve request kind,
  path, endpoint generation, and reentrancy guards.
- [x] Parse GET/capture/ACK/error payloads through separate paths, enforce exact
  action/outcome/status validation, apply ManualRedirectPolicy/NoProxy, and
  schedule one same-generation GET after every terminal inference POST result.
- [x] Add the 3500 ms single-shot freshness timer and endpoint/shutdown invalidation.
- [x] Run the client tests GREEN, then rerun the legacy capture-control cases.

### Task 4 — Pure `InferenceUiState` RED/GREEN

Create `tests/inference_ui_state_tests.cpp` first, with these table-oriented
cases:

- `initial_unknown_masks_everything`
- `unavailable_and_legacy_are_read_only`
- `manual_worker_state_table_is_exact`
- `operation_precedence_and_error_are_visible`
- `camera_gate_only_affects_start_side`
- `busy_and_reconcile_mask_actions_without_faking_state`
- `malformed_state_fails_closed`

Create QtCore-only `InferenceUiState.h/.cpp`; implement only the frozen pure
projection and rerun the helper target before UI work. The helper target passed
before MainWindow integration.

### Task 5 — MainWindow RED integration tests

Add the fake-network cases to the established MainWindow harness, and expose
the frozen standalone target through
`tests/main_window_inference_controls_tests.cpp` (which selects the Task 02
cases from that harness):

- `testTask02SharedHostWidgetsAndHttpOnlyControls`
- `testTask02RefreshAndInferenceUseOnlyCommittedHost`
- `testTask02CaptureAndInferenceErrorsStaySeparated`
- `testTask02ApplyHostBlocksQueuedMutation`
- `testTask02CloseNeverImplicitlyStopsRemoteWork`
- `testTask02RemoteAndDirectEndpointWidgetsStayDistinct`
- existing `testVisionCaptureActionDefersWindowClose` for close deferral

The wrapper intentionally reuses the current fake controller/loopback helpers;
it does not add a second production network path.

Run the integration target before adding widgets or MainWindow bindings; RED
identified missing controls/behavior rather than a fixture or Qt installation
problem. The final target and the existing `main_window_tests` both pass.

### Task 6 — Minimal UI and endpoint integration

- [x] Add `VisionClient::endpointBusy()` and
  `endpointActivityChanged(bool)` from the existing socket state only.
- [x] Build all MainWindow widgets before connecting initialization signals.
  Move Host to one top-level `piHost`, add `Apply Host`/hint, separate Remote
  TCP ports from Direct serial widgets, and preserve defaults/NoButtons.
- [x] Wire Start/Stop/Refresh and capture actions through the committed Host;
  use `makeInferenceUiState` in one refresh helper; keep inference errors out
  of Capture Error.
- [x] Implement host syntax validation, dirty/committed transaction,
  confirmation recheck, endpoint generation preservation, and close behavior.
- [x] Register the pure helper and MainWindow integration test targets in CMake
  with QtCore-only and existing Widgets/core wiring respectively.
- [x] Run all new integration tests GREEN, then the affected existing tests.

### Task 7 — Regression and review package

- [x] Build the full new worktree build directory with the verified kit.
- [x] Run the focused CTest regex and full Console CTest with actual exit codes.
- [x] Run `git diff --check`, `git diff --stat`, status, and forbidden-scope audits.
- [x] Map Q01–Q44 below to real tests/assertions and capture representative
  fake HTTP requests/responses, freshness, Host, and close evidence.
- [x] Generate an uncommitted complete diff artifact including every new file.
- [x] Report blockers/should-fix/NITs and unexecuted real hardware/UI acceptance;
  stop without commit, push, PR, merge, deployment, or Task 03.

## Q01–Q44 evidence map

The following anchors are the required review mapping; one test may cover a
closely related row only when the assertion is explicitly named in the test.

| ID | Test/assertion anchor |
| --- | --- |
| Q01 | `configured_disabled_status_parses_capabilities_and_null_operation` |
| Q02 | `strict_capability_and_operation_impostors_fail_closed` |
| Q03 | `legacy_status_is_readable_but_manual_actions_are_unavailable` |
| Q04 | `capture_post_preserves_inference_metadata_and_freshness` |
| Q05 | `zero_metrics_and_full_sha_are_preserved` |
| Q06 | `manual_start_and_stop_use_exact_paths_body_and_injected_port` |
| Q07 | `retry_and_clear_error_send_one_post_each` |
| Q08 | `inference_ack_preserves_state_and_reconciles_once` |
| Q09 | `malformed_or_fake_inference_ack_cannot_change_state` |
| Q10 | `inference_error_preserves_code_message_and_capture_state` |
| Q11 | `second_mutation_is_rejected_while_one_is_active_or_queued` |
| Q12 | `failed_get_cancels_queued_action_and_revalidates_capability` |
| Q13 | `reply_finalization_has_no_reentrant_idle_gap` |
| Q14 | `ambiguous_inference_post_is_uncertain_without_auto_retry` |
| Q15 | `freshness_expires_and_capture_post_does_not_renew_it` |
| Q16 | `failed_get_disables_actions_until_refresh_recovers` |
| Q17 | `inference_actions_wait_for_reconciliation_get` |
| Q18 | `sent_post_blocks_low_level_endpoint_change` |
| Q19 | `endpoint_switch_cancels_get_and_stale_reply_cannot_cross_generation` |
| Q20 | `testTask02ApplyHostBlocksQueuedMutation` holds an active GET plus queued Start and asserts Apply is disabled while the committed endpoint stays unchanged |
| Q21 | `redirect_is_rejected_without_following_or_resending` |
| Q22 | Existing `stopRecordingAllowsLongerServerFinalization` regression |
| Q23 | `initial_unknown_masks_everything`, `unavailable_and_legacy_are_read_only`, and `manual_worker_state_table_is_exact` |
| Q24 | `operation_precedence_and_error_are_visible` |
| Q25 | `camera_gate_only_affects_start_side` |
| Q26 | `busy_and_reconcile_mask_actions_without_faking_state`, plus diagnostics assertions in `testTask02RefreshAndInferenceUseOnlyCommittedHost` |
| Q27 | `operation_precedence_and_error_are_visible` (operation error remains visible) |
| Q28 | `testTask02SharedHostWidgetsAndHttpOnlyControls` and `testTask02RemoteAndDirectEndpointWidgetsStayDistinct` |
| Q29 | `testTask02RemoteAndDirectEndpointWidgetsStayDistinct` |
| Q30 | `testTask02RefreshAndInferenceUseOnlyCommittedHost` (all requests use the committed injected endpoint) |
| Q31 | `testTask02SharedHostWidgetsAndHttpOnlyControls` (injected port/host are preserved without constructor request) |
| Q32 | `testTask02RefreshAndInferenceUseOnlyCommittedHost` |
| Q33 | `testVisionConnectionIsIndependentFromControlTransport`, `testVisionCaptureControlsAreIndependentFromRobotTransport`, and `testTask02RefreshAndInferenceUseOnlyCommittedHost` |
| Q34 | `refreshPiHostUi` locks remote Opening/Connected/Closing and `VisionClient::endpointBusy`; raw socket coverage is `endpointBusyTracksRawSocketActivity` |
| Q35 | `testTask02RemoteAndDirectEndpointWidgetsStayDistinct` and the existing direct serial lifecycle tests |
| Q36 | `testTask02SharedHostWidgetsAndHttpOnlyControls` plus the dirty-host/committed-stop assertion in `testTask02RefreshAndInferenceUseOnlyCommittedHost` |
| Q37 | `testTask02SharedHostWidgetsAndHttpOnlyControls` (valid commit, invalid candidates, Escape cancel), `testTask02ApplyHostBlocksQueuedMutation`, and `VisionControlClient::sent_post_blocks_low_level_endpoint_change` |
| Q38 | `testVisionCaptureActionDefersWindowClose` uses a scheduled dialog answer and post-close recheck; `applyPiHost` rechecks locks after its dialog |
| Q39 | invalid URL/host:port/whitespace and valid normalized hostname assertions in `testTask02SharedHostWidgetsAndHttpOnlyControls` |
| Q40 | `testVisionCaptureActionDefersWindowClose` (busy deferral and accepted-work warning) |
| Q41 | `testTask02CloseNeverImplicitlyStopsRemoteWork` and the confirmed-close path in `testVisionCaptureActionDefersWindowClose` |
| Q42 | Every MainWindow fake-controller action asserts `FakeTransport::writes()` unchanged |
| Q43 | Full Console CTest and existing `main_window_tests`, `vision_client_tests`, `remote_robot_controller_tests` |
| Q44 | `testDashboardLayout` checks the approved normal/minimum window geometry; Task 03 remains responsible for final size |

## Exact build and test commands

Use the fresh build directory and matching Qt kit; the shell must prepend only
for the command process:

```text
cmake -S RoboBeetleConsole -B RoboBeetleConsole/build-task02-qtkit -G Ninja -DCMAKE_PREFIX_PATH=D:/Qt/6.11.2/mingw_64 -DCMAKE_CXX_COMPILER=D:/Qt/Tools/mingw1310_64/bin/g++.exe -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
cmake --build RoboBeetleConsole/build-task02-qtkit --config Debug --parallel 2
ctest --test-dir RoboBeetleConsole/build-task02-qtkit -C Debug --output-on-failure -R "^(vision_control_client_tests|inference_ui_state_tests|main_window_inference_controls_tests|main_window_tests|remote_robot_controller_tests|vision_client_tests|video_view_tests|rbrp_client_session_tests)$"
ctest --test-dir RoboBeetleConsole/build-task02-qtkit -C Debug --output-on-failure
git diff --check
```

CTest commands run with `QT_QPA_PLATFORM=offscreen` and PATH entries for
`D:/Qt/6.11.2/mingw_64/bin` and `D:/Qt/Tools/mingw1310_64/bin`. No dependency
installation or hardware access is permitted.
