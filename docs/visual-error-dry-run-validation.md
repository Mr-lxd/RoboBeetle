# Visual error DRY_RUN validation — 2026-10-03

Scope: Qt centroid/error display only. No physical camera, FOMO deployment, robot command, firmware flashing or water validation was performed. The screen/camera experiment remains pending. Existing manual controls retain their behavior.

## Reproduction environment

- Baseline: `243755648dc0ac050bae8f39170adf7fea08eaf1` (main/origin/main at preparation).
- Qt 6.11.2 MinGW, GCC 13.1.0, CMake/Ninja, Release, BUILD_TESTING=ON.
- `QT_QPA_PLATFORM=offscreen`, `QT_QPA_FONTDIR=C:\Windows\Fonts`.
- Source and portable EXE hashes are recorded in [desktop instructions](desktop-visual-error-dry-run.md).

```powershell
$env:PATH = 'D:\Qt\6.11.2\mingw_64\bin;D:\Qt\Tools\mingw1310_64\bin;' + $env:PATH
$env:QT_QPA_PLATFORM = 'offscreen'
$env:QT_QPA_FONTDIR = 'C:\Windows\Fonts'
& 'D:\Qt\Tools\CMake_64\bin\ctest.exe' --test-dir build/qt-visual-dry-run --output-on-failure --parallel 4
& 'D:\Qt\Tools\CMake_64\bin\ctest.exe' --test-dir build/qt-visual-dry-run --output-on-failure -R '^(target_state_tests|video_view_tests|main_window_visual_error_tests|detection_stream_decoder_tests)$'
```

The three visual targets verify numerical selection/normalization, rendered geometry/clearing, and real loopback MainWindow metadata/status integration. The fourth target preserves decoder/renderability boundary checks and tests display reasons.

The integration test checks `POST /api/v1/vision/inference/stop`, accepts an HTTP 200 stop acknowledgement, verifies reconciliation clears the target/errors, then confirms authoritative `operation=stopping` displays `INFERENCE_OFF`. Fresh empty detections show `NO_TARGET`; unknown worker state, expired HTTP status and stale/missing detection metadata show `STALE`. All diagnostic/Stop/status transitions leave FakeTransport robot writes empty. This does not prohibit a user from using the existing manual controls.

## Baseline failure evidence

Unmodified baseline was extracted with `git archive` into ignored build storage and built independently using the same toolchain. Independent logs avoid relying on the overwritten CTest `LastTest.log`:

- `build/qt-visual-dry-run/baseline-controller-tests.log`
- `build/qt-visual-dry-run/baseline-font-ui-tests.log`

The relevant baseline excerpts are:

```text
robot_controller_tests
FAIL: scheduled Enable ACK should establish the sustained-load precondition
FAIL: multiple user commands should be accepted into the APC220 bounded queue
(the queue assertion failed seven times)
APC220 sustained load: RTT=170 ms, watchdog=500 ms,
configured hard budget=490 ms, configured margin=10 ms,
observed max heartbeat gap=341 ms
```

The first failure is in `testApc220SustainedLoadPreservesHeartbeatSafetyMargin`: Enable was not confirmed within the test's 450 ms wait, so subsequent ordinary commands failed their enabled precondition. It passed in one complete feature run and failed again in the final complete run. The specific timer/scheduler root cause remains unconfirmed; no hardware failure is inferred and no controller timing/safety code was changed.

```text
main_window_tests
FAIL: IMU card status/metric labels must not be vertically clipped
FAIL: Depth card status/metric labels must not be vertically clipped
FAIL: Protocol card TX/RX/CRC/Timeout/ACK RTT labels must not be vertically clipped
FAIL: Leak card label must not be vertically clipped

main_window_layout_tests
startup_available=800x800 expected=1100x720 actual=1100x720
FAIL: feedback: Motion/Gait is fully visible at the comfortable window size
```

These are pre-existing normal-font layout assertion failures reproduced on the unmodified baseline. The final complete feature run returned 18/21 (all three baseline failures); a preceding run returned 19/21 with the two layout failures. Neither run is reported as all-green. Final focused validation returned 4/4. No unrelated layout assertion or safety constraint was weakened.

Full feature log: `build/qt-visual-dry-run/pr-full-tests.log`; final focused log: `build/qt-visual-dry-run/pr-visual-tests.log`. These local build logs are not Git artifacts; the excerpts above provide reviewable baseline evidence in the PR.

## Literal GNU grep check

Run from the feature worktree after the commits, with local main at the baseline above:

```powershell
$pattern = 'IConsoleController|RobotController|RemoteRobotController|MotionManager|SimpleGait|CPG|ServoService|ServoId|ServoPwm|ServoAngle|MotionMode|startMotion|stopMotion|setGaitBackend|setFrontRearCoordination|enableServo|disableServo|disableAll|neutralServo|acquireControl|releaseControl|controller_->|#include.*"(controller|robot|remote|transport)/'
& 'D:\Git\cmd\git.exe' diff --unified=0 main...HEAD -- RoboBeetleConsole/src |
    & 'D:\Git\usr\bin\grep.exe' -E '^\+[^+]' |
    & 'D:\Git\usr\bin\grep.exe' -nE $pattern
"grep_exit=$LASTEXITCODE"

& 'D:\Git\usr\bin\grep.exe' -nE $pattern `
    RoboBeetleConsole/src/vision/TargetState.h RoboBeetleConsole/src/vision/TargetState.cpp `
    RoboBeetleConsole/src/vision/VideoView.h RoboBeetleConsole/src/vision/VideoView.cpp `
    RoboBeetleConsole/src/vision/DetectionMetadata.h RoboBeetleConsole/src/vision/DetectionMetadata.cpp
"grep_exit=$LASTEXITCODE"
```

Both scans produce no matched lines and exit 1 (GNU grep's **no match** result, not a command failure). The first scans added production lines, including MainWindow additions; the second scans the complete pure calculation/rendering modules. Tests are excluded because they deliberately use FakeTransport/RobotController to observe zero writes. Existing MainWindow manual handlers are excluded from the added-lines scan because their references predate this feature.

This is a concrete interface/header-reference check, supported by the integration zero-write assertion, rather than a formal proof against every possible runtime behavior. The production scope contains only Qt/C++ vision data and drawing; no robot controller, transport, gait or firmware implementation is changed.

## Portable smoke check

The new `D:\RoboBeetleConsole-portable-visual-error-dry-run-status-20261003` package is deployed with `windeployqt --release --no-translations --include-plugins qjpeg,qoffscreen`. With PATH restricted to Windows system directories, its EXE stays running for three seconds and loads its own `platforms/qoffscreen.dll`; its `imageformats/qjpeg.dll` is present. The check stops only the PID it launched. This verifies dependency deployment/startup, not live video or robot control.
