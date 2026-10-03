# Desktop visual error display implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add one independently testable step: show the selected FOMO centroid, normalized horizontal/vertical errors, image center and error line in the existing Console video view. This visual diagnostic path is always DRY_RUN and has no motion output.

**Architecture:** Preserve CameraOwner, inference, TCP streams, MainWindow's inference/status/size/timestamp freshness gate, controllers and firmware. A pure `TargetState` selector uses existing `DetectionFrame` original pixels, chooses the highest-confidence valid observation (first in stream on a tie), and computes `ex = (u - W/2)/(W/2)` and `ey = (v - H/2)/(H/2)`. VideoView renders these values within the displayed image, retaining all existing amber detection markers. No yaw controller, identity tracking, motion dispatch, PWM, safety-limit changes or real-actuation toggle is included in this step.

**Tech Stack:** Existing Qt 6 / C++20 / CMake / Ninja / MinGW console and its CTest harness.

## Baseline and boundaries

- Source baseline: `243755648dc0ac050bae8f39170adf7fea08eaf1`. Initial refresh failed with a transport/TLS error; the later user-authorized draft-PR closeout successfully fetched origin/main and verified this same SHA. The stale local main was fast-forwarded to it without switching the feature worktree.
- Work only in the attached `visual-error-dry-run` worktree on `codex/visual-error-dry-run`. Preserve the user's original checkouts and `D:\RoboBeetleConsole-portable-motion-gait-flex-ui53a0e7`.
- Existing manual controls retain their behavior. The DRY_RUN label describes the new visual diagnostic path, not a global interlock on manual control.

## Task 1: baseline and failing display test

- [x] Configure `build/qt-visual-dry-run` with Release, BUILD_TESTING=ON, Qt 6.11.2 and the installed MinGW compiler; build the existing console/tests.
- [x] Run baseline `ctest --test-dir build/qt-visual-dry-run --output-on-failure` with Qt runtime on PATH and `QT_QPA_PLATFORM=offscreen`; record any baseline failure separately.
- [x] Extend `RoboBeetleConsole/tests/video_view_tests.cpp` with a 640x480 image displayed in a 400x400 widget: image center must map to (200,200), a (160,120) target to (100,125), and the error line must disappear after clearing detection metadata. Run `video_view_tests` and observe the new assertions fail on the existing rendering.

## Task 2: target state and calculation

- [x] Add `RoboBeetleConsole/src/vision/TargetState.h` and `.cpp`, carrying frame ID, capture timestamp, source size, selected observation, ex and ey. Expose `selectTargetState(const DetectionFrame&)` returning an optional state.
- [x] Add `RoboBeetleConsole/tests/target_state_tests.cpp` for left/center/right, above/below, a different source resolution, highest confidence, stable ties, empty input, zero dimensions, non-finite confidence/coordinates and out-of-image coordinates.
- [x] Register the module and tests in `RoboBeetleConsole/CMakeLists.txt`. First use a null-result stub to observe numerical/selection tests fail; then implement validation, selection and the two normalization formulas and rerun to GREEN.

## Task 3: display integration

- [x] In `VideoView.h/.cpp`, expose the current optional target state and diagnostic text for the existing view, without introducing a controller/transport dependency.
- [x] Render a cyan image-center cross, cyan error line to the selected target, and a compact translucent diagnostic panel with `VISION DRY_RUN`, selection policy, target class/coordinates, confidence, ex/ey or `NO_TARGET`. Clip graphics to the image, not the surrounding widget.
- [x] Calculate state only from accepted overlay metadata. Clear it with existing overlay/video lifecycle and reject source size mismatches. Preserve all existing amber markers and labels.
- [x] Extend view tests to inspect the actual selected numerical state, diagnostic text, clearing lifecycle and rendered geometry. Add a MainWindow integration assertion that valid detection updates display errors without sending robot commands, retaining existing stale/stop/disconnect assertions.

## Task 4: verify and handoff

- [x] Rebuild and run the full Console CTest suite. If the known dashboard clipping failure reproduces in baseline, report it separately and leave it outside this change.
- [x] Save and inspect offscreen rendered images of left/center/right/no-target to confirm legibility, selected target and scaling. These are simulated display evidence, not camera or hardware acceptance.
- [x] Build a new portable folder `D:\RoboBeetleConsole-portable-visual-error-dry-run-20261003` using `windeployqt --release --no-translations`; do not overwrite the user's old folder. Include a short Chinese README with launch/observation steps and scope.
- [x] Add `docs/desktop-visual-error-dry-run.md` with formulas, selection caveat, existing freshness semantics, build/run commands and the left-center-right acceptance table. Report changed files, tests, pending real-camera observation and Git state. No commit, push, PR, firmware flashing or physical actuation.

## User-authorized draft PR closeout (2026-10-03)

The subsequent user request authorizes commit, push and a draft PR on this branch, replacing the earlier no-publication boundary. No merge or actuation is authorized.

- [x] Distinguish fresh empty metadata (`NO_TARGET`), unknown/stale metadata or status (`STALE`) and authoritative inactive/stopping inference (`INFERENCE_OFF`); retain the original renderability gate and thresholds.
- [x] First observe the new status/Stop assertions fail, then implement display-reason clearing. Verify actual Stop POST/ACK/reconciliation clears ex/ey and sends no robot command.
- [x] Build and run font-aware tests; final full result 18/21, focused result 4/4. Record three visual tests and three reproduced baseline failures; the preceding 19/21 run illustrates the controller timing sensitivity.
- [x] Deploy the new status-aware portable folder without overwriting earlier packages; code commit `657d1262f7865be06b3b69caaa55c65f1ce03aba`, EXE SHA-256 `955BE5F63467653CE16F9892AA0D4833BA5DFA3B1D7C017A39985AAB94CD7821`. Record dimensionless ex/ey sign conventions.
- [x] Literal GNU grep on production additions and complete pure modules returns no interface matches. Commit code separately from documentation.
- [ ] Publication handoff: push the branch, create/attach a draft PR, paste `git diff --stat main...HEAD` and verify the remote head. Leave main unmerged. The PR and final handoff provide the publication evidence rather than a self-referencing document commit.
