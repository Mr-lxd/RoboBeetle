# Qt regression stability validation

Date: 2026-10-04. Baseline: `db216e74e5cde3c81351804568241442567aff1a` (merged PR #45). Branch: `codex/qt-regression-stability`.

## Scope and cause evidence

- Telemetry clipping reproduced at 1420x880. Styled IMU/Depth cards needed 115px but received 105px; Protocol needed 125px but received 110px; Leak needed 63px but received 60px. Explicit compact card minima allowed layouts to compress labels below their font minimum. `QLayout::SetMinimumSize` now recomputes the four affected card minima after style/live status updates. The unaffected Actuator 145px floor remains intact. Existing label assertions remain, also reject zero height, and print actual/required geometry on failure.
- The offscreen screen reports 800x800 and clamps startup to 1100x720. Resizing to 1600x1000 preserves the startup splitter allocation: Motion viewport 155px versus content minimum 160px. The geometry test now explicitly allocates content plus measured pane chrome, checks the viewport minimum and zero scrolling, and retains the user splitter preservation assertion. This isolates a screen-dependent startup assumption; it does not promise automatic splitter reset on resize.
- Historical `task05-arming-delivery-ctest.log` records LOST/CSV failures (lines 58-59; 25/28 suite passed; visual test 15.20s). The baseline timer test was **not reproduced locally**: eight serial repetitions passed (89.05s total). Wall-clock integration tests depend on timely event delivery: a >=500ms gap correctly invalidates the uninterrupted NO_TARGET sequence before LOST. The fixture now measures its cadence using elapsed monotonic time instead of counting nominal coarse timer sleeps. CTest runs `robot_controller_tests`, `main_window_tests`, and `main_window_visual_error_tests` serially even under `-j4`. This isolates CTest scheduling competition; it cannot prevent unrelated host stalls and does not prove the cause of the historical stall. Production timer/state-machine semantics and all five CSV transition checks remain unchanged.

## Reproduction environment

Worktree: `C:/Users/laixindong/.codex/worktrees/qt-regression-stability/RoboBeetle`

Build: `C:/Users/laixindong/.codex/worktrees/qt-regression-stability-build/qt`

Qt 6.11.2 MinGW, GCC 13.1.0, Release, Ninja; `QT_QPA_PLATFORM=offscreen`, `QT_QPA_FONTDIR=C:/Windows/Fonts`. PATH includes `D:/Qt/6.11.2/mingw_64/bin` and `D:/Qt/Tools/mingw1310_64/bin`.

```powershell
& D:/Qt/Tools/CMake_64/bin/cmake.exe -S RoboBeetleConsole -B C:/Users/laixindong/.codex/worktrees/qt-regression-stability-build/qt -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DCMAKE_PREFIX_PATH=D:/Qt/6.11.2/mingw_64 -DCMAKE_MAKE_PROGRAM=D:/Qt/Tools/Ninja/ninja.exe -DCMAKE_CXX_COMPILER=D:/Qt/Tools/mingw1310_64/bin/g++.exe
& D:/Qt/Tools/CMake_64/bin/cmake.exe --build C:/Users/laixindong/.codex/worktrees/qt-regression-stability-build/qt -j 4
& D:/Qt/Tools/CMake_64/bin/ctest.exe --test-dir C:/Users/laixindong/.codex/worktrees/qt-regression-stability-build/qt --output-on-failure -j 4
& D:/Qt/Tools/CMake_64/bin/ctest.exe --test-dir C:/Users/laixindong/.codex/worktrees/qt-regression-stability-build/qt --output-on-failure -j 1
```

## Results and local evidence

Log directory: `C:/Users/laixindong/.codex/worktrees/qt-regression-stability-build/`.

| Evidence | Result |
| --- | --- |
| `baseline-red.log` | Two layout binaries fail; visual integration passes |
| `baseline-diagnostic-red.log` | Same two failures, with the dimensions above |
| `timer-baseline-repeat.log` | Unmodified timer behavior, 8/8 serial passes; intermittent failure not reproduced |
| `geometry-green.log` | Both affected layout binaries pass |
| `targeted-green-repeat.log` | Three targeted binaries, three runs each: 9 passing executions, 89.01s; followed by a small clock-origin/zero-height assertion tightening |
| `final-build.log` | Final source builds successfully |
| `full-qt-green-parallel.log` | Final source, 27/27 pass, 0 failures, `-j4`, 47.68s |
| `full-qt-green-serial.log` | Final source, 27/27 pass, 0 failures, `-j1`, 60.06s |

PR #45 baseline has 27 tests. The historical PR #46 worktree includes one additional test (28 total); that separate change is outside this branch. Evidence covers desktop/offscreen Qt and loopback simulation only; no physical robot, firmware programming, ARM, or water validation was performed.

## Test executable copies

Only the three requested test executables were copied after testing ended. Destination: `D:/RoboBeetle-results/qt-regression-stability-20261004/`. These require the Qt/MinGW runtime PATH above; no application executable is delivered.

| File | SHA256 |
| --- | --- |
| `main_window_tests.exe` | `471AD566522CEDBC6B7A7AA08FCDAAB32396276645BEA87BFDB129367E803401` |
| `main_window_layout_tests.exe` | `D7D5E40DBB6FFDD970F39E3C1132A41D635950B4073AE9A47623738BDEEF683A` |
| `main_window_visual_error_tests.exe` | `120556CBA240881A697014564D6C2DC535E2FD6674FAAEBDE7D72BD0318559A8` |
