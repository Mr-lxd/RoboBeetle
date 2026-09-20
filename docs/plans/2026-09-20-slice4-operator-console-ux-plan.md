# Slice 4 Task 03 — Operator Console UX Implementation Plan

## Ordered execution

1. Reconfirm the frozen base, worktree, toolchain, and clean status; configure
   `RoboBeetleConsole/build-task03-qtkit` and run the unchanged 16-target
   baseline under `QT_QPA_PLATFORM=offscreen`.
2. Add this design/plan pair and the test-only fixture, layout test target, and
   preview target. Run the new layout tests against the old UI and retain the
   failing RED evidence before changing production widgets.
3. Implement `ElidedLabel` and add it to every MainWindow-compiling target
   through the local `RB_CONSOLE_UI_SOURCES` CMake list.
4. Refactor MainWindow construction to the frozen widget tree. Create all
   widgets and details first, then install one-time vision/controller bindings.
   Preserve existing controller guards, Task 02 state projections, and the
   single VideoView frame destination.
5. Move diagnostics into Vision Details, add the compact telemetry sidebar,
   persistent operator action bar, exact seven-tab order, actuator reflow, and
   the shared action-role stylesheet. Keep all operation handlers and endpoint
   semantics unchanged.
6. Migrate only presentation locators in existing tests, then build and run
   focused tests and the full 17-target CTest suite under offscreen QPA.
7. Build the real preview executable. Capture S01–S16 at the prescribed
   logical sizes/scales (and S10 lower scroll), inspect every PNG, and repair
   only deterministic presentation defects.
8. Produce the external review directory with the complete unified diff,
   SHA-256 manifest, screenshot manifest, test/geometry evidence, and review
   report. Leave the worktree uncommitted and stop at the Task 03 reviewer
   boundary.

## Test gates

The RED gate must show the old tree fails at least the new structural/layout
assertions. GREEN requires all existing 16 targets plus
`main_window_layout_tests` (17 total), with no weakened behavior assertions.
The preview uses only fake/loopback/synthetic inputs and reports requested and
settled sizes, available screen, QPA, scale, DPR, tab, geometry, and output
metadata. Native Windows and offscreen results are never conflated.

## Safety and stop rules

No source outside the console UI/test/documentation scope may change. Do not
commit, push, merge, deploy, or access physical hardware. If the required
base, toolchain, or contract cannot be reconciled with the checked-out code,
preserve completed work and report a concrete BLOCKER instead of silently
changing the architecture.
