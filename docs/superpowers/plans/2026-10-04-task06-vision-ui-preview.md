# Task06 Vision UI preview implementation plan

> Use subagent-driven-development for independent direction/test and preview tasks; root integrates MainWindow/VideoView and verifies all results.

Goal: deliver the user-specified Auto follow preview and screenshots without changing the existing safety policy.
Architecture: MainWindow derives independent readiness and presentation; existing session provides arming/outcomes. VideoView paints the armed indication. Preview uses fake command/diagnostic inputs, never hardware.
Tech: Qt6.11.2 Widgets/C++20, MinGW13.1, CMake/Ninja, PowerShell.

- [x] Establish isolated main-based branch and clean baseline Qt build (full regression verified after implementation).
- [x] RED: session-start direction equals VisualPolicyConfig config constant; remove UI sign selectors and assert readiness/button/banner/geometry in main_window_visual_dispatch_tests.cpp.
- [x] GREEN: VisualPolicyConfig constant defaults; VisualDispatchSession initialization only. Preserve pure policy and remote implementation. Adapt only former direction confirmation tests.
- [x] MainWindow.cpp/.h: move dispatch widgets into Auto follow in Vision column, add independent prerequisite labels and fault/state/action rendering, keep all existing manual/feature-off and diagnostic wiring. VideoView.cpp paints green edge and confirmed command badge; presentation signature accepts badge command.
- [x] operator_console_preview.cpp: add five state scenarios at both requested sizes using fake controller and real session; include geometry/state metadata and truthful base provenance.
- [x] Build and focused checks using cmake --build <task06-build>/qt --parallel4 and ctest -R visual_dispatch --output-on-failure.
- [x] Full ctest --output-on-failure -j4. Inspect all10 rendered images and adjust clipping only when evidence demands.
- [x] Commit tested source; windeployqt Release portable build, record EXE/test hashes and image/source manifest.
- [x] Push and create draft PR; attach it. Preserve old worktrees/packages; no merge or physical actuation.

Delivery: draft PR49 (https://github.com/Mr-lxd/RoboBeetle/pull/49). Binary source 93a640ff8b10119c2aafa284f2120c063e9fcab1; 30/30 PASS; ten screenshots and portable provenance in docs/task06-vision-ui-preview-validation.md. Operator/Claude review and physical trial pending; not merged.
