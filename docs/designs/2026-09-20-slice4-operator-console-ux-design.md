# Slice 4 Task 03 — Operator Console UX Design

## Contract and scope

This record implements the frozen Slice 4 Task 03 contract against
`origin/main` `d84548c46f8edd862595b9edf38ffad1963b318c` in the isolated
`codex/slice4-operator-console-ux` worktree. The change is presentation-only
inside `RoboBeetleConsole`: Task 01/02 network, controller, lifecycle, and
status semantics remain authoritative and unchanged.

The console must not create a second camera, video view, poller, control path,
robot write, or automatic action. Real hardware, Pi, serial, and robot access
are out of scope; tests use only fake/loopback/synthetic inputs.

## Frozen widget hierarchy

`MainWindow` owns one `appCanvas` vertical layout:

1. `topHeaderBar` contains exactly two horizontal rows. Row one contains the
   title, shared Pi Host editor, explicit Apply button, and Vision port. Row
   two contains the existing Robot/Serial controls, state, and authority. A
   single-line dirty-host hint sits below the two rows.
2. `workspaceSplitter` is the only `QSplitter`, vertical, with non-collapsible
   children. Its top child is the dashboard and its bottom child is the
   operator-tools pane.
3. The dashboard is a horizontal layout with one existing `VideoView` in a
   video card at stretch 4 and a compact telemetry sidebar at stretch 1.
4. The operator-tools pane has a persistent action bar followed by one
   `QTabWidget` in this exact order: Motion / Gait, Actuators, Vision Details,
   Telemetry Details, Protocol Details, Log, Data Plots.

There is exactly one video view and one vertical splitter. Details actions
only select the corresponding tab; construction, resize, tab changes, and
style changes never issue network, capture, inference, or robot commands.

## Geometry and sizing invariants

The root minimum remains exactly 1100 x 720. The application accepts and
records settled sizes 1100x720, 1420x880, and 1600x1000. The splitter is
initialized once (tools height 190 below the 760-pixel client threshold,
otherwise 240), with top stretch one and bottom stretch zero; no recurring
`setSizes` loop or persisted QSettings state is used. Sidebar width is bounded
to 272..320 pixels and the video/sidebar stretch is 4:1. The sidebar has no
scrollbar at the required sizes and Leak is the smallest card.

Actuator cards retain the existing five card objects, order, callbacks, and
state. A single host grid reflows existing widgets only when its column count
changes: five columns at widths >= 5*248+4*8, three at >= 3*248+2*8, two at
>= 2*248+8, otherwise one. Required layouts are five columns at 1420/1600
and a 3+2 arrangement at 1100.

## Presentation and ownership

The primary, secondary, stop, danger, and disabled roles are represented by
`consoleActionRole` properties and one shared stylesheet. The frozen color,
typography, radius, border, gap, and minimum button-height tokens are applied
without changing handlers or guards. `ElidedLabel` is a pure one-line QLabel
utility: it stores `fullText`, elides with `Qt::ElideRight`, exposes the full
text as tooltip, and has no timer/network behavior.

The persistent action bar owns the existing Motion Stop and motion status,
`disableAllButton_`, and a disabled Emergency Stop. The old d-pad stop is not
duplicated; its center cell is inert `Manual`. Motion/Gait retains the exact
`Motion / Gait — Bench` title and existing handlers. Long diagnostics move to
Vision Details, including the full read-only 64-character inference SHA.

The video card keeps the existing `VideoView`, compact inference/capture rows,
and one-line `visionControlMessage` with the frozen priority Inference >
Capture > Control > ACK. RX FPS is derived from existing VisionClient
diagnostics and is explicitly not inference or unique-display FPS.

## Required helper boundaries

MainWindow construction creates the complete widget tree before bindings.
Bindings are grouped in `bindVisionUi()` and `bindControllerUi()`. Refresh
helpers are presentation-only (`refreshVideoDiagnosticsUi`,
`refreshProtocolUi`, `refreshVisionNoticeUi`, `reflowActuatorCards`, and
`initializeWorkspaceSizes`). Existing `applyPiHost`, `closeEvent`, and refresh
methods retain their Task 02 behavior; only their presentation locators are
updated.

## Verification artefacts

The layout test target maps U01–U40 to concrete object-tree, geometry, style,
locator, and no-side-effect assertions. The preview executable constructs the
real MainWindow with a fake controller, loopback vision client, and synthetic
video, then captures unscaled S01–S16 PNGs plus the additional S10 lower
actuator view and a JSON manifest. Native Windows and offscreen evidence are
labelled separately; unavailable native acceptance remains pending.
