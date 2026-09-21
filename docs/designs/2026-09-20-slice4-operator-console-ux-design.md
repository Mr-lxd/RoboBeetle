# Slice 4 Task 03 — Operator Console UX Final Design

## Status and authority

This document records the **final, human-reviewed Task 03 UI** that was accepted
before Vision Slice 5 started.

Repository baseline for this Task 03 branch:

- base main: `d84548c46f8edd862595b9edf38ffad1963b318c`
- accepted implementation commit: `c3b2fc8f38d8e8c38d7b866269c8b449c65e8b5b`
- branch: `codex/slice4-operator-console-ux`

Earlier Task 03 drafts in this file were refined during iterative native Qt review.
The rules below are the authoritative final presentation contract.

Task 01/02 transport/controller/protocol semantics remain authoritative. Task 03
does not redesign RBRP, STM32, robot motion, model inference, confidence threshold,
or camera ownership.

## Root workspace

MainWindow keeps:

1. one compact top connection bar;
2. exactly one vertical `workspaceSplitter`;
3. an upper video-first dashboard;
4. a lower Operator tools area.

Minimum window size remains exactly `1100x720`.

Screen-aware startup uses:

- width: `max(1100, min(1420, availableWidth - 32))`
- height: `max(720, min(1000, availableHeight - 80))`
- fallback: `1420x1000`

The UI remains usable at 1100x720, 1420x880, and 1600x1000.

## Shared Pi endpoint bar

The compact top bar exposes one shared authoritative Pi Host.

Remote mode uses:

- Robot TCP: host + 47000
- RBVS video: host + 47010
- Vision HTTP control: host + 47011

Direct mode keeps serial configuration for robot control while Vision continues to
use the committed Pi Host.

The edited host and committed host remain distinct. Endpoint mutation is blocked
while relevant Vision operations are active. Host changes must not couple Robot,
Video, and Vision lifecycles.

## Video-first dashboard

The upper dashboard has exactly two columns:

- Realtime Video / Vision Status card
- compact telemetry/status rail

There is exactly one `VideoView`.

The VideoView surface is sized to an **exact integer multiple of the source aspect
ratio**. For the deployed 640x480 source this is exact 4:3, preventing the
KeepAspectRatio renderer from producing a narrow black edge.

The Vision Status panel expands into the remaining space beside the video rather
than leaving an unused middle column.

### Vision Status summary

The main status panel shows only operator-level information:

- Video connection state with prominent status light/text
- Video FPS
- source resolution
- Storage Free from Pi `free_disk_bytes` (disk storage, not RAM)
- Inference state with prominent status light/text
- inference FPS / capture-to-inference latency
- detections count
- HTTP reachability
- Capture state
- recorded/snapshot counters

Video FPS and inference FPS use distinct emphasized presentation styles.

Detailed diagnostics remain in Vision Details.

### Vision Controls

Controls are vertically arranged below Vision Status:

1. Connect / Disconnect Video
2. one state-driven Inference toggle
3. Snapshot
4. one state-driven Recording toggle

The visible Inference action is authoritative-status driven:

- Disabled -> Start Inference
- Starting / Running -> Stop Inference
- Stopping -> Stopping Inference...
- Retrying -> Retrying Inference...
- Failed -> Retry Inference, with the existing exceptional Clear Error recovery
  action retained

The visible Recording action is authoritative-status driven:

- Idle -> Start Recording
- Recording -> Stop Recording
- Stopping -> Stopping Recording...

No optimistic state transition is allowed. POST responses are reconciled by
authoritative status GET. Vision actions emit zero RobotController writes.

The old Refresh Status control remains an internal compatibility action rather than
an operator-facing duplicate. Normal status polling is established with the Vision
connection path.

## Right telemetry/status rail

The right rail is one vertical column in this order:

1. Leak Detection
2. IMU
3. Depth Sensor
4. Protocol
5. Actuator Control

Card sizing is content-aware so IMU/Depth/Protocol text is not clipped while the
five cards visually fill the video-region height.

Actuator Control is a **status-only** panel. It contains five semantic servo rows:

- FrontRight
- FrontLeft
- Depth
- RearRight
- RearLeft

Each row contains only servo name plus status light/text. It does not duplicate
PWM, angle, Enable, Neutral, Apply, or slider controls.

Confirmed enabled/ACKed servo state is highlighted in blue.

The five status rows distribute through the available Actuator Control height
rather than clustering at the top.

## Operator tools

The final top-level tab order is:

1. Motion / Gait
2. Servo Fine Control
3. Vision Details
4. Telemetry Details
5. Protocol Details
6. Data Plots

At minimum window height, Motion/Gait and Servo Fine Control may use vertical
scrolling. There is no global workspace scroll.

### Motion / Gait

Motion/Gait contains three horizontal functional areas:

- Motion Control
- Gait / Vertical
- Log

The runtime Log is intentionally integrated into Motion/Gait in the final reviewed
layout.

The center Motion Stop remains directly reachable.

The bottom motion label is `Brake`, but the underlying existing
`MotionMode::Backward` action remains disabled/pending verification. Task 03 does
not introduce a new Brake protocol command.

The persistent action bar keeps:

- Enable All
- Disable All
- disabled Emergency Stop

Emergency Stop remains disabled because no corresponding frozen protocol command
exists.

### Servo Fine Control

All per-servo editable controls live here, not in Actuator Control:

- PWM range
- PWM numeric value
- PWM slider
- Angle numeric value
- Apply
- Enable / Release
- Neutral

There is no separate Set Angle button. Angle editors hide spin arrows and preserve
the existing edit/commit behavior.

Remote RBRP mode continues to reject unsupported raw PWM operations.

## Visual status semantics

Video Connected uses the established green status treatment.

Inference Running uses a distinct blue status light.

Enabled/ACKed actuator status uses blue.

True danger/error states retain red; ordinary Stop actions are not presented as a
red emergency action.

`Depth Sensor — ROVMAKER` is shortened to `Depth Sensor`.

## Safety and lifecycle invariants

Task 03 presentation changes must not:

- create a second VideoView or camera
- create a second VisionControlClient/poller
- redesign RBRP
- modify STM32
- issue robot motion from Vision actions
- start inference merely by connecting video
- couple Video disconnect to Inference stop
- reset capture counters when inference stops
- treat stale HTTP data as current state

The deployed Slice 4 Pi backend supports explicit manual inference Start/Stop.

## Verification and acceptance

Final accepted code at `c3b2fc8...` passed:

- focused MainWindow / layout / inference / remote-controller tests
- full CTest: 17/17 PASS
- `git diff --check`: PASS apart from expected Windows LF/CRLF warnings

Native Qt was repeatedly reviewed during iterative layout refinement.

Hardware integration was also completed before Slice 5:

- Pi shared host / Robot TCP connectivity exercised
- RBVS 47010 video exercised with USB camera
- Vision HTTP 47011 reachable
- manual inference Start -> Running -> Stop -> Disabled verified on Pi
- Snapshot exercised
- servo enabled/ACKed presentation observed
- user-level Vision service established on Pi
- `loginctl enable-linger pi` enabled for persistent user service startup

A Windows portable deployment directory was produced with `windeployqt` and
verified to start without manually adding Qt/MinGW paths.
