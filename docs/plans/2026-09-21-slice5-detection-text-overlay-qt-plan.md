# Vision Slice 5 — Qt Detection Text Overlay Plan

## Baseline

RoboBeetle base:

`97620aeeb23c6a64c5f4663dcc427de514ddce08`

Backend contract:

`fomo-visual-servo main @ 1c220d1056108734acb790eb7e16a6d908493957`

Working branch:

`chatgpt/slice5-detection-text-overlay`

## Phase B01 — Capability parsing

Extend `VisionCaptureStatus` with optional:

- detection stream supported
- detection stream port
- detection stream version

Acceptance:

- strict types/ranges
- missing fields remain unsupported
- endpoint reset clears capability
- capture/inference actions do not invent capability
## Phase B02 — Metadata codec/client

Add:

- `DetectionMetadata`
- `DetectionStreamDecoder`
- `DetectionClient`

Acceptance:

- bounded NDJSON
- exact v1/type/coordinate-space validation
- 64 KiB line maximum
- 256 detection maximum
- 128-byte UTF-8 class-name maximum
- finite/range validation
- strictly increasing metadata frame IDs
- no system proxy
- stale endpoint callbacks fenced by generation
- metadata failure isolated from video/control

## Phase B03 — Freshness policy

Keep RBVS latest-frame display.

Render only when:

- HTTP status fresh
- inference Running
- source sizes equal
- detection timestamp <= video timestamp
- age <= 1500 ms
- detections non-empty
Do not use wall-clock time for frame association.

Do not cache/replay old video frames.

HTTP freshness expiry hides text while preserving a healthy metadata
connection.

Within one Video session, do not repeatedly reconnect a failed identical
metadata endpoint.

## Phase B04 — VideoView text renderer

Retain current RBVS frame capture timestamp.

Render `class confidence` near the centroid using the actual image target
rectangle.

Allowed:
- white text
- small dark shadow/outline

Not allowed:
- bbox
- marker
- dot/circle
- crosshair
- line/trail
- target lock indicator
- detection background box
## Phase B05 — MainWindow composition

MainWindow receives one DetectionClient from application composition.

Connect 47012 only after:

- Video is Connected
- fresh 47011 status advertises support
- advertised version is exactly v1

Clear/suppress overlay on stale/disabled/mismatch/disconnect conditions.

A Video disconnect or host switch resets the detection session fence.

No RobotController/RBRP write may originate from this path.

## Phase B06 — Automated gate

Focused:

- vision_control_client_tests
- detection_stream_decoder_tests
- detection_client_tests
- video_view_tests
- main_window_tests

Then full CTest.

Expected final suite: 19 tests, zero failures.

## Phase B07 — Native Pi acceptance — completed

The real Pi/native Qt gate completed successfully:

1. Pi 47010 / 47011 / 47012 reachable.
2. Inference was forced to authoritative Disabled before the native UI gate.
3. Connect Video established the real 47010 stream.
4. Fresh advertised `detection_stream_supported=true`, port 47012, version 1
   established one real 47012 Qt connection.
5. With inference Running, native Qt visibly rendered only
   `class confidence` text for fresh detections.
6. No rectangle, dot, circle, crosshair, centroid glyph, or background marker was
   visible.
7. Stop Inference returned the Pi to authoritative Disabled.
8. Overlay text disappeared while both the existing 47010 video connection and
   47012 metadata connection remained established.
9. Automated MainWindow loopback coverage verifies metadata failure isolation,
   old-Pi capability suppression, HTTP-stale display suppression,
   one-attempt-per-video-session behavior, session reset/reconnect semantics, and
   zero Robot transport writes.
10. The separate GUI Disconnect Video socket-release step was not re-run in the
    final native hardware gate; that lifecycle is covered by the automated tests.
11. Pi inference was left Disabled.

Release gate after hardware acceptance:

- final full CTest
- `git diff --check`
- commit/push branch
- GitHub final review
- create PR
- merge only after review PASS
- rebuild portable Windows bundle only if requested
