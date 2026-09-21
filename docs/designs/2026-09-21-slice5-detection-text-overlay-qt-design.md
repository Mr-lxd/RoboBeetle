# Vision Slice 5 — Qt Detection Overlay Design

## Status

This document records the Qt-side Slice 5 contract.

Authoritative baselines:

- RoboBeetle main: `97620aeeb23c6a64c5f4663dcc427de514ddce08`
- fomo-visual-servo Slice 5 backend merged as:
  `1c220d1056108734acb790eb7e16a6d908493957`
- Qt branch: `chatgpt/slice5-detection-text-overlay`

The operator overlay is display-only. It is not a control signal.

## Scope

Slice 5 adds fresh, frame-associated detection visualization to the existing
Realtime Video surface.

Rendered form:

`centroid marker + <class_name> <confidence>`

Example:

`fish 0.88`

No bounding box, target selection, target lock, tracking identity, trail,
PID, visual servo, STM32 change, or robot motion is included.
## Network boundaries

Existing endpoints remain unchanged:

- 47000: Robot TCP / RBRP
- 47010: RBVS v1 JPEG video
- 47011: Vision HTTP status/capture/manual inference
- 47012: Slice 5 detection metadata

47010 is not extended or version-bumped.

Qt connects to 47012 only after a fresh authoritative 47011 status
advertises all of:

- `detection_stream_supported == true`
- a valid detection stream port
- `detection_stream_version == 1`

Missing capability means unsupported, not an error.

A future/unknown version fails closed and is not probed.

## Metadata client

`DetectionClient` owns one independent no-proxy QTcpSocket.

It uses a bounded NDJSON decoder with:

- max record: 64 KiB
- max detections: 256
- max UTF-8 class name: 128 bytes
- finite numeric validation
- source-coordinate range validation
- strictly increasing metadata frame IDs
Endpoint replacement uses a generation fence so callbacks from an old
socket cannot publish into the new endpoint session.

Within one Video session, a given continuously-advertised metadata endpoint
is actively attempted at most once. A failed 47012 connection is not retried on
each HTTP poll.

A new Video session resets that one-attempt fence. A fresh authoritative
capability withdrawal followed by a later re-advertisement also starts a new
capability generation and permits one new attempt.

47012 disconnect/error never disconnects 47010 or invalidates 47011.

## Frame association and freshness

RBVS remains latest-frame realtime video. Qt does not cache or replay old
video frames to wait for inference.

VideoView retains the Pi capture timestamp delivered by RBVS.

For the newest detection result:

```text
age_ns =
    current_video_capture_timestamp_ns
    - detection_capture_timestamp_ns
```

The detection overlay is renderable only when:

- Vision HTTP status is fresh
- inference state is Running with no active transition
- metadata stream is connected
- metadata dimensions match the current video image
- metadata timestamp is not newer than the video timestamp
- age is <= 1,500,000,000 ns
- detections is non-empty
HTTP freshness expiry suppresses the overlay immediately but does not tear down
an otherwise healthy 47012 socket.

Inference Disabled/Starting/Stopping/Retrying/Failed suppresses the overlay.

Video disconnect, host switch, unsupported fresh capability, metadata
disconnect, empty detections, dimension mismatch, future metadata, or
over-age metadata clears/suppresses the overlay.

The 1500 ms threshold is UI-only. It must never become a tracking or
control threshold.

## Rendering

VideoView maps original-frame centroid coordinates through the exact
KeepAspectRatio image target rectangle already used to draw the live image.

For each renderable detection it draws:

- one Bright Amber (`#FFB000`) crosshair centered at the mapped centroid,
  approximately 13 display pixels across with a 2-pixel stroke
- one Bright Amber filled center dot with an approximately 3-pixel radius
- Bright Amber class/confidence text offset from the centroid marker
- a one-pixel dark text shadow for readability

The painter is clipped to the actual video image rectangle.

No bounding box, background detection box, trail/history, target-selection or
target-lock indicator, tracking identity, or control/servo marker is drawn.

Label baselines and horizontal placement are clamped to the visible image.
## Lifecycle ownership

MainWindow owns presentation policy only.

`VisionControlClient` parses optional 47012 capability fields.

`DetectionClient` owns transport/decoder state.

`VideoView` owns image-coordinate mapping and display-only overlay painting.

Vision actions and detection metadata produce zero RobotController writes.

Connect Video does not start inference.
Start Inference remains explicit.
Stop Inference leaves video connected.
Detection stream failure leaves video/control usable.

## Compatibility

Old Pi + new Qt:
- no detection capability -> no 47012 connection attempt
- video/manual inference behavior remains unchanged

New Pi + old Qt:
- old Qt ignores extra 47011 fields and port 47012

New Pi + new Qt:
- a fresh frame-associated detection overlay appears on the latest live video
## Automated acceptance

Required tests include:

- detection NDJSON fragmentation/multiple records
- malformed/version/range/size rejection
- max 256 detections / 64 KiB bound
- endpoint generation fence
- strictly increasing metadata frame IDs
- old-Pi capability suppression
- unknown-version suppression
- 1500 ms freshness boundary
- HTTP-stale overlay suppression without tearing down healthy 47012
- inference-state suppression
- isolated 47012 failure
- one-attempt-per-video-session behavior
- VideoView text-and-centroid-marker render lifecycle
- zero Robot transport writes

## Native hardware acceptance

The completed real-Pi/native-Windows acceptance predates the centroid-marker
and Bright Amber closeout. It verified the text-only baseline:

- Pi 47010 / 47011 / 47012 were reachable.
- Inference was first forced to Disabled for a deterministic starting state.
- Connect Video established the real 47010 stream.
- Fresh advertised v1 capability established one real 47012 metadata connection.
- With inference Running, the native video showed
  `class confidence` text at real detections.
- That gate did not include or validate the later centroid-marker presentation.
- Stop Inference returned the Pi to Disabled and the text disappeared while the
  47010 video connection remained established.
- The 47012 transport remained isolated from video/control lifecycle.
- Automated MainWindow loopback coverage verified the overlay path emits zero
  Robot transport writes.

Final Pi inference state was left Disabled.

The final Bright Amber centroid-marker and text appearance is host-verified in
this closeout. Native visual revalidation of that appearance remains Pending.
