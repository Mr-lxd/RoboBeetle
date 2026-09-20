# Slice 4 Task 02 — Qt Manual Inference Control + Shared Pi Host

> **Status:** Frozen Task 02 implementation record. Software verification is
> recorded in the final review report; real Pi/camera/serial/UI acceptance is
> pending. This is not a new architecture proposal.

## Authority and scope

This record implements the approved `Slice4_Codex_Task02_Frozen_Pack.md` against
RoboBeetle baseline `c06dd44419c9b56dd97c6424d1b3b555f5697453`. The read-only
backend contract is fomo-visual-servo `main` at
`10e01bb4e2130c0a866f15bd6ee5ba5dd36ac20d`.

Only the Qt console host/client/UI surface is in scope. The implementation does
not modify fomo, RBRP, Pi gateway, STM32, robot authority, motion/gait, servo
logic, camera topology, model policy, or deployment files. Task 03 owns the
large-dashboard/sidebar/layout redesign and minimum-size acceptance.

## Components and ownership

### `VisionControlClient`

The existing single `QNetworkAccessManager` remains the sole HTTP transport and
the existing one-second polling timer remains the only network poller. The
client owns JSON parsing, generation fencing, request admission, mutation
queueing, typed ACK/error outcomes, freshness, and one-shot GET reconciliation.
It never knows robot authority or RBVS state.

Inference actions use the existing control endpoint and send exactly `{}` with
`Content-Type: application/json`:

| Action | Path | Accepted result |
| --- | --- | --- |
| Start / Retry | `POST /api/v1/vision/inference/start` | 202 `accepted`; 200 `already_starting`, `already_running`, or `already_retrying` |
| Stop / Clear Error | `POST /api/v1/vision/inference/stop` | 202 `accepted`; 200 `already_disabled` or `already_stopping` |

An ACK is never a worker state. Every terminal inference POST result schedules
one immediate GET on the same endpoint generation. Only that subsequent GET can
confirm state. Retry emits one Start POST; Clear Error emits one Stop POST; no
client-generated Stop→Start chain and no automatic POST retry are allowed.

`actionInFlight`, `actionBusy`, and `inferenceReconcilePending` remain distinct.
At most one request is active and at most one user mutation is queued behind a
normal GET. A second mutation is rejected. Poll ticks are skipped while busy,
and Refresh Status coalesces with the existing GET.

### Status and freshness

`VisionCaptureStatus` keeps all capture and inference diagnostics and adds typed
presence/capability metadata:

```cpp
bool haveCameraStatus{false};
bool haveCaptureStatus{false};
bool haveInferenceStatus{false};
bool haveInferenceState{false};
std::optional<bool> inferenceConfigured;
std::optional<bool> inferenceControlSupported;
bool inferenceOperationValid{false};
QString inferenceOperation;
```

Each successful GET resets presence flags before parsing. Capabilities accept
only JSON booleans; `operation` is valid only for JSON null or `stopping` /
`retrying`. Missing or malformed metadata fails closed. A legacy GET remains
readable but cannot enable inference controls. A GET without an inference object
shows `Inference Unavailable`, not confirmed Disabled. Capture POSTs update only
capture fields and never renew inference freshness or replace the last GET
inference sample. Inference ACKs bypass `applyPayload`.

The last successful GET is fresh for 3500 ms, tracked by one local single-shot
expiry timer. GET failure, endpoint change, or client shutdown immediately
invalidates freshness. Freshness is not renewed by POST. `setTransferTimeout`
remains 2000 ms for normal GET/actions and 15000 ms for Stop Recording. Manual
Refresh remains available after a stale or failed GET.

HTTP redirects are rejected with `redirect_rejected` using
`QNetworkRequest::ManualRedirectPolicy`; the direct control endpoint keeps
`QNetworkProxy::NoProxy`. Application-error JSON is parsed before transport
errors are collapsed. A sent ambiguous POST reports `outcomeUncertain=true`,
does not alter server-owned capture/inference error fields, and is never
resent automatically.

### `InferenceUiState`

The new QtCore-only helper is a pure projection:

```cpp
struct InferenceUiState {
    QString stateText;
    QString startText;
    QString stopText;
    QString reason;
    bool startEnabled{false};
    bool stopEnabled{false};
    bool showActiveMetrics{false};
};

InferenceUiState makeInferenceUiState(
    const VisionCaptureStatus &status,
    bool statusFresh,
    bool actionBusy,
    bool reconcilePending);
```

Priority is Unknown/freshness, missing inference, unknown state, read-only
legacy, invalid capability/operation, stopping, retrying, then the known worker
states. Disabled enables Start only with fresh confirmed camera-running status;
Starting and Running enable Stop; Failed offers Retry and Clear Error with the
same camera rule for Retry. Busy or reconciliation disables both actions after
computing the display text. Active FPS/latency/detection metrics are shown only
for fresh Running with a valid idle operation; null/absent values render `--`.

### Shared Pi Host and MainWindow

`MainWindow` owns exactly one top-level editable `piHost` and one committed
`committedPiHost_`. The editor is a candidate until explicit `Apply Host` or
Enter; Escape restores the committed value. A dirty candidate never silently
changes request destinations. While dirty, new Robot/Video/Refresh/Start/Retry/
Snapshot/Start Recording actions are disabled, while Stop/Clear and disconnect
actions continue to use the visibly committed host.

Remote mode has independent `robotTcpPort` (47000) and `visionPort` (47010)
spin boxes with `NoButtons`; Direct Serial keeps editable serial port and baud
widgets and has no remote robot TCP editor. HTTP has no editable port and uses
the client-injected/default 47011. Host syntax is validated without DNS and
supports IPv4, DNS/`.local`, and valid IPv6 literals through `QUrl::setHost`.

Host application rechecks all locks after any confirmation dialog, calls only
`VisionControlClient::setEndpoint`, and never auto-connects Robot/Video or stops
remote inference/recording. Host changes are blocked for active remote
transport, any non-unconnected RBVS socket, action busy, or an in-progress
transaction. `VisionClient` adds only `endpointBusy()` and
`endpointActivityChanged(bool)` from the existing socket state.

The current dashboard arrangement remains. The Video card retains the stream
and capture controls, adds Refresh Status, control reachability/message labels,
and inference Start/Stop buttons. Error attribution stays in the Vision
control message rather than converting inference failures to Capture Error.
Close defers while a sent/queued action is busy, warns when remote work or an
ambiguous outcome may continue, and after explicit confirmation shuts down local
clients/controller only; it never POSTs implicit inference/recording Stop.

## Verification boundary

Tests use fake controllers/transports, loopback `QTcpServer`/ephemeral ports,
synthetic JSON and UI data, and `QT_QPA_PLATFORM=offscreen`. They do not access
real Pi, camera, serial, robot, ONNX, or deployment infrastructure. The final
report maps Q01–Q44 to actual test functions/assertions and records the exact
Qt/CMake/compiler, build, CTest, RED, GREEN, diff, and hardware-status evidence.
