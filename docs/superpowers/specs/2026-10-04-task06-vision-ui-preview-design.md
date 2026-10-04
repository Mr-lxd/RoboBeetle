# Task06 PR-A Vision UI preview

User-supplied scope: presentation-only preview, except the explicitly requested session direction-source change. Base main a5a235421ecfe3fe6a7560212e164064f90a61a4. No depth/Task06 PR-B, pitch/PR-C, firmware or gateway changes. Draft PR only; operator and Claude review plus physical trial precede any merge.

## Presentation

A right-side Auto follow card contains the session-only Visual dispatch checkbox, state, three individually evaluated prerequisite rows, one >=52px Arm/Disarm button, confirmed-mode/operator-STOP detail and Turn +1 (config). Remove the four old dispatch/sign/control/status rows from Vision Details. Vision Controls are 2x2 with >=44px buttons. Vision status shows three rows: Video / Inference / Recording, retaining detailed measurements via tooltips/details.

State precedence: FAULT (latched STOP timeout or pose mismatch), ARMED, DRY RUN (gate OFF/direct), READY (three requirements met), NOT READY. Each prerequisite is independently computed: remote link and authority, required enabled+known pose mask, TRACKING. Arm requires gate ON, all prerequisites and no fault; Disarm stays available when armed. NOT READY counts missing conditions. Green frame and upper-left AUTO / ARMED / ACK-confirmed command badge belong to VideoView presentation only. Red fault banner at card top. Text distinguishes states as well as shade.

## Direction exception and safety boundary

VisualPolicyConfig defines +1 once as the configured default; sessions take it directly as confirmed direction. Remove operator direction-selection/confirmation UI. CSV continues recording the same config-derived direction/provenance. Keep pure-policy +/-1 mapping tests and safety admission tests. Do not alter VisualDispatchStateMachine, RemoteRobotController, dwell, manual takeover, feature-off STOP rules, event-driven STALE/INFERENCE_OFF/LOST or outcome correlation.

## Verification/delivery

RED then GREEN for config startup, UI readiness matrix, geometry, banner and overlay. Preserve existing safety assertions; only direction-workflow assertions change. Full Qt regression passes. operator_console_preview supplies five auto-follow states at1420x880 and1100x720 (10 images), simulation only. Visually inspect exports, package Release EXE with windeployqt, record exact source and SHA256. Output D:/RoboBeetle-results/task06-ui-preview-20261004/. Push draft PR, do not merge.
