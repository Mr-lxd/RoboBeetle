# Visual approach / stop (Task20)

`Exec` defaults to `Discrete` each launch. Select `Proportional` while disarmed and with no unconfirmed STOP, enable Visual dispatch, and Arm explicitly. `Dry run` follows the same area/state logic without calling controller send APIs; it bypasses hardware eligibility, while Pitch/Both still require valid calibrated depth.

The existing visual command policy selects Forward, Turn Left/Right, or Ascend/Descend. The proportional executor maps those commands to throttle/turn/pitch, refreshes the latest input at each evaluation, and waits for Start ACK before streaming. No target retains the preceding setpoint for the existing 1.5 s grace. LOST stops while keeping Arm; STALE, inference off, manual input, rejected/unknown Start, or firmware setpoint timeout stop and disarm. Depth safety checks both pending and current pitch and bypasses ordinary switching dwell.

Only the selected target's area is filtered, once per new frame. Video catch-up preserves dwell and missing-area timing. Continuous missing area over 500 ms becomes LOST. ARRIVED stops and remains armed, including eccentric targets. Initial Arm/reacquisition with area at least A_stop enters ARRIVED immediately without Start. Resume requires area below A_resume for its dwell, an accepted STOP at least 1000 ms old, and actual STOPPED.

Visual proportional configuration is independent of gamepad settings and saved under `Visual/Approach/v1`. All defaults are provisional:

| Parameter | Default |
|---|---:|
| alpha_area | 0.30 |
| A_slow / A_resume / A_stop | 8 / 14 / 20 cells |
| arrive / resume dwell | 1000 / 1000 ms |
| t_max / t_min / t_turn_min | 0.60 / 0.15 / 0.30 |
| k_turn / k_pitch | 1.0 / 0.50 |
| max_scale / turn_gain | 1.0 / 1.0 |
| pitch_limit / slew_per_second | 10 degrees / 2.0 |

Throttle decreases linearly between A_slow and A_stop; turns retain t_turn_min so differential turning still has propulsion. Turn Left maps to negative turn, which reduces the left side under the existing firmware formula. Ascend maps to positive pitch, which makes the front-axis offset negative. Both retains horizontal priority and a single nonzero steering axis.

Visual ownership prevents the gamepad mapper from treating a visual session as its own. Manual takeover stops visual execution and waits for STOPPED and fresh centered gamepad input. No arbiter or token framework is added.

The logger always writes `visual-csv-v7`, preserving the first 48 columns and appending 9 columns: `exec_mode,dry_run,area_f,approach_state,throttle,turn,pitch_setpoint,actually_submitted,approach_event`. Discrete extensions are blank. Values join existing frame/dispatch records; no periodic approach rows are added. Sparse events are ARRIVED_ENTER/EXIT, AREA_MISSING_LOST, SETPOINT_TIMEOUT, START_FAILED, STREAM_REJECTED, and CONFIG. Setpoints use protocol units (0..1000 or -1000..1000); IMU `pitch` stays separate. Submission status describes a successful input/request submission, not proof of physical motion. Existing v6 recordings and Task17 named-column reads remain valid.

## Verification and boundaries

New tests cover pure mapping/area transitions, continuous streaming and safety, Dry run with zero controller calls, ordinary correlated Remote Start/STOP, unified CSV, and UI ownership/selector behavior. Existing behavioral tests are retained; only CSV schema/version/count assertions change in visual_csv_logger_tests.cpp, main_window_tests.cpp and main_window_visual_dispatch_tests.cpp. The two additional files were explicitly authorized after the first regression identified their old format checks. Release build succeeded; the full run passed37/40, then all3 format-only failures passed after the approved assertion update. All40 targets are verified; only failed targets were repeated. Task17 compatibility used74 synthetic sampled frames with identical v6/v7 depth joins and does not claim physical synchronization.

Fake-controller native previews below expand Operator tools at 1280×992; the manifest records zero robot-control writes. No hardware connection, firmware change, Pi deployment, burn, or robot motion is part of this implementation. Compatibility targets the existing Task19 firmware/gateway `8f1aedb` protocol.

## Operator acceptance (pending)

1. On a dry desktop with unloaded servos, select Proportional + Dry run. Move a real toy forward/back (not a screen recording) and inspect area_f, throttle, and APPROACH/ARRIVED in CSV.
2. Turn Dry run off and repeat; verify decreasing servo amplitude, arrival stop, and restart after moving the toy away.
3. Trigger target occlusion, inference off, manual keys, gamepad input, depth boundaries, and network loss individually.

Area is a coarse proximity cue, not distance. A large/truncated target may have area near 1 cell, causing continued t_max propulsion. Keep operator STOP ready and observe that case explicitly. No additional approach timeout is included. Default t_max=0.60 also limits same-side turn amplitude to 0.6 of discrete full-throttle turning, so response is intentionally weaker.

## Previews

![Discrete](Task20-Discrete.png)
![Approach](Task20-Approach.png)
![Arrived](Task20-Arrived.png)
![Dry run](Task20-Dry-run.png)
