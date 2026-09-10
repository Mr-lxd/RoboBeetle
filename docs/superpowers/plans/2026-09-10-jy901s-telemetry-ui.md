# JY901S telemetry and Qt monitor implementation plan

## Checkpoint 0 — audit and contract

- Confirm the actual Protocol V2 IDs, 64-byte payload cap, CRC/COBS layout,
  independent telemetry sequence, APC220 Heartbeat/ACK cadence, LeakStatus
  priority, and existing full Firmware regression list.
- Add the approved design/spec and this plan. Keep `.ioc` and generated
  CubeMX CMake unchanged because PR #10 already supplies USART3 hardware
  bring-up and PR #11 uses the existing USART1 telemetry path.

## Checkpoint 1 — Firmware pure-C red/green

- Add failing tests for fixed payload layout, schema/flags, little-endian
  signed fixed-point rounding and clamping, invalid-domain zeroing, counters,
  and exact Protocol V2 wire-vector size.
- Add the pure-C JY901S telemetry encoder and make the tests green.
- Add failing scheduler/policy tests for the one-second interval, wrap-safe
  timing, LeakStatus priority, one optional slot, and IMU recovery after a
  LeakStatus opportunity; implement the minimal pure-C policy/selector.

## Checkpoint 2 — Firmware app integration

- Add `ImuSnapshot` as Protocol V2 `0x21` at both Firmware/Console enum
  boundaries without changing base framing.
- Integrate snapshot construction into `app_main.c` only after successful
  Heartbeat ACK and through the pure-C selector. Preserve the existing Leak
  policy mark-after-success behavior and telemetry/command sequence isolation.
- Run the complete existing Firmware host regression set plus the new codec and
  scheduler tests, then run the app/API and HAL/generated-style syntax checks.

## Checkpoint 3 — Console pure Qt red/green

- Add failing tests for payload encode/decode, schema/flag validation,
  signed/endian/range behavior, partial validity, malformed payloads, and the
  0x21 Protocol V2 golden vector.
- Add `ImuSnapshot` value/codec code and make those tests green.
- Add failing `ImuMonitor` tests for Unknown/Receiving/Stale/Error,
  disconnect/liveness reset, invalidation, recovery, and ACK/Leak isolation;
  implement the monitor with a 3500 ms stale threshold.

## Checkpoint 4 — Console integration

- Feed `ImuSnapshot` in `RobotController` without touching pending ACK state or
  scheduler queues; reset it on transport disconnect/error and APC220 liveness
  loss, and tick stale handling from the existing retry timer.
- Add only the `IMU — JY901S` status/value/diagnostic panel. Verify invalid and
  stale values render as `--` and no 3D/history/control UI appears.
- Update the Console CMake source/test registration and run all Console CTest
  tests, including the unchanged servo/USART1/APC220 regressions.

## Checkpoint 5 — documentation and gates

- Synchronize Firmware README, Console README, canonical Protocol V2 docs,
  hardware handoff, and engineering lessons with the exact payload, timing,
  bandwidth estimate, evidence labels, and non-scope.
- Run all current Firmware regressions (not only the historical pure-C subset),
  new Firmware tests, all Console tests, app/HAL syntax checks, and an ARM
  target build if the toolchain is available. Do not program hardware in this
  phase; Program Verify and Hardware Verified remain Pending for PR #11.
- Perform an independent code review. Fix actionable findings and rerun the
  affected gates before pushing `feature/jy901s-telemetry-ui` and opening a
  stacked PR to `feature/jy901s-bringup`. Do not merge.
