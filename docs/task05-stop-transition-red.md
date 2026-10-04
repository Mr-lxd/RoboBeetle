# Task 05 — STOP transition characterization and disconnect investigation

2026-10-04; PR #45; baseline `99515ce563ee143c2a2a0224352ee0a9b6321a5a`.
Only tests and documentation change. No automatic sender, arming, motion
connection, firmware programming or hardware operation is introduced.
The existing `ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md` holds the Task 01–05
handoff summary; no `docs/HANDOFF.md` is created.

## Reviewed STOP semantics

The operator review selects **acceptance immediately preempts the original
transition, followed by a smooth return to zero with a 750 ms logical duration of elapsed
stop-ramp time from acceptance**. Completion before the original mode-transition
end is not required. The initial commit `42e55ec3e397e4c6e39ba7b13cf1162b60095815`
recorded two RED assertions for that stricter interpretation; those assertions
are superseded by characterization tests, with no production fix needed.

Production anchors (unchanged in this PR):

| Location | Finding |
| --- | --- |
| `RoboBeetleFirmware/Core/Motion/motion_config.h:5` | 750 ms transition / stop ramp; tick cadence is 10 ms at line 4. |
| `RoboBeetleFirmware/Core/Communication/protocol_dispatcher.c:411` | START calls `motion_manager_start`; STOP calls `motion_manager_request_stop_at(manager, now_ms)` separately. |
| `RoboBeetleFirmware/Core/Motion/motion_manager.c:755` | Different non-STOP mode during a transition returns BUSY. |
| `RoboBeetleFirmware/Core/Motion/motion_manager.c:857` | STOP is idempotent OK in STOPPED/FAULTED/STOPPING; running transitions do not reject STOP as Busy. |
| `RoboBeetleFirmware/Core/Motion/motion_manager.c:876` | Captures last targets, resets stop elapsed and clock at acceptance, enters STOPPING and returns OK without servo writes. |
| `RoboBeetleFirmware/Core/Motion/motion_manager.c:409` | STOPPING interpolates to zero and returns before gait advance. |
| `RoboBeetleFirmware/Core/Motion/motion_manager.c:430` | At stop elapsed >=750 ms, enter STOPPED / MOTION_STOP and release ownership. |
| `RoboBeetleFirmware/Core/Inc/rb_protocol_v2.h:55` | Wire Busy is 7. |

Thus normal STOP immediately ACKs OK and preempts, rather than returning Busy=7.
ACK establishes acceptance, not physical stopping. Repeated STOP cannot restart
or extend the return-to-zero ramp. The 750 ms is the logical ramp duration;
completion is applied on an eligible `process()` call. The 10 ms cadence and
foreground scheduling must be included in any actual wall-clock budget; a
stalled main loop cannot provide a hard real-time or physical stopping guarantee.

## Characterization tests and verification

The startup and mode-transition tests in
`RoboBeetleFirmware/tests/protocol_dispatcher_tests.c` send real typed Protocol
V2 frames through the production dispatcher and MotionManager; only hardware
servo operations are faked. They check:

- STOP injected during START or MODE transition ACKs OK and immediately enters
  STOPPING; ACK performs no servo writes.
- Sample at acceptance +100, +300, +500, +700, +739, +749 and +759 ms; refresh
  heartbeat at every observation and require `process()` OK.
- A fresh-sequence STOP at +300 ms ACKs OK and remains nonblocking. Completion
  remains bounded by the original acceptance +750 ms plus one tick, not repeated acceptance +750 ms.
- All five logical joints' absolute distance to zero is monotonically
  nonincreasing at the samples; all equal zero at completion.
- +749 ms remains STOPPING on a real eligible tick (verified by last_tick_ms and stop elapsed); +759 ms is STOPPED / MOTION_STOP / ownership released.
  Samples +739 -> +749 -> +759 each execute eligible ticks. The completion
  upper bound is `MOTION_TRANSITION_DURATION_MS + MOTION_GAIT_TICK_MS`
  (750 + 10 = 760 ms), assuming the foreground is serviced every tick.

```powershell
./RoboBeetleFirmware/tests/run_host_tests.ps1 -BuildRoot C:/Users/laixindong/.codex/worktrees/task05-stop-transition-red-build/cadence
```

Full Firmware host gate: **PASS**, 37 executables + 13 app/backend/benchmark/
diagnostics compile-contract objects + USART2 source/config contract. The
compiler uses `-std=c11 -Wall -Wextra -Werror`. The reviewed tests and existing
MotionManager suite pass. Log:
`C:/Users/laixindong/.codex/worktrees/task05-stop-transition-cadence.log`.
Original baseline/RED logs remain preserved outside the repo for provenance.
This is host characterization, not hardware or water validation.

## What Linux abort actually sends

**No STOP frame is sent by `LinuxOnboardApplicationPort::abort()`.**

| Location | Actual action |
| --- | --- |
| `RoboBeetlePi/gateway/linux/src/linux_onboard_application_port.cpp:304` | Delegates to `application_.abort()` and maps events. |
| `RoboBeetlePi/application/src/onboard_application.cpp:71` | Delegates to `runtime_.abort()`. |
| `RoboBeetlePi/runtime/src/link_runtime.cpp:56` | Delegates to `session_.abort(clock_fn_())`. |
| `RoboBeetlePi/session/src/serial_session.cpp:131` | Abort enters teardown. Close TX admission, abort LinkCore, drop pending TX, `tcflush(TCOFLUSH)`, close fd, enter ReopenRequired. |
| `RoboBeetlePi/link_core/src/link_core.cpp:123` | `abort_session()` enters Lost, not a motion command submission. |
| `RoboBeetlePi/session/src/serial_session.cpp:13` | 575 ms SafetyQuiet is a reopening guard, not a STOP ACK or measured stopping time. |

This stops future serial heartbeats. Bytes already on the wire, in the STM32
receive queue or accepted before teardown can still update the final heartbeat
time. Pending normal STOP cannot be assumed delivered after abort drops TX.

## Qt disconnect path and timing budget

Current active-authority path:

1. TCP EOF/fatal read → `TcpAdapter::close_current(...Disconnected)`
   (`RoboBeetlePi/gateway/linux/src/tcp_adapter.cpp:418`).
2. Source loss is queued to the owner; its next bridge snapshot handles losses
   first (`RoboBeetlePi/gateway/linux/src/gateway_owner.cpp:186`).
3. `ControlGatewayCore::source_lost()` → `revoke()` → `abort_once()`
   (`control_gateway_core.cpp:649`, `:798`, `:786`): revoke authority and abort
   active serial session. No ordinary STOP frame is inserted.
4. Heartbeats cease. STM32 `safety_supervisor_process()` detects
   `now-last_heartbeat >500 ms` (`Core/Safety/safety_supervisor.c:35`, timeout
   constant `safety_supervisor.h:7`): first integer-ms opportunity is +501 ms.
5. `app_main.c:719` calls `app_main_apply_safety_stop()` (`:105`), invoking
   `motion_manager_stop_immediate()` and `servo_service_disable_all()`.
   This is an immediate safety abort, not the ordinary 750 ms graceful ramp.
   Active PWM may finish at its falling edge (`Core/Servo/servo_driver_stm32.c:305`);
   logical shutdown does not establish when a physical joint stops moving.

Budget from Qt connection loss to logical safety shutdown:

`D_tcp_detection + D_owner_dispatch + D_residual_heartbeat + 501 ms + D_firmware_loop`.
Add PWM falling-edge / IRQ latency for electrical output cessation, and measured
mechanical time for a physical stop. No finite TCP detection bound follows from
silent network loss alone. If TCP stays connected but control heartbeats stop,
lease expiry revokes authority after 1000 ms from the last accepted control
heartbeat (`control_gateway_core.cpp:12`, `:854`), then uses the same abort path.

With the default serial heartbeat interval 100 ms
(`link_core/include/robobeetle/link_core/link_config.hpp:9`), an online owner
normally returns from serial poll by the next LinkCore deadline and processes
source losses before/after `run_once()` (`gateway_owner.cpp:249`). This gives a
nominal owner dispatch allowance of up to about 100 ms plus processing/OS
scheduling, not a hard upper bound: runtime polls to its dynamic deadline
(`runtime/src/link_runtime.cpp:31`), and custom config/load can change the delay.
After TCP loss notification, nominal budget is therefore about **601 ms plus
residual-heartbeat and scheduling delays**. For silent loss caught by the lease,
use **1000 + 100 + 501 = 1601 ms** from last accepted control heartbeat, plus the
same residual/scheduling terms. These are code-derived budgets, not measurements.
The 575 ms reopening guard neither adds to stopping time nor proves the machine
has physically stopped.

Consequently the existing disconnect fail-safe cannot be described as
"Qt disconnect immediately transmits STOP". A later motion-sending PR must
explicitly test default-OFF arming, immediate disarm on Qt/gateway loss, STOP
attempt while transport is usable, and the watchdog fallback when it is not.

## Artifact provenance

Reviewed host test EXE (does not control a robot):
`C:/Users/laixindong/.codex/worktrees/task05-stop-transition-red-build/cadence/protocol_dispatcher_tests.exe`.
Test source commit: `8a3a0d90248792855022b047234f2a45cdaafb35`.
Fresh test EXE SHA-256:
`583C7A30E3E80B44622850542DE417FACDDA4E8AD49AD3C09905444498FD506B`.
Subsequent provenance edits are documentation-only; executable build inputs
remain unchanged. The final delivery head is recorded in the PR description.
No new Qt EXE is built. The retained Task 04 DRY_RUN EXE
is `D:/RoboBeetleConsole-portable-target-temporal-association-dry-run-20261003/RoboBeetleConsole.exe`;
original source `ec5841f30ace8eadc551b5893a849087c553aa67`, SHA-256
`C6A525E920FC1DB7624F60967FF161AAC50C07C4678816A13C73A1405F6E1C78`.
This is retained artifact provenance, not a motion-enabled Task 05 build.

## Later motion PR requirements (pending)

| Requirement | Coverage required |
| --- | --- |
| Arming defaults OFF | Startup/reconnect disarmed; Qt and gateway loss each disarm and request STOP with transport-loss fallback. |
| Manual takeover | Every manual input immediately takes authority and disarms automatic mode, including dwell/in-flight cases. |
| STALE / LOST / INFERENCE_OFF | Each immediately requests STOP in the same evaluation, bypassing dwell and transitions. |
| Non-STOP dwell | At least 1000 ms between sends; 999/1000 boundary; Busy=7 causes no immediate retry; STOP bypasses dwell. |
| `turn_sign` | No default; unset/invalid/unconfirmed direction refuses arming; both explicitly confirmed signs tested. |
| STOP semantics | Reviewed acceptance +750 ms ramp contract covered here; ACK never labelled physical completion. |
| Desktop hardware validation | Servos unloaded and out of water only; physical direction still unconfirmed. |

No automatic movement or arming is delivered by PR #45.
