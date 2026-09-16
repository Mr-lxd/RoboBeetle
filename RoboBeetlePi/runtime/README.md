# LinkRuntime (Implementation Slice 5)

Linux-only, single-thread owner above `SerialSession`. The future application
owns repeated calls and explicit reopen decisions. There is no application loop,
thread, actuator integration, or automatic reconnect here. Runtime does not copy
the session's quiet, resync, liveness, queue, or teardown policies.

`open(device_path)`, `submit_request(type, payload)`, and `abort()` sample
`std::chrono::steady_clock` monotonic milliseconds and delegate to the session.
There is no mutable session accessor. The fd accessor is borrowed diagnostics;
do not read/write/close it or change its flags externally.

`run_once()` performs this bounded ordering:

1. Sample time and call session `tick`. Closed/ReopenRequired returns NeedsOpen
   with any tick events, without polling an invalid fd.
2. Reacquire the fd and monitor POLLIN. Add POLLOUT **only** when `wants_write()`
   is true. Sample fresh monotonic time immediately before obtaining/converting
   the session deadline: due/past = 0, future delta clamped to INT_MAX, no
   deadline = -1 (only with a valid descriptor). Pre-tick work or scheduling
   crossing a deadline therefore cannot produce a stale positive poll timeout.
3. Call `poll` exactly once. EINTR returns Interrupted without retry or teardown.
   Other errno aborts through the session and returns PollFatal with original errno.
4. Sample time after poll. POLLNVAL/ERR/HUP aborts through the session before
   any I/O. Otherwise service POLLIN once, then POLLOUT once if registered,
   still wanted, and the session/fd remains open. No speculative RX draining.
5. Sample fresh time after readiness work and tick again, including after a
   timeout. This prevents continuously readable input from starving timers.

Events are returned once in pre-tick, readable, writable, post-tick order.
Fatal readiness and poll errors append session abort events in their place.
Allocation/programming exceptions may propagate; timeout/EINTR/NeedsOpen do not
use exceptions. All physical cleanup remains owned by `SerialSession`.

Status precedence is deterministic: pre-poll closed (including loss during
pre-tick) = NeedsOpen; poll EINTR = Interrupted; other poll syscall errors =
PollFatal; fatal readiness or loss during services/post-tick = SessionLost;
otherwise poll zero = Timeout and poll readiness = Progress. Timeout may include
timer events and queue new bytes; Progress need not include a LinkEvent. EINTR
retains effects/events of the required pre-poll tick but adds no post-poll tick.
NeedsOpen/SessionLost from session operations uses the session's last error.
POLLNVAL reports EBADF, ERR/HUP reports EIO (NVAL wins combined flags); a poll
syscall error retains its original errno even if teardown flush also fails.

Linux CMake creates `rbp2_link_runtime` and `rbp2_link_runtime_tests`; portable
targets never include the runtime or Linux headers. Tests A-W use private clock
and poll seams and the real session/transport, with transport syscall observation
through existing private friends. W uses real poll and PTY I/O with a controlled
clock to verify 574ms silence, raw zero at 575ms, then first heartbeat without a
real quiet-duration wait. No Raspberry Pi/UART/physical/water result is implied.

Linux verification: configure this directory with `-DBUILD_TESTING=ON`, build,
then run CTest. Windows portable results do not establish Linux runtime PASS.
