# Serial session lifecycle (Slice 4)

`robobeetle::session::SerialSession` is a Linux-only, single-owner composition
of `LinkCore` and `PosixSerialTransport`, built as `rbp2_serial_session`.
The portable Protocol, LinkCore and FrameTxQueue targets are unchanged.
The future caller supplies monotonic `TimeMs` values and fd readiness; this
class does not implement a poll loop, threads, sleeps, reconnect/backoff,
LinkRuntime, command replay or application integration.

## Lifecycle and scheduler contract

Construction is `ReopenRequired`, with no fd and no scheduled retry.
`retry_open(path, now)` is the only open operation and returns zero or errno.
The first attempt anchors `safe_after = now + 575 ms`; addition saturates at
the maximum `TimeMs`. An unsuccessful open remains closed and must be retried
explicitly. Subsequent attempts retain this deadline. After a loss, retries
retain the new loss deadline. An already owned fd returns `EBUSY` unchanged.

A successful open immediately disables transport admission and enters
`SafetyQuiet`. It can own its fd early, but no raw byte, heartbeat, ordinary
request or LinkCore poll is allowed before `safe_after`. At exactly 575 ms,
`tick` discards stale kernel input with `TCIFLUSH`, then queues one standalone
raw `0x00` using the existing transport/FrameTxQueue path. It is not a Codec
frame and has no sequence, CRC, ACK or Heartbeat semantics. Only this private
enqueue briefly opens the admission gate, and it closes again before returning.

The state is now `Resynchronizing`. A writable service call pumps the front
queue through the existing transport; zero writes, backpressure, and EINTR
follow that transport's semantics. LinkCore remains stopped and its poll is
not called while the raw byte is queued. Once that byte leaves the software
queue, `LinkCore.start(now)` resets logical session state without resetting
sequence allocation. Normal transport admission opens and the state becomes
`Online`. The initial heartbeat is due at that timestamp and is dispatched by
the next `tick`. Software queue drain establishes ordering into the kernel;
it does not claim UART last-bit, device receipt, hardware or water validation.

`Online` means the transport session is ready; ordinary submission additionally
requires LinkCore `Active`. `submit_request` rejects quiet, resynchronizing and
closed sessions before calling LinkCore. An Online Unconfirmed/Degraded link
still applies LinkCore's existing Active gate and ACK timeout rules. ACK
timeouts remain anchored at complete transport queue acceptance.

`service_readable` makes one `read_some` call into a 512-byte buffer for each
readable notification. Each Online Progress result is forwarded once,
unmodified, to `LinkCore.receive`. Input serviced during quiet/resynchronization
is discarded. Every result ends the round, including a full 512-byte Progress;
remaining input requires a fresh readiness notification from the caller. A
full buffer cannot establish readiness for a second read. This avoids probing
a drained `VMIN=0` descriptor: the frozen transport contract still treats every
actual `read()==0` as Fatal, including a no-data zero. `service_writable` makes
one front-frame pump attempt; the transport retries EINTR internally, retains
partial remainders and returns on backpressure. Neither service blocks for
readiness or uses `tcdrain`.

`next_wakeup_ms()` returns only the 575-ms deadline while quiet, LinkCore's
earliest heartbeat/ACK/liveness deadline while Online, and no timer while
resynchronizing or closed. `wants_write()` identifies pending resync/Online
software TX. These seams permit a future scheduler without adding one here.
The caller must service a returned due timer promptly. Backward timestamps
are defensively clamped to the largest timestamp already observed.

## One teardown for every loss

Fatal read, fatal write, LinkCore Lost detected by `tick`, and explicit `abort`
all run the same ordered teardown:

1. Disable transport admission.
2. Call `LinkCore.abort_session(now)` and retain its completion events.
3. Drop every pending software TX remainder and successor.
4. Call `tcflush(owned_fd, TCOFLUSH)` to discard kernel TX before close.
5. Close the owned fd.
6. Enter `ReopenRequired` and anchor a new saturating 575-ms deadline.

A pending ordinary request becomes OutcomeUnknown and queued unsent requests
become Cancelled. No old command is stored for replay. Liveness Lost may have
already emitted these outcomes from LinkCore; its idempotent abort does not
duplicate them. Repeated session abort/closed service calls do not repeat the
teardown, emit duplicate outcomes or move the loss deadline.

Every flush failure, including EINTR, is reported through `last_error()`; a
failed TCOFLUSH takes precedence over the initiating errno. The session still
closes and remains reopen-required, with no automatic retry or LinkCore start.
An explicit successful retry clears the error and must complete quiet/resync
again. Failure to discard initial RX or queue the raw delimiter also uses the
same teardown. Destruction uses this lifecycle too; explicitly call `abort`
first when completion outcomes must be collected by the owner.

Logical abort constructs completion events and can fail to allocate. Its
exception propagates from explicit service/abort calls only after a private
non-allocating cleanup tail drops software TX, attempts TCOFLUSH before close,
closes the fd and records `ReopenRequired` with the new safety deadline.
Allocation failure records `ENOMEM` (other logical exceptions record `EIO`);
a flush failure still takes precedence. Completion outcomes cannot be promised
when their allocation fails. The destructor is explicitly `noexcept` and
contains logical-abort exceptions after this same physical cleanup. Private
trace callbacks cannot interrupt cleanup, even if a callback itself throws.

Only Linux-specific session code calls `tcflush`. It is never used during
ordinary Online traffic. Private syscall/ordering seams let tests inspect
the actual component states at teardown boundaries; no POSIX/test API is
added to LinkCore, Transport or Protocol.

## Verification

The Linux test executable covers the frozen A-U matrix: exact 574/575 quiet
boundary, stale RX discard, raw delimiter backpressure/order, ordinary Active
gate, raw receive forwarding, exact-full 512-byte bursts without speculative
second reads, fatal/EOF/liveness unified teardown, partial TX
discard, Unknown/Cancelled outcomes and no replay, early/late reopen, open and
flush failures, idempotence, saturated/monotonic time and earliest wakeups.
Exception-path tests inject abort allocation failure and throwing trace
callbacks, checking queue disposal, flush-before-close, safe state/deadline,
explicit exception propagation and destructor containment.
It uses real LinkCore/FrameTxQueue/transport with private deterministic syscall
seams, plus Linux PTYs and `/proc/self/fd` leak/ownership checks.

```sh
cmake -S RoboBeetlePi -B RoboBeetlePi/build-linux -DCMAKE_CXX_FLAGS="-Wall -Wextra -Werror"
cmake --build RoboBeetlePi/build-linux
ctest --test-dir RoboBeetlePi/build-linux --output-on-failure
```

Linux full repository CMake/CTest was unavailable on the implementation's
Windows host (no installed WSL distribution or Docker). The new Linux source
and behavioral tests were not compiled or executed there. The missing-feature
RED executable and fresh Windows portable CMake/CTest regression were run;
that portable result does not establish Slice 4 Linux GREEN or PTY evidence.
Hardware/serial waveform/physical/water validation remains PENDING.
