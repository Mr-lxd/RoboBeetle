# Linux serial transport (Slice 3)

`robobeetle::transport::PosixSerialTransport` is built in the Linux-only
`rbp2_posix_serial_transport` target. The portable `rbp2_transport` target and
its `Transport`/`FrameTxQueue` headers have no Linux dependencies. This class
owns one descriptor and one bounded `FrameTxQueue`; it is intended for a single
thread/event loop and is neither copyable nor movable.

`open(path)` returns zero on success or an errno value on failure. It opens with
`O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC` and configures raw 115200 8N1,
`CLOCAL | CREAD`, no parity or hardware/software flow control, and
`VMIN=0, VTIME=0`. Configuration failure closes the candidate descriptor and
leaves the transport closed. An already open descriptor or retained queue
causes `EBUSY` without changes. `native_fd()` is borrowed for readiness polling;
callers must not close it or change its flags.

`write(frame)` accepts an entire frame into owned queue storage or returns false
without changing the queue. Acceptance requires an open, nonfailed descriptor,
an enabled admission gate, and both queue capacity limits. Acceptance is not
physical completion, `tcdrain`, or UART last-bit evidence. Frame copying may
throw an allocation exception; constructors and `write` are therefore not
`noexcept`.

`pump_tx()` attempts only the current front remainder, returning after one
positive write. It consumes exactly the reported count, so a later frame cannot
overtake a partial frame. `EINTR` retries the same bytes; `EAGAIN`/`EWOULDBLOCK`
returns `WouldBlock` without consumption. A zero write also returns `WouldBlock`
without spinning. An accepted empty frame is retired as `Progress` with zero
bytes and no syscall. An empty queue returns `Idle` on an open healthy fd.

`read_some(buffer, capacity)` returns raw bytes without decoding. Zero capacity
returns `Idle` without reading. `EINTR` retries; `EAGAIN`/`EWOULDBLOCK` is
`WouldBlock`. Per the frozen Slice 3 contract, every `read()==0` is `Fatal` with
error number zero, **including a no-data zero return possible with `VMIN=0`**.
Callers should arrange readiness before reading. Other fatal results report
errno. Null storage with nonzero capacity reports `Fatal/EINVAL`.

`IoResult` contains `Idle`, `Progress`, `WouldBlock`, or `Fatal`, the actual byte
count for this call, and a fatal error number. Fatal RX or TX closes admission
before returning and latches failure across subsequent I/O calls. No pending
bytes are silently discarded, no completion is fabricated, and toggling the
gate cannot revive the failed descriptor. `set_accept_new_tx(false)` on a
healthy descriptor only blocks new admission; already queued frames may drain.

`drop_pending_tx()` explicitly discards the partial remainder and queued frames.
`close()` releases the descriptor once and disables admission, retaining the
queue for an explicit drop; it never drains, flushes, sleeps, or reconnects.
Before a new fd session, close and drop retained TX. The destructor closes the
fd and destroys the owned queue. Linux `close` is not retried after `EINTR` to
avoid closing an unrelated reused descriptor.

The private read/write syscall seam is used only by deterministic transport
tests. It does not appear in `Transport` or `LinkCore`. The Linux test target
covers frame admission/capacity, partial ordering, backpressure, interruption,
fatal gates, explicit drop, raw RX, real PTY flags/termios, and fd ownership and
failure cleanup. It uses `/proc/self/fd` for leak checks. Run on Linux:

```sh
cmake -S RoboBeetlePi -B RoboBeetlePi/build-linux -DCMAKE_CXX_FLAGS="-Wall -Wextra -Werror"
cmake --build RoboBeetlePi/build-linux
ctest --test-dir RoboBeetlePi/build-linux --output-on-failure
```

This slice contains no session orchestration, LinkCore calls, quiet interval,
resynchronization bytes, protocol semantics, CRC/ACK/heartbeat handling, or
hardware validation. PTY evidence is host software evidence only.
