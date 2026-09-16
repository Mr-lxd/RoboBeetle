# Portable Raspberry Pi LinkCore Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a deterministic, platform-neutral LinkCore that owns Protocol V2 Heartbeats, one ordinary in-flight request, bounded ordinary FIFO work, ACK correlation, timeout outcomes, and basic liveness state without adding POSIX, GUI, ROS2, or firmware dependencies.

**Architecture:** LinkCore consumes the existing `robobeetle::protocol::Frame`, `Codec`, and `StreamDecoder` APIs. A minimal transport interface accepts encoded wire bytes, while callers inject monotonic millisecond timestamps through `poll(now_ms)` and `receive(bytes, now_ms)`; tests provide in-memory FakeTransport and FakeClock. Heartbeats use an independent lane and sequence history, while ordinary ACKed commands use one in-flight slot plus a bounded FIFO. This slice records local liveness and terminal outcomes; POSIX session management, safety-quiescence reopen, and application adapters remain future work.

**Tech Stack:** Portable C++17, CMake/CTest, existing Protocol V2 COBS/CRC codec, no Qt/POSIX/ROS2/STM32 HAL.

---

### Task 1: Add portable LinkCore contract tests (RED)

**Files:**
- Modify: `RoboBeetlePi/CMakeLists.txt`
- Create: `RoboBeetlePi/tests/fake_clock.hpp`
- Create: `RoboBeetlePi/tests/fake_transport.hpp`
- Create: `RoboBeetlePi/tests/link_core_tests.cpp`

- [x] **Step 1: Define the wished-for test seam**

Add tests that construct `FakeTransport`, `LinkCoreConfig`, and `LinkCore`, call `start()`/`poll()`/`receive()`, and inspect returned events and captured encoded frames. Cover:

```text
heartbeat interval boundary, next deadline, sequence increment and uint16 wrap;
matching/wrong-sequence/wrong-type/malformed/non-OK ACK;
ordinary timeout exactly at deadline, exactly once, and stale ACK suppression;
fragmented ACK, ACK plus telemetry, malformed-frame recovery, unknown frame plus ACK;
initial/active/degraded/lost state and recovery after a later healthy heartbeat;
one ordinary in-flight plus bounded FIFO, with Heartbeat continuing independently.
```

Use a FakeClock that only advances an integer `std::uint64_t now_ms`; no sleeps or wall clock.

- [x] **Step 2: Run the RED build**

Run:

```powershell
cmake -S RoboBeetlePi -B <red-build-dir> -DBUILD_TESTING=ON
cmake --build <red-build-dir> --config Debug
```

Expected result: compilation fails because the new LinkCore/transport headers and implementation target do not yet exist. Record the missing-symbol/header failure; do not commit the RED state.

Observed RED failure: `fake_clock.hpp` could not find `robobeetle/link_core/link_core.hpp` before the production LinkCore headers existed. The RED state was not committed.

### Task 2: Add the platform-neutral transport/time/event contracts

**Files:**
- Create: `RoboBeetlePi/transport/include/robobeetle/transport/transport.hpp`
- Create: `RoboBeetlePi/link_core/include/robobeetle/link_core/link_events.hpp`
- Create: `RoboBeetlePi/link_core/include/robobeetle/link_core/link_config.hpp`
- Create: `RoboBeetlePi/link_core/include/robobeetle/link_core/link_core.hpp`

- [x] **Step 1: Define the minimal transport boundary**

Expose only `virtual bool write(const protocol::Bytes &wire_bytes) = 0`; do not include termios, poll, Qt, ROS2, or HAL.

- [x] **Step 2: Define deterministic public data**

Use `std::uint64_t` monotonic millisecond timestamps, raw Protocol V2 message-type bytes, raw ACK result bytes, and explicit enums for `Unconfirmed`, `Active`, `Degraded`, and `Lost`. Events must distinguish dispatched/completed/timed-out requests, malformed/ignored ACKs, decode errors, state changes, and received non-ACK frames.

- [x] **Step 3: Define configurable limits**

Default heartbeat interval to 100 ms, ordinary ACK timeout to 200 ms, local liveness timeout to 450 ms, initial sequence to zero, and a finite ordinary queue/correlation history capacity. No retry or application command semantics are added in this slice.

### Task 3: Implement LinkCore scheduling and correlation (GREEN)

**Files:**
- Create: `RoboBeetlePi/link_core/src/link_core.cpp`
- Modify: `RoboBeetlePi/CMakeLists.txt`

- [x] **Step 1: Implement sequence allocation and wire dispatch**

Allocate uint16 sequences modulo 65536 without reusing an active ordinary or recent Heartbeat correlation. Encode every request with the existing `Codec::encodeWire`; Heartbeats and ordinary requests must call the same transport `write()` path.

- [x] **Step 2: Implement Heartbeat lane**

At each due timestamp dispatch a fresh `MessageType::Heartbeat` frame with a four-byte little-endian `host_uptime_ms` equal to `(now_ms - process_start_ms) mod 2^32`. Keep a bounded recent Heartbeat ACK window; an ACK miss must not stop future Heartbeat dispatch while the link remains in a recoverable state.

- [x] **Step 3: Implement ordinary FIFO and outcomes**

Allow one ordinary request in flight; queue later requests up to the configured bound. Matching result zero yields `Accepted`; matching nonzero result yields `Rejected` while preserving the raw result. A deadline expiration yields exactly one `OutcomeUnknown`, retires the correlation, and cancels queued unsent work without replay.

- [x] **Step 4: Implement receive processing**

Feed every byte chunk through the single existing `StreamDecoder`. Parse ACK payloads only when exactly four bytes are present. Wrong sequence/type, duplicate/late ACKs, malformed ACKs, valid unknown message types, telemetry, and decode errors must be observable without completing the wrong request or breaking the next frame.

- [x] **Step 5: Implement basic liveness state**

Start `Unconfirmed`; a successful matching Heartbeat ACK enters `Active`; ordinary timeout may enter `Degraded`; reaching the configured local heartbeat liveness deadline enters `Lost`; later fresh Heartbeat communication can recover only through the explicit LinkCore recovery/start path, with no actuator replay. STM32 remains the physical Safety authority.

### Task 4: Run GREEN and regression verification

**Files:**
- No additional production files.

- [x] **Step 1: Run portable tests**

Run the Pi CTest, direct executable, and a separate `-Wall -Wextra -Werror` build. Confirm every LinkCore contract test passes without sleeps.

- [x] **Step 2: Run existing Firmware protocol checks**

Run the existing `protocol_golden_vectors` executable and the complete Firmware host runner. Do not modify Firmware or Console.

- [x] **Step 3: Audit scope and whitespace**

Run `git diff --check`, verify no `termios.h`, `poll.h`, `unistd.h`, Qt, ROS2, or HAL imports under `RoboBeetlePi`, and verify no changed files under `RoboBeetleFirmware` or `RoboBeetleConsole`.

### Task 5: Commit and push the reviewed slice

**Files:**
- Only files listed by `git diff --name-only` after the implementation.

- [x] **Step 1: Inspect and commit**

Stage only the plan, LinkCore contracts/implementation, CMake, and tests; commit with:

```text
feat: add portable Raspberry Pi LinkCore
```

- [x] **Step 2: Push without integration**

Push `codex/raspberry-pi-link-core` to its existing upstream using a normal non-force push. Do not create or merge a PR, and report the exact local/remote HEAD SHA, upstream, status, changed files, diff stat, RED/GREEN evidence, and scope audit.
