# Raspberry Pi Slice 7 Remote Control Gateway Core + TCP Adapter Implementation Plan

**Status:** APPROVED IMPLEMENTATION PLAN  
**Repository:** `Mr-lxd/RoboBeetle`  
**Authoritative base:** `main @ 9bc66130467ebb4810d49c2a509c81f838855c6e`  
**Frozen design:** `docs/superpowers/specs/2026-09-17-raspberry-pi-remote-control-gateway-design.md`  
**Branch:** `codex/raspberry-pi-remote-control-gateway`  
**Plan owner:** ChatGPT  
**Implementation owner:** Codex  
**Hardware acceptance owner:** User  

This plan implements the frozen Slice 7 design. Codex must not redesign architecture while executing it. If actual source contradicts the plan or requires a frozen Slice 1-6 production change, stop and report the evidence.

---

## 1. Preconditions

Before changing production code:

1. Verify `origin/main` is still:

```text
9bc66130467ebb4810d49c2a509c81f838855c6e
```

2. Verify current worktree is the isolated Slice 7 worktree/branch:

```text
codex/raspberry-pi-remote-control-gateway
```

3. Verify `git status --short` is clean.

4. Replace the earlier local Slice 7 design draft with the ChatGPT-authored frozen design document supplied by the user.

5. Amend the existing local design-only commit so that it contains the frozen design and still changes only the design document.

6. Add this implementation-plan document as a separate docs-only commit.

7. Do not touch:
   - `D:\RoboBeetle`
   - `dsh/qt-ui`
   - Slice 6 worktree
   - `RoboBeetleConsole/`
   - `RoboBeetleFirmware/`
   - existing Slice 1-6 production implementations except `RoboBeetlePi/CMakeLists.txt`

---

## 2. Allowed implementation paths

Production additions:

```text
RoboBeetlePi/gateway/
RoboBeetlePi/tools/robobeetle_pi_gateway.cpp
```

Test additions:

```text
RoboBeetlePi/tests/rbrp_codec_tests.cpp
RoboBeetlePi/tests/control_gateway_core_tests.cpp
RoboBeetlePi/tests/linux_onboard_application_port_tests.cpp
RoboBeetlePi/tests/tcp_gateway_integration_tests.cpp
```

Build integration:

```text
RoboBeetlePi/CMakeLists.txt
```

Documentation:

```text
docs/superpowers/specs/2026-09-17-raspberry-pi-remote-control-gateway-design.md
docs/superpowers/plans/2026-09-17-raspberry-pi-remote-control-gateway.md
```

No other production path is expected.

If implementation appears to require modifying:

```text
RoboBeetlePi/protocol/
RoboBeetlePi/link_core/
RoboBeetlePi/transport/
RoboBeetlePi/session/
RoboBeetlePi/runtime/
RoboBeetlePi/application/
RoboBeetleFirmware/
RoboBeetleConsole/
```

STOP and report before doing it.

---

## 3. CMake target architecture

Add two portable libraries outside the Linux conditional:

```text
rbp2_rbrp_codec
rbp2_gateway_core
```

Suggested dependencies:

```text
rbp2_rbrp_codec
  -> standard library only

rbp2_gateway_core
  -> standard library only
  -> gateway headers
```

Do not link the portable Core against `rbp2_onboard_application`, `rbp2_link_runtime`, or POSIX targets.

Portable tests:

```text
rbp2_rbrp_codec_tests
rbp2_gateway_core_tests
```

Inside `if(CMAKE_SYSTEM_NAME STREQUAL "Linux")` add:

```text
rbp2_gateway_linux_application_port
rbp2_gateway_tcp
rbp2_gateway_owner
robobeetle_pi_gateway
```

Suggested dependencies:

```text
rbp2_gateway_linux_application_port
  -> rbp2_gateway_core
  -> rbp2_onboard_application

rbp2_gateway_tcp
  -> rbp2_rbrp_codec
  -> Threads::Threads

rbp2_gateway_owner
  -> rbp2_gateway_core
  -> rbp2_gateway_linux_application_port
  -> rbp2_gateway_tcp
  -> Threads::Threads

robobeetle_pi_gateway
  -> rbp2_gateway_owner
```

Linux tests:

```text
rbp2_gateway_linux_application_port_tests
rbp2_tcp_gateway_integration_tests
```

Use `find_package(Threads REQUIRED)` only for the Linux gateway layer.

Keep all socket, `eventfd`, `poll`, pthread/std::thread implementation below the Linux conditional.

---

## 4. Frozen implementation-facing interfaces

Names may follow normal project style, but semantics must match these interfaces.

### 4.1 `gateway_types.hpp`

Define:

```cpp
using ControlSourceId = std::uint64_t;
using RequestId = std::uint32_t;
using GatewayTimeMs = std::uint64_t;
```

Define typed `RobotCommand` alternatives for:

```text
EnableServos
DisableServos
SetServoAngle
NeutralServos
StartMotion
StopMotion
SetGaitBackend
```

Define neutral enums/DTOs from the frozen design, including:

```text
GatewayApplicationSubmitStatus
GatewayApplicationSubmitResult
GatewayApplicationRunStatus
GatewayApplicationSessionState
GatewayApplicationLinkState
GatewayCommandOutcomeEvent
GatewayTelemetryEvent
GatewayTelemetryMalformedDiagnostic
GatewayStateLinkEvent
GatewayApplicationEvent
RemoteMessage
RemoteEnvelope
GatewayMessage
GatewayOutbound
SourceLostSignal
CloseSourceSignal
```

Every cross-thread output/lifecycle signal must carry `ControlSourceId`.

Do not add raw Protocol V2 request type to `GatewayCommandOutcomeEvent`.

### 4.2 `gateway_application_port.hpp`

Portable abstract interface. A recommended shape is:

```cpp
class GatewayApplicationPort {
public:
    virtual ~GatewayApplicationPort() = default;

    virtual int open() = 0;
    virtual GatewayApplicationRunResult run_once() = 0;
    virtual GatewayApplicationAbortResult abort() = 0;
    virtual GatewayApplicationSubmitResult submit(const RobotCommand &) = 0;

    virtual GatewayApplicationSessionState session_state() const noexcept = 0;
    virtual GatewayApplicationLinkState link_state() const = 0;
};
```

Tests use a fake implementation.

Only the gateway-owner thread calls this interface.

### 4.3 `ControlGatewayCore`

Use deterministic caller-supplied monotonic time. Do not call wall clock.

Recommended public responsibilities:

```text
source_connected(source)
process(RemoteEnvelope, owner_now_ms)
source_lost(SourceLostSignal, owner_now_ms)
consume_application_run_result(result, owner_now_ms)
consume_application_abort_result(result, owner_now_ms)
check_time(owner_now_ms)
network_output_failed(source, reason, owner_now_ms)
shutdown(owner_now_ms)
```

The Core may hold a non-owning reference to `GatewayApplicationPort`.

The Core owns:

```text
Hello/lifecycle state
authority
lease deadline
request-id history
sequence correlation
domain state transitions
```

The owner layer owns bounded cross-thread queues and performs network enqueueing.

### 4.4 Application sequence correlation

Live map:

```text
uint16_t stm32_sequence
    ->
ControlSourceId
RequestId
RobotCommandKind
```

Do not key final outcomes by raw Protocol V2 request type.

Do not create an application command token.

Unknown/stale final sequence:

```text
local diagnostic only
```

### 4.5 Time behavior

Initial Acquire success:

```text
open() succeeds
-> authority committed Owned
-> initial_deadline = owner_now_ms + 1000
```

Subsequent heartbeat:

```text
if envelope.received_at_ms < current_deadline:
    deadline = envelope.received_at_ms + 1000
else:
    late; do not revive authority
```

---

## 5. Commit sequence

The branch should be implemented in the following logical commits. Do not squash/rewrite these implementation commits during development.

### Commit 1 — freeze design document

```text
docs: freeze Slice 7 remote control gateway design
```

Action:

- replace the earlier design draft with the ChatGPT-authored frozen Design Spec;
- amend the existing design-only commit if practical because it has not been pushed;
- confirm design commit contains only that spec.

No production changes.

### Commit 2 — add implementation plan

```text
docs: add Slice 7 remote control gateway plan
```

Add this plan only.

### Commit 3 — define portable RBRP tests

```text
test: define Slice 7 RBRP v1 contracts
```

Add:

```text
RoboBeetlePi/tests/rbrp_codec_tests.cpp
```

Add the test target to CMake.

Use the project's existing lightweight test support/style.

Before implementation, the new target must fail because RBRP implementation is missing or incomplete; existing baseline tests must remain unaffected.

Contracts must cover:

- exact 16-byte header;
- explicit LE fields;
- all message kinds;
- all exact payload sizes;
- fragmented header;
- fragmented payload;
- multiple frames per input;
- bad magic/version/flags;
- oversize payload;
- unknown top-level kind;
- fatal framing classification;
- typed command decoding;
- signed angle;
- Backward representation;
- server message encoding;
- 3/58/40 telemetry payload sizes.

### Commit 4 — implement portable RBRP

```text
feat: add Slice 7 RBRP v1 codec
```

Add:

```text
gateway/include/robobeetle/gateway/gateway_types.hpp
gateway/include/robobeetle/gateway/rbrp_codec.hpp
gateway/src/rbrp_codec.cpp
```

Implement only wire/framing/types needed for codec tests.

Requirements:

- no POSIX includes;
- no Slice 6 Linux-only includes;
- no struct-memcpy serialization;
- bounded parser storage;
- no application/authority policy inside codec;
- fatal framing represented to caller, not handled by socket code in portable library.

Gate:

```text
rbp2_rbrp_codec_tests PASS
existing Windows portable tests PASS
```

### Commit 5 — define portable Core contracts

```text
test: define Slice 7 gateway core contracts
```

Add:

```text
RoboBeetlePi/tests/control_gateway_core_tests.cpp
```

Use:

```text
FakeGatewayApplicationPort
```

Tests must cover at least:

- Hello required;
- Acquire from ReopenRequired;
- explicit open exactly once;
- Acquire OpenFailed;
- Acquire InvalidState;
- initial lease grant-time anchor;
- heartbeat trusted-receive-time refresh;
- timely heartbeat processed after deadline;
- actually late heartbeat;
- large bounded backlog followed by timely heartbeat;
- Release -> abort;
- SourceLost -> abort;
- no second authority;
- request ID validation;
- duplicate outstanding request ID;
- recent completed ID rejection;
- local command rejections;
- Backward PendingQualification with no submit;
- Submitted produces sequence correlation;
- Accepted outcome;
- Rejected outcome;
- OutcomeUnknown;
- Cancelled;
- stale/unknown sequence ignored as diagnostic;
- lease expiry;
- session/link loss;
- network-output failure;
- shutdown;
- no robot actuator shadow state.

The new target may fail before Core implementation.

### Commit 6 — implement portable ControlGatewayCore

```text
feat: add Slice 7 portable gateway core
```

Add:

```text
gateway/include/robobeetle/gateway/gateway_application_port.hpp
gateway/include/robobeetle/gateway/control_gateway_core.hpp
gateway/src/control_gateway_core.cpp
```

Implement only frozen portable policy.

Important:

- Core does not know TCP fd/IP/DDS/Qt/device path;
- no wall clock;
- no POSIX;
- no automatic reconnect;
- no retry/replay;
- no actuator shadow state;
- no raw PWM;
- no raw Protocol V2 frames;
- fatal protocol classification remains adapter-facing;
- application outcomes correlate by live STM32 sequence only.

Gate:

```text
rbp2_rbrp_codec_tests PASS
rbp2_gateway_core_tests PASS
all existing Windows portable tests PASS
```

### Commit 7 — define Linux application-port mapping tests

```text
test: define Slice 7 Linux application port mapping
```

Add:

```text
tests/linux_onboard_application_port_tests.cpp
```

Test the exhaustive translation boundary:

- submit status mapping;
- optional sequence invariant;
- Servo/Gait/Motion typed mapping;
- Backward remains PendingQualification;
- no SetServoPwm exposed;
- session-state mapping;
- link-state mapping;
- Accepted/Rejected/Unknown/Cancelled mapping;
- typed Leak/IMU/Depth mapping;
- malformed telemetry diagnostic mapping;
- no raw LinkEvent crosses the neutral port;
- no duplicate telemetry/outcome generated.

Prefer source-level/PTY behavior where necessary; do not modify Slice 6 to add testing seams unless review explicitly authorizes it.

### Commit 8 — implement LinuxOnboardApplicationPort

```text
feat: add Slice 7 Linux application port
```

Add:

```text
gateway/linux/include/robobeetle/gateway/linux_onboard_application_port.hpp
gateway/linux/src/linux_onboard_application_port.cpp
```

The class owns one `OnboardApplication`.

It stores the configured device path.

No second OnboardApplication may be created elsewhere.

Special telemetry mapping rule:

Slice 6 currently emits raw `FrameReceived` followed immediately by typed telemetry/malformed notice. The Linux adapter may use that local ordering only to associate the raw STM32 telemetry sequence with the immediately following typed DTO. Raw frame bytes are then discarded and never cross the portable boundary.

Gate on Linux:

```text
rbp2_gateway_linux_application_port_tests PASS
existing Linux CTest remains PASS
```

### Commit 9 — define TCP bridge/integration contracts

```text
test: define Slice 7 TCP gateway integration contracts
```

Add:

```text
tests/tcp_gateway_integration_tests.cpp
```

The initial test skeleton must specify and progressively exercise:

- one connected TCP source;
- second source immediate close;
- source IDs increase;
- loopback listener/client;
- fragmented RBRP;
- eventfd owner->worker wakeup;
- source-addressed outbound;
- SourceLost;
- CloseSource;
- stale SourceLost ignored;
- stale CloseSource ignored;
- old-source output not sent to new source;
- partial telemetry TX isolation;
- inbound queue exhaustion;
- critical output exhaustion;
- normal shutdown.

### Commit 10 — implement Linux TCP adapter and bridge

```text
feat: add Slice 7 Linux TCP adapter
```

Add:

```text
gateway/linux/include/robobeetle/gateway/tcp_adapter.hpp
gateway/linux/src/tcp_adapter.cpp
```

Implement:

```text
listener fd
one current client fd
poll(listener, client, eventfd)
nonblocking TCP
RBRP stream parser
bounded inbound queue
source-addressed output
partial write state
telemetry latest-value coalescing
SourceLostSignal
CloseSourceSignal
```

Connection IDs must be monotonically increasing nonzero IDs.

If the counter wraps to zero, skip zero.

A stale signal/output never applies to a different source.

TCP worker never calls the application port.

Use Linux `eventfd` for owner->worker wakeup.

The worker must tolerate `EINTR` and nonblocking `EAGAIN/EWOULDBLOCK` correctly.

Fatal socket/protocol errors close the matching source.

### Commit 11 — implement GatewayOwner composition

```text
feat: add Slice 7 gateway owner
```

Add:

```text
gateway/linux/include/robobeetle/gateway/gateway_owner.hpp
gateway/linux/src/gateway_owner.cpp
```

Owner thread owns:

```text
ControlGatewayCore
LinuxOnboardApplicationPort
```

Frozen loop:

```text
SourceLost
-> drain all currently queued RemoteEnvelope values
-> check lease
-> one application run_once when appropriate
-> SourceLost again
-> drain current queue again
-> check lease
-> publish/wake worker
```

Do not introduce an 8-message budget.

When application is `ReopenRequired` and there is no immediate work, use a condition-variable/bounded-wait strategy rather than busy spinning.

Critical output enqueue failure must call Core network-failure handling and post source-addressed CloseSource through the independent control path.

### Commit 12 — complete TCP + PTY end-to-end integration

```text
test: harden Slice 7 gateway end-to-end lifecycle
```

Extend `tcp_gateway_integration_tests.cpp` so the real path is:

```text
TCP client
-> TCP worker
-> GatewayOwner
-> ControlGatewayCore
-> LinuxOnboardApplicationPort
-> OnboardApplication
-> LinkRuntime
-> PTY simulated STM32
```

Required scenarios:

1. Hello/Acquire.
2. 575 ms quiet/resync -> heartbeat -> Active.
3. CommandRequest -> CommandSubmitted.
4. STM32 ACK -> CommandOutcome.
5. Rejected ACK.
6. Leak/IMU/Depth forwarding.
7. Telemetry coalescing.
8. TCP disconnect -> pending Unknown / queued Cancelled -> abort.
9. Lease timeout -> abort.
10. UART/PTTY loss while TCP exists -> authority revoked.
11. Same-process reconnect -> new source -> explicit Acquire.
12. STM32 sequence continuity.
13. No replay.
14. A disconnect then B immediate reconnect; late A outcomes never reach B.
15. stale CloseSource cannot close B.
16. inbound flood is bounded/fails safe.
17. slow client critical backpressure fails safe.
18. Backward creates zero UART command bytes.
19. normal shutdown ordering.

Set a sensible CTest timeout; do not hide deadlocks with excessively long timeouts.

### Commit 13 — add gateway executable

```text
feat: add Raspberry Pi remote gateway executable
```

Add:

```text
tools/robobeetle_pi_gateway.cpp
```

CLI:

```text
robobeetle_pi_gateway
  [--device <path>]
  [--bind <IPv4-address>]
  [--port <1..65535>]
```

Defaults:

```text
--device /dev/serial0
--bind 127.0.0.1
--port 47000
```

Requirements:

- invalid CLI arguments fail clearly;
- no implicit public bind;
- non-loopback prints trusted-network/no-TLS warning;
- SIGINT/SIGTERM request graceful gateway shutdown;
- no daemon/systemd behavior;
- no automatic UART reconnect;
- no actuator command synthesized on exit.

### Commit 14 — verification/docs sync

```text
docs: record Slice 7 software verification
```

Update the implementation plan with actual verification evidence/checkmarks or add a concise gateway README if needed.

Do not claim hardware acceptance.

Record:

- exact implementation HEAD;
- Windows toolchain/config used;
- Windows portable CTest results;
- Pi/Linux native build result;
- Linux CTest result;
- gateway executable build/link result;
- known qualification items still pending.

---

## 6. Required Windows portable gate

Run on the Windows host with the repository's existing MinGW environment.

Configure/build at least:

```text
rbp2_protocol_tests
rbp2_robot_codec_tests
rbp2_rbrp_codec_tests
rbp2_gateway_core_tests
```

Run CTest for all Windows-compatible tests.

If the known MinGW `0xc0000139` loader issue occurs, fix only the test-process `PATH` to include the matching MinGW runtime DLL directory. Do not copy random DLLs into the repo and do not treat the environment issue as a product-code failure.

Required result:

```text
all Windows-portable tests PASS
```

---

## 7. Required Linux/Pi software gate

Before any hardware acceptance, on Raspberry Pi / ARM Linux:

1. Use the exact Slice 7 implementation commit.
2. Native configure/build.
3. Run full `ctest --output-on-failure`.
4. Verify all previous Slice 5/6 tests still pass.
5. Verify new:
   - RBRP codec tests;
   - portable Core tests;
   - Linux application-port tests;
   - TCP/PTTY integration tests.
6. Build `robobeetle_pi_gateway`.
7. Run CLI validation/help/safe loopback smoke without actuator commands.

No hardware gate starts until this software gate is PASS.

---

## 8. Review gate before hardware

After Codex completes implementation and software tests:

STOP.

Do not create PR.

Report to ChatGPT:

```text
branch
base SHA
current HEAD
git status --short
git log --oneline base..HEAD
git diff --name-only base..HEAD
git diff --stat base..HEAD

Windows configure/build/test evidence
Linux/Pi configure/build/CTest evidence
gateway executable build evidence
```

Also report explicitly:

```text
Firmware changed?               NO
Qt changed?                     NO
Protocol/LinkCore changed?      NO
Transport/Session/Runtime?      NO
application Slice 6 changed?    NO
dsh/qt-ui touched?              NO
Slice 6 worktree touched?       NO
```

ChatGPT performs external code review before hardware acceptance.

---

## 9. Hardware acceptance after code review

Only after external review returns:

```text
BLOCKER = 0
SHOULD FIX = 0
```

perform user-led Gates A-E from the frozen Design Spec.

Codex may provide commands/logging support but the user performs:

- cable/network break;
- servo/mechanical observation;
- STM32/Pi physical setup;
- final safety validation.

---

## 10. PR gate

PR is created only after:

```text
Design Frozen              PASS
Implementation             PASS
Windows Portable Gate      PASS
Linux/Pi Native Gate       PASS
External Code Review       PASS
Hardware Gate A            PASS
Hardware Gate B            PASS
Hardware Gate C            PASS
Hardware Gate D            PASS
Hardware Gate E            PASS
Final Documentation        PASS
```

No direct merge.

PR must document:

- base/head SHA;
- architecture boundary;
- no Qt/Firmware/frozen lower-layer changes;
- Windows test evidence;
- Linux/Pi test evidence;
- Hardware Gates A-E;
- remaining non-goals;
- trusted-LAN/no-TLS limitation.

---

## 11. Stop conditions

Codex must STOP and report instead of improvising if any of these occurs:

1. Current `main` is no longer the expected base and materially conflicts.
2. Frozen Slice 1-6 production code appears to require modification.
3. A second `OnboardApplication` seems necessary.
4. TCP worker would need to call `OnboardApplication`/`LinkRuntime`.
5. Portable Core would need Linux-only headers.
6. Source tagging cannot be maintained across all bridge objects.
7. Timely heartbeat semantics cannot be tested deterministically.
8. `abort()` behavior differs from the frozen safety model.
9. Backward would generate UART traffic.
10. Old source data can be observed by a new source.
11. A test requires weakening no-replay or explicit-reopen semantics.
12. Qt/Firmware changes appear necessary.
13. The user-protected worktrees would need modification.
14. Any safety behavior would need to be guessed rather than supported by current code/design.

Do not solve these by redesigning on the fly.

---

## 12. Definition of implementation complete

Implementation phase is complete only when:

```text
[ ] frozen Design Spec is present
[ ] this Implementation Plan is present
[ ] portable RBRP codec implemented
[ ] portable ControlGatewayCore implemented
[ ] LinuxOnboardApplicationPort implemented
[ ] Linux TCP worker implemented
[ ] eventfd/bridge implemented
[ ] GatewayOwner implemented
[ ] gateway executable implemented
[ ] all portable tests PASS
[ ] all Linux tests PASS
[ ] old Slice 1-6 tests still PASS
[ ] no frozen lower-layer behavior modified
[ ] no Qt/Firmware changes
[ ] no automatic reconnect/retry/replay
[ ] single application/UART owner preserved
[ ] source-generation isolation tests PASS
[ ] lease tests PASS
[ ] backpressure tests PASS
[ ] graceful shutdown tests PASS
[ ] code is ready for ChatGPT external review
```

Hardware acceptance and PR are later phases.

---

## 13. Commit 14 verification record (2026-09-17)

This section records the implementation evidence available on the current
Windows host. The implementation HEAD below is the exact code target before
this documentation-only commit.

### Implementation state

```text
branch: codex/raspberry-pi-remote-control-gateway
base:   9bc66130467ebb4810d49c2a509c81f838855c6e
implementation HEAD for original pre-review handoff:
a84ce0180ddf3918966759e48d8ecddc7942240f
```

The implementation follows the frozen design and plan. Corrective commit
`4e9f003` orders SourceLost processing before connection registration and
ignores a registration whose TCP source generation has already closed; the
follow-up `a84ce01` registers a still-live source before draining its first
envelope, preserving both immediate A-to-B isolation and the initial Hello.

### Windows portable gate

```text
[x] Configure: CMake + Ninja, GNU 16.1.0 MinGW, BUILD_TESTING=ON
[x] Build:    cmake --build --parallel 4
[x] CTest:    ctest --output-on-failure
```

Result:

```text
4/4 passed
rbp2_rbrp_codec_tests       PASS
rbp2_gateway_core_tests     PASS
rbp2_protocol_tests         PASS
rbp2_robot_codec_tests      PASS
```

The test-process PATH contains the selected MinGW compiler `bin` directory;
no runtime DLL was copied into the repository or product targets.

### Linux/Pi gate

```text
[ ] Raspberry Pi/native Linux configure/build: NOT RUN
[ ] Native full CTest:                         NOT RUN
[ ] robobeetle_pi_gateway native build/link:   NOT RUN
[ ] CLI/help/loopback smoke:                   NOT RUN
```

The current Windows host has no installed WSL Linux environment, native Linux
compiler, or container runtime. A diagnostic `CMAKE_SYSTEM_NAME=Linux`
MinGW attempt configured but is not a native result and stopped at missing
POSIX headers (`termios.h`, `poll.h`, and `arpa/inet.h`). The Linux/Pi gate
must be run on Raspberry Pi/ARM Linux against implementation HEAD
`a84ce01e9691e2f41b16a39293789d4a55c1394d`.

### Scope and qualification

```text
Firmware changed?               NO
Qt changed?                     NO
Protocol/LinkCore changed?      NO
Transport/Session/Runtime?      NO
application Slice 6 changed?    NO
dsh/qt-ui touched?              NO
Slice 6 worktree touched?       NO
Hardware acceptance?            NOT RUN
```

No hardware acceptance, PR, push, merge, or Slice 8 work is part of this
record. External code review remains the next gate; Linux/Pi software evidence
is still required before hardware qualification.

## 14. External Code Review Round 1 correction record (2026-09-17)

Round 1 reviewed the pushed handoff at:

```text
reviewed HEAD: 9a785af758712a590a163260eb822d8e8fb0f635
base:          9bc66130467ebb4810d49c2a509c81f838855c6e
```

The correction implementation is complete at the new, unrevised normal
commit:

```text
test commit:         ec19449f0a34c3bd26f04ff99540e7d5b9240b1a
implementation HEAD: c84ce6672a779886292afb4fc49a3b5ba7193e86
```

The six blockers were addressed as follows:

```text
[x] Linux application-port SessionState namespace uses robobeetle::session
[x] ProtocolPty declares protocol_ack before use; HelloReply checks 0/512
[x] Submitted correlation is live before CommandSubmitted publication;
    ambiguous application invariants fail safe and late outcomes stay local
[x] TCP worker snapshots fd/source generation and discards stale revents
[x] owner registration coalesces to the newest source generation
[x] non-Lost link transitions publish unsolicited ControlState
[x] initial lease uses an injected post-open monotonic sample
[x] owner drains bridge work under a synchronized lease-evaluation boundary
```

Regression coverage added or updated:

```text
[x] failed CommandSubmitted publication -> abort, CloseSource, zero correlation
[x] late old sequence after a new source remains diagnostic only
[x] Submitted-without-sequence and live-sequence collision fail safe
[x] post-open lease anchor and Active/Degraded/Active state forwarding
[x] queued replacement survives A generation rollover with fresh source ID
[x] newest pending source registration replaces an older pending generation
[x] real PTY lifecycle waits for ControlState(link=Active)
```

Windows verification after the correction commits:

```text
[x] fresh CMake/Ninja configure: GNU 16.1.0 MinGW, BUILD_TESTING=ON
[x] fresh build: cmake --build build-slice7-round1-final --parallel 4
[x] full portable CTest: 4/4 PASS
```

Linux/Pi native configure/build and CTest were not run on this Windows host;
no Linux success is claimed here. Hardware acceptance, PR creation, merge,
Slice 8, and forced history operations remain out of scope.

Frozen scope remains unchanged:

```text
Firmware changed?               NO
Qt changed?                     NO
Protocol/LinkCore changed?      NO
Transport/Session/Runtime?      NO
application Slice 6 changed?    NO
dsh/qt-ui touched?              NO
Slice 6 worktree touched?       NO
```
