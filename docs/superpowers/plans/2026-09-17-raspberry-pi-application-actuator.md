# Raspberry Pi Slice 6 Application / Actuator Integration Implementation Plan

> For agentic workers: REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Add a typed, Linux-safe OnboardApplication facade, firmware-parity telemetry decoders, and a manual Raspberry Pi smoke CLI above the frozen Slice 5 LinkRuntime.

**Architecture:** Portable robot_types and robot_codec validate and encode Protocol V2 payloads and decode the three existing telemetry payloads. Linux-only OnboardApplication delegates open/run/submit/abort to LinkRuntime, preserves every raw LinkEvent, and appends typed telemetry or malformed notices. The smoke executable issues one explicit command at a time; it never owns the serial descriptor, makes serial-fd calls, or owns a scheduler, retry policy, or robot-state cache. It may use bounded readiness checks on stdin so runtime service continues while an operator types.

**Tech Stack:** C++17, existing CMake/CTest targets, std::variant, std::optional, existing rbp2_protocol, rbp2_link_runtime, and Linux PTY tests.

---

## File map

- Create RoboBeetlePi/application/include/robobeetle/application/robot_types.hpp for portable enums, command result types, and Leak/IMU/Depth values.
- Create RoboBeetlePi/application/include/robobeetle/application/robot_codec.hpp and RoboBeetlePi/application/src/robot_codec.cpp for payload encoding and Firmware-parity decoding.
- Create RoboBeetlePi/application/include/robobeetle/application/onboard_application.hpp and RoboBeetlePi/application/src/onboard_application.cpp for the Linux facade and event translation.
- Create RoboBeetlePi/application/README.md for API, safety, CLI, and evidence boundaries.
- Create RoboBeetlePi/tests/robot_codec_tests.cpp and RoboBeetlePi/tests/onboard_application_tests.cpp for portable contract and Linux PTY integration tests.
- Create RoboBeetlePi/tools/robobeetle_pi_smoke.cpp for the manual Linux CLI.
- Modify only RoboBeetlePi/CMakeLists.txt to register rbp2_robot_codec, rbp2_onboard_application, tests, and smoke.
- Do not modify RoboBeetleFirmware, RoboBeetleConsole, or production files under RoboBeetlePi/protocol, link_core, transport, session, or runtime.

### Task 1: Build boundary and portable codec RED tests

**Files:**
- Modify: RoboBeetlePi/CMakeLists.txt
- Create: RoboBeetlePi/application/include/robobeetle/application/robot_types.hpp
- Create: RoboBeetlePi/application/include/robobeetle/application/robot_codec.hpp
- Create: RoboBeetlePi/tests/robot_codec_tests.cpp

- [x] Step 1: Register a portable rbp2_robot_codec target and rbp2_robot_codec_tests outside the Linux conditional. The target includes application/include and protocol/include and links no POSIX library. Until the codec header exists, the test compiles a guarded main that prints exactly "FAIL: Slice 6 robot codec is not implemented" and returns 1. **Executed: PASS.**

- [x] Step 2: Write command RED assertions before implementing robot_codec.cpp. Use these exact values. **Executed: RED observed, then GREEN.**

~~~cpp
expect(encode_servo_mask(0x001f).payload == bytes({0x1f, 0x00}), ...);
expect(encode_servo_mask(0).status == CodecStatus::InvalidMask, ...);
expect(encode_servo_mask(0x8000).status == CodecStatus::InvalidMask, ...);
expect(encode_servo_angle(ServoId::FrontLeft, 0x1234).payload ==
       bytes({1, 1, 0x34, 0x12}), ...);
expect(encode_servo_angle(ServoId::RearLeft, -2).payload ==
       bytes({1, 4, 0xfe, 0xff}), ...);
expect(encode_servo_angle(static_cast<ServoId>(5), 0).status ==
       CodecStatus::InvalidServoId, ...);
expect(encode_servo_pwm(ServoId::FrontRight, 2000).payload ==
       bytes({1, 0, 0xd0, 0x07}), ...);
expect(encode_motion(MotionMode::Forward, MotionAction::Start).payload ==
       bytes({1, 1, 1}), ...);
expect(encode_motion(MotionMode::Stop, MotionAction::Stop).payload ==
       bytes({1, 0, 0}), ...);
expect(encode_motion(MotionMode::Stop, MotionAction::Start).status ==
       CodecStatus::InvalidMotionMode, ...);
expect(encode_motion(static_cast<MotionMode>(7), MotionAction::Start).status ==
       CodecStatus::InvalidMotionMode, ...);
expect(encode_gait_backend(GaitBackend::SimpleGait).payload == bytes({0}), ...);
expect(encode_gait_backend(GaitBackend::CPG).payload == bytes({1}), ...);
expect(encode_gait_backend(static_cast<GaitBackend>(2)).status ==
       CodecStatus::InvalidGaitBackend, ...);
~~~

Run the target before implementation:

~~~powershell
$env:PATH = "C:\Users\laixindong\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin;$env:PATH"
cmake --build C:\Users\laixindong\AppData\Local\Temp\robobeetle-slice6-red-20260917-01 --target rbp2_robot_codec_tests
& C:\Users\laixindong\AppData\Local\Temp\robobeetle-slice6-red-20260917-01\rbp2_robot_codec_tests.exe
~~~

Expected: exit 1 with the missing-feature message.

- [x] Step 3: Add RED fixtures for Leak (payload 1), IMU (payload 56), and Depth (payload 38). Fixtures must be built from current Firmware encoders: IMU schema 1/flags 0x07, signed triples at offsets 2/8/14, diagnostics at 20/24/28/32/36/40/44/48/52; Depth schema 1/flags 0x03, signed fields at 2 and 6, age at 8, diagnostics at 10/14/18/22/26/30/34. Assert valid values, wrong length/schema, reserved flags, signed values, and nonzero invalid-domain data are rejected. **Executed: PASS.**

- [x] Step 4: Commit the build and RED tests. **Committed as `b6ae2e2`, tightened as `50a1109`.**

~~~powershell
git add RoboBeetlePi/CMakeLists.txt RoboBeetlePi/application RoboBeetlePi/tests/robot_codec_tests.cpp
git commit -m "test: define Raspberry Pi application codec contracts"
~~~

### Task 2: Implement and verify the portable robot codec

**Files:**
- Modify: RoboBeetlePi/application/include/robobeetle/application/robot_types.hpp
- Modify: RoboBeetlePi/application/include/robobeetle/application/robot_codec.hpp
- Create: RoboBeetlePi/application/src/robot_codec.cpp
- Modify: RoboBeetlePi/tests/robot_codec_tests.cpp

- [x] Step 1: Define the exact wire-facing enums and mask. **Executed: PASS.**

~~~cpp
enum class ServoId : std::uint8_t {
    FrontRight = 0, FrontLeft = 1, FrontAxis = 2,
    RearRight = 3, RearLeft = 4,
};
enum class MotionMode : std::uint8_t {
    Stop = 0, Forward = 1, Backward = 2, TurnLeft = 3,
    TurnRight = 4, Ascend = 5, Descend = 6,
};
enum class MotionAction : std::uint8_t { Stop = 0, Start = 1 };
enum class GaitBackend : std::uint8_t { SimpleGait = 0, CPG = 1 };
constexpr std::uint16_t SupportedServoMask = 0x001f;
~~~

Use named diagnostics structs and fixed-width arrays for IMU/Depth. Expose raw validity flags and sample_age_ms; do not add calibration or stale policy.

- [x] Step 2: Implement command encoders that return a CodecResult containing CodecStatus and protocol::Bytes. Mask commands are two little-endian bytes. Angle/PWM commands are [1, servo_id, value_lo, value_hi]. Motion is [1, mode, action]; gait is one byte. Reject invalid enum casts, masks, and STOP/START combinations. Do not call Codec::encodeWire. **Committed as `d28dce5`.**

- [x] Step 3: Implement decode_leak, decode_imu, and decode_depth as non-throwing optional-returning functions with a TelemetryMalformedReason output. Enforce exact size, schema, reserved flags, zero invalid domains, and Depth unknown age 0xffff when depth is invalid. Decode every named diagnostic offset from Firmware source. Do not enforce Qt-only physical ranges or freshness. **Executed: PASS.**

- [x] Step 4: Run codec GREEN and a fresh warning build. **Windows GCC 16.1.0 / `-Wall -Wextra -Werror`: PASS; CTest 2/2 PASS.**

~~~powershell
$env:PATH = "C:\Users\laixindong\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin;$env:PATH"
cmake -S RoboBeetlePi -B C:\Users\laixindong\AppData\Local\Temp\robobeetle-slice6-portable-20260917-01 -G Ninja -DBUILD_TESTING=ON -DCMAKE_CXX_FLAGS="-Wall -Wextra -Werror"
cmake --build C:\Users\laixindong\AppData\Local\Temp\robobeetle-slice6-portable-20260917-01
ctest --test-dir C:\Users\laixindong\AppData\Local\Temp\robobeetle-slice6-portable-20260917-01 --output-on-failure
~~~

Expected: portable existing tests plus rbp2_robot_codec_tests report 100% pass and no warnings.

- [x] Step 5: Commit. **Committed as `d28dce5`.**

~~~powershell
git add RoboBeetlePi/application RoboBeetlePi/tests/robot_codec_tests.cpp RoboBeetlePi/CMakeLists.txt
git commit -m "feat: add Raspberry Pi robot command and telemetry codec"
~~~

### Task 3: Linux OnboardApplication RED tests and facade

**Files:**
- Modify: RoboBeetlePi/CMakeLists.txt
- Create: RoboBeetlePi/application/include/robobeetle/application/onboard_application.hpp
- Create: RoboBeetlePi/application/src/onboard_application.cpp
- Create: RoboBeetlePi/tests/onboard_application_tests.cpp

- [x] Step 1: Register rbp2_onboard_application only inside the Linux conditional, link it to rbp2_link_runtime and rbp2_robot_codec, and add rbp2_onboard_application_tests. Until the facade header exists, the test must print "FAIL: Slice 6 OnboardApplication is not implemented" and return 1. **Executed: guarded RED observed; Windows omits Linux target.**

- [x] Step 2: Write PTY tests before the facade implementation. Use only existing LinkRuntime/SerialSession private test seams. Cover open and 575-ms quiet/raw-zero/Heartbeat activation; exact typed ServoEnable request bytes; matching OK RequestAccepted and non-OK RequestRejected; ACK plus Leak/IMU/Depth interleave; pending OutcomeUnknown and queued Cancelled on loss; explicit reopen with a new sequence and no replay; invalid arguments; and start_motion(Backward) returning PendingQualification with no write. **Executed as test-first RED scaffolding; Linux execution remains NOT RUN.**

- [x] Step 3: Define the smallest Linux facade. **Implemented as `c31e1e9`; PTY harness hardening `9d3f824`.**

~~~cpp
class OnboardApplication final {
public:
    explicit OnboardApplication(link_core::LinkCoreConfig config = {},
                                std::size_t max_tx_frames = 64U,
                                std::size_t max_tx_bytes = 65536U);
    int open(const char *device_path);
    ApplicationRunResult run_once();
    CommandSubmitResult enable_servos(std::uint16_t mask);
    CommandSubmitResult disable_servos(std::uint16_t mask);
    CommandSubmitResult set_servo_angle(ServoId id, std::int16_t angle_cdeg);
    CommandSubmitResult neutral_servos(std::uint16_t mask);
    CommandSubmitResult start_motion(MotionMode mode);
    CommandSubmitResult stop_motion();
    CommandSubmitResult set_gait_backend(GaitBackend backend);
    CommandSubmitResult set_servo_pwm_maintenance(ServoId id,
                                                   std::uint16_t pulse_us);
    ApplicationAbortResult abort();
    [[nodiscard]] session::SessionState session_state() const noexcept;
    [[nodiscard]] link_core::LinkState link_state() const;
};
~~~

ApplicationEvent is std::variant<link_core::LinkEvent, LeakTelemetry,
ImuTelemetry, DepthTelemetry, TelemetryMalformed>. Every raw LinkEvent is
returned first; a telemetry FrameReceived adds one typed value or malformed
notice. ApplicationRunResult preserves RuntimeStatus and errno. Command
mapping only converts existing SubmitStatus values and never caches or replays
commands.

- [ ] Step 4: Run Linux GREEN when available. **NOT RUN: no usable WSL distribution, Docker, or Linux runner.**

~~~sh
cmake -S RoboBeetlePi -B /tmp/robobeetle-slice6-linux-20260917 -G Ninja \
  -DCMAKE_CXX_FLAGS="-Wall -Wextra -Werror" -DBUILD_TESTING=ON
cmake --build /tmp/robobeetle-slice6-linux-20260917
ctest --test-dir /tmp/robobeetle-slice6-linux-20260917 --output-on-failure
~~~

Expected: rbp2_protocol_tests, rbp2_posix_serial_transport_tests,
rbp2_serial_session_tests, rbp2_link_runtime_tests, rbp2_robot_codec_tests,
and rbp2_onboard_application_tests all pass. If Linux is unavailable, mark
these native and PTY checks NOT RUN and do not claim them as PASS.

- [x] Step 5: Commit. **Committed as `c31e1e9`; harness fix `9d3f824`.**

~~~powershell
git add RoboBeetlePi/CMakeLists.txt RoboBeetlePi/application/include/robobeetle/application/onboard_application.hpp RoboBeetlePi/application/src/onboard_application.cpp RoboBeetlePi/tests/onboard_application_tests.cpp
git commit -m "feat: add Raspberry Pi typed onboard application"
~~~

### Task 4: Manual Linux smoke tool and documentation

**Files:**
- Modify: RoboBeetlePi/CMakeLists.txt
- Create: RoboBeetlePi/tools/robobeetle_pi_smoke.cpp
- Create: RoboBeetlePi/application/README.md

- [x] Step 1: Register robobeetle_pi_smoke only on Linux and link it to rbp2_onboard_application. **Executed: Windows target inventory omits it.**

- [x] Step 2: Implement a line-oriented CLI requiring one device path. Support link status, enable/disable/neutral mask, angle servo/cdeg, gait simple/cpg, motion forward/turn_left/turn_right/ascend/descend/stop/backward, and telemetry display. Parse numeric arguments with checked conversion, print Submitted plus sequence or the typed rejection, and print Pending hardware qualification for backward without sending bytes. Do not add daemonization, background threads, retry/backoff, automatic reconnect, batch motion, or serial-fd calls; bounded stdin-only readiness polling is allowed so the runtime remains serviced while input is incomplete. **Implemented as `da99d3f`/`b82a8cc`; explicit same-process `reopen` fix resolved in `e20dc46`.**

- [x] Step 3: Document that ACK/admission is not physical execution, no authoritative Pi robot state is stored, and Hardware Acceptance is PENDING USER VERIFICATION. Document that every action is explicitly entered by the operator. **Executed in README.**

- [ ] Step 4: Build the smoke target with -Wall -Wextra -Werror as part of the Linux command in Task 3. **Linux smoke build: NOT RUN; Windows CMake omission and portable codec build: PASS.**

- [x] Step 5: Commit. **Committed as `da99d3f`/`b82a8cc`; reopen fix committed in follow-up `e20dc46`.**

~~~powershell
git add RoboBeetlePi/CMakeLists.txt RoboBeetlePi/tools/robobeetle_pi_smoke.cpp RoboBeetlePi/application/README.md
git commit -m "feat: add Raspberry Pi actuator smoke tool"
~~~

### Task 5: Regression, scope audit, review handoff, and push

**Files:**
- Modify: RoboBeetlePi/application/README.md with final software evidence.
- Modify: docs/superpowers/specs/2026-09-17-raspberry-pi-application-actuator-design.md only if implementation reveals a factual contradiction.

- [x] Step 1: Run a fresh Windows portable warning build and CTest. **PASS: GCC 16.1.0, `-Wall -Wextra -Werror`, CTest 2/2, both direct executables.**

- [x] Step 2: Run the existing Firmware host regression using its established temporary build procedure. **PASS: 36 executables + 13 compile-contract objects + USART2 source/config contract; no Firmware diff.**

- [ ] Step 3: Run the complete Linux CMake/CTest command from Task 3 when WSL/Docker/Pi Linux is available. **NOT RUN: no usable WSL distribution, Docker, or Pi/Linux runner.**

- [x] Step 4: Execute the exact scope audit. **PASS: base-to-HEAD diff contains only the expected Slice 6 files; forbidden paths absent.**

~~~powershell
git status --short
git diff --check
git diff --stat 488760688a9735248337779e7e0e3038f1d57f3a..HEAD
git diff --name-only 488760688a9735248337779e7e0e3038f1d57f3a..HEAD
git -C D:\RoboBeetle status --short
git -C D:\RoboBeetle-worktrees\dsh-qt-ui status --short
~~~

Confirm no RoboBeetleConsole/, no RoboBeetleFirmware/, and no frozen Pi production layer files are listed. Do not alter either protected worktree.

- [x] Step 5: Dispatch spec-compliance review against the approved design, then code-quality review only after spec review has no Critical/Important gaps. Re-run reviews after valid fixes. **Internal task/final reviews passed; ChatGPT external review found one smoke-tool reopen blocker, resolved in `e20dc46`; awaiting re-review.**

- [x] Step 6: Push and stop. **Initial feature push was `5bb5941`; follow-up fix `e20dc46` was pushed normally without force.**

~~~powershell
git push -u origin codex/raspberry-pi-application-actuator
git rev-parse HEAD
git ls-remote origin refs/heads/codex/raspberry-pi-application-actuator
git status --short
~~~

Do not create a PR, merge, delete the feature branch, or start Slice 7. Final handoff is: "Implementation complete. PR not created. Merge not performed. Ready for ChatGPT external review."

## Current execution status

- [x] Windows portable RED/GREEN implementation work
- [x] Portable `-Wall -Wextra -Werror` build
- [x] Portable CTest and direct executables
- [x] Firmware host regression: 36 executables + 13 compile-contract objects + USART2 contract
- [x] Scope audit and feature branch push
- [ ] Linux native / PTY verification — **NOT RUN** (no usable WSL distribution, Docker, or Pi/Linux runner)
- [ ] Raspberry Pi native verification — **NOT RUN**
- [ ] Hardware Acceptance — **PENDING USER VERIFICATION**

ChatGPT external review found one smoke-tool same-process `reopen` blocker;
the fix is resolved in `e20dc46` and awaits re-review.

## Plan self-review

- All typed command payloads, invalid arguments, the Backward gate, and exact little-endian bytes have explicit test tasks.
- Leak, IMU, and Depth fixtures cover valid values, lengths, schemas, flags, signed fields, diagnostics, and invalid-domain invariants from current Firmware.
- PTY tests cover explicit open/resync/heartbeat, ACK outcomes, telemetry interleave, loss outcomes, reopen, and no replay.
- CMake boundaries keep POSIX headers out of portable codec targets and leave Slice 1–5 production files unchanged.
- No automatic hardware claim is included; unavailable Linux execution is explicitly reported as NOT RUN.
