# PR #8 Design: Five-Servo End-to-End Bring-Up

Date: 2026-09-08
Base: `main` at `54a8631194645a83e65afa21660be0adb38e27de`
Branch: `feature/five-servo-bringup`

## Scope and invariants

PR #8 brings the five installed servo channels through the existing Protocol V2,
Firmware service/driver, and Qt Console without changing the wire format or the
APC220 scheduler. It is limited to servo descriptors, channel/timer wiring,
per-servo validation/UI, regression tests, and the associated documentation.

The following remain frozen: Protocol V2 magic/version/CRC/COBS/message IDs and
payloads; APC220 stop-and-wait, heartbeat admission, fail-closed, stale-command
purge, Disable priority, and reconnect semantics; Firmware Safety Supervisor
timeout semantics; UART/USART1 settings; Servo PWM frequency; and all existing
hardware-verified Servo1/Set Angle behavior unless it is explicitly superseded
by the final semantic mapping below.

No CPG, Raspberry Pi, sensors, telemetry, Protocol V2 extension, or unrelated
architecture refactor is included.

## Frozen semantic mapping

The numeric IDs and masks are an API contract and will be asserted at compile
time and by both Firmware and Console tests:

| ID | Semantic name | Display name | Servo | Final timer/channel | Mask |
|---:|---|---|---|---|---:|
| 0 | `FrontRight` | Front Right / 前足右 | SAVOX SW-0250MG+ | TIM3_CH1 / PA6 | `0x0001` |
| 1 | `FrontLeft` | Front Left / 前足左 | SAVOX SW-0250MG+ | TIM3_CH2 / PA7 | `0x0002` |
| 2 | `FrontAxis` | Front Axis / 升潜前足轴 | HDKJ S3150D | TIM3_CH3 / PB0 | `0x0004` |
| 3 | `RearRight` | Rear Right / 后足右 | GDW IPX896HV | TIM4_CH1 / PD12 | `0x0008` |
| 4 | `RearLeft` | Rear Left / 后足左 | GDW IPX896HV | TIM4_CH2 / PD13 | `0x0010` |

The aggregate supported mask is fixed at `0x001F`. Firmware uses semantic
`FRONT_RIGHT`, `FRONT_LEFT`, `FRONT_AXIS`, `REAR_RIGHT`, and `REAR_LEFT` names.
Qt exposes the same semantic names. `ServoId::Servo1` remains only as a
deprecated/historical source-compatibility alias of `FrontRight`; there is no
`Servo2` alias.

The old Servo1 hardware bring-up mapping of RearLeft to TIM3_CH1/PA6 is retained
only as Historical Reference. In PR #8 PA6 is officially FrontRight and RearLeft
is PD13/TIM4_CH2.

## Firmware architecture

### Descriptor and calibration data

`Core/Servo/servo_descriptor.c/.h` will own one five-entry, immutable metadata
table. Each entry contains:

- semantic numeric ID and mask;
- supported and angle-supported flags;
- electrical calibration (min/neutral/max pulse and angle limits);
- stricter mechanical command pulse/angle envelope;
- symbolic timer selection (TIM3 or TIM4) and HAL channel.

The table is pure C and has no HAL dependency. It is the Firmware source of
truth for validation and is exposed through bounded lookup/count APIs for tests.

The SAVOX calibration is provisional 1000/1500/2000 us and -50/+50 degrees;
its mechanical command envelope is 1050/1500/1950 us and -45/+45 degrees.
The GDW calibration remains the historical 520/1520/2520 us and -90/+90
degrees, while normal mechanical commands are limited to 1020/1520/2020 us
and -45/+45 degrees. FrontAxis is PWM-only with a provisional 1450/1500/1550
us clamp and no angle capability.

`servo_calibration` becomes a generic mapping helper over a supplied
calibration record; it contains no Servo1-only global state.

### Service and driver

`servo_service` obtains descriptors by ID, rejects zero/unsupported masks and
invalid IDs, requires the relevant enabled bit for motion commands, enforces
mechanical command limits before electrical limits, and iterates descriptors
for multi-servo Enable/Neutral/Disable/Disable All. FrontAxis SetAngle returns
UnsupportedServo. Invalid IDs never reach driver operations. Enable/Disable
state is a five-bit mask and Disable All stops every currently enabled channel.

`servo_driver_stm32` owns a small runtime binding table derived from the pure-C
descriptor table. It resolves each symbolic timer/channel to the `TIM_HandleTypeDef`
provided by `app_main_init`, then performs HAL CCR writes and PWM start/stop by
descriptor lookup. No new TX/RX, DMA, RTOS, or safety policy is introduced.

### Application and CubeMX wiring

`app_main_init` accepts both TIM3 and TIM4 handles and binds them to the driver.
`main.c` initializes the existing peripherals plus TIM4 and passes both handles;
its responsibility remains CubeMX/HAL startup, peripheral init, app delegation,
the thin UART callback, and error/assert handling.

The `.ioc`, generated `main.c` initialization, and HAL MSP post-init are kept
consistent: TIM3 CH1/CH2/CH3 on PA6/PA7/PB0 and TIM4 CH1/CH2 on PD12/PD13,
all approximately 333 Hz with the existing 1 us tick/3003 us period. USART1
PA9/PA10, SWD, DBG_LED, APC220, and Safety wiring are unchanged. The
user-maintained Firmware CMake file adds only any new descriptor source/header;
the generated CubeMX CMake is not edited.

## Console architecture

`RobotCommand` defines the semantic enum and a five-entry descriptor collection
with display labels, supported/angle-supported capability, electrical calibration,
and mechanical command ranges. `RobotControllerConfig` defaults to the five-bit
supported mask. Controller validation and enabled/pending signal loops iterate
the descriptor collection rather than assuming two indices. Existing APC220
scheduler state transitions are preserved; only the actuator IDs and per-servo
ranges become descriptor-driven.

`MainWindow` builds five panels by iterating the descriptor collection. Each card
has independent Enable/Disable, Neutral, PWM, Apply PWM, angle input, Set Angle,
and status controls. FrontAxis visibly reports Calibration Pending / PWM
Bring-up Only and has Set Angle disabled. All other cards use their descriptor
limits. The old Servo2 Unsupported/Planned card is removed. The global Disable
All action remains; there is no Enable All action. No commanded/actual telemetry
model is introduced.

## Protocol and safety behavior

Protocol V2 payloads are unchanged. Enable/Disable/Neutral continue to carry a
uint16 mask; Set PWM and Set Angle continue to carry count=1, servo ID, and
little-endian value. Dispatcher routing remains generic and delegates all
per-ID/capability checks to `servo_service`.

All five enabled channels participate in the existing Safety Supervisor
fail-close path. A watchdog transition stops all enabled PWM channels and
invalidates the duplicate-action cache exactly as before. The Console APC220
scheduler keeps one ACK-requiring request in flight, heartbeat coalescing and
490 ms host-side admission, stale actuator purge, Disable priority, fail-closed
liveness recovery, and fresh Enable+ACK re-arm requirements.

## Verification strategy

Tests are written/updated before production changes where practical and must
show the expected failure before the implementation is completed. Firmware
pure-C tests will cover:

- compile-time and runtime ID/mask/table count invariants;
- every timer/channel abstraction and invalid-ID fail-closed behavior;
- independent five-bit Enable/Disable state;
- SAVOX, GDW, and FrontAxis calibration/range distinctions;
- FrontAxis SetAngle rejection;
- Neutral and Disable All across five channels;
- Safety/dispatcher routing for IDs 0--4, duplicate cache, and watchdog stop.

Console tests will cover the five semantic descriptors and supported mask,
per-servo PWM/angle/mechanical ranges, FrontAxis angle rejection, five-servo
enable/disable/Neutral/Disable All behavior, Protocol payload IDs, APC220
scheduler/fail-closed/reconnect regressions, and the existing DirectUart,
protocol, and controller suites.

Verification commands will include a fresh Firmware configure/build, all
available Firmware pure-C regressions and Protocol golden vectors, a fresh Qt
configure/build, `protocol_tests`, `robot_controller_tests`, and `ctest`. Target
STM32CubeIDE/download and physical five-servo regression remain a user hardware
acceptance gate; documentation will mark them pending until performed.

## Documentation updates

The implementation will update only the relevant sections of:

- `RoboBeetleFirmware/README.md`;
- `RoboBeetleConsole/README.md`;
- `RoboBeetleConsole/docs/protocol.md`;
- `docs/engineering-lessons.md`;
- `ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md`.

Documentation will state the final semantic/timer mapping, shared TIM3/TIM4
frequency, SAVOX datasheet-compatible 333 Hz context, S3150D compatibility and
mechanical calibration limitations, the historical PA6 mapping, and the
distinction between prior Hardware Verified Servo1 behavior and pending PR #8
five-servo hardware regression.

## Out of scope / rollback boundaries

No Protocol V2 format or message-ID changes, firmware safety timeout changes,
APC220 timing changes, calibration expansion beyond the explicitly frozen
records, servo direction inversion guesses, CPG/Pi/sensor work, or broad UI/state
architecture refactor is allowed. If CubeMX generation changes unrelated files,
the change will be reduced to the required timer/pin initialization and reviewed
against the `.ioc`; unrelated generated churn is not accepted.
