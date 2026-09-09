# Leak Sensor Bring-up Design

## Scope

This stacked sensor bring-up branch adds only the first digital leak-detection
path after the PR #8 five-servo branch. The hardware input is the leak module's
digital output `D0` wired to STM32 `PA11`, powered from 3.3 V with a common
ground; the analog `A0` output remains disconnected. No IMU, depth parser,
telemetry, Qt UI, Protocol V2 field, or automatic actuator safety action is part
of this change.

The branch is intentionally stacked on the current PR #8 head because PR #8 is
not yet merged into `main`. Once PR #8 is merged, this branch can be rebased
onto the resulting `main` and its pull request base changed without changing
the sensor design.

## Proposed architecture

```text
leak module D0
    -> PA11 GPIO input (CubeMX/main GPIO initialization)
    -> leak_sensor_stm32 raw-level adapter (HAL-aware)
    -> leak_sensor pure-C state mapper/service (HAL-independent)
    -> app_main-owned internal state (polled once per main-loop pass)
```

`leak_sensor.c/.h` owns only the state vocabulary and explicit polarity mapping:

- GPIO high maps to `LEAK_SENSOR_STATE_DRY`.
- GPIO low maps to `LEAK_SENSOR_STATE_WET`.
- Initialization starts at `LEAK_SENSOR_STATE_UNKNOWN` until the first poll.
- No debounce, persistence timer, latch, interrupt, alarm, or side effect is
  introduced.

`leak_sensor_stm32.c/.h` is a thin HAL adapter that stores a GPIO port/pin and
returns a boolean raw level from `HAL_GPIO_ReadPin`. It does not include leak
policy, safety, Protocol, Servo, or Qt code. The adapter is kept separate so
the pure mapper can be host-tested without STM32 headers.

`app_main` owns one mapper state and one adapter instance. `app_main_init`
receives the already-configured port/pin from `main.c`; `app_main_process`
polls the adapter and updates the mapper before continuing the existing UART,
Protocol, and Safety processing. The leak state is internal only and is not
reported on the wire or used to disable servos.

## GPIO and electrical assumptions

Before this branch, the active `.ioc` had no PA11 assignment, and no source or
HAL MSP code used PA11. A source-level resource scan confirms PA11 is free with
respect to the current USART1, TIM3/TIM4, SWD, and GPIO assignments. The branch
adds PA11 as a plain `GPIO_MODE_INPUT` with `GPIO_NOPULL`, using the existing
`MX_GPIO_Init` clock/configuration path. The
module documentation indicates a digital HIGH/LOW output, but its output-stage
type is not fully established in the repository; therefore `GPIO_NOPULL` is a
bring-up assumption, not a verified electrical conclusion. If physical tests
show a floating or unstable D0, pull configuration is a separate hardware-led
follow-up.

## Verification boundary

Host tests prove only the pure polarity/state logic. ARM target build and
program verification are reported separately when actually run. PA11 voltage
levels, wet/dry polarity, response/recovery delay, and chatter require the
user's physical test and remain pending until then. No leak Hardware Verified
claim is made by this design.

## Deferred work

Later, separately reviewed work may add EXTI or debounce/persistence, a latched
alarm, Safety Supervisor integration, Protocol V2 sensor telemetry, Qt display,
and a defined recovery policy. The bring-up order remains leak detection,
JY901S IMU, depth/sensor board, sensor telemetry, then Qt visualization because
the leak input is the smallest digital-input baseline.

