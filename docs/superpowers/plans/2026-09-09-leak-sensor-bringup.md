# Leak Sensor Bring-up Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Add a minimal, polled, digital leak-sensor D0 path on STM32 PA11 with pure-C polarity/state tests, without changing Protocol, Servo, Safety, or Qt behavior.

**Architecture:** A HAL-independent `leak_sensor` mapper stores `UNKNOWN`, `DRY`, or `WET` and maps HIGH to DRY and LOW to WET. A two-field HAL adapter reads the configured GPIO port/pin; `app_main` owns both objects and polls once per loop. CubeMX/main configure PA11 as an input with `GPIO_NOPULL`, explicitly documented as a bring-up assumption.

**Tech Stack:** C11 Firmware, STM32F4 HAL/CubeMX/CMake, existing manual host GCC regressions, Markdown handoff/engineering docs, and the existing Ninja ARM build when the toolchain is available.

---

### Task 1: Add the pure-C leak-state regression first

**Files:**
- Create: `RoboBeetleFirmware/tests/leak_sensor_tests.c`
- Create (after the red run): `RoboBeetleFirmware/Core/Sensors/leak_sensor.h`
- Create (after the red run): `RoboBeetleFirmware/Core/Sensors/leak_sensor.c`

- [x] **Step 1: Write the failing test.**

Create `tests/leak_sensor_tests.c` with an `expect` helper and these concrete
checks: the initial state is `LEAK_SENSOR_STATE_UNKNOWN`, HIGH maps to
`LEAK_SENSOR_STATE_DRY`, LOW maps to `LEAK_SENSOR_STATE_WET`, and updating a
state object stores each result.

- [x] **Step 2: Run the red test.**

Run from the Firmware directory:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -I Core/Sensors tests/leak_sensor_tests.c Core/Sensors/leak_sensor.c -o build/leak_sensor_tests.exe
```

Expected result before the implementation exists: compilation fails because
`Core/Sensors/leak_sensor.h` and `Core/Sensors/leak_sensor.c` do not exist.

- [x] **Step 3: Implement the minimal pure-C API.**

`leak_sensor.h` defines:

```c
typedef enum {
    LEAK_SENSOR_STATE_UNKNOWN = 0,
    LEAK_SENSOR_STATE_DRY,
    LEAK_SENSOR_STATE_WET
} leak_sensor_state_t;

typedef struct {
    leak_sensor_state_t state;
} leak_sensor_t;

void leak_sensor_init(leak_sensor_t *sensor);
leak_sensor_state_t leak_sensor_state_from_gpio_level(bool gpio_high);
void leak_sensor_update_from_gpio_level(
    leak_sensor_t *sensor,
    bool gpio_high);
leak_sensor_state_t leak_sensor_state(const leak_sensor_t *sensor);
```

`leak_sensor.c` initializes to UNKNOWN, returns DRY for true and WET for false,
and performs no filtering or side effect.

- [x] **Step 4: Run the pure test green.**

Run the same GCC command and execute `build/leak_sensor_tests.exe`; expect the
success line `All firmware leak sensor tests passed`.

- [x] **Step 5: Commit the isolated pure-C cycle.**

```powershell
git add RoboBeetleFirmware/Core/Sensors/leak_sensor.h RoboBeetleFirmware/Core/Sensors/leak_sensor.c RoboBeetleFirmware/tests/leak_sensor_tests.c
git commit -m "feat(firmware): add pure-C leak state mapper"
```

### Task 2: Add the thin STM32 reader and wire polling into app_main

**Files:**
- Create: `RoboBeetleFirmware/Core/Sensors/leak_sensor_stm32.h`
- Create: `RoboBeetleFirmware/Core/Sensors/leak_sensor_stm32.c`
- Modify: `RoboBeetleFirmware/Core/App/app_main.h`
- Modify: `RoboBeetleFirmware/Core/App/app_main.c`
- Modify: `RoboBeetleFirmware/Core/Src/main.c`
- Modify: `RoboBeetleFirmware/Core/Inc/main.h`

- [x] **Step 1: Define the adapter boundary.**

Use this HAL-aware structure and API:

```c
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} leak_sensor_stm32_t;

void leak_sensor_stm32_init(
    leak_sensor_stm32_t *reader,
    GPIO_TypeDef *port,
    uint16_t pin);
bool leak_sensor_stm32_read_level(
    const leak_sensor_stm32_t *reader);
```

The implementation returns
`HAL_GPIO_ReadPin(reader->port, reader->pin) == GPIO_PIN_SET` and contains no
polarity or safety policy.

- [x] **Step 2: Wire the existing app loop.**

Add a static `leak_sensor_t` and `leak_sensor_stm32_t` to `app_main.c`.
Extend `app_main_init` with `GPIO_TypeDef *leak_gpio_port, uint16_t
leak_gpio_pin`, initialize both objects, and at the start of
`app_main_process` call `leak_sensor_update_from_gpio_level` with the
adapter's raw result. Update the single `main.c` call to pass
`LEAK_SENSOR_GPIO_Port` and `LEAK_SENSOR_Pin`.

- [x] **Step 3: Configure PA11 in the existing CubeMX path.**

Add `LEAK_SENSOR_Pin`/`LEAK_SENSOR_GPIO_Port` defines to `main.h`, add PA11
as `GPIO_Input` labelled `LEAK_SENSOR` to `RoboBeetleFirmware.ioc`, and
extend `MX_GPIO_Init` with:

```c
GPIO_InitStruct.Pin = LEAK_SENSOR_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(LEAK_SENSOR_GPIO_Port, &GPIO_InitStruct);
```

Keep the existing GPIOA clock enable and do not add EXTI/NVIC code.

- [x] **Step 4: Run adapter/app syntax checks.**

Run the existing host pure-C tests and the repository's ARM configure/build if
the configured `arm-none-eabi-gcc` toolchain is available. Use the existing
CubeMX CMake files; only the user-maintained top-level CMake source/include
lists are updated in Task 3.

### Task 3: Register the new module in user-maintained CMake and run regressions

**Files:**
- Modify: `RoboBeetleFirmware/CMakeLists.txt`
- Test: all existing `RoboBeetleFirmware/tests/*.c` manual regressions

- [x] **Step 1: Add source/include entries.**

Append `Core/Sensors/leak_sensor.c` and
`Core/Sensors/leak_sensor_stm32.c` to `target_sources`, and add
`\${CMAKE_CURRENT_SOURCE_DIR}/Core/Sensors` to
`target_include_directories`. Do not modify
`cmake/stm32cubemx/CMakeLists.txt`.

- [x] **Step 2: Run the complete available Firmware validation.**

Run the new leak test plus the existing seven pure-C regressions with their
documented `-Wall -Wextra -Werror` commands, then run:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Record ARM Build as PASS only if these commands complete successfully; if the
toolchain is absent or fails for an unrelated environment reason, report it as
Pending/blocked rather than changing production behavior.

- [x] **Step 3: Run style and diff checks.**

```powershell
git diff --check
git status --short --branch
git diff --stat
```

### Task 4: Synchronize canonical documentation

**Files:**
- Modify: `RoboBeetleFirmware/README.md`
- Modify: `ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md`
- Modify: `docs/engineering-lessons.md`

- [x] **Step 1: Record the implemented boundary.**

Document D0→PA11, 3.3 V/common GND, A0 unused, polling, HIGH→Dry/LOW→Wet,
`GPIO_NOPULL` as an unverified output-stage assumption, and the fact that no
Safety/Protocol/Qt/Servo action is connected.

- [x] **Step 2: Record the exact hardware procedure and evidence taxonomy.**

Include the dry/wet voltage and logic measurements, response/recovery delays,
and chatter fields. Mark PA11 polarity, wet/dry detection, Program Verify, and
leak Hardware Verified as Pending until the user supplies physical results.

- [x] **Step 3: Record sequence and preserved Depth facts.**

Keep the PR #8 approximately 1100–2500 μs bench observation, below-1100 μs
ACK-timeout caution, deferred final mechanical endpoints/center/angle mapping,
and provisional 1500 μs wording. State the planned sequence leak → JY901S IMU
→ depth board → Protocol telemetry → Qt visualization.

### Task 5: Final review and publication

**Files:**
- Review all changed files and branch metadata

- [x] **Step 1: Re-run focused and full tests after documentation changes.**

Run the leak test, all existing Firmware pure-C tests, Qt CTest if its
dependencies are available, ARM build if available, and `git diff --check`.

- [x] **Step 2: Inspect the final diff for scope.**

Confirm no Protocol V2, Servo, Safety behavior, Qt source, IMU/depth code,
ADC/A0, EXTI, debounce, or generated CubeMX CMake changes are present.

- [x] **Step 3: Commit and publish without merging.**

Use three focused commits: `feat(firmware): add pure-C leak state mapper` for
the mapper and its test, `feat(firmware): poll leak D0 on PA11` for the HAL
adapter/app/GPIO/CMake integration, and `docs: record leak sensor bring-up
status` for the handoff, README, lessons, and process documents. Push
`feature/sensor-bringup`, and open a PR whose base is
`feature/five-servo-bringup` while PR #8 remains unmerged. Do not merge either
PR. If GitHub connectivity is unavailable, retain local commits and report the
exact push/PR blocker.

## Self-review

- PA11 conflict was checked against `.ioc`, `main.c`, HAL MSP, peripherals, and
  sensor/Safety sources; none was found.
- The plan covers pure-C polarity/state tests before production logic, the
  existing GPIO initialization path, top-level CMake registration, all required
  validation categories, documentation, and the stacked-branch relationship.
- No task changes Protocol V2, Servo behavior/calibration, Safety actions, Qt,
  IMU, depth, ADC, EXTI, debounce, or generated CubeMX CMake.




