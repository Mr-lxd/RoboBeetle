# USART2 Raspberry Pi Primary Host Link

**Status:** Approved design, pending implementation

**Baseline:** `016436ea7e04a370de8a51c5679593ef0f272375` (the squash merge of PR #20),
whose parent is `33ff57067aab6c99a594deced7143b64dffca7a6`.

**Feature branch:** `codex/usart2-pi-primary-host`

## Goal

Add the STM32F407 USART2 hardware path used by the Raspberry Pi primary host:
PA2 as `USART2_TX`, PA3 as `USART2_RX`, 115200 baud, 8-N-1, TX/RX mode,
no hardware flow control, oversampling 16, and USART2 interrupt priority 0/0.
Change only the production binding passed to `app_main_init()` so the existing
single `uart_transport_stm32` instance uses `&huart2`.

USART1 remains initialized at 9600 on PA9/PA10 with its existing IRQ for the
APC220/legacy hardware path, but it is no longer the active Protocol V2 host
transport. More precisely, `uart_transport_stm32` binds only `&huart2`; USART1
does not arm an RX receive for that transport, and USART1 bytes do not enter
Protocol V2, create Heartbeats, affect Safety, or gain control authority. The
retained USART1/APC220 hardware must not be described as an already-available
backup control link. USART3 and USART6 remain unchanged.

## Architecture and data flow

1. CubeMX source configuration records USART2, PA2/PA3, and its NVIC entry in
   `RoboBeetleFirmware/RoboBeetleFirmware.ioc`.
2. Cube-style generated startup code declares `huart2`, initializes USART1,
   USART2, USART3, and USART6 in that order, and passes `&huart2` as the first
   argument to the unchanged `app_main_init()` API.
3. `HAL_UART_MspInit()` configures USART2's APB1 clock, GPIOA PA2/PA3 AF7 pins,
   and `USART2_IRQn`; its deinit branch reverses those resources. Existing
   USART1/3/6 branches are retained.
4. `USART2_IRQHandler()` dispatches to `HAL_UART_IRQHandler(&huart2)`. The
   existing USART1 handler remains present for the retained peripheral.
5. The existing unified `HAL_UART_RxCpltCallback`, `HAL_UART_TxCpltCallback`,
   `HAL_UART_ErrorCallback`, and `HAL_UART_AbortTransmitCpltCallback`
   dispatchers remain the only application callback path. No USART2-specific
   callback branch is added. `uart_transport_stm32` is not refactored; PR #20's
   exact `UART_HandleTypeDef *` binding predicate continues to decide which
   callbacks are accepted, so callbacks from the retained but unbound USART1
   handle are ignored by the active transport.

No second transport instance, failover, arbitration, APC command path, or
Raspberry Pi application code is introduced.

## Source/configuration changes

Production changes are limited to:

- `RoboBeetleFirmware/RoboBeetleFirmware.ioc`: add the USART2 IP, PA2/PA3
  signals, `Mcu.IP*`/`Mcu.Pin*` bookkeeping, function-list entry, and
  `NVIC.USART2_IRQn`; preserve the existing USART1/3/6 values and avoid pin
  conflicts.
- `RoboBeetleFirmware/Core/Src/main.c`: add `huart2`,
  `MX_USART2_UART_Init()`, the USART2 init call, and the `app_main_init()` first
  argument switch. All other UART fields and the application API remain stable.
- `RoboBeetleFirmware/Core/Src/stm32f4xx_hal_msp.c`: add only the USART2
  init/deinit resource branches described above.
- `RoboBeetleFirmware/Core/Src/stm32f4xx_it.c` and
  `RoboBeetleFirmware/Core/Inc/stm32f4xx_it.h`: add the USART2 handle declaration,
  handler, and prototype while retaining the USART1 handler/prototype.

The active README statements that explicitly identify USART1 as the current
primary host may receive the smallest wording correction needed to describe
USART2. Historical APC220/USART1 evidence is retained rather than rewritten.

## Error handling and safety

Protocol V2 framing, Heartbeat liveness, the 500 ms SafetySupervisor timeout,
STOP/disable behavior, recovery semantics, TX queue behavior, Motion, Servo,
and sensor transports are unchanged. HAL initialization failures continue to
call the existing `Error_Handler()`. No host failover or authority-selection
policy is added.

## Test-first verification

The implementation will follow a red-green cycle:

1. Run the clean baseline host gate from the feature worktree.
2. Add a focused source/`.ioc` compile-contract that asserts the USART2 fields,
   generated-code structure, binding switch, retained USART1 settings/IRQ, and
   unchanged USART3/6 settings. Run it against the baseline and record the
   expected RED before changing production files.
3. Implement the minimal production/configuration diff, then run the focused
   transport and TX queue tests, the new contract, and the complete host gate.
4. Audit source/configuration parity (including PA2/PA3 AF7, USART2 clock/NVIC,
   IRQ-to-handle dispatch, init order, and the absence of unrelated peripheral
   or protocol changes), run `git diff --check`, and inspect the diff/stat.
5. ARM firmware build/flash is outside this workstation's responsibility and
   will be reported as `PENDING USER TARGET VERIFICATION`; no CMake toolchain or
   machine-specific path is changed.

Acceptance requires all host executables and existing compile-contract checks
to pass, the new USART2 contract to pass, a clean diff check, and no changes to
the excluded Protocol/Safety/Motion/Servo/USART3/USART6 surfaces.

The new contract must explicitly assert: USART2/PA2/PA3/115200/NVIC `.ioc`
values; `huart2` and `MX_USART2_UART_Init()` fields; an exact
`app_main_init(&huart2, ...)` first argument; retained USART1 initialization,
9600 settings, PA9/PA10 MSP, and IRQ; USART2 PA2/PA3 AF7, APB1 clock, and IRQ
0/0; the exact `USART2_IRQHandler()` HAL call; unchanged USART3/USART6
configuration; and the absence of edits to `uart_transport_stm32.c` or the
unified HAL callback dispatcher.

## Alternatives considered

- **CubeMX CLI regeneration:** not selected because the CLI is unavailable in
  this environment and regeneration could introduce unrelated generated-file
  churn that is harder to audit.
- **Generic multi-UART transport/configuration abstraction:** not selected;
  it would expand this single-instance binding change into a new architecture.
