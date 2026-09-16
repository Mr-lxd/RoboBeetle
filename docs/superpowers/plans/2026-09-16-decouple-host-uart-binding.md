# Host UART Binding Decoupling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the single STM32 host transport accept callbacks from the exact UART handle bound at initialization, regardless of its `Instance`, while preserving the current USART1 production path and transport semantics.

**Architecture:** Keep the existing singleton transport, queue ownership, interrupt-driven TX, RX re-arm, and recovery state machine. Replace only the callback admission predicate in `uart_transport_stm32.c`; use the existing host test fixture to prove pointer identity, non-USART1 bound instances, and safe rejection of NULL/non-bound handles. Leave production initialization, ISR routing, CubeMX files, and protocol behavior unchanged.

**Tech Stack:** C11 firmware sources, STM32F4 HAL, GCC host tests driven by `RoboBeetleFirmware/tests/run_host_tests.ps1`, CMake ARM firmware build.

---

### Task 1: Add identity regression tests and observe the red baseline

**Files:**
- Modify: `RoboBeetleFirmware/tests/uart_transport_stm32_tests.c`

- [ ] **Step 1: Relax only the host HAL stub's peripheral assumption.** In the test-local `HAL_UART_Receive_IT()` guard, keep the NULL-handle and one-byte-size checks but remove the `huart->Instance != USART1` condition. In `HAL_UART_Transmit_IT()`, likewise reject only a NULL handle and preserve all existing observation, status, and state behavior. This changes no production code or HAL implementation.

- [ ] **Step 2: Add a bound non-USART1 callback test.** Reset the fixture, set `uart1.Instance = USART3`, initialize with `uart_transport_stm32_init(&uart1)`, and assert that the existing RX delivery/re-arm and TX completion paths work. Then drive an ORE error and foreground re-arm, and perform an active-frame reinitialize with an inline abort-complete callback; assert the same IDLE/recovery and diagnostics outcomes as the USART1 path.

```c
static void test_bound_non_usart1_instance_is_accepted(void)
{
    const uint8_t tx[] = {0xB1U};
    uint8_t rx = 0U;
    uart_transport_stm32_diagnostics_t value;

    reset_mock();
    uart1.Instance = USART3;
    uart_transport_stm32_init(&uart1);

    deliver_byte(0xB2U);
    expect(uart_transport_stm32_pop(&rx),
           "bound non-USART1 handle should deliver RX bytes");
    expect(rx == 0xB2U,
           "bound non-USART1 RX should preserve the received byte");

    (void)uart_transport_stm32_enqueue(
        tx, (uint16_t)sizeof tx, UART_TX_MESSAGE_ACK);
    complete_tx();
    value = diagnostics();
    expect(value.completed_count[UART_TX_MESSAGE_ACK] == 1U,
           "bound non-USART1 TX completion should release its frame");

    uart1.RxState = HAL_UART_STATE_READY;
    uart1.ErrorCode = HAL_UART_ERROR_ORE;
    uart_transport_stm32_on_error(&uart1);
    expect(diagnostics().rx_needs_rearm,
           "bound non-USART1 error should request RX rearm");
    uart_transport_stm32_process();
    expect(diagnostics().rx_armed && !diagnostics().rx_needs_rearm,
           "bound non-USART1 error should rearm in foreground");

    (void)uart_transport_stm32_enqueue(
        tx, (uint16_t)sizeof tx, UART_TX_MESSAGE_ACK);
    abort_inline_callback_count = 1U;
    expect(uart_transport_stm32_reinitialize() == HAL_OK,
           "bound non-USART1 abort completion should finish recovery");
    expect(uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_IDLE,
           "bound non-USART1 abort recovery should return IDLE");
}
```

- [ ] **Step 3: Add the same-instance non-bound and NULL rejection test.** After binding `uart1`, set `uart3.Instance = uart1.Instance`, start a frame, enter the existing abort-pending state, snapshot diagnostics and call counters, then invoke all four callbacks with `&uart3` and `NULL`. Assert that state, queue ownership, receive/transmit/abort call counts, and diagnostics are unchanged; finish recovery through the bound handle.

```c
static void test_same_instance_non_bound_and_null_callbacks_are_ignored(void)
{
    const uint8_t tx[] = {0xB3U};
    uart_transport_stm32_diagnostics_t before;
    uart_transport_stm32_diagnostics_t after;
    uart_transport_stm32_state_t state_before;
    uint32_t receives_before;
    uint32_t transmits_before;
    uint32_t aborts_before;

    init_transport();
    uart3.Instance = uart1.Instance;
    (void)uart_transport_stm32_enqueue(
        tx, (uint16_t)sizeof tx, UART_TX_MESSAGE_ACK);
    abort_status = HAL_BUSY;
    expect(uart_transport_stm32_reinitialize() == HAL_BUSY,
           "bound transport should expose pending abort state");

    before = diagnostics();
    state_before = uart_transport_stm32_get_state();
    receives_before = receive_call_count;
    transmits_before = transmit_call_count;
    aborts_before = abort_call_count;

    uart_transport_stm32_on_rx_complete(&uart3);
    uart_transport_stm32_on_tx_complete(&uart3);
    uart_transport_stm32_on_error(&uart3);
    uart_transport_stm32_on_abort_transmit_complete(&uart3);
    uart_transport_stm32_on_rx_complete(NULL);
    uart_transport_stm32_on_tx_complete(NULL);
    uart_transport_stm32_on_error(NULL);
    uart_transport_stm32_on_abort_transmit_complete(NULL);

    after = diagnostics();
    expect(uart_transport_stm32_get_state() == state_before,
           "non-bound or NULL callbacks must not change recovery state");
    expect(receive_call_count == receives_before,
           "non-bound or NULL callbacks must not touch RX HAL state");
    expect(transmit_call_count == transmits_before,
           "non-bound or NULL callbacks must not touch TX ownership");
    expect(abort_call_count == aborts_before,
           "non-bound or NULL callbacks must not retry abort");
    expect(memcmp(&after, &before, sizeof after) == 0,
           "non-bound or NULL callbacks must not change diagnostics");

    abort_status = HAL_OK;
    abort_inline_callback_count = 1U;
    uart_transport_stm32_process();
    expect(uart_transport_stm32_get_state() == UART_TRANSPORT_STATE_IDLE,
           "bound callback should still complete the pending recovery");
}
```

- [ ] **Step 4: Register both tests in `main()`, run the focused transport test through the existing script, and confirm RED.**

Run:

```powershell
$wt = 'C:\Users\laixindong\.config\superpowers\worktrees\RoboBeetle\codex-decouple-host-uart-binding'
$build = 'C:\Users\laixindong\.config\superpowers\worktrees\RoboBeetle\codex-decouple-host-uart-binding-build-red'
powershell -NoProfile -ExecutionPolicy Bypass -File `
  "$wt\RoboBeetleFirmware\tests\run_host_tests.ps1" -BuildRoot $build
```

Expected: the script reaches `uart_transport_stm32_tests`, reports failures for the bound non-USART1 callback behavior under the old `Instance == USART1` predicate, and exits non-zero. Do not change production code before recording this failure.

- [ ] **Step 5: Commit the test-only red state.**

```powershell
git add -- RoboBeetleFirmware/tests/uart_transport_stm32_tests.c
git commit -m "test: cover bound host UART identity"
```

### Task 2: Remove USART1 identity coupling with the minimal production change

**Files:**
- Modify: `RoboBeetleFirmware/Core/Communication/uart_transport_stm32.c:87-93, 633-765`

- [ ] **Step 1: Replace the helper with pointer-only identity.** Rename `uart_transport_is_usart1()` to `uart_transport_matches_bound_uart()` and implement:

```c
static bool uart_transport_matches_bound_uart(
    const UART_HandleTypeDef *huart)
{
    return (huart != NULL) &&
           (uart_handle != NULL) &&
           (huart == uart_handle);
}
```

- [ ] **Step 2: Update all four callback guards.** Use the renamed helper in `uart_transport_stm32_on_rx_complete()`, `uart_transport_stm32_on_tx_complete()`, `uart_transport_stm32_on_abort_transmit_complete()`, and `uart_transport_stm32_on_error()`. Do not alter the callback bodies, queue transitions, diagnostics, HAL IT calls, or recovery state machine.

- [ ] **Step 3: Run the focused transport test and the full host suite.** The focused test must pass, including the non-USART1 bound and same-instance non-bound/NULL cases. Then run the existing `run_host_tests.ps1` with a fresh build directory and record the exit code and complete PASS summary.

### Task 3: Verify scope, compile contracts, and ARM firmware

**Files:**
- Inspect only: `RoboBeetleFirmware/Core/Src/main.c`, `RoboBeetleFirmware/Core/Src/stm32f4xx_it.c`, `RoboBeetleFirmware/RoboBeetleFirmware.ioc`, and `RoboBeetleFirmware/CMakeLists.txt`

- [ ] **Step 1: Confirm no forbidden production files changed.** Run `git diff --name-only 33ff57067aab6c99a594deced7143b64dffca7a6...HEAD` and verify only the approved design/plan/test/transport files appear. Search `uart_transport_stm32.c` and `.h` for `USART1`; any remaining match must be unrelated to callback identity (preferably none in the core files).

- [ ] **Step 2: Run the required focused regressions and compile-contract checks.** Re-run `uart_transport_stm32_tests`, `uart_tx_queue_tests`, the complete `run_host_tests.ps1` suite (which includes protocol, app, and clock compile contracts), and `git diff --check`.

- [ ] **Step 3: Configure and build the ARM firmware using the existing CMake project.** Use a fresh build directory under the isolated worktree and the repository's ARM toolchain file:

```powershell
$wt = 'C:\Users\laixindong\.config\superpowers\worktrees\RoboBeetle\codex-decouple-host-uart-binding'
$build = 'C:\Users\laixindong\.config\superpowers\worktrees\RoboBeetle\codex-decouple-host-uart-binding-build-arm'
cmake -S "$wt\RoboBeetleFirmware" -B $build -G Ninja `
  -DCMAKE_TOOLCHAIN_FILE="$wt\RoboBeetleFirmware\cmake\gcc-arm-none-eabi.cmake" `
  -DCMAKE_BUILD_TYPE=Debug
cmake --build $build --parallel
```

Expected: configure and build exit 0 and produce the firmware ELF. If the local ARM toolchain or generator is unavailable, record the exact failure as an unverified item rather than changing the build system.

- [ ] **Step 4: Commit the minimal production change only after green verification.**

```powershell
git add -- RoboBeetleFirmware/Core/Communication/uart_transport_stm32.c
git commit -m "refactor: decouple host transport from USART1"
```

### Task 4: Final evidence and handoff

- [ ] **Step 1: Re-run final verification after the production commit.** Capture the full host-suite result, focused transport/queue results, ARM build result, `git diff --stat 33ff57067aab6c99a594deced7143b64dffca7a6...HEAD`, `git status --short --branch`, and `git rev-parse HEAD`.

- [ ] **Step 2: Confirm original workspace preservation.** From `D:\RoboBeetle`, verify the detached HEAD and both pre-existing untracked files are unchanged; do not clean, reset, or commit them.

- [ ] **Step 3: Report without merging `main`.** Include baseline SHA, branch/worktree path, commit SHAs, modified-file list, per-file changes, why USART1 behavior is unchanged, every test/build command and result, diff stat, final status/HEAD, and remaining risks such as no physical re-validation of the current target image.
