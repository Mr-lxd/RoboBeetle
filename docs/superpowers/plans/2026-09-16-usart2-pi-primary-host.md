# USART2 Raspberry Pi Primary Host Link Implementation Plan

> For agentic workers: REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

Goal: Add the STM32F407 USART2 PA2/PA3 115200 primary-host path and bind the existing single host transport to &huart2 while preserving the USART1/APC220 peripheral and all existing application semantics.

Architecture: Extend the CubeMX .ioc source configuration and its generated-style main.c, MSP, and IRQ files with one USART2 instance. Keep the unified HAL callback dispatcher and PR #20 exact-handle transport predicate unchanged; the only application binding change is the first app_main_init argument. Add a source/configuration contract so the .ioc, generated code, retained USART1 path, and unchanged USART3/6 settings cannot drift.

Tech Stack: STM32F407VET6 C11 firmware, STM32F4 HAL/CMSIS, CubeMX .ioc text configuration, PowerShell host contract tests, GCC host tests, CMake/Ninja target source audit.

---

## File map

- Create: RoboBeetleFirmware/tests/usart2_host_binding_contract_tests.ps1
- Modify: RoboBeetleFirmware/tests/run_host_tests.ps1
- Modify: RoboBeetleFirmware/RoboBeetleFirmware.ioc
- Modify: RoboBeetleFirmware/Core/Src/main.c
- Modify: RoboBeetleFirmware/Core/Src/stm32f4xx_hal_msp.c
- Modify: RoboBeetleFirmware/Core/Src/stm32f4xx_it.c
- Modify: RoboBeetleFirmware/Core/Inc/stm32f4xx_it.h
- Modify only stale active wording: RoboBeetleFirmware/README.md

Do not modify RoboBeetleFirmware/Core/Communication/uart_transport_stm32.c, the unified HAL callback block in main.c, Protocol/Safety/Motion/Servo sources, or USART3/USART6 source/configuration.

### Task 1: Add and observe the failing contract

Files:
- Create RoboBeetleFirmware/tests/usart2_host_binding_contract_tests.ps1
- Test RoboBeetleFirmware/tests/usart2_host_binding_contract_tests.ps1

- [ ] Step 1: Write the contract before production edits.

The script reads main.c, stm32f4xx_hal_msp.c, stm32f4xx_it.c, stm32f4xx_it.h, RoboBeetleFirmware.ioc, and uart_transport_stm32.c. Normalize CRLF with a replacement of the regex \r\n? by [char]10. Define Has, NotHas, Ioc, and InOrder helpers. Ioc must use a multiline regex anchored to exactly one key and compare the value ordinally.

Use these helper implementations so a malformed test reports a contract
failure rather than a PowerShell interpolation error:

    function Has([string]$text, [string]$needle, [string]$label) {
        if (-not $text.Contains($needle)) {
            throw ('Missing ' + $label + ': ' + $needle)
        }
    }
    function NotHas([string]$text, [string]$needle, [string]$label) {
        if ($text.Contains($needle)) {
            throw ('Unexpected ' + $label + ': ' + $needle)
        }
    }
    function Ioc([string]$key, [string]$value) {
        $matches = [regex]::Matches($ioc, '(?m)^' + [regex]::Escape($key) + '=([^\n]*)$')
        if ($matches.Count -ne 1) { throw ('Expected one .ioc key: ' + $key) }
        if ($matches[0].Groups[1].Value -cne $value) {
            throw ('.ioc mismatch for ' + $key)
        }
    }
    function InOrder([string]$text, [string[]]$needles, [string]$label) {
        $last = -1
        foreach ($needle in $needles) {
            $at = $text.IndexOf($needle)
            if ($at -lt 0 -or $at -le $last) {
                throw ('Order failure (' + $label + '): ' + $needle)
            }
            $last = $at
        }
    }

Assert these exact .ioc values:

    Mcu.IP5=USART1
    Mcu.IP6=USART2
    Mcu.IP7=USART3
    Mcu.IP8=USART6
    Mcu.IPNb=9
    Mcu.Pin16=PA2
    Mcu.Pin17=PA3
    Mcu.PinsNb=18
    PA2.Mode=Asynchronous
    PA2.Signal=USART2_TX
    PA3.Mode=Asynchronous
    PA3.Signal=USART2_RX
    NVIC.USART2_IRQn=true\:0\:0\:false\:false\:true\:true\:true\:true
    USART1.BaudRate=9600
    USART1.IPParameters=VirtualMode,BaudRate
    USART1.VirtualMode=VM_ASYNC
    USART2.BaudRate=115200
    USART2.IPParameters=VirtualMode,BaudRate
    USART2.VirtualMode=VM_ASYNC
    USART3.BaudRate=9600
    USART3.IPParameters=VirtualMode,BaudRate
    USART3.VirtualMode=VM_ASYNC
    USART6.BaudRate=115200
    USART6.IPParameters=VirtualMode,BaudRate
    USART6.VirtualMode=VM_ASYNC
    ProjectManager.functionlistsort=1-SystemClock_Config-RCC-false-HAL-false,2-MX_GPIO_Init-GPIO-false-HAL-true,3-MX_TIM3_Init-TIM3-false-HAL-true,4-MX_TIM4_Init-TIM4-false-HAL-true,5-MX_USART1_UART_Init-USART1-false-HAL-true,6-MX_USART2_UART_Init-USART2-false-HAL-true,7-MX_USART3_UART_Init-USART3-false-HAL-true,8-MX_USART6_UART_Init-USART6-false-HAL-true

Assert that main.c contains huart2 declaration/prototype, the exact USART2 fields Instance=USART2, BaudRate=115200, WordLength=8B, StopBits=1, Parity=None, Mode=TX_RX, HwFlowCtl=None, OverSampling=16, and HAL_UART_Init(&huart2). Assert existing huart1=9600, huart3=9600, and huart6=115200 fields. Assert the ordered calls MX_USART1_UART_Init, MX_USART2_UART_Init, MX_USART3_UART_Init, MX_USART6_UART_Init. Assert the exact normalized call prefix app_main_init followed by four spaces and &huart2, and reject the equivalent &huart1 prefix.

Assert MSP content for retained USART1 (Instance branch, PA9|PA10, AF7_USART1, clock, priority/enable and matching deinit) and for USART2 (Instance branch, USART2 clock, GPIOA clock, PA2|PA3, AF_PP, NOPULL, VERY_HIGH, AF7_USART2, priority/enable and matching deinit). Assert stm32f4xx_it.c has the huart2 extern and the generated-style USART2_IRQHandler block calling HAL_UART_IRQHandler(&huart2); retain the USART1 handler call and both header prototypes.

Extract the USER CODE BEGIN 4 to USER CODE END 4 block from main.c. Assert all existing unified RX/TX/abort/error dispatcher calls are present and reject both USART2 and huart2 in that block. Reject USART2 in uart_transport_stm32.c. End with Write-Host PASS usart2_host_binding_contract_tests.

- [ ] Step 2: Run the new script against the unchanged baseline.

    powershell -NoProfile -ExecutionPolicy Bypass -File .\RoboBeetleFirmware\tests\usart2_host_binding_contract_tests.ps1

Expected: nonzero exit with a missing .ioc key or huart2 assertion, proving RED is caused by the absent USART2 path. Do not edit production files before this failure is recorded.

### Task 2: Add the USART2 .ioc configuration

File: RoboBeetleFirmware/RoboBeetleFirmware.ioc

- [ ] Step 1: Add the ordered IP and pin bookkeeping.

Set the USART IP block and counts to Mcu.IP5=USART1, Mcu.IP6=USART2, Mcu.IP7=USART3, Mcu.IP8=USART6, Mcu.IPNb=9. Append Mcu.Pin16=PA2 and Mcu.Pin17=PA3 and set Mcu.PinsNb=18. Add PA2.Mode=Asynchronous, PA2.Signal=USART2_TX, PA3.Mode=Asynchronous, PA3.Signal=USART2_RX, the exact NVIC.USART2_IRQn value from Task 1, USART2.BaudRate=115200, USART2.IPParameters=VirtualMode,BaudRate, and USART2.VirtualMode=VM_ASYNC. Set ProjectManager.functionlistsort to the exact USART1 → USART2 → USART3 → USART6 value above. Preserve every existing USART1, USART3, USART6, RCC, timer, and GPIO key.

- [ ] Step 2: Re-run the contract.

It must pass all .ioc assertions and fail on the still-missing huart2 source assertions. Do not add unrelated CubeMX regeneration output.

### Task 3: Add huart2 and switch only the production binding

File: RoboBeetleFirmware/Core/Src/main.c

- [ ] Step 1: Add declarations and initialization order.

Use:

    UART_HandleTypeDef huart1;
    UART_HandleTypeDef huart2;
    UART_HandleTypeDef huart3;
    UART_HandleTypeDef huart6;

    static void MX_USART1_UART_Init(void);
    static void MX_USART2_UART_Init(void);
    static void MX_USART3_UART_Init(void);
    static void MX_USART6_UART_Init(void);

    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();

- [ ] Step 2: Insert the generated-style MX_USART2_UART_Init function between USART1 and USART3.

    static void MX_USART2_UART_Init(void)
    {
      huart2.Instance = USART2;
      huart2.Init.BaudRate = 115200;
      huart2.Init.WordLength = UART_WORDLENGTH_8B;
      huart2.Init.StopBits = UART_STOPBITS_1;
      huart2.Init.Parity = UART_PARITY_NONE;
      huart2.Init.Mode = UART_MODE_TX_RX;
      huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      huart2.Init.OverSampling = UART_OVERSAMPLING_16;
      if (HAL_UART_Init(&huart2) != HAL_OK)
      {
        Error_Handler();
      }
    }

Retain the surrounding CubeMX comment and USER CODE marker style from the existing UART init functions.

- [ ] Step 3: Replace only the first app_main_init argument with &huart2. Keep the other six arguments and the entire unified callback dispatcher byte-for-byte unchanged.

### Task 4: Add USART2 MSP resources

File: RoboBeetleFirmware/Core/Src/stm32f4xx_hal_msp.c

- [ ] Step 1: Add after USART1 init:

    else if(huart->Instance==USART2)
    {
      __HAL_RCC_USART2_CLK_ENABLE();
      __HAL_RCC_GPIOA_CLK_ENABLE();
      GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(USART2_IRQn);
    }

Retain generated USER CODE markers and PA2/PA3 mapping comments.

- [ ] Step 2: Add before USART3 deinit:

    else if(huart->Instance==USART2)
    {
      __HAL_RCC_USART2_CLK_DISABLE();
      HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
      HAL_NVIC_DisableIRQ(USART2_IRQn);
    }

Do not alter USART1/3/6 branches.

### Task 5: Add USART2 interrupt dispatch

Files:
- RoboBeetleFirmware/Core/Src/stm32f4xx_it.c
- RoboBeetleFirmware/Core/Inc/stm32f4xx_it.h

- [ ] Step 1: Add extern UART_HandleTypeDef huart2 and prototype void USART2_IRQHandler(void); beside the existing UART entries.

- [ ] Step 2: Insert this handler between USART1 and USART3:

    void USART2_IRQHandler(void)
    {
      HAL_UART_IRQHandler(&huart2);
    }

Use the generated USER CODE marker/comment structure already present in stm32f4xx_it.c. Keep USART1_IRQHandler calling HAL_UART_IRQHandler(&huart1).

### Task 6: Make minimal docs wording and register the contract

Files:
- RoboBeetleFirmware/README.md
- RoboBeetleFirmware/tests/run_host_tests.ps1

- [ ] Step 1: Correct only active README statements.

State that the active primary host-link transport is STM32 USART2 at 115200 8-N-1 on PA2/PA3. State that USART1 remains initialized at 9600 on PA9/PA10 for retained APC220/legacy hardware, is not bound to uart_transport_stm32, does not arm host transport RX, and is not a backup control link. Qualify prior USART1 DAP runs as historical; do not rewrite historical acceptance sections or claim Raspberry Pi hardware execution.

- [ ] Step 2: Invoke the contract after the existing clock contract call.

    $usart2ContractScript = Join-Path $PSScriptRoot 'usart2_host_binding_contract_tests.ps1'
    & powershell -NoProfile -ExecutionPolicy Bypass -File $usart2ContractScript
    if ($LASTEXITCODE -ne 0) {
        throw 'USART2 host binding contract check failed'
    }

The contract script itself prints PASS usart2_host_binding_contract_tests; the
runner only propagates its exit status so the result appears once.

Append + USART2 source/config contract to the final summary while preserving the historical 36 executable and 13 object counts.

### Task 7: Run all GREEN verification

- [ ] Step 1: Run the focused contract.

    powershell -NoProfile -ExecutionPolicy Bypass -File .\RoboBeetleFirmware\tests\usart2_host_binding_contract_tests.ps1

Expected: PASS usart2_host_binding_contract_tests and exit code 0.

- [ ] Step 2: Run the full host gate and focused binaries.

    $repo = (git rev-parse --show-toplevel)
    $hostBuild = Join-Path $repo 'build\host-usart2-pi-primary-host'
    powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $repo 'RoboBeetleFirmware\tests\run_host_tests.ps1') -BuildRoot $hostBuild
    & (Join-Path $hostBuild 'uart_transport_stm32_tests.exe')
    & (Join-Path $hostBuild 'uart_tx_queue_tests.exe')

Expected: all 36 executables, all 13 existing compile-contract objects, the clock contract, and the USART2 contract pass. Both focused binaries exit 0.

- [ ] Step 3: Audit source/config parity without an ARM toolchain.

    $repo = (git rev-parse --show-toplevel)
    rg -n 'huart2|MX_USART2_UART_Init|USART2_IRQHandler|GPIO_PIN_2|GPIO_PIN_3|GPIO_AF7_USART2|USART2_IRQn|USART2\.BaudRate|PA2\.Signal|PA3\.Signal' (Join-Path $repo 'RoboBeetleFirmware')
    git diff --name-only -- RoboBeetleFirmware/Core/Communication/uart_transport_stm32.c

The first command must show consistent USART2 declarations in .ioc, main, MSP, and IRQ files. The second command must print no path. ARM build/flash is PENDING USER TARGET VERIFICATION; do not install a toolchain or modify CMake.

- [ ] Step 4: Run scope/format checks.

    git diff --check
    git diff --stat
    git status --short --branch
    git diff -- RoboBeetleFirmware/Core/Communication/uart_transport_stm32.c

Expected: diff check exits 0, only the file map plus approved docs is changed, transport source has no diff, and ignored build output is absent from status.

### Task 8: Commit the tested implementation and stop for review

- [ ] Step 1: Before committing, verify the only production behavior change is app_main_init(&huart2,...), the only new peripheral is USART2 PA2/PA3, and USART1 remains initialized but cannot enter Protocol V2, create Heartbeats, affect Safety, or gain control authority. Verify unified callbacks, Protocol V2, Heartbeat/Safety, TX queue, recovery, Motion, Servo, USART3, and USART6 are unchanged.

- [ ] Step 2: Commit after all green checks.

    git add RoboBeetleFirmware/RoboBeetleFirmware.ioc RoboBeetleFirmware/Core/Src/main.c RoboBeetleFirmware/Core/Src/stm32f4xx_hal_msp.c RoboBeetleFirmware/Core/Src/stm32f4xx_it.c RoboBeetleFirmware/Core/Inc/stm32f4xx_it.h RoboBeetleFirmware/tests/usart2_host_binding_contract_tests.ps1 RoboBeetleFirmware/tests/run_host_tests.ps1 RoboBeetleFirmware/README.md
    git commit -m "feat: add USART2 Raspberry Pi host link"

Do not create or merge a pull request. After committing, rerun git status --short --branch, git diff --check, and the focused contract. Report branch, committed HEAD, changed files, tests, source/config evidence, diff stat, and clean status. Stop and await external review.
