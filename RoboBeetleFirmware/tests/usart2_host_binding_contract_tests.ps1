$ErrorActionPreference = 'Stop'

$firmwareRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
function Read-Normalized([string]$path) {
    return ((Get-Content -Raw -LiteralPath $path) -replace '\r\n?', [char]10)
}

$main = Read-Normalized (Join-Path $firmwareRoot 'Core\Src\main.c')
$msp = Read-Normalized (Join-Path $firmwareRoot 'Core\Src\stm32f4xx_hal_msp.c')
$interrupts = Read-Normalized (Join-Path $firmwareRoot 'Core\Src\stm32f4xx_it.c')
$interruptHeader = Read-Normalized (Join-Path $firmwareRoot 'Core\Inc\stm32f4xx_it.h')
$ioc = Read-Normalized (Join-Path $firmwareRoot 'RoboBeetleFirmware.ioc')
$transport = Read-Normalized (Join-Path $firmwareRoot 'Core\Communication\uart_transport_stm32.c')

function Assert-Contains([string]$text, [string]$needle, [string]$label) {
    if (-not $text.Contains($needle)) {
        throw ('Missing ' + $label + ': ' + $needle)
    }
}

function Assert-NotContains([string]$text, [string]$needle, [string]$label) {
    if ($text.Contains($needle)) {
        throw ('Unexpected ' + $label + ': ' + $needle)
    }
}

function Assert-IocValue([string]$key, [string]$expected) {
    $matches = [regex]::Matches(
        $ioc,
        '(?m)^' + [regex]::Escape($key) + '=([^\n]*)$')
    if ($matches.Count -ne 1) {
        throw ('Expected one .ioc key: ' + $key)
    }
    if ($matches[0].Groups[1].Value -cne $expected) {
        throw ('.ioc mismatch for ' + $key + ': expected ' + $expected +
            ', got ' + $matches[0].Groups[1].Value)
    }
}

function Assert-Ordered([string]$text, [string[]]$needles, [string]$label) {
    $last = -1
    foreach ($needle in $needles) {
        $position = $text.IndexOf($needle)
        if (($position -lt 0) -or ($position -le $last)) {
            throw ('Order failure (' + $label + '): ' + $needle)
        }
        $last = $position
    }
}

@(
    @{ Key = 'Mcu.IP5'; Value = 'USART1' },
    @{ Key = 'Mcu.IP6'; Value = 'USART2' },
    @{ Key = 'Mcu.IP7'; Value = 'USART3' },
    @{ Key = 'Mcu.IP8'; Value = 'USART6' },
    @{ Key = 'Mcu.IPNb'; Value = '9' },
    @{ Key = 'Mcu.Pin16'; Value = 'PA2' },
    @{ Key = 'Mcu.Pin17'; Value = 'PA3' },
    @{ Key = 'Mcu.PinsNb'; Value = '18' },
    @{ Key = 'PA2.Mode'; Value = 'Asynchronous' },
    @{ Key = 'PA2.Signal'; Value = 'USART2_TX' },
    @{ Key = 'PA3.Mode'; Value = 'Asynchronous' },
    @{ Key = 'PA3.Signal'; Value = 'USART2_RX' },
    @{ Key = 'NVIC.USART2_IRQn'; Value = 'true\:0\:0\:false\:false\:true\:true\:true\:true' },
    @{ Key = 'USART1.BaudRate'; Value = '9600' },
    @{ Key = 'USART1.IPParameters'; Value = 'VirtualMode,BaudRate' },
    @{ Key = 'USART1.VirtualMode'; Value = 'VM_ASYNC' },
    @{ Key = 'USART2.BaudRate'; Value = '115200' },
    @{ Key = 'USART2.IPParameters'; Value = 'VirtualMode,BaudRate' },
    @{ Key = 'USART2.VirtualMode'; Value = 'VM_ASYNC' },
    @{ Key = 'USART3.BaudRate'; Value = '9600' },
    @{ Key = 'USART3.IPParameters'; Value = 'VirtualMode,BaudRate' },
    @{ Key = 'USART3.VirtualMode'; Value = 'VM_ASYNC' },
    @{ Key = 'USART6.BaudRate'; Value = '115200' },
    @{ Key = 'USART6.IPParameters'; Value = 'VirtualMode,BaudRate' },
    @{ Key = 'USART6.VirtualMode'; Value = 'VM_ASYNC' },
    @{ Key = 'ProjectManager.functionlistsort'; Value = '1-SystemClock_Config-RCC-false-HAL-false,2-MX_GPIO_Init-GPIO-false-HAL-true,3-MX_TIM3_Init-TIM3-false-HAL-true,4-MX_TIM4_Init-TIM4-false-HAL-true,5-MX_USART1_UART_Init-USART1-false-HAL-true,6-MX_USART2_UART_Init-USART2-false-HAL-true,7-MX_USART3_UART_Init-USART3-false-HAL-true,8-MX_USART6_UART_Init-USART6-false-HAL-true' }
) | ForEach-Object {
    Assert-IocValue $_.Key $_.Value
}

@(
    'UART_HandleTypeDef huart2;',
    'static void MX_USART2_UART_Init(void);',
    'huart2.Instance = USART2;',
    'huart2.Init.BaudRate = 115200;',
    'huart2.Init.WordLength = UART_WORDLENGTH_8B;',
    'huart2.Init.StopBits = UART_STOPBITS_1;',
    'huart2.Init.Parity = UART_PARITY_NONE;',
    'huart2.Init.Mode = UART_MODE_TX_RX;',
    'huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;',
    'huart2.Init.OverSampling = UART_OVERSAMPLING_16;',
    'if (HAL_UART_Init(&huart2) != HAL_OK)',
    'huart1.Init.BaudRate = 9600;',
    'huart3.Init.BaudRate = 9600;',
    'huart6.Init.BaudRate = 115200;'
) | ForEach-Object {
    Assert-Contains $main $_ 'main UART initialization'
}

Assert-Ordered $main @(
    '  MX_USART1_UART_Init();',
    '  MX_USART2_UART_Init();',
    '  MX_USART3_UART_Init();',
    '  MX_USART6_UART_Init();'
) 'UART initialization order'

$binding = '  app_main_init(' + [char]10 + '      &huart2,'
$oldBinding = '  app_main_init(' + [char]10 + '      &huart1,'
Assert-Contains $main $binding 'primary host binding'
Assert-NotContains $main $oldBinding 'old primary host binding'

@(
    'if(huart->Instance==USART1)',
    'GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;',
    'GPIO_InitStruct.Alternate = GPIO_AF7_USART1;',
    'HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);',
    'HAL_NVIC_EnableIRQ(USART1_IRQn);',
    '__HAL_RCC_USART1_CLK_DISABLE();',
    'HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);',
    'HAL_NVIC_DisableIRQ(USART1_IRQn)'
) | ForEach-Object {
    Assert-Contains $msp $_ 'retained USART1 MSP'
}

@(
    'if(huart->Instance==USART2)',
    '__HAL_RCC_USART2_CLK_ENABLE();',
    '__HAL_RCC_GPIOA_CLK_ENABLE();',
    'PA2     ------> USART2_TX',
    'PA3     ------> USART2_RX',
    'GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;',
    'GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;',
    'GPIO_InitStruct.Pull = GPIO_NOPULL;',
    'GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;',
    'GPIO_InitStruct.Alternate = GPIO_AF7_USART2;',
    'HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);',
    'HAL_NVIC_EnableIRQ(USART2_IRQn);',
    '__HAL_RCC_USART2_CLK_DISABLE();',
    'HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);',
    'HAL_NVIC_DisableIRQ(USART2_IRQn)'
) | ForEach-Object {
    Assert-Contains $msp $_ 'USART2 MSP'
}

Assert-Contains $interrupts 'extern UART_HandleTypeDef huart2;' 'USART2 extern'
$usart2Handler = (@(
    'void USART2_IRQHandler(void)',
    '{',
    '  /* USER CODE BEGIN USART2_IRQn 0 */',
    '',
    '  /* USER CODE END USART2_IRQn 0 */',
    '  HAL_UART_IRQHandler(&huart2);',
    '  /* USER CODE BEGIN USART2_IRQn 1 */',
    '',
    '  /* USER CODE END USART2_IRQn 1 */',
    '}'
) -join [char]10)
Assert-Contains $interrupts $usart2Handler 'USART2 IRQ dispatch'
Assert-Contains $interrupts 'HAL_UART_IRQHandler(&huart1);' 'retained USART1 IRQ dispatch'
Assert-Contains $interruptHeader 'void USART2_IRQHandler(void);' 'USART2 IRQ prototype'
Assert-Contains $interruptHeader 'void USART1_IRQHandler(void);' 'retained USART1 IRQ prototype'

$callbackStart = $main.IndexOf('/* USER CODE BEGIN 4 */')
$callbackEnd = $main.IndexOf('/* USER CODE END 4 */')
if (($callbackStart -lt 0) -or ($callbackEnd -le $callbackStart)) {
    throw 'Unified HAL callback dispatcher block is missing'
}
$callbackBlock = $main.Substring($callbackStart, $callbackEnd - $callbackStart)
@(
    'HAL_UART_RxCpltCallback',
    'uart_transport_stm32_on_rx_complete(huart);',
    'jy901s_transport_stm32_on_rx_complete(huart);',
    'depth_transport_stm32_on_rx_complete(huart);',
    'HAL_UART_TxCpltCallback',
    'uart_transport_stm32_on_tx_complete(huart);',
    'HAL_UART_AbortTransmitCpltCallback',
    'uart_transport_stm32_on_abort_transmit_complete(huart);',
    'HAL_UART_ErrorCallback',
    'uart_transport_stm32_on_error(huart);',
    'jy901s_transport_stm32_on_error(huart);',
    'depth_transport_stm32_on_error(huart);'
) | ForEach-Object {
    Assert-Contains $callbackBlock $_ 'unified HAL callback dispatcher'
}
Assert-NotContains $callbackBlock 'USART2' 'USART2-specific callback branch'
Assert-NotContains $callbackBlock 'huart2' 'USART2-specific callback handle branch'
Assert-NotContains $transport 'USART2' 'transport source change'

Write-Host 'PASS usart2_host_binding_contract_tests'
