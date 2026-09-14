$ErrorActionPreference = 'Stop'

$firmwareRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$main = Get-Content -Raw -LiteralPath (Join-Path $firmwareRoot 'Core\Src\main.c')
$ioc = Get-Content -Raw -LiteralPath (Join-Path $firmwareRoot 'RoboBeetleFirmware.ioc')
$interrupts = Get-Content -Raw -LiteralPath (Join-Path $firmwareRoot 'Core\Src\stm32f4xx_it.c')

function Assert-Contains([string]$text, [string]$needle) {
    if (-not $text.Contains($needle)) {
        throw ('Missing clock contract: ' + $needle)
    }
}

@(
    'RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;',
    'RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;',
    'RCC_OscInitStruct.PLL.PLLM = 16;',
    'RCC_OscInitStruct.PLL.PLLN = 336;',
    'RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;',
    'RCC_OscInitStruct.PLL.PLLQ = 7;',
    'RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;',
    'RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;',
    'RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;',
    'RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;',
    'HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5)',
    'htim3.Init.Prescaler = 83;',
    'htim4.Init.Prescaler = 83;',
    'htim3.Init.Period = 3002;',
    'htim4.Init.Period = 3002;',
    'huart1.Init.BaudRate = 9600;',
    'huart3.Init.BaudRate = 9600;',
    'huart6.Init.BaudRate = 115200;'
) | ForEach-Object { Assert-Contains $main $_ }

if ($main.IndexOf('  SystemClock_Config();') -lt 0 -or
    $main.IndexOf('  MX_USART1_UART_Init();') -lt $main.IndexOf('  SystemClock_Config();') -or
    $main.IndexOf('  MX_USART3_UART_Init();') -lt $main.IndexOf('  SystemClock_Config();') -or
    $main.IndexOf('  MX_USART6_UART_Init();') -lt $main.IndexOf('  SystemClock_Config();')) {
    throw 'SystemClock_Config must run before UART initialization'
}

Assert-Contains $interrupts 'HAL_IncTick();'

@(
    'TIM3.Prescaler=83',
    'TIM4.Prescaler=83'
) | ForEach-Object { Assert-Contains $ioc $_ }

if ($main.Contains('RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;')) {
    throw 'PLL must not remain disabled'
}

@(
    'RCC.AHBFreq_Value=168000000',
    'RCC.APB1Freq_Value=42000000',
    'RCC.APB2Freq_Value=84000000',
    'RCC.CortexFreq_Value=168000000',
    'RCC.PLLCLKFreq_Value=168000000',
    'RCC.PLLQCLKFreq_Value=48000000',
    'RCC.VCOInputFreq_Value=1000000',
    'RCC.VCOOutputFreq_Value=336000000',
    'RCC.SYSCLKFreq_VALUE=168000000'
) | ForEach-Object { Assert-Contains $ioc $_ }

$timerKernelHz = 84000000
$prescaler = 83
if (($timerKernelHz / ($prescaler + 1)) -ne 1000000) {
    throw 'TIM3/TIM4 1 us timer contract does not evaluate to 1 MHz'
}

Write-Host 'PASS clock_config_contract_tests'
