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

function Assert-IocValue([string]$key, [string]$expected) {
    $iocMatches = [regex]::Matches($ioc, '(?m)^' + [regex]::Escape($key) + '=([^\r\n]*)\r?$')
    if ($iocMatches.Count -ne 1) {
        throw ('Expected exactly one .ioc key: ' + $key)
    }
    if ($iocMatches[0].Groups[1].Value -cne $expected) {
        throw ('.ioc value mismatch for ' + $key + ': expected ' + $expected + ', got ' + $iocMatches[0].Groups[1].Value)
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
    '__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);',
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
    @{ Key = 'MxCube.Version'; Value = '6.18.1' },
    @{ Key = 'ProjectManager.FirmwarePackage'; Value = 'STM32Cube FW_F4 V1.28.3' },
    @{ Key = 'Mcu.CPN'; Value = 'STM32F407VET6' },
    @{ Key = 'PCC.Vdd'; Value = '3.3' },
    @{ Key = 'RCC.48MHZClocksFreq_Value'; Value = '48000000' },
    @{ Key = 'RCC.AHBCLKDivider'; Value = 'RCC_SYSCLK_DIV1' },
    @{ Key = 'RCC.AHBFreq_Value'; Value = '168000000' },
    @{ Key = 'RCC.APB1CLKDivider'; Value = 'RCC_HCLK_DIV4' },
    @{ Key = 'RCC.APB1Freq_Value'; Value = '42000000' },
    @{ Key = 'RCC.APB1TimFreq_Value'; Value = '84000000' },
    @{ Key = 'RCC.APB2CLKDivider'; Value = 'RCC_HCLK_DIV2' },
    @{ Key = 'RCC.APB2Freq_Value'; Value = '84000000' },
    @{ Key = 'RCC.APB2TimFreq_Value'; Value = '168000000' },
    @{ Key = 'RCC.CortexFreq_Value'; Value = '168000000' },
    @{ Key = 'RCC.FCLKCortexFreq_Value'; Value = '168000000' },
    @{ Key = 'RCC.HCLKFreq_Value'; Value = '168000000' },
    @{ Key = 'RCC.PLLCLKFreq_Value'; Value = '168000000' },
    @{ Key = 'RCC.PLLM'; Value = '16' },
    @{ Key = 'RCC.PLLN'; Value = '336' },
    @{ Key = 'RCC.PLLP'; Value = 'RCC_PLLP_DIV2' },
    @{ Key = 'RCC.PLLQ'; Value = '7' },
    @{ Key = 'RCC.PLLQCLKFreq_Value'; Value = '48000000' },
    @{ Key = 'RCC.PLLSourceVirtual'; Value = 'RCC_PLLSOURCE_HSI' },
    @{ Key = 'RCC.PWR_Regulator_Voltage_Scale'; Value = 'PWR_REGULATOR_VOLTAGE_SCALE1' },
    @{ Key = 'RCC.SYSCLKSource'; Value = 'RCC_SYSCLKSOURCE_PLLCLK' },
    @{ Key = 'RCC.VCOInputFreq_Value'; Value = '1000000' },
    @{ Key = 'RCC.VCOOutputFreq_Value'; Value = '336000000' },
    @{ Key = 'RCC.SYSCLKFreq_VALUE'; Value = '168000000' }
) | ForEach-Object { Assert-IocValue $_.Key $_.Value }

$iocIpParametersMatch = [regex]::Match($ioc, '(?m)^RCC\.IPParameters=(.*)$')
if (-not $iocIpParametersMatch.Success) {
    throw 'RCC.IPParameters is missing'
}
$iocIpParameters = $iocIpParametersMatch.Groups[1].Value.Split(',')
@(
    'AHBCLKDivider',
    'APB1CLKDivider',
    'APB1TimFreq_Value',
    'APB2CLKDivider',
    'APB2TimFreq_Value',
    'FCLKCortexFreq_Value',
    'HCLKFreq_Value',
    'PLLM',
    'PLLN',
    'PLLP',
    'PLLQ',
    'PLLSourceVirtual',
    'PWR_Regulator_Voltage_Scale',
    'SYSCLKSource'
) | ForEach-Object {
    if ($iocIpParameters -notcontains $_) {
        throw ('RCC.IPParameters does not persist clock input: ' + $_)
    }
}

@(
    @{ Key = 'TIM3.Period'; Value = '3002' },
    @{ Key = 'TIM3.Prescaler'; Value = '83' },
    @{ Key = 'TIM4.Period'; Value = '3002' },
    @{ Key = 'TIM4.Prescaler'; Value = '83' },
    @{ Key = 'USART1.BaudRate'; Value = '9600' },
    @{ Key = 'USART3.BaudRate'; Value = '9600' },
    @{ Key = 'USART6.BaudRate'; Value = '115200' }
) | ForEach-Object { Assert-IocValue $_.Key $_.Value }

$timerKernelHz = 84000000
$prescaler = 83
if (($timerKernelHz / ($prescaler + 1)) -ne 1000000) {
    throw 'TIM3/TIM4 1 us timer contract does not evaluate to 1 MHz'
}

Write-Host 'PASS clock_config_contract_tests'
