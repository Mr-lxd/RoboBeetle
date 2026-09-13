[CmdletBinding()]
param(
    [string]$BuildRoot = ''
)

$ErrorActionPreference = 'Stop'

if ([string]::IsNullOrWhiteSpace($BuildRoot)) {
    $BuildRoot = Join-Path $PSScriptRoot '..\..\build\firmware-host-tests'
}

$firmwareRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$buildRootPath = (Resolve-Path (New-Item -ItemType Directory -Force -Path $BuildRoot)).Path
$gcc = (Get-Command gcc -ErrorAction Stop).Source

$includeArgs = @(
    '-DSTM32F407xx',
    '-I', (Join-Path $firmwareRoot 'Core\Inc'),
    '-I', (Join-Path $firmwareRoot 'Core\App'),
    '-I', (Join-Path $firmwareRoot 'Core\Communication'),
    '-I', (Join-Path $firmwareRoot 'Core\Motion'),
    '-I', (Join-Path $firmwareRoot 'Core\Safety'),
    '-I', (Join-Path $firmwareRoot 'Core\Sensors'),
    '-I', (Join-Path $firmwareRoot 'Core\Servo'),
    '-I', (Join-Path $firmwareRoot 'Drivers\STM32F4xx_HAL_Driver\Inc'),
    '-I', (Join-Path $firmwareRoot 'Drivers\CMSIS\Device\ST\STM32F4xx\Include'),
    '-I', (Join-Path $firmwareRoot 'Drivers\CMSIS\Include')
)

$warningArgs = @('-std=c11', '-Wall', '-Wextra', '-Werror')
$halWarningArgs = @('-Wno-pointer-to-int-cast', '-Wno-int-to-pointer-cast')

$cases = @(
    @{ Name = 'protocol_golden_vectors'; Sources = @('tests/protocol_golden_vectors.c', 'Core/Src/rb_protocol_v2.c'); Link = @() },
    @{ Name = 'ring_buffer_tests'; Sources = @('tests/ring_buffer_tests.c', 'Core/Communication/ring_buffer.c'); Link = @() },
    @{ Name = 'servo_descriptor_tests'; Sources = @('tests/servo_descriptor_tests.c', 'Core/Servo/servo_descriptor.c'); Link = @() },
    @{ Name = 'servo_calibration_tests'; Sources = @('tests/servo_calibration_tests.c', 'Core/Servo/servo_calibration.c', 'Core/Servo/servo_descriptor.c'); Link = @() },
    @{ Name = 'servo_service_tests'; Sources = @('tests/servo_service_tests.c', 'Core/Servo/servo_service.c', 'Core/Servo/servo_calibration.c', 'Core/Servo/servo_descriptor.c'); Link = @() },
    @{ Name = 'safety_supervisor_tests'; Sources = @('tests/safety_supervisor_tests.c', 'Core/Safety/safety_supervisor.c'); Link = @() },
    @{ Name = 'leak_sensor_tests'; Sources = @('tests/leak_sensor_tests.c', 'Core/Sensors/leak_sensor.c', 'Core/Sensors/leak_telemetry_policy.c'); Link = @() },
    @{ Name = 'simple_gait_generator_tests'; Sources = @('tests/simple_gait_generator_tests.c', 'Core/Motion/simple_gait_generator.c'); Link = @('-lm') },
    @{ Name = 'motion_manager_tests'; Sources = @('tests/motion_manager_tests.c', 'Core/Motion/motion_manager.c', 'Core/Motion/simple_gait_generator.c', 'Core/Servo/servo_service.c', 'Core/Servo/servo_calibration.c', 'Core/Servo/servo_descriptor.c', 'Core/Safety/safety_supervisor.c'); Link = @('-lm') },
    @{ Name = 'protocol_dispatcher_tests'; Sources = @('tests/protocol_dispatcher_tests.c', 'Core/Communication/protocol_dispatcher.c', 'Core/Motion/motion_manager.c', 'Core/Motion/simple_gait_generator.c', 'Core/Servo/servo_service.c', 'Core/Servo/servo_calibration.c', 'Core/Servo/servo_descriptor.c', 'Core/Safety/safety_supervisor.c'); Link = @('-lm') },
    @{ Name = 'jy901s_parser_tests'; Sources = @('tests/jy901s_parser_tests.c', 'Core/Sensors/jy901s_parser.c'); Link = @() },
    @{ Name = 'jy901s_telemetry_tests'; Sources = @('tests/jy901s_telemetry_tests.c', 'Core/Sensors/jy901s_telemetry.c', 'Core/Src/rb_protocol_v2.c'); Link = @() },
    @{ Name = 'telemetry_scheduler_tests'; Sources = @('tests/telemetry_scheduler_tests.c', 'Core/Communication/imu_telemetry_policy.c', 'Core/Communication/telemetry_scheduler.c'); Link = @() },
    @{ Name = 'depth_parser_tests'; Sources = @('tests/depth_parser_tests.c', 'Core/Sensors/depth_parser.c'); Link = @() },
    @{ Name = 'depth_telemetry_tests'; Sources = @('tests/depth_telemetry_tests.c', 'Core/Communication/depth_telemetry.c'); Link = @() },
    @{ Name = 'servo_driver_stm32_tests'; Sources = @('tests/servo_driver_stm32_tests.c', 'Core/Servo/servo_driver_stm32.c', 'Core/Servo/servo_service.c', 'Core/Servo/servo_calibration.c', 'Core/Servo/servo_descriptor.c'); Link = @(); Extra = $halWarningArgs },
    @{ Name = 'jy901s_transport_stm32_tests'; Sources = @('tests/jy901s_transport_stm32_tests.c', 'Core/Communication/jy901s_transport_stm32.c', 'Core/Communication/ring_buffer.c'); Link = @(); Extra = $halWarningArgs },
    @{ Name = 'depth_transport_stm32_tests'; Sources = @('tests/depth_transport_stm32_tests.c', 'Core/Communication/depth_transport_stm32.c', 'Core/Communication/ring_buffer.c'); Link = @(); Extra = $halWarningArgs },
    @{ Name = 'app_main_depth_telemetry_tests'; Sources = @(
        'tests/app_main_depth_telemetry_tests.c',
        'Core/App/app_main.c',
        'Core/Src/rb_protocol_v2.c',
        'Core/Communication/ring_buffer.c',
        'Core/Communication/uart_transport_stm32.c',
        'Core/Communication/jy901s_transport_stm32.c',
        'Core/Communication/depth_transport_stm32.c',
        'Core/Communication/depth_telemetry.c',
        'Core/Communication/imu_telemetry_policy.c',
        'Core/Communication/telemetry_scheduler.c',
        'Core/Communication/protocol_dispatcher.c',
        'Core/Motion/simple_gait_generator.c',
        'Core/Motion/motion_manager.c',
        'Core/Sensors/leak_sensor.c',
        'Core/Sensors/leak_sensor_stm32.c',
        'Core/Sensors/leak_telemetry_policy.c',
        'Core/Sensors/jy901s_parser.c',
        'Core/Sensors/jy901s_telemetry.c',
        'Core/Sensors/depth_parser.c',
        'Core/Servo/servo_calibration.c',
        'Core/Servo/servo_descriptor.c',
        'Core/Servo/servo_service.c',
        'Core/Servo/servo_driver_stm32.c',
        'Core/Safety/safety_supervisor.c'
    ); Link = @('-lm'); Extra = $halWarningArgs }
)

function Invoke-HostCase {
    param(
        [hashtable]$Case
    )

    $outputPath = Join-Path $buildRootPath ($Case.Name + '.exe')
    $sourcePaths = @($Case.Sources | ForEach-Object { Join-Path $firmwareRoot $_ })
    $extraArgs = @()
    if ($null -ne $Case.Extra) {
        $extraArgs = @($Case.Extra)
    }

    $compileArgs = @($warningArgs + $includeArgs + $extraArgs + @('-o', $outputPath) + $sourcePaths + $Case.Link)
    & $gcc @compileArgs
    if ($LASTEXITCODE -ne 0) {
        throw "Host compile failed: $($Case.Name)"
    }

    & $outputPath
    if ($LASTEXITCODE -ne 0) {
        throw "Host test failed: $($Case.Name)"
    }

    Write-Host ("PASS " + $Case.Name)
}

foreach ($case in $cases) {
    Invoke-HostCase -Case $case
}

$apiObject = Join-Path $buildRootPath 'app_main_jy901s_api_tests.o'
$apiSources = @(
    '-c',
    $warningArgs,
    $includeArgs,
    $halWarningArgs,
    (Join-Path $firmwareRoot 'tests\app_main_jy901s_api_tests.c'),
    '-o',
    $apiObject
)
& $gcc @apiSources
if ($LASTEXITCODE -ne 0) {
    throw 'Compile-contract check failed: app_main_jy901s_api_tests'
}
Write-Host 'PASS app_main_jy901s_api_tests compile contract'

Write-Host ("All Firmware host tests passed: " + $cases.Count + " executables + 1 compile contract")
