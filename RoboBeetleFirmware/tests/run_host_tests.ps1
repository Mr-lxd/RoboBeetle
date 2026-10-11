[CmdletBinding()]
param(
    [string]$BuildRoot = '',
    [string[]]$OnlyCases = @()
)

$ErrorActionPreference = 'Stop'

if ([string]::IsNullOrWhiteSpace($BuildRoot)) {
    $BuildRoot = Join-Path $PSScriptRoot '..\..\build\firmware-host-tests'
}

$firmwareRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$buildRootPath = (Resolve-Path (New-Item -ItemType Directory -Force -Path $BuildRoot)).Path
$gcc = (Get-Command gcc -ErrorAction Stop).Source
$traceOutputRoot = Join-Path $buildRootPath 'gait-trace'
$traceTestOutputPathA = Join-Path $traceOutputRoot 'test-a'
$traceTestOutputPathB = Join-Path $traceOutputRoot 'test-b'
$traceToolOutputPath = Join-Path $traceOutputRoot 'tool'
foreach ($tracePath in @(
        $traceTestOutputPathA,
        $traceTestOutputPathB,
        $traceToolOutputPath)) {
    New-Item -ItemType Directory -Force -Path $tracePath | Out-Null
}

$includeArgs = @(
    '-DSTM32F407xx',
    '-I', (Join-Path $firmwareRoot 'Core\Inc'),
    '-I', (Join-Path $firmwareRoot 'Core\App'),
    '-I', (Join-Path $firmwareRoot 'Core\Diagnostics'),
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
    @{ Name = 'jy901s_oneshot_config_tests'; Sources = @('tests/jy901s_oneshot_config_tests.c'); Link = @() },
    @{ Name = 'motion_state_sampler_tests'; Sources = @('tests/motion_state_sampler_tests.c', 'Core/Communication/motion_state_sampler.c', 'Core/Communication/uart_tx_queue.c'); Link = @() },
    @{ Name = 'protocol_golden_vectors'; Sources = @('tests/protocol_golden_vectors.c', 'Core/Src/rb_protocol_v2.c'); Link = @() },
    @{ Name = 'ring_buffer_tests'; Sources = @('tests/ring_buffer_tests.c', 'Core/Communication/ring_buffer.c'); Link = @() },
    @{ Name = 'uart_tx_queue_tests'; Sources = @('tests/uart_tx_queue_tests.c', 'Core/Communication/uart_tx_queue.c'); Link = @() },
    @{ Name = 'uart_transport_stm32_tests'; Sources = @('tests/uart_transport_stm32_tests.c', 'Core/Communication/uart_transport_stm32.c', 'Core/Communication/uart_tx_queue.c', 'Core/Communication/ring_buffer.c'); Link = @(); Extra = @($halWarningArgs + '-DROBOBEETLE_UART_TRANSPORT_HOST_TEST=1') },
    @{ Name = 'servo_descriptor_tests'; Sources = @('tests/servo_descriptor_tests.c', 'Core/Servo/servo_descriptor.c'); Link = @() },
    @{ Name = 'servo_calibration_tests'; Sources = @('tests/servo_calibration_tests.c', 'Core/Servo/servo_calibration.c', 'Core/Servo/servo_descriptor.c'); Link = @() },
    @{ Name = 'servo_pwm_stop_policy_tests'; Sources = @('tests/servo_pwm_stop_policy_tests.c', 'Core/Servo/servo_pwm_stop_policy.c'); Link = @() },
    @{ Name = 'servo_service_tests'; Sources = @('tests/servo_service_tests.c', 'Core/Servo/servo_service.c', 'Core/Servo/servo_calibration.c', 'Core/Servo/servo_descriptor.c'); Link = @() },
    @{ Name = 'safety_supervisor_tests'; Sources = @('tests/safety_supervisor_tests.c', 'Core/Safety/safety_supervisor.c'); Link = @() },
    @{ Name = 'leak_sensor_tests'; Sources = @('tests/leak_sensor_tests.c', 'Core/Sensors/leak_sensor.c', 'Core/Sensors/leak_telemetry_policy.c'); Link = @() },
    @{ Name = 'cpg_core_tests'; Sources = @('tests/test_cpg_core.c', 'Core/Motion/cpg_core.c'); Link = @('-lm') },
    @{ Name = 'cpg_gait_generator_tests'; Sources = @('tests/test_cpg_gait_generator.c', 'Core/Motion/cpg_gait_generator.c', 'Core/Motion/cpg_core.c'); Link = @('-lm') },
    @{ Name = 'cpg_safety_catchup_tests'; Sources = @('tests/test_cpg_safety_catchup.c', 'Core/Motion/cpg_gait_generator.c', 'Core/Motion/cpg_core.c', 'Core/Motion/motion_manager.c', 'Core/Servo/servo_service.c', 'Core/Servo/servo_calibration.c', 'Core/Servo/servo_descriptor.c', 'Core/Safety/safety_supervisor.c'); Link = @('-lm') },
    @{ Name = 'cpg_period_tests'; Sources = @('tests/test_cpg_period.c', 'Core/Motion/cpg_gait_generator.c', 'Core/Motion/cpg_core.c'); Link = @('-lm') },
    @{ Name = 'simple_gait_generator_tests'; Sources = @('tests/simple_gait_generator_tests.c', 'Core/Motion/simple_gait_generator.c'); Link = @('-lm') },
    @{ Name = 'experimental_flex_gait_generator_tests'; Sources = @('tests/experimental_flex_gait_generator_tests.c', 'Core/Motion/experimental_flex_gait_generator.c'); Link = @('-lm') },
    @{ Name = 'motion_timing_diagnostics_tests'; Sources = @('tests/motion_timing_diagnostics_tests.c', 'Core/Diagnostics/motion_timing_diagnostics.c'); Link = @() },
    @{ Name = 'motion_timing_diagnostics_hooks_tests'; Sources = @('tests/motion_timing_diagnostics_hooks_tests.c', 'Core/Diagnostics/motion_timing_diagnostics.c'); Link = @(); Extra = @($halWarningArgs + '-DROBOBEETLE_MOTION_TIMING_DIAGNOSTICS=1' + '-DROBOBEETLE_MOTION_TIMING_HOST_TEST=1') },
    @{ Name = 'motion_timing_diagnostics_lifecycle_tests'; Sources = @('tests/motion_timing_diagnostics_lifecycle_tests.c', 'Core/Diagnostics/motion_timing_diagnostics.c'); Link = @(); Extra = @($halWarningArgs + '-DROBOBEETLE_MOTION_TIMING_DIAGNOSTICS=1' + '-DROBOBEETLE_MOTION_TIMING_HOST_TEST=1') },
    @{ Name = 'app_main_timing_diagnostics_tests'; Sources = @(
        'tests/app_main_timing_diagnostics_tests.c',
        'Core/App/app_main.c',
        'Core/Diagnostics/motion_timing_diagnostics.c',
        'Core/Src/rb_protocol_v2.c',
        'Core/Communication/ring_buffer.c',
        'Core/Communication/uart_tx_queue.c',
        'Core/Communication/uart_transport_stm32.c',
        'Core/Communication/jy901s_transport_stm32.c',
        'Core/Communication/depth_transport_stm32.c',
        'Core/Communication/depth_telemetry.c',
        'Core/Communication/motion_state_codec.c',
        'Core/Communication/motion_state_sampler.c',
        'Core/Communication/imu_telemetry_policy.c',
        'Core/Communication/telemetry_scheduler.c',
        'Core/Communication/protocol_dispatcher.c',
        'Core/Motion/cpg_core.c',
        'Core/Motion/cpg_gait_generator.c',
        'Core/Motion/experimental_flex_gait_generator.c',
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
        'Core/Servo/servo_pwm_stop_policy.c',
        'Core/Safety/safety_supervisor.c'
    ); Link = @('-lm'); Extra = @($halWarningArgs + '-DROBOBEETLE_MOTION_TIMING_DIAGNOSTICS=1' + '-DROBOBEETLE_MOTION_TIMING_HOST_TEST=1' + '-DROBOBEETLE_UART_TRANSPORT_HOST_TEST=1') },
    @{ Name = 'app_main_reduced_telemetry_tests'; Sources = @(
        'tests/app_main_reduced_telemetry_tests.c',
        'Core/App/app_main.c',
        'Core/Diagnostics/motion_timing_diagnostics.c',
        'Core/Src/rb_protocol_v2.c',
        'Core/Communication/ring_buffer.c',
        'Core/Communication/uart_tx_queue.c',
        'Core/Communication/uart_transport_stm32.c',
        'Core/Communication/jy901s_transport_stm32.c',
        'Core/Communication/depth_transport_stm32.c',
        'Core/Communication/depth_telemetry.c',
        'Core/Communication/motion_state_codec.c',
        'Core/Communication/motion_state_sampler.c',
        'Core/Communication/imu_telemetry_policy.c',
        'Core/Communication/telemetry_scheduler.c',
        'Core/Communication/protocol_dispatcher.c',
        'Core/Motion/cpg_core.c',
        'Core/Motion/cpg_gait_generator.c',
        'Core/Motion/experimental_flex_gait_generator.c',
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
        'Core/Servo/servo_pwm_stop_policy.c',
        'Core/Safety/safety_supervisor.c'
    ); Link = @('-lm'); Extra = @($halWarningArgs + '-DROBOBEETLE_MOTION_TIMING_DIAGNOSTICS=1' + '-DROBOBEETLE_MOTION_TIMING_REDUCED_TELEMETRY=1' + '-DROBOBEETLE_MOTION_TIMING_HOST_TEST=1' + '-DROBOBEETLE_UART_TRANSPORT_HOST_TEST=1') },
    @{ Name = 'gait_trace_compare_tests'; Sources = @('tests/gait_trace_compare_tests.c', 'tests/tools/gait_trace_compare.c', 'Core/Motion/simple_gait_generator.c', 'Core/Motion/cpg_gait_generator.c', 'Core/Motion/cpg_core.c'); Link = @('-lm'); Arguments = @($traceTestOutputPathA, $traceTestOutputPathB) },
    @{ Name = 'gait_trace_compare_tool'; Sources = @('tests/tools/gait_trace_compare_main.c', 'tests/tools/gait_trace_compare.c', 'Core/Motion/simple_gait_generator.c', 'Core/Motion/cpg_gait_generator.c', 'Core/Motion/cpg_core.c'); Link = @('-lm'); Arguments = @($traceToolOutputPath) },
    @{ Name = 'motion_manager_tests'; Sources = @('tests/motion_manager_tests.c', 'Core/Diagnostics/motion_timing_diagnostics.c', 'Core/Motion/motion_manager.c', 'Core/Motion/simple_gait_generator.c', 'Core/Motion/cpg_gait_generator.c', 'Core/Motion/experimental_flex_gait_generator.c', 'Core/Motion/cpg_core.c', 'Core/Servo/servo_service.c', 'Core/Servo/servo_calibration.c', 'Core/Servo/servo_descriptor.c', 'Core/Safety/safety_supervisor.c'); Link = @('-lm'); Extra = @($halWarningArgs + '-DROBOBEETLE_MOTION_TIMING_DIAGNOSTICS=1' + '-DROBOBEETLE_MOTION_TIMING_HOST_TEST=1') },
    @{ Name = 'protocol_dispatcher_tests'; Sources = @('tests/protocol_dispatcher_tests.c', 'Core/Communication/protocol_dispatcher.c', 'Core/Motion/motion_manager.c', 'Core/Motion/simple_gait_generator.c', 'Core/Motion/cpg_gait_generator.c', 'Core/Motion/experimental_flex_gait_generator.c', 'Core/Motion/cpg_core.c', 'Core/Servo/servo_service.c', 'Core/Servo/servo_calibration.c', 'Core/Servo/servo_descriptor.c', 'Core/Safety/safety_supervisor.c'); Link = @('-lm') },
    @{ Name = 'jy901s_parser_tests'; Sources = @('tests/jy901s_parser_tests.c', 'Core/Sensors/jy901s_parser.c'); Link = @() },
    @{ Name = 'jy901s_telemetry_tests'; Sources = @('tests/jy901s_telemetry_tests.c', 'Core/Sensors/jy901s_telemetry.c', 'Core/Src/rb_protocol_v2.c'); Link = @() },
    @{ Name = 'telemetry_scheduler_tests'; Sources = @('tests/telemetry_scheduler_tests.c', 'Core/Communication/imu_telemetry_policy.c', 'Core/Communication/telemetry_scheduler.c'); Link = @() },
    @{ Name = 'depth_parser_tests'; Sources = @('tests/depth_parser_tests.c', 'Core/Sensors/depth_parser.c'); Link = @() },
    @{ Name = 'depth_telemetry_tests'; Sources = @('tests/depth_telemetry_tests.c', 'Core/Communication/depth_telemetry.c'); Link = @() },
    @{ Name = 'servo_driver_stm32_tests'; Sources = @('tests/servo_driver_stm32_tests.c', 'Core/Servo/servo_driver_stm32.c', 'Core/Servo/servo_pwm_stop_policy.c', 'Core/Servo/servo_service.c', 'Core/Servo/servo_calibration.c', 'Core/Servo/servo_descriptor.c'); Link = @(); Extra = $halWarningArgs },
    @{ Name = 'jy901s_transport_stm32_tests'; Sources = @('tests/jy901s_transport_stm32_tests.c', 'Core/Communication/jy901s_transport_stm32.c', 'Core/Communication/ring_buffer.c'); Link = @(); Extra = $halWarningArgs },
    @{ Name = 'depth_transport_stm32_tests'; Sources = @('tests/depth_transport_stm32_tests.c', 'Core/Communication/depth_transport_stm32.c', 'Core/Communication/ring_buffer.c'); Link = @(); Extra = $halWarningArgs },
    @{ Name = 'app_main_depth_telemetry_tests'; Sources = @(
        'tests/app_main_depth_telemetry_tests.c',
        'Core/App/app_main.c',
        'Core/Src/rb_protocol_v2.c',
        'Core/Communication/ring_buffer.c',
        'Core/Communication/uart_tx_queue.c',
        'Core/Communication/uart_transport_stm32.c',
        'Core/Communication/jy901s_transport_stm32.c',
        'Core/Communication/depth_transport_stm32.c',
        'Core/Communication/depth_telemetry.c',
        'Core/Communication/motion_state_codec.c',
        'Core/Communication/motion_state_sampler.c',
        'Core/Communication/imu_telemetry_policy.c',
        'Core/Communication/telemetry_scheduler.c',
        'Core/Communication/protocol_dispatcher.c',
        'Core/Motion/cpg_core.c',
        'Core/Motion/cpg_gait_generator.c',
        'Core/Motion/experimental_flex_gait_generator.c',
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
        'Core/Servo/servo_pwm_stop_policy.c',
        'Core/Safety/safety_supervisor.c'
    ); Link = @('-lm'); Extra = @($halWarningArgs + '-DROBOBEETLE_UART_TRANSPORT_HOST_TEST=1') }
    @{ Name = 'app_main_backend_template'; Sources = @(
        'tests/app_main_backend_tests.c',
        'Core/App/app_main.c',
        'Core/Src/rb_protocol_v2.c',
        'Core/Communication/ring_buffer.c',
        'Core/Communication/uart_tx_queue.c',
        'Core/Communication/uart_transport_stm32.c',
        'Core/Communication/jy901s_transport_stm32.c',
        'Core/Communication/depth_transport_stm32.c',
        'Core/Communication/depth_telemetry.c',
        'Core/Communication/motion_state_codec.c',
        'Core/Communication/motion_state_sampler.c',
        'Core/Communication/imu_telemetry_policy.c',
        'Core/Communication/telemetry_scheduler.c',
        'Core/Communication/protocol_dispatcher.c',
        'Core/Motion/cpg_core.c',
        'Core/Motion/cpg_gait_generator.c',
        'Core/Motion/experimental_flex_gait_generator.c',
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
        'Core/Servo/servo_pwm_stop_policy.c',
        'Core/Safety/safety_supervisor.c'
    ); Link = @('-lm', '-Wl,--wrap=motion_manager_init_with_backends'); Extra = @($halWarningArgs + '-DROBOBEETLE_UART_TRANSPORT_HOST_TEST=1') }
)

$backendTemplate = $cases | Where-Object {
    $_.Name -eq 'app_main_backend_template'
}
$cases = @($cases | Where-Object {
    $_.Name -ne 'app_main_backend_template'
})
$cases += @{
    Name = 'app_main_backend_default_tests'
    Sources = $backendTemplate.Sources
    Link = $backendTemplate.Link
    Extra = @($halWarningArgs + '-DROBOBEETLE_UART_TRANSPORT_HOST_TEST=1')
}
$cases += @{
    Name = 'app_main_backend_simple_override_tests'
    Sources = $backendTemplate.Sources
    Link = $backendTemplate.Link
    Extra = @($halWarningArgs + '-DROBOBEETLE_UART_TRANSPORT_HOST_TEST=1' + '-DMOTION_DEFAULT_GAIT_BACKEND_CPG=0')
}
$cases += @{
    Name = 'app_main_backend_cpg_override_tests'
    Sources = $backendTemplate.Sources
    Link = $backendTemplate.Link
    Extra = @($halWarningArgs + '-DROBOBEETLE_UART_TRANSPORT_HOST_TEST=1' + '-DMOTION_DEFAULT_GAIT_BACKEND_CPG=1')
}

# CPG configuration is linked whenever a generator/manager is under test.
foreach ($case in $cases) {
    if ($case.Sources -contains 'Core/Motion/cpg_gait_generator.c') {
        $case.Sources += 'Core/Motion/cpg_parameters.c'
    }
}
$task19Sources = @('Core/Motion/cpg_parameters.c', 'Core/Motion/cpg_gait_generator.c',
    'Core/Motion/cpg_core.c', 'Core/Motion/motion_manager.c', 'Core/Servo/servo_service.c',
    'Core/Servo/servo_calibration.c', 'Core/Servo/servo_descriptor.c', 'Core/Safety/safety_supervisor.c')
$cases += @{ Name = 'cpg_parameters_tests'; Sources = @('tests/cpg_parameters_tests.c') + $task19Sources; Link = @('-lm') }
$cases += @{ Name = 'cpg_protocol_tests'; Sources = @('tests/cpg_protocol_tests.c','Core/Communication/protocol_dispatcher.c') + $task19Sources; Link = @('-lm') }
$cases += @{ Name = 'motion_state_codec_tests'; Sources = @('tests/motion_state_codec_tests.c','Core/Communication/motion_state_codec.c'); Link = @() }
foreach ($coord in @('same','opposite')) {
    foreach ($apply in @(0,1)) {
        $name = 'task19_trace_' + $coord + '_apply_' + $apply
        $traceFile = Join-Path $buildRootPath ($name + '.bin')
        $coordArg = if ($coord -eq 'same') { '0' } else { '1' }
        $extra = if ($apply -eq 1) { @('-DTASK19_APPLY_DEFAULT=1') } else { @() }
        $cases += @{ Name = $name; Sources = @('tests/task19_default_trace.c') + $task19Sources;
            Link = @('-lm'); Extra = $extra; Arguments = @($traceFile, $coordArg);
            TraceFile = $traceFile; ReferenceFile = Join-Path $PSScriptRoot ('fixtures/task19_default_' + $coord + '.bin') }
    }
}
if ($OnlyCases.Count -gt 0) {
    $cases = @($cases | Where-Object { $OnlyCases -contains $_.Name })
    if ($cases.Count -ne $OnlyCases.Count) { throw 'Unknown or duplicate OnlyCases name' }
}

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

    $arguments = @()
    if ($null -ne $Case.Arguments) {
        $arguments = @($Case.Arguments)
    }

    & $outputPath @arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Host test failed: $($Case.Name)"
    }

    if ($null -ne $Case.ReferenceFile) {
        $actual = [IO.File]::ReadAllBytes($Case.TraceFile)
        $reference = [IO.File]::ReadAllBytes($Case.ReferenceFile)
        if ($actual.Length -ne $reference.Length) { throw "Trace length differs: $($Case.Name)" }
        for ($i = 0; $i -lt $actual.Length; ++$i) {
            if ($actual[$i] -ne $reference[$i]) { throw "Trace differs at byte ${i}: $($Case.Name)" }
        }
    }
    Write-Host ("PASS " + $Case.Name)
}

foreach ($case in $cases) {
    Invoke-HostCase -Case $case
}

if ($OnlyCases.Count -gt 0) { Write-Host ('Focused Firmware cases passed: ' + $cases.Count); exit 0 }

$diagnosticsCompileContract = Join-Path $buildRootPath 'motion_timing_diagnostics_compile_contract.o'
foreach ($contract in @(
        @{ Name = 'off'; Diagnostics = 0; ReducedTelemetry = 0; ExpectedDiagnostics = 0; ExpectedReduced = 0 },
        @{ Name = 'reduced_without_diagnostics'; Diagnostics = 0; ReducedTelemetry = 1; ExpectedDiagnostics = 0; ExpectedReduced = 0 },
        @{ Name = 'diagnostics'; Diagnostics = 1; ReducedTelemetry = 0; ExpectedDiagnostics = 1; ExpectedReduced = 0 },
        @{ Name = 'diagnostics_reduced'; Diagnostics = 1; ReducedTelemetry = 1; ExpectedDiagnostics = 1; ExpectedReduced = 1 }
    )) {
    $contractObject = Join-Path $buildRootPath (
        'motion_timing_diagnostics_compile_contract_' + $contract.Name + '.o')
    $contractArgs = @($warningArgs + $includeArgs + @(
        ('-DROBOBEETLE_MOTION_TIMING_DIAGNOSTICS=' + $contract.Diagnostics),
        ('-DROBOBEETLE_MOTION_TIMING_REDUCED_TELEMETRY=' + $contract.ReducedTelemetry),
        ('-DEXPECT_DIAGNOSTICS_ACTIVE=' + $contract.ExpectedDiagnostics),
        ('-DEXPECT_REDUCED_TELEMETRY_ACTIVE=' + $contract.ExpectedReduced),
        (Join-Path $firmwareRoot 'tests\motion_timing_diagnostics_compile_contract.c')
    ))
    if ($contract.Diagnostics -eq 0) {
        $contractExecutable = Join-Path $buildRootPath (
            'motion_timing_diagnostics_compile_contract_' + $contract.Name + '.exe')
        & $gcc @($contractArgs + @('-o', $contractExecutable))
        if ($LASTEXITCODE -eq 0) {
            & $contractExecutable
        }
    }
    else {
        & $gcc @(@('-c') + $contractArgs + @('-o', $contractObject))
    }
    if ($LASTEXITCODE -ne 0) {
        throw ('Compile-contract check failed: motion timing diagnostics ' +
            $contract.Name)
    }
    Write-Host ('PASS motion_timing_diagnostics compile contract ' +
        $contract.Name)
}

foreach ($benchmark in @(0, 1)) {
    foreach ($backend in @(1, 0)) {
        $apiObject = Join-Path $buildRootPath (
            "app_main_jy901s_api_backend_" + $backend +
            "_benchmark_" + $benchmark + '.o')
        $apiSources = @(
            '-c',
            $warningArgs,
            $includeArgs,
            $halWarningArgs,
            ("-DMOTION_DEFAULT_GAIT_BACKEND_CPG=" + $backend),
            ("-DROBOBEETLE_CPG_TARGET_BENCHMARK=" + $benchmark),
            (Join-Path $firmwareRoot 'tests\app_main_jy901s_api_tests.c'),
            '-o',
            $apiObject
        )
        & $gcc @apiSources
        if ($LASTEXITCODE -ne 0) {
            throw ("Compile-contract check failed: app_main_jy901s_api_tests backend " +
                $backend + " benchmark " + $benchmark)
        }
        Write-Host ("PASS app_main_jy901s_api_tests compile contract backend " +
            $backend + " benchmark " + $benchmark)

        $appObject = Join-Path $buildRootPath (
            "app_main_compile_contract_backend_" + $backend +
            "_benchmark_" + $benchmark + '.o')
        $appSources = @(
            '-c',
            $warningArgs,
            $includeArgs,
            $halWarningArgs,
            ("-DMOTION_DEFAULT_GAIT_BACKEND_CPG=" + $backend),
            ("-DROBOBEETLE_CPG_TARGET_BENCHMARK=" + $benchmark),
            (Join-Path $firmwareRoot 'Core\App\app_main.c'),
            '-o',
            $appObject
        )
        & $gcc @appSources
        if ($LASTEXITCODE -ne 0) {
            throw ("Compile-contract check failed: app_main backend " +
                $backend + " benchmark " + $benchmark)
        }
        Write-Host ("PASS app_main compile contract backend " +
            $backend + " benchmark " + $benchmark)
    }
}

$benchmarkObject = Join-Path $buildRootPath 'cpg_target_benchmark_compile_contract.o'
$benchmarkSources = @(
    '-c',
    $warningArgs,
    $includeArgs,
    $halWarningArgs,
    '-DROBOBEETLE_CPG_TARGET_BENCHMARK=1',
    '-DROBOBEETLE_CPG_HOST_COMPILE_CONTRACT=1',
    (Join-Path $firmwareRoot 'Core\App\cpg_target_benchmark.c'),
    '-o',
    $benchmarkObject
)
& $gcc @benchmarkSources
if ($LASTEXITCODE -ne 0) {
    throw 'Compile-contract check failed: cpg_target_benchmark.c'
}
Write-Host 'PASS cpg_target_benchmark.c compile contract'

$clockContractScript = Join-Path $PSScriptRoot 'clock_config_contract_tests.ps1'
& powershell -NoProfile -ExecutionPolicy Bypass -File $clockContractScript
if ($LASTEXITCODE -ne 0) {
    throw 'Clock configuration contract check failed'
}

$usart2ContractScript = Join-Path $PSScriptRoot 'usart2_host_binding_contract_tests.ps1'
& powershell -NoProfile -ExecutionPolicy Bypass -File $usart2ContractScript
if ($LASTEXITCODE -ne 0) {
    throw 'USART2 host binding contract check failed'
}

Write-Host ("All Firmware host tests passed: " + $cases.Count +
    " executables + 13 app/backend/benchmark/diagnostics compile-contract objects + USART2 source/config contract")
