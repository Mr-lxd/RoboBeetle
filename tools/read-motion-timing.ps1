[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$ElfPath,

    [string]$ReportSymbol = 'motion_timing_report',

    [string]$RawReportPath = '',

    [string]$OutputPath = '',

    [string]$NmPath = 'arm-none-eabi-nm',

    [string]$OpenOcdPath = 'openocd',

    [string[]]$OpenOcdArgs = @(),

    [string]$BranchSha = '',

    [string]$TrialId = '',

    [string]$Workload = '',

    [string]$DiagnosticCondition = '',

    [string]$ToolchainVersion = '',

    [string]$CMakeGenerator = '',

    [string]$BuildType = '',

    [string]$LinkerScript = '',

    [string]$ImageSize = ''
)

$ErrorActionPreference = 'Stop'

function Stop-WithError {
    param([string]$Message)
    throw "motion timing readout failed: $Message"
}

function Read-U32 {
    param(
        [byte[]]$Bytes,
        [int]$Offset
    )

    if (($Offset -lt 0) -or (($Offset + 4) -gt $Bytes.Length)) {
        Stop-WithError "report offset $Offset is outside the dump"
    }
    return [BitConverter]::ToUInt32($Bytes, $Offset)
}

function Read-Dist {
    param(
        [byte[]]$Bytes,
        [int]$Offset
    )

    $histogram = @()
    for ($index = 0; $index -lt 8; ++$index) {
        $histogram += Read-U32 $Bytes ($Offset + 24 + (4 * $index))
    }

    return [ordered]@{
        count = Read-U32 $Bytes $Offset
        min_value = Read-U32 $Bytes ($Offset + 4)
        max_value = Read-U32 $Bytes ($Offset + 8)
        total_lo = Read-U32 $Bytes ($Offset + 12)
        total_hi = Read-U32 $Bytes ($Offset + 16)
        worst_interval_value = Read-U32 $Bytes ($Offset + 20)
        histogram = $histogram
        saturated_count = Read-U32 $Bytes ($Offset + 56)
    }
}

if (-not (Test-Path -LiteralPath $ElfPath -PathType Leaf)) {
    Stop-WithError "ELF was not found: $ElfPath"
}

if ([BitConverter]::IsLittleEndian -eq $false) {
    Stop-WithError 'the host byte decoder is not little-endian'
}

$resolvedElfPath = (Resolve-Path -LiteralPath $ElfPath).Path
$nm = Get-Command $NmPath -ErrorAction SilentlyContinue
if ($null -eq $nm) {
    Stop-WithError "required symbol tool was not found: $NmPath"
}

$nmOutput = & $nm.Source '--defined-only' $resolvedElfPath 2>&1
if ($LASTEXITCODE -ne 0) {
    Stop-WithError "nm could not inspect $resolvedElfPath"
}

$symbolPattern = '^\s*([0-9A-Fa-f]+)\s+\S\s+' +
    [regex]::Escape($ReportSymbol) + '\s*$'
$symbolMatches = @(
    $nmOutput | Where-Object { $_ -match $symbolPattern } |
        ForEach-Object { [regex]::Match($_, $symbolPattern) }
)
if ($symbolMatches.Count -ne 1) {
    Stop-WithError "expected exactly one defined symbol '$ReportSymbol', found $($symbolMatches.Count)"
}

$reportAddress = [Convert]::ToUInt32(
    $symbolMatches[0].Groups[1].Value,
    16)
$reportSize = 1172
$temporaryDump = $false

if ([string]::IsNullOrWhiteSpace($RawReportPath)) {
    if ($OpenOcdArgs.Count -eq 0) {
        Stop-WithError 'OpenOCD arguments are required for a live read; use -RawReportPath for an existing debugger dump'
    }

    $openOcd = Get-Command $OpenOcdPath -ErrorAction SilentlyContinue
    if ($null -eq $openOcd) {
        Stop-WithError "required target tool was not found: $OpenOcdPath"
    }

    $RawReportPath = [IO.Path]::GetTempFileName()
    $temporaryDump = $true
    $dumpCommand = 'init; halt; dump_image "{0}" 0x{1:X8} {2}; shutdown' -f `
        $RawReportPath,
        $reportAddress,
        $reportSize
    & $openOcd.Source @OpenOcdArgs '-c' $dumpCommand
    if ($LASTEXITCODE -ne 0) {
        Stop-WithError 'OpenOCD failed to read the diagnostic report'
    }
}

try {
    if (-not (Test-Path -LiteralPath $RawReportPath -PathType Leaf)) {
        Stop-WithError "raw report dump was not found: $RawReportPath"
    }

    [byte[]]$bytes = [IO.File]::ReadAllBytes(
        (Resolve-Path -LiteralPath $RawReportPath).Path)
    if ($bytes.Length -ne $reportSize) {
        Stop-WithError "raw report length $($bytes.Length) does not match fixed ABI size $reportSize"
    }

    $magic = Read-U32 $bytes 0
    if ($magic -ne 0x4D54494D) {
        Stop-WithError ('report magic mismatch: 0x{0:X8}' -f $magic)
    }

    $abiVersion = Read-U32 $bytes 4
    if ($abiVersion -ne 2) {
        Stop-WithError "report ABI version mismatch: $abiVersion"
    }

    $declaredSize = Read-U32 $bytes 8
    if ($declaredSize -ne $reportSize) {
        Stop-WithError "report size mismatch: declared $declaredSize, expected $reportSize"
    }

    $rxNames = @('host', 'jy901s', 'depth')
    $rxReports = [ordered]@{}
    for ($index = 0; $index -lt 3; ++$index) {
        $base = 152 + (72 * $index)
        $rxReports[$rxNames[$index]] = [ordered]@{
            call_count = Read-U32 $bytes $base
            nonempty_count = Read-U32 $bytes ($base + 4)
            byte_count = Read-U32 $bytes ($base + 8)
            duration = Read-Dist $bytes ($base + 12)
        }
    }

    $txNames = @('ack', 'leak', 'imu', 'depth')
    $txReports = [ordered]@{}
    for ($index = 0; $index -lt 4; ++$index) {
        $base = 368 + (84 * $index)
        $txReports[$txNames[$index]] = [ordered]@{
            call_count = Read-U32 $bytes $base
            byte_count = Read-U32 $bytes ($base + 4)
            last_status = Read-U32 $bytes ($base + 8)
            ok_count = Read-U32 $bytes ($base + 12)
            error_count = Read-U32 $bytes ($base + 16)
            timeout_count = Read-U32 $bytes ($base + 20)
            duration = Read-Dist $bytes ($base + 24)
        }
    }

    $motionBase = 704
    $motion = [ordered]@{
        accepted_tick_count = Read-U32 $bytes $motionBase
        last_elapsed_ms = Read-U32 $bytes ($motionBase + 4)
        requested_elapsed_ms = Read-Dist $bytes ($motionBase + 8)
        actual_interval_cycles = Read-Dist $bytes ($motionBase + 68)
        actual_interval_ms = Read-Dist $bytes ($motionBase + 128)
        gap_gt_10_ms_count = Read-U32 $bytes ($motionBase + 188)
        gap_gt_12_ms_count = Read-U32 $bytes ($motionBase + 192)
        gap_gt_15_ms_count = Read-U32 $bytes ($motionBase + 196)
        gap_gt_20_ms_count = Read-U32 $bytes ($motionBase + 200)
        gap_gt_30_ms_count = Read-U32 $bytes ($motionBase + 204)
        tick_duration_cycles = Read-Dist $bytes ($motionBase + 208)
        generator_advance_cycles = Read-Dist $bytes ($motionBase + 268)
        sample_cycles = Read-Dist $bytes ($motionBase + 328)
        apply_cycles = Read-Dist $bytes ($motionBase + 388)
        result_ok_count = Read-U32 $bytes ($motionBase + 448)
        result_error_count = Read-U32 $bytes ($motionBase + 452)
    }

    $branch = $BranchSha
    if ([string]::IsNullOrWhiteSpace($branch)) {
        try {
            $branch = (& git -C (Split-Path -Parent $resolvedElfPath) rev-parse HEAD 2>$null).Trim()
        }
        catch {
            $branch = ''
        }
    }

    $decoded = [ordered]@{
        report_symbol = $ReportSymbol
        report_address = ('0x{0:X8}' -f $reportAddress)
        elf_path = $resolvedElfPath
        branch_sha = $branch
        trial_metadata = [ordered]@{
            trial_id = $TrialId
            workload = $Workload
            diagnostic_condition = $DiagnosticCondition
            toolchain_version = $ToolchainVersion
            cmake_generator = $CMakeGenerator
            build_type = $BuildType
            linker_script = $LinkerScript
            image_size = $ImageSize
        }
        magic = ('0x{0:X8}' -f $magic)
        abi_version = $abiVersion
        report_size = $declaredSize
        system_core_clock_hz = Read-U32 $bytes 12
        diagnostic_flags = Read-U32 $bytes 16
        runtime_backend = Read-U32 $bytes 20
        run_marker = Read-U32 $bytes 24
        reset_marker = Read-U32 $bytes 28
        app_loop_body = Read-Dist $bytes 32
        app_loop_interval = Read-Dist $bytes 92
        rx_drain = $rxReports
        tx = $txReports
        motion = $motion
        diagnostic_saturation_count = Read-U32 $bytes 1160
        diagnostic_counter_wrap_count = Read-U32 $bytes 1164
        diagnostic_invalid_count = Read-U32 $bytes 1168
        raw_report_path = (Resolve-Path -LiteralPath $RawReportPath).Path
    }

    $json = $decoded | ConvertTo-Json -Depth 10
    if ([string]::IsNullOrWhiteSpace($OutputPath)) {
        Write-Output $json
    }
    else {
        $resolvedOutputDirectory = Split-Path -Parent $OutputPath
        if ([string]::IsNullOrWhiteSpace($resolvedOutputDirectory)) {
            $resolvedOutputDirectory = (Get-Location).Path
        }
        if (-not (Test-Path -LiteralPath $resolvedOutputDirectory -PathType Container)) {
            New-Item -ItemType Directory -Force -Path $resolvedOutputDirectory | Out-Null
        }
        $json | Set-Content -LiteralPath $OutputPath -Encoding UTF8
        Write-Output "validated report written to $OutputPath"
    }
}
finally {
    if ($temporaryDump -and (Test-Path -LiteralPath $RawReportPath)) {
        Remove-Item -LiteralPath $RawReportPath -Force
    }
}
