$ErrorActionPreference = 'Stop'

$readoutScript = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\read-motion-timing.ps1')).Path
$source = Get-Content -LiteralPath $readoutScript -Raw

$windowsPath = 'C:\Users\name\AppData\Local\Temp\tmp123.tmp'
$expectedOpenOcdPath = 'C:/Users/name/AppData/Local/Temp/tmp123.tmp'
$actualOpenOcdPath = $windowsPath -replace '\\', '/'
if ($actualOpenOcdPath -cne $expectedOpenOcdPath) {
    throw "Windows path normalization regression: expected '$expectedOpenOcdPath', got '$actualOpenOcdPath'"
}

foreach ($fragment in @(
        '$openOcdDumpPath',
        '$RawReportPath -replace',
        "'\\'",
        "'/'"
    )) {
    if (-not $source.Contains($fragment)) {
        throw "readout helper is missing the OpenOCD path conversion fragment: $fragment"
    }
}

foreach ($fragment in @(
        '$reportSize = 1432',
        '$abiVersion -ne 4',
        'uart_transport = $uartTransport',
        'Read-U32 $bytes (1300 + (4 * $index))'
    )) {
    if (-not $source.Contains($fragment)) {
        throw "readout helper is missing the fixed ABI v4 decoder fragment: $fragment"
    }
}

$dumpCommandIndex = $source.IndexOf('$dumpCommand =', [StringComparison]::Ordinal)
if ($dumpCommandIndex -lt 0) {
    throw 'readout helper does not construct a dump_image command'
}

$dumpCommandSection = $source.Substring($dumpCommandIndex)
if (-not $dumpCommandSection.Contains('$openOcdDumpPath')) {
    throw 'dump_image does not use the normalized OpenOCD path'
}

Write-Output 'read-motion-timing OpenOCD Windows path regression: PASS'
