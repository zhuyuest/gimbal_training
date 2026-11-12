param([string]$Linker = '')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$testDir = Join-Path $projectRoot 'target/host-tests'
New-Item -ItemType Directory -Force $testDir | Out-Null
Push-Location $projectRoot
try {
    $compilerArgs = @('--edition=2024', '--test', 'tests/host.rs', '-o', (Join-Path $testDir 'host-tests.exe'))
    if ($Linker) { $compilerArgs += @('-C', "linker=$Linker") }
    & rustc @compilerArgs
    if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed.' }
    & (Join-Path $testDir 'host-tests.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Host tests failed.' }
} finally {
    Pop-Location
}
