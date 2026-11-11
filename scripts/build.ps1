# Build from Windows PowerShell. No CMake required. Requires GCC C11 (MSYS2 UCRT64 supported).
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$gcc = (Get-Command gcc -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty Source)
if (-not $gcc -and (Test-Path 'C:\msys64\ucrt64\bin\gcc.exe')) {
    $gcc = 'C:\msys64\ucrt64\bin\gcc.exe'
}
if (-not $gcc) { throw 'GCC not found. Install MSYS2 UCRT64 GCC or add gcc.exe to PATH.' }
$build = Join-Path $root 'build'
New-Item -ItemType Directory -Path $build -Force | Out-Null
$flags = @('-std=c11', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-O2', '-I', (Join-Path $root 'include'))
$core = @('motor_controller.c', 'motor_hal.c', 'motor_plant.c', 'uart_protocol.c', 'console.c') |
    ForEach-Object { Join-Path $root (Join-Path 'src' $_) }
$targets = @(
    @{ Name = 'motor_sim.exe'; Source = (Join-Path $root 'src\main.c') },
    @{ Name = 'motor_tests.exe'; Source = (Join-Path $root 'tests\test_motor_controller.c') },
    @{ Name = 'motor_trace.exe'; Source = (Join-Path $root 'tools\trace_export.c') }
)
foreach ($t in $targets) {
    $output = Join-Path $build $t.Name
    & $gcc @flags @core $t.Source '-o' $output '-lm'
    if ($LASTEXITCODE -ne 0) { throw "GCC failed compiling $($t.Name)" }
    Write-Host "Built build\$($t.Name)"
}
