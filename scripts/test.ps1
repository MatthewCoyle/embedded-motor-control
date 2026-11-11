# Full host verification. Does not validate electrical hardware or real-time timing.
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
& (Join-Path $PSScriptRoot 'build.ps1')
$binary = Join-Path $root 'build\motor_sim.exe'
& (Join-Path $root 'build\motor_tests.exe')
if ($LASTEXITCODE -ne 0) { throw 'Unit test failure' }
$cases = [ordered]@{
    normal_operation = 'NONE'
    overtemperature = 'OVER_TEMP'
    overcurrent = 'OVER_CURRENT'
    stall = 'STALL'
    sensor_failure = 'SENSOR'
    undervoltage = 'BUS_VOLTAGE'
    estop_recovery = 'ESTOP'
    overspeed = 'OVERSPEED'
    stale_sensor = 'SENSOR'
}
foreach ($scenario in $cases.Keys) {
    $path = Join-Path $root "examples\$scenario.txt"
    $output = (& $binary '--script' $path) -join "`n"
    if ($LASTEXITCODE -ne 0 -or $output -match '(?m)^ERR ') { throw "Scenario rejected a command: $scenario" }
    $pattern = '(?m)^STATUS[^\r\n]*state=(\w+)[^\r\n]*\r?\n  output: enable=(\d) pwm=([0-9.]+);[^\r\n]*\r?\n  latched_faults=([^\r\n]+)'
    $records = [regex]::Matches($output, $pattern)
    if ($records.Count -eq 0) { throw "Missing status records: $scenario" }
    foreach ($record in $records) {
        $state = $record.Groups[1].Value
        $enabled = $record.Groups[2].Value
        $pwm = [double]::Parse($record.Groups[3].Value, [Globalization.CultureInfo]::InvariantCulture)
        if ($pwm -lt 0 -or $pwm -gt 1) { throw "PWM bounds: $scenario" }
        if ($state -ne 'RUNNING' -and ($enabled -ne '0' -or $pwm -ne 0)) { throw "Output enabled outside RUNNING: $scenario" }
    }
    if ($records[$records.Count-1].Groups[1].Value -ne 'IDLE') { throw "Final state not IDLE: $scenario" }
    if ($scenario -eq 'normal_operation') {
        if ($output -match 'state=FAULT|state=RECOVERY' -or -not $output.Contains('direction=REV')) { throw 'Normal/reverse scenario failed' }
    } else {
        $faultPosition = $output.IndexOf("latched_faults=$($cases[$scenario])")
        $recoveryPosition = $output.IndexOf('state=RECOVERY', [Math]::Max(0,$faultPosition))
        $idlePosition = $output.IndexOf('state=IDLE', [Math]::Max(0,$recoveryPosition))
        if ($faultPosition -lt 0 -or $recoveryPosition -lt $faultPosition -or $idlePosition -lt $recoveryPosition) { throw "Missing ordered fault/recovery/idle: $scenario" }
        if (-not $output.Contains("fault_history=$($cases[$scenario])")) { throw "Fault history missing: $scenario" }
    }
    Write-Host "[PASS] $scenario (ordered states, output inhibition, final IDLE)"
}
& (Join-Path $root 'build\motor_trace.exe') | Out-File -Encoding ascii (Join-Path $root 'assets\simulation_trace.csv')
if ($LASTEXITCODE -ne 0) { throw 'Trace export failed' }
$trace = @(Import-Csv (Join-Path $root 'assets\simulation_trace.csv'))
if ($trace.Count -ne 626) { throw 'Trace sample count incorrect' }
$faults = @($trace | Where-Object state -eq 'FAULT')
if ($faults.Count -eq 0 -or @($faults | Where-Object { [double]::Parse($_.pwm,[Globalization.CultureInfo]::InvariantCulture) -ne 0 }).Count -ne 0) { throw 'Trace lacks latched shutdown' }
Write-Host "ALL CHECKS PASSED: 34 C tests, 9 scenarios, 626 trace rows; first fault=$($faults[0].time_ms)ms."
