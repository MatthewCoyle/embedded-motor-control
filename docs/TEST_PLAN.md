# Verification methodology

Verification has three levels: C tests exercise controller APIs and numeric/state boundaries; end-to-end scripts execute byte framing, command handling, HAL, plant and diagnostics; a generated trace checks fault shutdown over an actual simulated time series. No result here is physical hardware verification.

## Requirements and evidence map

| Requirement | C test function or scenario |
|---|---|
| Startup/STOP/output inhibition | `test_initial_state`, `test_start_stop`, `test_start_coasting_and_idle_timing` |
| Speed/direction limits | `test_speed_limits`, `test_direction_locked_while_running`, `test_reverse_requires_stopped_rotor` |
| PI/PWM/ramping | `test_pwm_slew_bound`, `test_simulated_motor_normal`, `test_long_run_load_and_antiwindup` |
| Thermal/current/bus trips | `test_fault_overtemp`, `test_fault_overcurrent_latches`, `test_fault_bus`, `test_numeric_threshold_matrix` |
| Sensor validity/ranges/age | `test_fault_invalid_sensor`, `test_fault_nan_sensor`, `test_sample_bounds`, `test_sensor_age_and_overspeed` |
| Stall detection | `test_stall_on_mechanical_jam`, `test_stall_across_operating_range` |
| Held ESTOP, safe reset, dwell | `test_estop_synchronous`, `test_reset_blocked_while_hot`, `test_reset_requires_cooldown_and_manual_start`, `test_held_estop_and_reset_hysteresis` |
| Recovery retrip/history | `test_recovery_invalid_faults_again`, `test_invalid_state_and_fault_history` |
| Loop timing | `test_missed_tick_fault`, `test_start_coasting_and_idle_timing` |
| HAL read failure | `test_hal_acquisition_failure` |
| UART bounds/binary/malformed input | `test_parser_normal_crlf`, `test_parser_overflow_recovers`, `test_binary_frames_and_max_line`, `test_parser_adverse_values`, `test_command_interface` |
| Injection clearing | `test_injections_clear` |

The table groups requirements and is not a claim of branch or line coverage. There are 34 named C test functions, some with many assertions and table rows. All are required to pass; failed assertions return failure and the test executable exits nonzero. Assertions are not disabled by Release/NDEBUG builds because the harness uses its own CHECK macro.

## End-to-end checks

Nine scenarios cover normal/reversal, overtemperature, overcurrent, stall, sensor disconnect, undervoltage, ESTOP/recovery, overspeed and age 41-ms invalid feedback. PowerShell verifies native process exit codes, status records, duty bounds, disabled/zero output outside RUNNING, ordered fault then RECOVERY then IDLE, retained history, and final IDLE. The normal scenario must avoid FAULT/RECOVERY and demonstrate reverse. Expected command-rejection behaviour is tested at the C API/dispatcher level, not hidden in supposedly successful demo scripts.

`motor_trace` produces 626 samples from 0 through 12500 ms in 20-ms steps. Requested speed 1600 changes to 2200 at 4000 ms and jam begins 9000 ms. PowerShell checks sample count, existence of fault and zero PWM in all fault rows; the exporter itself returns nonzero unless the final fault includes STALL. The graph simply visualizes that CSV.

## Negative/boundary choices

Tests include inclusive trip/exclusive bus boundaries, each field with NaN/Infinity, age 40 vs41 ms, RPM 50 vs 50.1 for stopped eligibility, reset thermal/current limits, exactly 127 payload bytes, overflow recovery, NUL/non-ASCII, numeric overflow/trailing garbage, negative/zero/unaligned/excessive STEP, extra arguments, missing callbacks and read failure despite plausible returned data. Recovery dwell is interrupted just before completion to check it requires consecutive samples. Unreachable loaded speed is sustained 60 simulated seconds before lowering the target and checking bounded recovery.

## Verification scope

These tests establish specified behaviour for the simulated controller and input interfaces. Physical sensor failures, actuator feedback, scheduler timing and closed-loop stability on an actual motor require hardware-specific verification.
