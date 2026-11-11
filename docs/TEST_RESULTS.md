# Executed verification results

Date: 2026-11-09. Scope: native Windows host software simulation only.

## Environment

Windows PowerShell 5.1; GCC `gcc.exe (Rev5, Built by MSYS2 project) 16.1.0`, located at `C:\msys64\ucrt64\bin\gcc.exe`. The actual Windows scripts were run successfully. The shell's default execution policy blocks direct `.\scripts\test.ps1`; using the documented process-only option resolves this without changing saved policy.

Core build and tests do not require Python. Optional plot and transcript checks were run with Python 3.12.14 using only the standard library.

## Exact executed commands

From the repository root:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\test.ps1
.\build\motor_sim.exe --script .\examples\stall.txt
```

The first command invokes `scripts/build.ps1`, which compiles all three executables using:

```text
gcc -std=c11 -Wall -Wextra -Wpedantic -Werror -O2 -I include src/motor_controller.c src/motor_hal.c src/motor_plant.c src/uart_protocol.c src/console.c <target source> -o build/<target>.exe -lm
```

Target sources are `src/main.c`, `tests/test_motor_controller.c`, and `tools/trace_export.c` for `motor_sim`, `motor_tests` and `motor_trace` respectively. The script resolves absolute source paths at runtime so it also works when invoked from a different directory.

Optional Python verification commands:

```powershell
python .\scripts\plot_trace.py .\assets\simulation_trace.csv .\assets\simulation_trace.svg
python .\scripts\verify_scenarios.py .\build\motor_sim.exe
```

## Results

| Check | Observed result |
|---|---|
| Strict native build | Three executables built, no compiler warnings/errors |
| C regression tests | 34/34 test functions passed |
| PowerShell integration scenarios | 9/9 passed; ordered states, bounded PWM, inhibited output outside RUNNING, final IDLE |
| Generated trace | 626 rows, 0..12500 ms in 20-ms steps; first FAULT 9800 ms; all FAULT rows PWM 0 |
| Optional portable Python checker | 9/9 scenarios passed |
| Dependency-free SVG generation | Created from 626 C-generated CSV rows |
| Adverse-case regression tests | Startup while coasting, stale eligibility after invalid idle timing, held-ESTOP and low-speed stall cases passed |

The included [Windows transcript](../assets/verification_windows.txt), [stall transcript](../assets/stall_transcript.txt), [CSV](../assets/simulation_trace.csv) and [plot](../assets/simulation_trace.svg) are generated evidence. The graph is generated directly from CSV with no invented measurements. Native numerical results can vary slightly with compiler floating-point implementation; the specified limits and state invariants are the intended acceptance criteria.

No coverage percentage or exhaustive proof is claimed. 

## Remaining verification limits

This evidence covers host-side software behaviour. It does not demonstrate embedded target timing or hardware-in-the-loop performance; those require a separate hardware integration and verification effort.
