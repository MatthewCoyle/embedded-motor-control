"""Portable scenario assertions. Python standard library only."""
from pathlib import Path
import re
import subprocess
import sys

CASES = {"normal_operation": "NONE", "overtemperature": "OVER_TEMP",
         "overcurrent": "OVER_CURRENT", "stall": "STALL", "sensor_failure": "SENSOR",
         "undervoltage": "BUS_VOLTAGE", "estop_recovery": "ESTOP",
         "overspeed": "OVERSPEED", "stale_sensor": "SENSOR"}
STATUS = re.compile(r"^STATUS[^\n]*state=(\w+)[^\n]*\n  output: enable=(\d) pwm=([\d.]+);[^\n]*\n  latched_faults=([^\n]+)", re.M)

def main():
    if len(sys.argv) != 2:
        raise SystemExit("Usage: python scripts/verify_scenarios.py build/motor_sim[.exe]")
    root = Path(__file__).resolve().parent.parent
    for scenario, fault in CASES.items():
        result = subprocess.run([sys.argv[1], "--script", str(root / "examples" / f"{scenario}.txt")],
                                capture_output=True, text=True, check=True, timeout=10)
        records = STATUS.findall(result.stdout)
        assert records and records[-1][0] == "IDLE", scenario
        for state, enabled, pwm, _ in records:
            assert 0 <= float(pwm) <= 1, scenario
            assert state == "RUNNING" or (enabled == "0" and float(pwm) == 0), scenario
        if fault != "NONE":
            start = result.stdout.index(f"latched_faults={fault}")
            recovery = result.stdout.index("state=RECOVERY", start)
            result.stdout.index("state=IDLE", recovery)
            assert f"fault_history={fault}" in result.stdout, scenario
        else:
            assert "state=FAULT" not in result.stdout and "direction=REV" in result.stdout
        print(f"[PASS] {scenario}")
    print(f"SCENARIOS: {len(CASES)}/{len(CASES)} passed")

if __name__ == "__main__":
    main()
