#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 2
ctest --test-dir build --output-on-failure
./build/motor_trace > assets/simulation_trace.csv
python3 scripts/verify_scenarios.py ./build/motor_sim
