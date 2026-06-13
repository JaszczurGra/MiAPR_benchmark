#!/usr/bin/env bash
# Set up the OFFLINE harness on the host (no ROS, no GPU, no Docker) and run its tests.
# This exercises everything in `mb_benchmark` except the MoveIt / cuRobo adapters.
set -euo pipefail
cd "$(dirname "$0")/.."

python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ./benchmark pytest

echo "=== running offline test suite ==="
python -m pytest benchmark/tests -q

echo
echo "OK. Try the offline demo:"
echo "  source .venv/bin/activate && mb-benchmark demo --out results/demo"
