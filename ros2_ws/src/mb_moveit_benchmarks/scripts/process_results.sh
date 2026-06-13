#!/usr/bin/env bash
# Turn moveit_ros_benchmarks OMPL log files into a SQLite database for Planner Arena.
#
# Pipeline A produces one OMPL log per benchmark run in the output_directory from
# benchmark.yaml (default /results/moveit_benchmarks/). This script aggregates them into
# a .db that you can open locally with the Planner Arena tool or upload to
# https://plannerarena.org.
#
# Usage (inside the `ros` container):
#   ros2 run mb_moveit_benchmarks process_results.sh [LOG_DIR] [OUT_DB]
set -euo pipefail

LOG_DIR="${1:-/results/moveit_benchmarks}"
OUT_DB="${2:-/results/moveit_benchmarks/benchmark.db}"

# The statistics script ships with moveit_ros_benchmarks.
STATS_SCRIPT="$(ros2 pkg prefix moveit_ros_benchmarks)/lib/moveit_ros_benchmarks/moveit_benchmark_statistics.py"
if [[ ! -f "${STATS_SCRIPT}" ]]; then
  # fallback name / location across versions
  STATS_SCRIPT="$(find "$(ros2 pkg prefix moveit_ros_benchmarks)" -name 'moveit_benchmark_statistics.py' | head -n1 || true)"
fi
if [[ -z "${STATS_SCRIPT}" || ! -f "${STATS_SCRIPT}" ]]; then
  echo "Could not find moveit_benchmark_statistics.py -- check your moveit_ros_benchmarks install." >&2
  exit 1
fi

echo "Aggregating logs in ${LOG_DIR} -> ${OUT_DB}"
python3 "${STATS_SCRIPT}" "${LOG_DIR}"/*.log --database "${OUT_DB}"
echo "Done. Open ${OUT_DB} with Planner Arena (https://plannerarena.org) or the local viewer."
