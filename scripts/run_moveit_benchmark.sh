#!/usr/bin/env bash
# TASK 2 (Pipeline A): standard moveit_ros_benchmarks run -> Planner Arena database.
# Steps: start mongo, populate the warehouse from the shared scenarios, run the benchmark,
# then aggregate the logs.
set -euo pipefail
cd "$(dirname "$0")/.."
COMPOSE="docker compose -f docker/docker-compose.yml"

echo "[1/4] start MongoDB warehouse"
$COMPOSE up -d mongo

echo "[2/4] export scenes + (TODO: persist to warehouse -- see populate_warehouse.py)"
$COMPOSE run --rm ros \
  ros2 run mb_moveit_benchmarks populate_warehouse.py \
    --scenarios /workspace/scenarios/library --out /workspace/results/scenes

echo "[3/4] run moveit_run_benchmark (this can take a while)"
$COMPOSE run --rm ros \
  ros2 launch mb_moveit_benchmarks benchmark.launch.py

echo "[4/4] aggregate logs -> SQLite for Planner Arena"
$COMPOSE run --rm ros \
  ros2 run mb_moveit_benchmarks process_results.sh \
    /workspace/results/moveit_benchmarks /workspace/results/moveit_benchmarks/benchmark.db

echo "Done. Upload results/moveit_benchmarks/benchmark.db to https://plannerarena.org"
