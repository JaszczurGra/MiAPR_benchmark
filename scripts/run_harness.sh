#!/usr/bin/env bash
# TASKS 3+4 (Pipeline B): cross-framework harness over MoveIt planners AND cuRobo, on
# identical scenarios, scored by one uniform metric path.
#
# Flow:
#   1. generate the (seeded) query set once on the host -> scenarios/generated  (shared).
#   2. run MoveIt planners in the `ros` container      -> results/raw/...
#   3. run cuRobo in the `curobo` container (GPU)      -> results/raw/...
#   4. score everything + build the report (offline)   -> results/report/
set -euo pipefail
cd "$(dirname "$0")/.."
COMPOSE="docker compose -f docker/docker-compose.yml"

RUNS="${RUNS:-10}"
TIMEOUT="${TIMEOUT:-10}"
SEED="${SEED:-42}"
MOVEIT_PLANNERS="${MOVEIT_PLANNERS:-moveit:RRTConnect,moveit:RRT,moveit:PRM,moveit:BITstar,moveit:EST,moveit:KPIECE}"

echo "[1/4] generate identical queries -> scenarios/generated"
$COMPOSE run --rm ros \
  mb-benchmark generate --scenarios /workspace/scenarios/library \
    --out /workspace/scenarios/generated --num 20 --seed "$SEED"

echo "[2/4] run MoveIt planners (ros container)"
$COMPOSE run --rm ros \
  mb-benchmark run --scenarios /workspace/scenarios/generated --no-autogen \
    --planners "$MOVEIT_PLANNERS" --runs "$RUNS" --timeout "$TIMEOUT" --seed "$SEED" \
    --out /workspace/results

echo "[3/4] run cuRobo (curobo container, GPU)"
$COMPOSE run --rm curobo \
  mb-benchmark run --scenarios /workspace/scenarios/generated --no-autogen \
    --planners curobo --runs "$RUNS" --timeout "$TIMEOUT" --seed "$SEED" \
    --out /workspace/results

echo "[4/4] score + report"
$COMPOSE run --rm ros \
  mb-benchmark report --scenarios /workspace/scenarios/generated --no-autogen \
    --raw /workspace/results/raw --out /workspace/results/report

echo "Done. See results/report/report.md"
