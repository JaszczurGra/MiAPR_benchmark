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

SEED="${SEED:-42}"
# runs/timeout come from config/planners.yaml (defaults:) -- the CLI reads them. Set the
# RUNS/TIMEOUT env vars here only to override the YAML for this invocation.
RUNS="${RUNS:-}"
TIMEOUT="${TIMEOUT:-}"
# Which planners each container runs. @<group> is expanded by mb-benchmark from
# config/planners.yaml (harness:); override with an explicit comma list if you want.
MOVEIT_PLANNERS="${MOVEIT_PLANNERS:-@moveit}"
CUROBO_PLANNERS="${CUROBO_PLANNERS:-@curobo}"

# Forward --runs/--timeout only when overridden above; otherwise the CLI uses the YAML.
RUN_FLAGS=()
[ -n "$RUNS" ]    && RUN_FLAGS+=(--runs "$RUNS")
[ -n "$TIMEOUT" ] && RUN_FLAGS+=(--timeout "$TIMEOUT")

echo "[1/4] generate identical queries -> scenarios/generated (MoveIt-validated)"
# --validate-moveit: start/goal states are checked against MoveIt's mesh + self-collision
# model (run here in the `ros` container) so the shared query set is valid under the
# STRICTEST planner. cuRobo/straightline are more permissive, so the same queries work for
# them too -- keeping the comparison fair.


$COMPOSE run --rm ros \
  mb-benchmark generate --scenarios /workspace/scenarios/library \
    --out /workspace/scenarios/generated --num 20 --seed "$SEED" --validate-moveit

echo "[2/4] run MoveIt planners (ros container)"
$COMPOSE run --rm ros \
  mb-benchmark run --scenarios /workspace/scenarios/generated --no-autogen \
    --planners "$MOVEIT_PLANNERS" "${RUN_FLAGS[@]+"${RUN_FLAGS[@]}"}" --seed "$SEED" \
    --out /workspace/results

echo "[3/4] run cuRobo (curobo container, GPU)"
$COMPOSE run --rm curobo \
  mb-benchmark run --scenarios /workspace/scenarios/generated --no-autogen \
    --planners "$CUROBO_PLANNERS" "${RUN_FLAGS[@]+"${RUN_FLAGS[@]}"}" --seed "$SEED" \
    --out /workspace/results

echo "[4/4] score + report"
$COMPOSE run --rm ros \
  mb-benchmark report --scenarios /workspace/scenarios/generated --no-autogen \
    --raw /workspace/results/raw --out /workspace/results/report

echo "Done. See results/report/report.md"
