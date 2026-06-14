#!/usr/bin/env bash
# TASK 1: launch the UR5e simulation (mock hardware) + MoveIt + RViz in the ros container.
# Requires: built images (make build) and, for RViz, `xhost +local:` on the host.
set -euo pipefail
cd "$(dirname "$0")/.."

UR_TYPE="${1:-ur5e}"
exec docker compose -f docker/docker-compose.yml run --rm ros \
  ros2 launch mb_bringup sim.launch.py ur_type:="${UR_TYPE}" launch_rviz:=true
