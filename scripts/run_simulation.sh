#!/usr/bin/env bash
# TASK 1: launch the UR5e simulation (mock hardware) + MoveIt + RViz in the ros container.
# Requires: built images (make build) and, for RViz, `xhost +local:` on the host.
set -euo pipefail
cd "$(dirname "$0")/.."
UR_TYPE="${1:-ur5e}"

docker compose -f docker/docker-compose.yml run --rm ros bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   ros2 launch ur_description view_ur.launch.py ur_type:=${UR_TYPE}" &

sleep 5

exec docker compose -f docker/docker-compose.yml run --rm ros bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /workspace/ros2_ws/install/setup.bash && \
   ros2 launch ur_moveit_config ur_moveit.launch.py \
   ur_type:=${UR_TYPE} use_mock_hardware:=true launch_rviz:=true \
   warehouse_plugin:=warehouse_ros_sqlite::DatabaseConnection \
   warehouse_host:=/root/.ros/warehouse_ros.sqlite"
