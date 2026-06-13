#!/usr/bin/env bash
# Entrypoint for the `ros` service: source ROS + the built workspace, then exec the cmd.
set -e
source /opt/ros/humble/setup.bash
if [ -f /workspace/ros2_ws/install/setup.bash ]; then
  source /workspace/ros2_ws/install/setup.bash
fi
# Make the editable harness importable even if /workspace was re-mounted over the image.
pip install -q -e /workspace/benchmark >/dev/null 2>&1 || true
exec "$@"
