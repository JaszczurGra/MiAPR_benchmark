#!/usr/bin/env bash
# Entrypoint for the `ros` service: source ROS + the workspace overlay, then exec the cmd.
set -e
source /opt/ros/jazzy/setup.bash
# /workspace is bind-mounted from the host (docker-compose.yml), which shadows the
# workspace that was built into the image. Build it against the live source on first run;
# the result lands in the (git-ignored) host ros2_ws/install and persists, so later runs
# are fast. After editing a package's source, rebuild: `cd ros2_ws && colcon build`.
if [ ! -f /workspace/ros2_ws/install/setup.bash ] && [ -d /workspace/ros2_ws/src ]; then
  echo "[entrypoint] building ros2_ws against the live source (first run)..."
  (cd /workspace/ros2_ws && colcon build --symlink-install)
fi
if [ -f /workspace/ros2_ws/install/setup.bash ]; then
  source /workspace/ros2_ws/install/setup.bash
fi
# Refresh ONLY the harness code from the (possibly host-mounted) source. --no-deps avoids
# re-resolving the scientific stack (NumPy/etc.) under moveit_py; all deps are in the image.
# Non-editable (`-e` needs setuptools>=64 / PEP 660, which the base lacks).
mkdir -p /workspace/results/moveit_benchmarks
pip install -q --no-deps --force-reinstall /workspace/benchmark || true
exec "$@"
