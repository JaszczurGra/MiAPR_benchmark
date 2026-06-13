"""TASK 2 — run moveit_ros_benchmarks for UR5e (Pipeline A).

Launches the `moveit_run_benchmark` executable with (a) our benchmark config and (b) the
UR5e MoveIt configuration (so the planners, kinematics and joint groups are available).
Results are written to the ``output_directory`` from benchmark.yaml as OMPL log files,
then turned into a database by scripts/process_results.sh for Planner Arena.

Prerequisites:
  * MongoDB warehouse running (docker-compose service `mongo`).
  * Warehouse populated: `ros2 run mb_moveit_benchmarks populate_warehouse.py`.

Run:
  ros2 launch mb_moveit_benchmarks benchmark.launch.py

NOTE (later agents): building the MoveIt config for benchmarking is the part most likely
to need iteration on a real ROS box (warehouse plugin params, planner-id naming). The
custom harness (Pipeline B) is the independent cross-check, so delivery does not hinge on
this launch succeeding first try.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Build the UR5e MoveIt config. (UR ships ur_moveit_config; this mirrors how
    # ur_moveit.launch.py assembles parameters.)
    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
        .robot_description(mappings={"ur_type": "ur5e", "name": "ur"})
        .to_moveit_configs()
    )

    benchmark_yaml = os.path.join(
        get_package_share_directory("mb_moveit_benchmarks"), "config", "benchmark.yaml"
    )

    # Warehouse plugin so moveit_run_benchmark can read scenes/queries from MongoDB.
    warehouse_params = {
        "warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection",
        "warehouse_host": "localhost",  # mongo uses host networking (docker-compose.yml)
        "warehouse_port": 27017,
    }

    run_benchmark = Node(
        package="moveit_ros_benchmarks",
        executable="moveit_run_benchmark",
        output="screen",
        parameters=[
            benchmark_yaml,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_params,
        ],
    )

    return LaunchDescription([run_benchmark])
