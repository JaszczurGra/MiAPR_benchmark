"""TASK 1 — bring up a UR5e simulation in ROS 2 (mock hardware) + MoveIt + RViz.

Mock ("fake") hardware from ros2_control is enough for *planning* benchmarks: MoveIt
plans against the planning scene; no physics engine is required. For execution / visual
validation against physics, use ``gazebo.launch.py`` instead.

Run (inside the `ros` container, after sourcing the workspace):
    ros2 launch mb_bringup sim.launch.py
    # arguments: ur_type:=ur5e launch_rviz:=true

NOTE: with mock hardware the scaled_joint_trajectory_controller does not work; we fall
back to joint_trajectory_controller (see Universal Robots ROS 2 driver docs).

"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")

    args = [
        DeclareLaunchArgument("ur_type", default_value="ur5e",
                              description="UR model (ur5e default; ur5 for strict CB-series)."),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value="/workspace/results/moveit_benchmarks/warehouse_ros.sqlite",
            description=(
                "Path to warehouse_ros SQLite database. "
                "Pass a fresh path (e.g. /tmp/warehouse.sqlite) if you hit "
                "'no such column: M_planning_scene_id' — a known warehouse_ros_sqlite 1.0.5 bug."
            ),
        ),
    ]

    # 1) Driver with mock hardware -> publishes robot state + ros2_control controllers.
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": "0.0.0.0",  # ignored with mock hardware
            "use_mock_hardware": "true",
            "mock_sensor_commands": "true",
            "initial_joint_controller": "joint_trajectory_controller",
            "launch_rviz": "false",
        }.items(),
    )

    # 2) MoveIt (move_group) + RViz motion-planning plugin for the same robot.
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_mock_hardware": "true",
            "launch_rviz": launch_rviz,
            "warehouse_sqlite_path": warehouse_sqlite_path,
        }.items(),
    )

    return LaunchDescription(args + [driver, moveit])
