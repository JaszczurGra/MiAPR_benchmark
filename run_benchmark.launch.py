import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

UR_TYPE = "ur5e"
BENCHMARK_YAML = "/workspace/benchmark.yaml"
WAREHOUSE = "/workspace/results/moveit_benchmarks/warehouse_ros.sqlite"

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("ur_description"), "urdf", "ur.urdf.xacro"),
            mappings={"ur_type": UR_TYPE, "name": "ur", "tf_prefix": ""})
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": UR_TYPE})
        .robot_description_kinematics(Path("config") / "kinematics.yaml")
        .joint_limits(Path("config") / "joint_limits.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    moveit_cpp_pipeline_params = {
        "planning_pipelines": {
            "pipeline_names": ["ompl"],
        },
    }

    return LaunchDescription([
        Node(
            package="moveit_ros_benchmarks",
            executable="moveit_run_benchmark",
            output="screen",
            parameters=[
                BENCHMARK_YAML,
                moveit_config.to_dict(),
                moveit_cpp_pipeline_params,
                {"warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
                 "warehouse_host": WAREHOUSE},
            ],
        )
    ])
