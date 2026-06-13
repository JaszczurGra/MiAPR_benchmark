"""Optional: UR5e in Gazebo (physics) + MoveIt, for execution / visual validation.

Planning benchmarks do NOT need this (use ``sim.launch.py`` with mock hardware). This is
here for completeness, e.g. to actually execute a planned trajectory and watch it.

Run (inside the `ros` container):
    ros2 launch mb_bringup gazebo.launch.py ur_type:=ur5e

Requires the ``ur_simulation_gz`` package (Gazebo / gz-sim).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    args = [DeclareLaunchArgument("ur_type", default_value="ur5e")]

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_moveit.launch.py"])
        ),
        launch_arguments={"ur_type": ur_type}.items(),
    )
    return LaunchDescription(args + [gz])
