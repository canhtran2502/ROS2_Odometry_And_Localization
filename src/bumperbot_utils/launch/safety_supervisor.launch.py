import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_python = LaunchConfiguration("use_python")
    enforcement = LaunchConfiguration("enforcement")
    config = LaunchConfiguration("config")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use the simulation clock (the protection loop stays on a wall timer)")
    use_python_arg = DeclareLaunchArgument(
        "use_python", default_value="false",
        description="Start the Python node instead of the C++ node")
    enforcement_arg = DeclareLaunchArgument(
        "enforcement", default_value="filter",
        description="filter (limit safety/cmd_vel_in) or lock_only (drive the lock topic only)")
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_utils"), "config", "safety_supervisor.yaml"),
        description="Parameter file of the safety supervisor")

    # The launch arguments override the parameter file
    parameters = [config, {"use_sim_time": use_sim_time, "enforcement": enforcement}]

    safety_supervisor_cpp = Node(
        package="bumperbot_utils",
        executable="safety_supervisor",
        name="safety_supervisor",
        output="screen",
        parameters=parameters,
        condition=UnlessCondition(use_python),
    )

    safety_supervisor_py = Node(
        package="bumperbot_utils",
        executable="safety_supervisor.py",
        name="safety_supervisor",
        output="screen",
        parameters=parameters,
        condition=IfCondition(use_python),
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_python_arg,
        enforcement_arg,
        config_arg,
        safety_supervisor_cpp,
        safety_supervisor_py,
    ])
