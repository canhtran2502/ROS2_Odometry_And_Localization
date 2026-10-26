import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory("bumperbot_navigation")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use the simulation clock"
    )
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="false",
        description="Run the Python dock_manager instead of the C++ one"
    )
    use_battery_monitor_arg = DeclareLaunchArgument(
        "use_battery_monitor",
        default_value="true",
        description="Start the battery monitor (automatic docking on low battery)"
    )
    stations_file_arg = DeclareLaunchArgument(
        "stations_file",
        default_value=os.path.join(share, "config", "stations.yaml"),
        description="Stations file; its dock station is the docking staging pose"
    )
    dock_config_arg = DeclareLaunchArgument(
        "dock_config",
        default_value=os.path.join(share, "config", "dock_manager.yaml"),
        description="dock_manager parameter file"
    )
    battery_config_arg = DeclareLaunchArgument(
        "battery_config",
        default_value=os.path.join(share, "config", "battery_monitor.yaml"),
        description="battery_monitor parameter file"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_python = LaunchConfiguration("use_python")
    use_battery_monitor = LaunchConfiguration("use_battery_monitor")
    stations_file = LaunchConfiguration("stations_file")
    dock_config = LaunchConfiguration("dock_config")
    battery_config = LaunchConfiguration("battery_config")

    dock_parameters = [
        dock_config,
        {"stations_file": stations_file, "use_sim_time": use_sim_time}
    ]

    dock_manager_cpp = Node(
        package="bumperbot_navigation",
        executable="dock_manager",
        name="dock_manager",
        output="screen",
        parameters=dock_parameters,
        condition=UnlessCondition(use_python)
    )

    dock_manager_py = Node(
        package="bumperbot_navigation",
        executable="dock_manager.py",
        name="dock_manager",
        output="screen",
        parameters=dock_parameters,
        condition=IfCondition(use_python)
    )

    battery_monitor = Node(
        package="bumperbot_navigation",
        executable="battery_monitor.py",
        name="battery_monitor",
        output="screen",
        parameters=[battery_config, {"use_sim_time": use_sim_time}],
        condition=IfCondition(use_battery_monitor)
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_python_arg,
        use_battery_monitor_arg,
        stations_file_arg,
        dock_config_arg,
        battery_config_arg,
        dock_manager_cpp,
        dock_manager_py,
        battery_monitor
    ])
