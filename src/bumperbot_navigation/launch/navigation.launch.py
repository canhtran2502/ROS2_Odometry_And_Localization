import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use the simulation clock"
    )
    nav2_params_arg = DeclareLaunchArgument(
        "nav2_params",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "config",
            "nav2_params.yaml"
        ),
        description="Nav2 parameter file"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_params = LaunchConfiguration("nav2_params")

    # Nav2 velocity goes to the low-priority twist_mux input "nav_vel"
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel", "nav_vel")]
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}]
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel", "nav_vel")]
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}]
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator"
            ]
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        nav2_params_arg,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager
    ])
