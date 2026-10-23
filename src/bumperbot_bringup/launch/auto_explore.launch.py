import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_explorer(context, *args, **kwargs):
    # map_save_path overrides the YAML value only when it is not empty
    parameters = [
        LaunchConfiguration("explorer_config").perform(context),
        {"use_sim_time": True},
    ]
    map_save_path = LaunchConfiguration("map_save_path").perform(context)
    if map_save_path:
        parameters.append({"map_save_path": map_save_path})

    return [
        Node(
            package="bumperbot_mapping",
            executable="frontier_explorer.py",
            name="frontier_explorer",
            output="screen",
            parameters=parameters
        )
    ]


def generate_launch_description():
    simulator_arg = DeclareLaunchArgument(
        "simulator",
        default_value="gazebo",
        description="Physics simulator: gazebo or mujoco"
    )
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="small_house",
        description="World to load (empty, small_house, small_warehouse)"
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz"
    )
    viewer_arg = DeclareLaunchArgument(
        "viewer",
        default_value="true",
        description="Open the MuJoCo viewer window (simulator:=mujoco only)"
    )
    map_save_path_arg = DeclareLaunchArgument(
        "map_save_path",
        default_value="",
        description="Where the explored map is saved (empty = value of the explorer YAML)"
    )
    explorer_config_arg = DeclareLaunchArgument(
        "explorer_config",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "config",
            "frontier_explorer.yaml"
        ),
        description="frontier_explorer parameter file"
    )

    # Simulator + controller + twist_mux + safety_stop + slam_toolbox (+ RViz)
    simulated_robot = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_bringup"),
            "launch",
            "simulated_robot.launch.py"
        ),
        launch_arguments={
            "use_slam": "True",
            "simulator": LaunchConfiguration("simulator"),
            "world_name": LaunchConfiguration("world_name"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "viewer": LaunchConfiguration("viewer"),
        }.items()
    )

    return LaunchDescription([
        simulator_arg,
        world_name_arg,
        use_rviz_arg,
        viewer_arg,
        map_save_path_arg,
        explorer_config_arg,
        simulated_robot,
        OpaqueFunction(function=launch_explorer),
    ])
