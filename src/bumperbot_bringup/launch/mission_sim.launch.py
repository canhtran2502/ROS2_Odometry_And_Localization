import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    navigation_share = get_package_share_directory("bumperbot_navigation")

    simulator_arg = DeclareLaunchArgument(
        "simulator",
        default_value="gazebo",
        description="Physics simulator: gazebo or mujoco"
    )
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="small_house",
        description="World to load (must match the saved map used by AMCL)"
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
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="false",
        description="Use the Python mission_manager instead of the C++ one"
    )
    stations_file_arg = DeclareLaunchArgument(
        "stations_file",
        default_value=os.path.join(navigation_share, "config", "stations.yaml"),
        description="Stations file used by the mission manager"
    )
    mission_config_arg = DeclareLaunchArgument(
        "mission_config",
        default_value=os.path.join(navigation_share, "config", "mission_manager.yaml"),
        description="mission_manager parameter file"
    )

    use_python = LaunchConfiguration("use_python")

    # Simulator + controller + twist_mux + safety_stop + map_server/AMCL (+ RViz)
    simulated_robot = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_bringup"),
            "launch",
            "simulated_robot.launch.py"
        ),
        launch_arguments={
            "use_slam": "False",
            "simulator": LaunchConfiguration("simulator"),
            "world_name": LaunchConfiguration("world_name"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "viewer": LaunchConfiguration("viewer"),
        }.items()
    )

    # Nav2 planner, controller, behaviors and bt_navigator
    navigation = IncludeLaunchDescription(
        os.path.join(navigation_share, "launch", "navigation.launch.py"),
        launch_arguments={
            "use_sim_time": "true"
        }.items()
    )

    mission_parameters = [
        LaunchConfiguration("mission_config"),
        {"stations_file": LaunchConfiguration("stations_file"), "use_sim_time": True}
    ]

    mission_manager_cpp = Node(
        package="bumperbot_navigation",
        executable="mission_manager",
        name="mission_manager",
        output="screen",
        parameters=mission_parameters,
        condition=UnlessCondition(use_python)
    )

    mission_manager_py = Node(
        package="bumperbot_navigation",
        executable="mission_manager.py",
        name="mission_manager",
        output="screen",
        parameters=mission_parameters,
        condition=IfCondition(use_python)
    )

    return LaunchDescription([
        simulator_arg,
        world_name_arg,
        use_rviz_arg,
        viewer_arg,
        use_python_arg,
        stations_file_arg,
        mission_config_arg,
        simulated_robot,
        navigation,
        mission_manager_cpp,
        mission_manager_py
    ])
