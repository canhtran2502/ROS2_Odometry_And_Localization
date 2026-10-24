import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    navigation_share = get_package_share_directory("bumperbot_navigation")
    mujoco_share = get_package_share_directory("bumperbot_mujoco")
    bringup_share = get_package_share_directory("bumperbot_bringup")

    simulator_arg = DeclareLaunchArgument(
        "simulator",
        default_value="mujoco",
        description="Physics simulator: mujoco or gazebo (no dock marker emulator with gazebo)"
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
        description="Use the Python dock_manager and mission_manager instead of the C++ ones"
    )
    battery_initial_arg = DeclareLaunchArgument(
        "battery_initial",
        default_value="1.0",
        description="Initial battery charge of the simulated battery (0..1)"
    )
    battery_sim_config_arg = DeclareLaunchArgument(
        "battery_sim_config",
        default_value=os.path.join(navigation_share, "config", "battery_sim.yaml"),
        description="battery_sim parameter file (simulation only)"
    )
    dock_marker_config_arg = DeclareLaunchArgument(
        "dock_marker_config",
        default_value=os.path.join(mujoco_share, "config", "dock_marker_sim.yaml"),
        description="dock_marker_sim parameter file (simulation only)"
    )

    simulator = LaunchConfiguration("simulator")
    use_python = LaunchConfiguration("use_python")

    # Simulated stack with Nav2 and mission_manager
    mission_sim = IncludeLaunchDescription(
        os.path.join(bringup_share, "launch", "mission_sim.launch.py"),
        launch_arguments={
            "simulator": simulator,
            "world_name": LaunchConfiguration("world_name"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "viewer": LaunchConfiguration("viewer"),
            "use_python": use_python,
        }.items()
    )

    # dock_manager and battery_monitor
    docking = IncludeLaunchDescription(
        os.path.join(navigation_share, "launch", "docking.launch.py"),
        launch_arguments={
            "use_sim_time": "true",
            "use_python": use_python,
        }.items()
    )

    # SIMULATION ONLY: simulated battery
    battery_sim = Node(
        package="bumperbot_navigation",
        executable="battery_sim.py",
        name="battery_sim",
        output="screen",
        parameters=[
            LaunchConfiguration("battery_sim_config"),
            {
                "use_sim_time": True,
                "initial_percentage": ParameterValue(LaunchConfiguration("battery_initial"), value_type=float),
            }
        ]
    )

    # SIMULATION ONLY: emulated dock marker detector, needs the MuJoCo ground truth
    dock_marker_sim = Node(
        package="bumperbot_mujoco",
        executable="dock_marker_sim",
        name="dock_marker_sim",
        output="screen",
        parameters=[
            LaunchConfiguration("dock_marker_config"),
            {"use_sim_time": True}
        ],
        condition=IfCondition(PythonExpression(["'", simulator, "' == 'mujoco'"]))
    )

    return LaunchDescription([
        simulator_arg,
        world_name_arg,
        use_rviz_arg,
        viewer_arg,
        use_python_arg,
        battery_initial_arg,
        battery_sim_config_arg,
        dock_marker_config_arg,
        mission_sim,
        docking,
        battery_sim,
        dock_marker_sim
    ])
