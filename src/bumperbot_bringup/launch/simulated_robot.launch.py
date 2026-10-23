import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value = "false"
    )

    simulator_arg = DeclareLaunchArgument(
        "simulator",
        default_value = "gazebo",
        description = "Physics simulator: gazebo or mujoco"
    )
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value = "empty",
        description = "World to load (empty, small_house, small_warehouse)"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value = "true",
        description = "Start RViz"
    )
    viewer_arg = DeclareLaunchArgument(
        "viewer",
        default_value = "true",
        description = "Open the MuJoCo viewer window (simulator:=mujoco only)"
    )

    use_slam = LaunchConfiguration("use_slam")
    use_rviz = LaunchConfiguration("use_rviz")
    viewer = LaunchConfiguration("viewer")
    simulator = LaunchConfiguration("simulator")
    world_name = LaunchConfiguration("world_name")

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_description"),
            "launch",
            "gazebo.launch.py"
        ),
        launch_arguments={
            "world_name": world_name
        }.items(),
        condition=IfCondition(PythonExpression(["'", simulator, "' == 'gazebo'"]))
    )

    mujoco = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mujoco"),
            "launch",
            "mujoco.launch.py"
        ),
        launch_arguments={
            "world_name": world_name,
            "viewer": viewer
        }.items(),
        condition=IfCondition(PythonExpression(["'", simulator, "' == 'mujoco'"]))
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    safety_stop = Node(
        package="bumperbot_utils",
        executable="safety_stop",
        output="screen"
    )

    rviz_localization = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "rviz",
            "global_localization.rviz"
        )],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition = IfCondition(PythonExpression(
            ["'", use_rviz, "'.lower() == 'true' and '", use_slam, "'.lower() != 'true'"]))
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "rviz",
            "slam.rviz"
        )],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition = IfCondition(PythonExpression(
            ["'", use_rviz, "'.lower() == 'true' and '", use_slam, "'.lower() == 'true'"]))
    )
    
    return LaunchDescription([
        use_slam_arg,
        use_rviz_arg,
        viewer_arg,
        simulator_arg,
        world_name_arg,
        gazebo,
        mujoco,
        controller,
        joystick,
        safety_stop,
        localization,
        slam,
        rviz_localization,
        rviz_slam
    ])