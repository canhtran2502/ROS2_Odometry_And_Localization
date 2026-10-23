import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bumperbot_description = get_package_share_directory("bumperbot_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(bumperbot_description, "urdf", "bumperbot.urdf.xacro"),
        description="Absolute path to robot urdf file",
    )
    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="small_house",
        description="MuJoCo world in bumperbot_mujoco/mjcf (empty, small_house, small_warehouse)",
    )
    viewer_arg = DeclareLaunchArgument(
        name="viewer", default_value="True", description="Open the MuJoCo viewer window"
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model"), " is_sim:=True simulator:=mujoco"]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # The controller manager runs on its own because there is no simulator plugin hosting it
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True},
            os.path.join(
                get_package_share_directory("bumperbot_controller"),
                "config",
                "bumperbot_controllers.yaml",
            ),
        ],
    )

    mujoco_sim = Node(
        package="bumperbot_mujoco",
        executable="mujoco_sim_node",
        output="screen",
        parameters=[{
            "world": LaunchConfiguration("world_name"),
            "viewer": LaunchConfiguration("viewer"),
        }],
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        viewer_arg,
        robot_state_publisher_node,
        controller_manager,
        mujoco_sim,
    ])
