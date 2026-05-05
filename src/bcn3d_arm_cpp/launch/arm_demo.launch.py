from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("bcn3d_moveo", package_name="bcn3d_moveit_config")
        .to_moveit_configs()
    )

    move_arm_demo = Node(
        package="bcn3d_arm_cpp",
        executable="move_arm_demo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([move_arm_demo])
