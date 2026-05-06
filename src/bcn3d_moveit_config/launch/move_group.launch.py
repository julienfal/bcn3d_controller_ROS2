from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("bcn3d_moveo", package_name="bcn3d_moveit_config")
        .to_moveit_configs()
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_monitored_planning_scene = LaunchConfiguration(
        "publish_monitored_planning_scene"
    )

    move_group_configuration = {
        "use_sim_time": use_sim_time,
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": publish_monitored_planning_scene,
        "publish_geometry_updates": publish_monitored_planning_scene,
        "publish_state_updates": publish_monitored_planning_scene,
        "publish_transforms_updates": publish_monitored_planning_scene,
        "monitor_dynamics": False,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_configuration,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("allow_trajectory_execution", default_value="true"),
            DeclareLaunchArgument(
                "publish_monitored_planning_scene", default_value="true"
            ),
            DeclareLaunchArgument(
                "capabilities",
                default_value=moveit_config.move_group_capabilities["capabilities"],
            ),
            DeclareLaunchArgument(
                "disable_capabilities",
                default_value=moveit_config.move_group_capabilities[
                    "disable_capabilities"
                ],
            ),
            move_group_node,
        ]
    )
