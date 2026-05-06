import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_bcn3d_gazebo = get_package_share_directory("bcn3d_gazebo")

    xacro_file = os.path.join(pkg_bcn3d_gazebo, "urdf", "moveo_gazebo.urdf.xacro")
    robot_description = {
        "robot_description": xacro.process_file(xacro_file).toxml(),
        "use_sim_time": True,
    }

    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true",
        description="Set to false to run gzserver without gzclient (much faster)",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_bcn3d_gazebo, "worlds", "pick_place.world"),
        description="Gazebo world file",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "verbose": "true",
            "gui": LaunchConfiguration("gui"),
            "world": LaunchConfiguration("world"),
        }.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "moveo",
        ],
        output="screen",
    )

    load_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    load_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )

    load_gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
    )

    # Order: spawn entity, then load controllers once spawn completes
    return LaunchDescription([
        gui_arg,
        world_arg,
        gazebo,
        rsp,
        spawn_entity,
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[load_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=load_jsb, on_exit=[load_arm])),
        RegisterEventHandler(OnProcessExit(target_action=load_arm, on_exit=[load_gripper])),
    ])
