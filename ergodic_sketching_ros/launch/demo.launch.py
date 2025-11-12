import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    This generates the full launch setup
    """

    # Set launch arguments
    namespace_name = "namespace"
    namespace = LaunchConfiguration(namespace_name)

    # Ergodic sketching arguments
    config_name = "config"
    base_frame_name = "base_frame"
    drawing_frame_xyz_name = "drawing_frame_xyz"
    drawing_frame_rpy_name = "drawing_frame_rpy"
    image_path_name = "image_path"

    config_file = LaunchConfiguration(config_name)
    base_frame = LaunchConfiguration(base_frame_name)
    drawing_frame_xyz = LaunchConfiguration(drawing_frame_xyz_name)
    drawing_frame_rpy = LaunchConfiguration(drawing_frame_rpy_name)
    image_path = LaunchConfiguration(image_path_name)

    # Declare them all
    namespace_launch_argument = DeclareLaunchArgument(
        namespace_name,
        default_value="",
        description="Namespace for the robot. If not set, the robot will be launched in the root namespace.",
    )

    drawing_config_launch_argument = DeclareLaunchArgument(
        config_name,
        default_value=PathJoinSubstitution(
        [
            "demos",
            "demo_drawing",
            "drawing.yaml",
        ]),
        description="Path to the config file for ergoding sketching.",
    )
    drawing_base_frame_launch_argument = DeclareLaunchArgument(
        base_frame_name,
        default_value="base",
        description="Frame used as the base frame in ergoding sketching.",
    )
    drawing_frame_xyz_launch_argument = DeclareLaunchArgument(
        drawing_frame_xyz_name,
        default_value="'0 0 0'",
        description="XYZ coordinates of the drawing frame.",
    )
    drawing_frame_rpy_launch_argument = DeclareLaunchArgument(
        drawing_frame_rpy_name,
        default_value="'0 0 0'",
        description="RPY coordinates of the drawing frame.",
    )
    image_path_launch_argument = DeclareLaunchArgument(
        image_path_name,
        default_value="",
        description="Absolute path to example image to draw.",
    )

    # Load Ergodic Sketching
    cwd = FindPackageShare("cooked_runtime")

    ergodic_sketching_server = Node(
        package="ergodic_sketching_ros",
        executable="ergodic_sketching_ros",
        name="ergodic_sketching_ros",
        namespace=namespace,
        # prefix=['gdb -ex run --arg'],
        parameters=[
            {
                "path": cwd,
                "config_file": config_file,
                "base_frame": base_frame,
                "drawing_frame_rpy": drawing_frame_rpy,
                "drawing_frame_xyz": drawing_frame_xyz,
            },
        ],
        output="both",
    )

    sketcher_client = Node(
        package="ergodic_sketching_ros",
        executable="sketcher_client.py",
        name="sketcher_client",
        namespace=namespace,
        parameters=[
            {
                "image_path": image_path,
            },
        ],
        output="both",
    )

    return LaunchDescription(
        [
            namespace_launch_argument,
            drawing_config_launch_argument,
            drawing_base_frame_launch_argument,
            drawing_frame_xyz_launch_argument,
            drawing_frame_rpy_launch_argument,
            image_path_launch_argument,
            ergodic_sketching_server,
            sketcher_client,
        ]
    )
