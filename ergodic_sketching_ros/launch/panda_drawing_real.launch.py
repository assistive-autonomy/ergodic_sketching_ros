#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, AndSubstitution, NotSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip_parameter_name = 'robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    arm_id = "panda"

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    config_file = LaunchConfiguration("config_file")
    base_frame = LaunchConfiguration("base_frame")
    use_gui = LaunchConfiguration("use_gui")
    drawing_frame_xyz = LaunchConfiguration("drawing_frame_xyz")
    drawing_frame_rpy = LaunchConfiguration("drawing_frame_rpy")

    cwd = FindPackageShare("ergodic_sketching_ros")

    planner_config_file = PathJoinSubstitution(
        [
            FindPackageShare("ergodic_sketching_ros"),
            "config",
            "ilqr_planner_config_panda.yaml"
        ]
    )

    franka_xacro_file = os.path.join(get_package_share_directory('ergodic_sketching_ros'), 'descriptions',
                                     'panda_drawing.urdf.xacro')
    robot_description = Command(
        [
            FindExecutable(name='xacro'), ' ', franka_xacro_file,
            ' hand:=', load_gripper,
            ' arm_id:=', arm_id,
            ' robot_ip:=', robot_ip, 
            ' use_fake_hardware:=', use_fake_hardware,
            ' fake_sensor_commands:=', fake_sensor_commands
        ]
    )

    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')

    franka_controllers = PathJoinSubstitution(
        [
            FindPackageShare('py_panda2'),
            'config',
            'single_robot_controllers.yaml',
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value="drozbot_config_panda.yaml",
            description="Configuration file for ergodic sketcher",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="world",
            description="Base frame",
        ),
        DeclareLaunchArgument(
            "drawing_frame_xyz",
            default_value="0.58992866 0.19507239 0.00578767",
            description="Drawing frame position",
        ),
        DeclareLaunchArgument(
            "drawing_frame_rpy",
            default_value="0.00448539 -0.0175784  -2.47236401",
            description="Drawing frame orientation",
        ),
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            description='Hostname or IP address of the robot.'),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='true',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description="Fake sensor commands. Only valid when '{}' is true".format(
                use_fake_hardware_parameter_name)),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='false',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'source_list': ['/panda/franka_state_publisher/joint_states', '/panda/panda_gripper/joint_states'],
                 'rate': 30}],
        ),
        Node(
            package='franka_control2',
            executable='franka_control2_node',
            parameters=[{'robot_description': robot_description}, franka_controllers],
            remappings=[('joint_states', 'panda/franka_state_publisher/joint_states'),('/panda/robot_description', '/robot_description')],
            namespace=arm_id,
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            on_exit=Shutdown(),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=arm_id,
            arguments=['franka_state_publisher'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=["--inactive", "joint_impedance_controller","cartesian_impedance_controller","joint_velocity_controller","joint_torque_controller"],
            output='screen',
            namespace=arm_id,
            condition=UnlessCondition(use_fake_hardware),
        ),
        Node(
            package='franka_gripper',
            executable='franka_gripper_node',
            name=[arm_id, '_gripper'],
            parameters=[{'robot_ip': robot_ip, 'joint_names': [f"{arm_id}_finger_joint1",f"{arm_id}_finger_joint2"]}],
            namespace=arm_id,
            condition=IfCondition(AndSubstitution(NotSubstitution(use_fake_hardware),load_gripper))
        ),
        Node(
            package='franka_gripper',
            executable='fake_gripper_state_publisher.py',
            name=[arm_id, '_gripper'],
            parameters=[{'robot_ip': robot_ip, 'joint_names': [f"{arm_id}_finger_joint1",f"{arm_id}_finger_joint2"]}],
            namespace=arm_id,
            condition=IfCondition(AndSubstitution(use_fake_hardware,load_gripper))
        ),
        Node(
            package="ergodic_sketching_ros",
            executable="planner_action_server",
            name="planner_action_server",
            parameters=[
                planner_config_file,
                {
                    "robot_description":robot_description,
                    "drawing_frame_rpy":drawing_frame_rpy,
                    "drawing_frame_xyz":drawing_frame_xyz,

                }
            ],
            output="both",
        ),
        Node(
            package="ergodic_sketching_ros",
            executable="postprocessing_ros",
            name="postprocessing_ros",
            parameters=[
                {
                    "robot_description":robot_description,
                    "path":cwd,
                    "config_file":config_file,
                    "base_frame":base_frame,
                    "drawing_frame_rpy":drawing_frame_rpy,
                    "drawing_frame_xyz":drawing_frame_xyz,
                },
            ],
            output="both",
        ),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
             condition=IfCondition(use_rviz)
             )

    ])
