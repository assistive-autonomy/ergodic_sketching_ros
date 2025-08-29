#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the 'License');
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
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext

from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def robot_description_dependent_nodes_spawner(context: LaunchContext, arm_id, load_gripper, franka_hand):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    # Robot description
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        arm_id_str,
        arm_id_str + '.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str,
            'gazebo_effort': 'true'
        }
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Franka controllers
    franka_controllers = PathJoinSubstitution(
        [
            FindPackageShare("crisp_controllers_robot_demos"),
            "config",
            "fr3",
            "controllers.yaml",
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
        ]
    )

    crisp_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            franka_controllers,
            {"robot_description": robot_description},
        ],
        remappings=[("joint_states", "franka/joint_states")],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        on_exit=Shutdown(),
    )

    return [robot_state_publisher, crisp_controller_manager]

def generate_launch_description():
    """
    This generates the full launch setup
    """

    # Set launch arguments
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    arm_id_name = 'arm_id'
    namespace_name = 'namespace'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    arm_id = LaunchConfiguration(arm_id_name)
    namespace = LaunchConfiguration(namespace_name)

    # Ergodic sketching arguments
    drawing_config_name = 'config'
    drawing_base_frame_name = 'base_frame'
    drawing_frame_xyz_name = 'drawing_frame_xyz'
    drawing_frame_rpy_name = 'drawing_frame_rpy'

    drawing_config = LaunchConfiguration(drawing_config_name)
    drawing_base_frame = LaunchConfiguration(drawing_base_frame_name)
    drawing_frame_xyz = LaunchConfiguration(drawing_frame_xyz_name)
    drawing_frame_rpy = LaunchConfiguration(drawing_frame_rpy_name)

    # Declare them all
    load_gripper_launch_argument = DeclareLaunchArgument(
            load_gripper_name,
            default_value='false',
            description='true/false for activating the gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
            franka_hand_name,
            default_value='franka_hand',
            description='Default value: franka_hand')
    arm_id_launch_argument = DeclareLaunchArgument(
            arm_id_name,
            default_value='fr3',
            description='Available values: fr3, fp3 and fer')
    namespace_launch_argument = DeclareLaunchArgument(
        namespace_name,
        default_value='',
        description='Namespace for the robot. If not set, the robot will be launched in the root namespace.')

    drawing_config_launch_argument = DeclareLaunchArgument(
        drawing_config_name,
        default_value='',
        description='Path to the config file for ergoding sketching.')
    drawing_base_frame_launch_argument = DeclareLaunchArgument(
        drawing_base_frame_name,
        default_value='',
        description='Frame used as the base frame in ergoding sketching.')
    drawing_frame_xyz_launch_argument = DeclareLaunchArgument(
        drawing_frame_xyz_name,
        default_value='drawing_frame_xyz',
        description='XYZ coordinates of the drawing frame.')
    drawing_frame_rpy_launch_argument = DeclareLaunchArgument(
        drawing_frame_rpy_name,
        default_value='drawing_frame_rpy',
        description='RPY coordinates of the drawing frame.')

    # Get robot description dependent nodes
    robot_description_dependent_nodes_spawner_opaque_function = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[arm_id, load_gripper, franka_hand])

    # Gazebo Sim
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r', }.items(),
    )

    # Spawn
    spawn_gazebo = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    # Visualize in RViz
    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')
    rviz = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             namespace=namespace,
             arguments=['--display-config', rviz_file, '-f', 'world'],
    )

    # # Joint states
    # load_joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #             'joint_state_broadcaster'],
    #     output='screen'
    # )
    
    # Load joint state publisher
    joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[
            {
                "source_list": [
                    "franka/joint_states",
                    "franka_gripper/joint_states",
                ],
                "rate": 1000,
            }],
    )

    # CRISP controllers
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    cartesian_impedance_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cartesian_impedance_controller", "-c", "/controller_manager",],
        output="screen",
    )
    pose_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pose_broadcaster", "--inactive", "-c", "/controller_manager",],
        output="screen",
    )
    external_torque_broadcaster_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["external_torques_broadcaster", "--inactive", "-c", "/controller_manager",],
        output="screen",
    )

    # Load Ergodic Sketching
    cwd = FindPackageShare('ergodic_sketching_ros')
    planner_config_file = PathJoinSubstitution(
        [
            FindPackageShare('ergodic_sketching_ros'),
            'config',
            'ilqr_planner_config_panda.yaml'
        ]
    )

    ergodic_sketching_planner_action_server = Node(
            package='ergodic_sketching_ros',
            executable='planner_action_server',
            name='planner_action_server',
            parameters=[
                planner_config_file,
                {
                    # 'robot_description':robot_description,
                    'drawing_frame_rpy':drawing_frame_rpy,
                    'drawing_frame_xyz':drawing_frame_xyz,

                }
            ],
            output='both',
    )

    ergodic_sketching_postprocessing = Node(
            package='ergodic_sketching_ros',
            executable='postprocessing_ros',
            name='postprocessing_ros',
            parameters=[
                {
                    # 'robot_description':robot_description,
                    'path': cwd,
                    'config_file': drawing_config,
                    'base_frame': drawing_base_frame,
                    'drawing_frame_rpy': drawing_frame_rpy,
                    'drawing_frame_xyz': drawing_frame_xyz,
                },
            ],
            output='both',
    )

    # Fallback joint impedance controller
    joint_impedance_example_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_impedance_example_controller'],
        output='screen'
    )

    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        arm_id_launch_argument,
        namespace_launch_argument,
        drawing_config_launch_argument,
        drawing_base_frame_launch_argument,
        drawing_frame_xyz_launch_argument,
        drawing_frame_rpy_launch_argument,
        robot_description_dependent_nodes_spawner_opaque_function,
        gazebo_empty_world,
        rviz,
        spawn_gazebo,
        joint_state_broadcaster,
        joint_state_publisher,
        cartesian_impedance_controller,
        pose_broadcaster,
        external_torque_broadcaster_controller,
        # ergodic_sketching_planner_action_server,
        # ergodic_sketching_postprocessing,
        # RegisterEventHandler(
        #         event_handler=OnProcessExit(
        #             target_action=spawn_gazebo,
        #             on_exit=[load_joint_state_broadcaster],
        #         )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_broadcaster,
        #         on_exit=[joint_impedance_example_controller],
        #     )
        # ),
    ])
