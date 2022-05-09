'''
Simple demo to load up the aws world and spawn in one or more neobotix robots. 
'''

# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, \
                            SetEnvironmentVariable, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('multirobot_control')
    # nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    robot_model_dir = get_package_share_directory('neo_simulation2')
    warehouse_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    robot_base_dir = get_package_share_directory('robot_base')

    robots = [
        {'name': 'robot1', 'x_pose': 0.0, 'y_pose': 2.2, 'z_pose': 0.15, 'yaw_pose': 0.0},
        {'name': 'robot2', 'x_pose': 0.0, 'y_pose': 0.0, 'z_pose': 0.15, 'yaw_pose': 1.4},
        {'name': 'robot3', 'x_pose': 0.0, 'y_pose': 4.5, 'z_pose': 0.15, 'yaw_pose': 0.0},
    ]

    # Create the launch configuration variables
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    world = LaunchConfiguration('world')
    urdf = LaunchConfiguration('urdf')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value=os.path.join(
            # warehouse_dir, 'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world' ),
            # robot_model_dir, 'worlds', 'aws.world' ),
            robot_model_dir, 'worlds', 'neo_workshop.world' ),
        description="Full path to world file"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=os.path.join(
            # warehouse_dir, 'maps', '005', 'map.yaml' ),
            robot_model_dir, 'maps', 'neo_workshop.yaml' ),
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_urdf_cmd = DeclareLaunchArgument(
        'urdf',
        # default_value=os.path.join(bringup_dir, 'urdf', 'mp_400.urdf.xacro'),
        default_value=os.path.join(robot_base_dir, 'urdf', 'robot_base.xacro'),
        description='Full path to robot URDF to load'
    )

    # Launch commands

    gazebo_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world,
                'verbose': 'true',
            }.items()
        )

    # start_gazebo_server_cmd = ExecuteProcess(
    #     cmd=['gzserver', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so', world, '--verbose'],
    #     cwd=[warehouse_dir], output='screen' )

    # start_gazebo_client_cmd = ExecuteProcess(
    #     cmd=['gzclient'],
    #     cwd=[warehouse_dir], output='screen')

    spawn_robot_cmds = []
    for i, robot in enumerate(robots):
        spawn_robot_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'individual_bot.launch.py')
                ),
                launch_arguments={
                    'namespace': robot['name'],
                    'use_namespace': 'True',
                    'use_sim_time': 'True',
                    'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                    'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                    'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                    'yaw_pose': TextSubstitution(text=str(robot['yaw_pose'])),
                    'urdf': urdf,
                    'robot_num': str(i)  # To tag robots' colours
                }.items()
            )
        )

    start_navigation_cmds = []
    for robot in robots:
        start_navigation_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'individual_bot_nav_bringup.launch.py')
                ),
                launch_arguments={
                    'namespace': robot['name'],
                    'use_namespace': 'True',
                    'map': map_yaml_file,
                    'use_sim_time': 'True',
                    'params': params_file,
                    'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                    'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                    'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                    'yaw_pose': TextSubstitution(text=str(robot['yaw_pose'])),
                    'autostart': 'true'
                }.items()
            )
        )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_urdf_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(gazebo_cmd),
    # ld.add_action(start_gazebo_server_cmd)
    # ld.add_action(start_gazebo_client_cmd)
    for cmd in spawn_robot_cmds:
        ld.add_action(cmd)
    for cmd in start_navigation_cmds:
        ld.add_action(cmd)

    return ld
