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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')

    lifecycle_nodes = ['map_server', 'amcl']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    # This remaps the configuration of each node to take in the TF 'prefix' of each
    # individual robot, for instance for robot1, "base_footprint" --> "robot1base_footprint"
    # But requires knowledge of the structure of each parameter. It will be hardcoded.
    
    # ? Potential error down the line: 
    # `local_costmap` and a bunch of other stuff use `global_frame` to define different things.
    # For `local_costmap`, `global_frame` refers to `odom`
    # For `global_costmap` and other stuff, it refers to `map`

    append_base_footprint = PythonExpression(["'", namespace, "' + 'base_footprint'"]),
    append_odom_frame = PythonExpression(["'", namespace, "' + 'odom'"]),
    append_scan_topic = PythonExpression(["'/' + '", namespace, "' + '/scan'"]),
    append_odom_topic = PythonExpression(["'/' + '", namespace, "' + '/odom'"]),

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file,
        # AMCL node
        'base_frame_id': append_base_footprint,
        'odom_frame_id': append_odom_frame,
        'scan_topic': append_scan_topic,
        # bt_navigator node
        'robot_base_frame': append_base_footprint,
        'odom_topic': append_odom_topic,
    }

    # The root key is '' as the namespace is applied to each _frame_
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'),
            description='Full path to map yaml file to load'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description='Initial x-pose of robot'),

        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='Initial y-pose of robot'),

        DeclareLaunchArgument(
            'z_pose', default_value='0.01',
            description='Initial z-pose of robot'),

        DeclareLaunchArgument(
            'yaw_pose', default_value='3.14',
            description="Initial yaw of robot"),

        Node(
            package='nav2_map_server',
            executable='map_server',
            # name=PythonExpression(["'", namespace, "' + '_map_server'"]),
            name='map_server',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time' : use_sim_time,
                'yaml_filename' : map_yaml_file
            }],
            remappings=remappings
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            # name=PythonExpression(["'", namespace, "' + '_amcl'"]),
            name='amcl',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                # 'alpha1': 0.2,
                # 'alpha2': 0.2,
                # 'alpha3': 0.2,
                # 'alpha4': 0.2,
                # 'alpha5': 0.2,
                'base_frame_id': append_base_footprint,
                # 'beam_skip_distance': 0.5,
                # 'beam_skip_error_threshold': 0.9,
                # 'beam_skip_threshold': 0.3,
                # 'do_beamskip': false,
                'global_frame_id': "map",
                # 'lambda_short': 0.1,
                # 'laser_likelihood_max_dist': 2.0,
                # 'laser_max_range': 100.0,
                # 'laser_min_range': -1.0,
                # 'laser_model_type': "likelihood_field",
                # 'max_beams': 60,
                # 'max_particles': 2000,
                # 'min_particles': 500,
                'odom_frame_id': append_odom_frame,
                # 'pf_err': 0.05,
                # 'pf_z': 0.99,
                # 'recovery_alpha_fast': 0.0,
                # 'recovery_alpha_slow': 0.0,
                # 'resample_interval': 1,
                # 'robot_model_type': "differential",
                # 'save_pose_rate': 0.5,
                # 'sigma_hit': 0.2,
                # 'tf_broadcast': true,
                # 'transform_tolerance': 1.0,
                # 'update_min_a': 0.2,
                # 'update_min_d': 0.25,
                # 'z_hit': 0.5,
                # 'z_max': 0.05,
                # 'z_rand': 0.5,
                # 'z_short': 0.05,
                'scan_topic': append_scan_topic
            }],
            remappings=remappings
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            # name=PythonExpression(["'", namespace, "' + '_lifecycle_manager_localization'"]),
            name="lifecycle_manager_localization",
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]
        )
    ])
