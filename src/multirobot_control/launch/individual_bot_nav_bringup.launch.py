'''
Launch file for the navigation stack of an individual robot
'''
import os
from re import S
from tracemalloc import start

from click import launch

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, \
                            SetEnvironmentVariable, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, \
                            TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    bringup_dir = get_package_share_directory('multirobot_control')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    robot_model_dir = get_package_share_directory('neo_simulation2')
    warehouse_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    
    # Arguments for namespacing
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    remappings_tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    remap_cmd_vel = [('/cmd_vel', 'cmd_vel')]

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial x-pose of robot')

    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial y-pose of robot')

    declare_z_pose_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.01',
        description='Initial z-pose of robot')

    declare_yaw_pose_cmd = DeclareLaunchArgument(
        'yaw_pose',
        default_value='3.14',
        description="Initial yaw of robot"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=os.path.join(
            robot_model_dir, 'maps', 'neo_workshop.yaml' ),
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    start_gt_cmd = Node(
        package='multirobot_control',
        executable='gt_odom',
        name='base_footprint_gt',
        namespace=namespace,
        arguments=[
            '--link_name', PythonExpression(["'", namespace, "' + 'base_footprint'"]),
            '--namespace', namespace
        ],
        output='screen'
    )

    start_static_transform_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=PythonExpression(["'", namespace, "' + 'odom_to_map_tf_pub'"]),
        namespace=namespace,
        arguments=[ 
            '0', '0', '0', '0', '0', '0', '1',
            'map', PythonExpression(["'", namespace, "' + 'odom'"]),
        ],  # Same frame
        output='screen'
    )

    # Launch localization to give map -> transform
    start_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'map_yaml_file': map_yaml_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'x_pose' : x_pose,
            'y_pose' : y_pose,
            'z_pose' : z_pose,
            'yaw_pose' : yaw_pose,
        }.items()
    )

    start_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'map_yaml_file': map_yaml_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'x_pose' : x_pose,
            'y_pose' : y_pose,
            'z_pose' : z_pose,
            'yaw_pose' : yaw_pose,
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_z_pose_cmd)
    ld.add_action(declare_yaw_pose_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    
    ld.add_action(start_gt_cmd)
    ld.add_action(start_static_transform_cmd)
    ld.add_action(start_localization_cmd)
    # ld.add_action(start_navigation_cmd)

    return ld