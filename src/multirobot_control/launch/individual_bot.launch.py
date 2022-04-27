'''
Launch file for an individual robot, complete with its own namespace etc.
'''
import os
from tracemalloc import start

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
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    urdf = LaunchConfiguration('urdf')

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

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=os.path.join(
            warehouse_dir, 'maps', '005', 'map.yaml' ),
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_urdf_cmd = DeclareLaunchArgument(
        'urdf',
        default_value=os.path.join(bringup_dir, 'urdf', 'mp_400.urdf'),
        description='Full path to robot URDF to load'
    )

    # os.environ["GAZEBO_MODEL_PATH"] = os.environ.get("GAZEBO_MODEL_PATH") + ':' \
    #     + os.path.join(robot_model_dir, 'components') + ':' \
    #     + os.path.join(robot_model_dir, 'robots')

    # print(os.environ.get("GAZEBO_MODEL_PATH"))

    spawn_entity_cmd = Node(
        package='multirobot_control',
        executable='spawn_single_bot',
        arguments=[
            '--robot_name', namespace,
            '--robot_namespace', namespace,
            '--urdf', urdf,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose
        ],
        output='screen'
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        remappings=remappings_tf,
        parameters=[{'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf]),
            'frame_prefix': namespace
        }]
    )

    start_tf_relay_cmd = Node(
        package='topic_tools',
        executable = 'relay',
        name=PythonExpression(["'", namespace, "' + '_relay_tf'"]),
        output='screen',
        namespace=namespace,
        parameters=[{
            # 'input_topic': '~/tf',
            'input_topic': PythonExpression(["'/'", "+", "'", namespace, "'", "+", "'/tf'"]),
            'output_topic': "/tf", 
            'lazy': False, 
            'stealth ': False, 
            'monitor_rate': 100.0
        }]
    )

    start_tf_static_relay_cmd = Node(
        package='topic_tools',
        executable = 'relay',
        name=PythonExpression(["'", namespace, "' + '_relay_tf_static'"]),
        output='screen',
        namespace=namespace,
        parameters=[{
            # 'input_topic': '~/tf_static',
            'input_topic': PythonExpression(["'/'", "+", "'", namespace, "'", "+", "'/tf_static'"]),
            'output_topic': "/tf_static", 
            'lazy': False, 
            'stealth ': False, 
            'monitor_rate': 100.0
        }]
    )

    # start_rviz_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
    #     launch_arguments={'namespace': namespace,
    #                       'use_namespace': 'True',
    #                       'use_sim_time': use_sim_time,
    #                       'rviz_config': os.path.join(
    #                             nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    #                       }.items()
    # )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        output='screen'
    )


    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_z_pose_cmd)
    ld.add_action(declare_yaw_pose_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_urdf_cmd)
    
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_tf_relay_cmd)
    ld.add_action(start_tf_static_relay_cmd)
    # ld.add_action(start_rviz_cmd)

    return ld