'''
Launch file for the navigation stack of an individual robot
'''
import os

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
    
    # Arguments for namespacing
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    robot_num = LaunchConfiguration('robot_num')
    log_level = LaunchConfiguration('log_level')
    local_planner = LaunchConfiguration('local_planner')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    remappings_tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    remap_cmd_vel = [('/cmd_vel', 'cmd_vel')]

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'planner_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_robot_num_cmd = DeclareLaunchArgument(
        'robot_num',
        default_value='15',
        description='The number of the robot, used to assign colours to distinguish robots'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description="Log level of nodes"
    )

    declare_local_planner_cmd = DeclareLaunchArgument(
        'local_planner',
        default_value='dwa_action_server',
        description="Local planner name"
    )

    start_static_transform_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=PythonExpression(["'", namespace, "' + 'odom_to_map_tf_pub'"]),
        namespace=namespace,
        arguments=[ 
            '0', '0', '0', '0', '0', '0', '1',
            'map', PythonExpression(["'", namespace, "' + 'odom'"]),
            '--ros-args', '--log-level', 'warn'
        ],  # Same frame
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    start_rrt_cmd = Node(
        package='multirobot_control',
        executable='rrt_server',
        namespace=namespace,
        output='screen',
        parameters=[{"robot_num": robot_num,
                     "use_sim_time": use_sim_time},
                     params_file],
        arguments=['--robot_num', robot_num, '--ros-args', '--log-level', log_level]
    )

    start_dwa_server_cmd = Node(
        package='multirobot_control',
        executable='dwa_server',
        namespace=namespace,
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}, params_file],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(PythonExpression(['\"', local_planner, '\" == "dwa_action_server"']))
    )

    start_dwa_multirobot_cmd = Node(
        package='multirobot_control',
        executable='dwa_multirobot',
        namespace=namespace,
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}, params_file],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(PythonExpression(['\"', local_planner, '\" == "dwa_multirobot_server"']))
    )

    start_dwa_replan_cmd = Node(
        package='multirobot_control',
        executable='dwa_replan',
        namespace=namespace,
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}, params_file],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(PythonExpression(['\"', local_planner, '\" == "dwa_replan_server"']))
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_robot_num_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_local_planner_cmd)
    
    ld.add_action(start_static_transform_cmd)
    ld.add_action(start_dwa_server_cmd)
    ld.add_action(start_dwa_multirobot_cmd)
    ld.add_action(start_dwa_replan_cmd)
    ld.add_action(start_rrt_cmd)

    return ld