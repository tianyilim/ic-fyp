import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, \
                            SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

from multirobot_control.set_rviz_config import set_rviz_config
from multirobot_control.parse_urdf import parse_urdf, parse_world, parse_scenario

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('multirobot_control')
    bringup_src = bringup_dir + "/../../../../src/multirobot_control"
    robot_base_dir = get_package_share_directory('robot_base')

    # Load in GAZEBO_MODEL_PATH (use src) directory
    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(bringup_src, "models") + ":" + os.environ.get("GAZEBO_MODEL_PATH")

    params_file_dir = os.path.join(bringup_src, 'params', 'params_custom.yaml')
    gazebo_param_path = os.path.join(bringup_src, 'params', 'gazebo_params.yaml')


    # Hardcoded scenario source
    scenario_file_dir = os.path.join(bringup_src, 'params', 'scenario_custom.yaml')
    if not os.path.isfile(scenario_file_dir):
        print(f"Ensure that {scenario_file_dir} exists!")
        return

    rviz_ref_file_dir = os.path.join(bringup_dir, 'rviz', 'rviz_config.rviz')
    rviz_file_dir = os.path.join(bringup_dir, 'rviz', 'rviz_config_.rviz')

    set_rviz_config(rviz_src=rviz_ref_file_dir, rviz_dest=rviz_file_dir, config_path=scenario_file_dir)

    with open(scenario_file_dir, 'r') as pf:
        scenario_configs = yaml.safe_load(pf)
    with open(params_file_dir, 'r') as f:
        param_configs = yaml.safe_load(f)

    robots = parse_scenario(scenario_configs)

    world_file = os.path.join(bringup_src, 'worlds', 'factory_world2.xacro')
    if 'realtime_factor' in scenario_configs['/**']['ros__parameters']:
        realtime_factor = scenario_configs['/**']['ros__parameters']['realtime_factor']
    else:
        realtime_factor = 1.0
    world = parse_world(world_file, realtime_factor)

    # Create the launch configuration variables
    params_file = LaunchConfiguration('params_file')
    headless_config = LaunchConfiguration('headless')
    log_level = LaunchConfiguration('log_level')
    goal_creation = LaunchConfiguration('goal_creation')
    rviz = LaunchConfiguration('rviz')

    # Ensure all logs come out in order
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file_dir,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description="Whether or not to launch full Gazebo GUI. If false, just launch RViz."
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description="Log level of nodes"
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description="Whether or not to launch RViz."
    )

    declare_goal_creation_cmd = DeclareLaunchArgument(
        'goal_creation',
        default_value='false',
        description='Whether to autostart goal_creation node'
    )

    # Launch commands
    gazebo_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world,
                'verbose': 'true',
                'extra_gazebo_args': f"--ros-args --params-file \"{gazebo_param_path}\"",
            }.items(),
            condition=UnlessCondition(headless_config)
        )

    gazebo_server_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={
                'world': world,
                'verbose': 'true',
                'extra_gazebo_args': f"--ros-args --params-file \"{gazebo_param_path}\"",
            }.items(),
            condition=IfCondition(headless_config)
        )

    spawn_robot_cmds = []
    start_navigation_cmds = []
    for i, robot in enumerate(robots):
        # Parse URDF here
        # ? URDF not using Launch arguments, hardcoded below
        urdf_filepath = os.path.join(robot_base_dir, 'urdf', 'robot_base.xacro')
        urdf = parse_urdf(urdf_filepath, i, robot['name'])

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
                    'robot_num': str(i),  # To tag robots' colours
                    'log_level': log_level
                }.items()
            )
        )
        start_navigation_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'individual_bot_nav_bringup.launch.py')
                ),
                launch_arguments={
                    'namespace': robot['name'],
                    'use_namespace': 'True',
                    'use_sim_time': 'True',
                    'params': params_file,
                    'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                    'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                    'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                    'yaw_pose': TextSubstitution(text=str(robot['yaw_pose'])),
                    'robot_num': str(i),  # To tag robots' colours
                    'local_planner': param_configs['/**']['ros__parameters']['local_planner'],
                    'log_level': log_level
                }.items()
            )
        )

    start_odom_distr_cmd = Node(
        package="multirobot_control",
        executable="odom_distribution",
        output='screen',
        parameters=[params_file, scenario_file_dir, {'use_sim_time': True}],
        arguments=['--ros-args', '--log-level', log_level]
    )

    start_map_viz_cmd = Node(
        package="multirobot_control",
        executable="map_visualisation",
        output="screen",
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': True}]
    )

    start_goal_creation_cmd = Node(
        package="multirobot_control",
        executable="goal_creation",
        output="screen",
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[scenario_file_dir, {'use_sim_time': True}],
        condition=IfCondition(goal_creation)
    )

    # Rviz does not need debug hooks
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d',  rviz_file_dir, '--ros-args', '--log-level', 'info'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz)
        # output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_goal_creation_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(gazebo_cmd)
    ld.add_action(gazebo_server_cmd)
    for cmd in spawn_robot_cmds:
        ld.add_action(cmd)
    for cmd in start_navigation_cmds:
        ld.add_action(cmd)

    ld.add_action(start_odom_distr_cmd)
    ld.add_action(start_map_viz_cmd)
    ld.add_action(start_goal_creation_cmd)
    ld.add_action(start_rviz_cmd)

    return ld