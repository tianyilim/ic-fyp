# Author: Addison Sears-Collins
# Date: September 23, 2021
# Description: Load a URDF and world file into Gazebo.
# https://automaticaddison.com

import os
from xmlrpc.client import gzip_decode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

  # Constants for paths to different files and folders
  gazebo_models_path = 'models'
  package_name = 'robot_base'
  robot_name_in_model = 'robot_base'
  rviz_config_file_path = 'urdf/urdf.rviz'
  urdf_file_path = 'urdf/robot_base.xacro'
  world_file_path = 'worlds/warehouse.world'
    
  # Pose where we want to spawn the robot
  spawn_x_val = '0.0'
  spawn_y_val = '0.0'
  spawn_z_val = '0.0'
  spawn_yaw_val = '-1.57'

  ############ You do not need to change anything below this line #############
  
  # Set the path to different files and folders.  
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_share = FindPackageShare(package=package_name).find(package_name)
  pkg_src = os.path.join(pkg_share, '../../../../src/robot_base')

  default_urdf_model_path = os.path.join(pkg_src, urdf_file_path)
  default_rviz_config_path = os.path.join(pkg_src, rviz_config_file_path)
  world_path = os.path.join(pkg_src, world_file_path)
  gazebo_models_path = os.path.join(pkg_src, gazebo_models_path)
  os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
  if 'GAZEBO_PLUGIN_PATH' in os.environ:
    os.environ["GAZEBO_PLUGIN_PATH"] = os.getenv["GAZEBO_PLUGIN_PATH"] + ":" + "/opt/ros/galactic/lib"
  else:
    os.environ["GAZEBO_PLUGIN_PATH"] = "/opt/ros/galactic/lib"
  
  # Launch configuration variables specific to simulation
  gui = LaunchConfiguration('gui')
  headless = LaunchConfiguration('headless')
  namespace = LaunchConfiguration('namespace')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  urdf_model = LaunchConfiguration('urdf_model')
  use_namespace = LaunchConfiguration('use_namespace')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
  gz_verbose = LaunchConfiguration('verbose')
  gz_pause = LaunchConfiguration('pause')
  
  # Declare the launch arguments  
  declare_use_joint_state_publisher = DeclareLaunchArgument(
    name='gui',
    default_value='True',
    description='Flag to enable joint_state_publisher_gui')
    
  declare_namespace = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')

  declare_use_namespace = DeclareLaunchArgument(
    name='use_namespace',
    default_value='false',
    description='Whether to apply a namespace to the navigation stack')
            
  declare_rviz_config_file = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_simulator = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')

  declare_urdf_model_path = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_model_path, 
    description='Absolute path to robot urdf file')
    
  declare_use_robot_state_pub = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_use_rviz = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
    
  declare_use_sim_time = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')

  declare_use_simulator = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')

  declare_world = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')

  declare_verbose = DeclareLaunchArgument(
    name='verbose',
    default_value='false',
    description='Whether to start simulator in verbose mode')

  declare_pause = DeclareLaunchArgument(
    name='pause',
    default_value='false',
    description='Whether to start simulator in paused state')
  
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
  start_robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': Command(['xacro ', urdf_model])}])

  # Publish the joint states of the robot
  start_joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    condition=UnlessCondition(gui))

  # Launch RViz
  start_rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])

  # Start Gazebo server
  start_gazebo_server = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world, 
                      'verbose': gz_verbose, 
                      'pause': gz_pause,
                      }.items())

  # Start Gazebo client    
  start_gazebo_client = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

  # start_gazebo = ExecuteProcess(
  #   cmd=['gazebo', '--verbose', '--pause', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so', world],
  #         output='screen'
  # )

  # Launch the robot
  spawn_entity = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', robot_name_in_model, 
                '-topic', '/robot_description',
                    '-x', spawn_x_val,
                    '-y', spawn_y_val,
                    '-z', spawn_z_val,
                    '-Y', spawn_yaw_val],
                    output='screen')

  # Start EKF node
  start_robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': use_sim_time}]
  )

  # Create the launch description and populate
  return LaunchDescription( 
    [   
    # Declare the launch options
    declare_use_joint_state_publisher,
    declare_namespace,
    declare_use_namespace,
    declare_rviz_config_file,
    declare_simulator,
    declare_urdf_model_path,
    declare_use_robot_state_pub,
    declare_use_rviz,
    declare_use_sim_time,
    declare_use_simulator,
    declare_world,
    declare_verbose,
    declare_pause,

    # Add actions
    start_robot_state_publisher,
    start_rviz,
    start_gazebo_server,
    start_gazebo_client,
    spawn_entity,
    start_robot_localization_node,
    ]
  )
