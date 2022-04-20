# Source: https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/

import os
from random import choices
from urllib.parse import uses_query
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_path = get_package_share_path('robot_base')
    pkg_src = os.path.join(package_path, '../../../../src/robot_base')
    urdf_path = os.path.join(pkg_src, 'urdf')

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')

    default_rviz_config_path = os.path.join(urdf_path, 'urdf.rviz')
    # This will not save back to src -> if any changes are done, point RVIZ back to the src file manually!
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                    description='Absolute path to rviz config file')

    use_sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='false', 
                                    choices=['true', 'false'], 
                                    description='Use simulation (Gazebo) clock if true')

    # Instanstiate robot model
    default_robot_model_path = os.path.join(urdf_path, 'robot_base.xacro')
    model_arg = DeclareLaunchArgument(name='robot_model', default_value=str(default_robot_model_path),
                                    description='Absolute path to robot urdf file')
    robot_base_desc = ParameterValue(Command(['xacro ', LaunchConfiguration('robot_model')]),
                                    value_type=str)
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        # output="screen",
        parameters=[{'robot_description': robot_base_desc}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    # For now just comment out the desired one in the launch file.
    # For now neither are used. The Gazebo plugin handles joint state publishing.
    joint_state_publisher_node = Node( package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node( package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    gazebo_executable = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen'
    )

    gazebo_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=["-topic", "/robot_description", "-entity", "robot_base"]
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        use_sim_time,
        
        robot_state_publisher_node,
        rviz_node,
        gazebo_executable,
        gazebo_node,
    ])