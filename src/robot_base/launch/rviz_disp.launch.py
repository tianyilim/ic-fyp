from ament_index_python.packages import get_package_share_path, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os

'''
Shows the URDF model of a simple robot in RVIZ for sanity check purposes
'''

def generate_launch_description():

    package_path = get_package_share_path('robot_base')
    pkg_src = os.path.join(package_path, '../../../../src/robot_base')
    urdf_path = os.path.join(pkg_src, 'urdf')

    default_model_path = os.path.join(urdf_path, 'robot_base.xacro')
    default_rviz_config_path = os.path.join(urdf_path, 'urdf.rviz')
    # This will not save back to src -> if any changes are done, point RVIZ back to the src file manually!

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        # joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])