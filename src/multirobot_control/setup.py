from setuptools import setup
import os
from glob import glob

package_name = 'multirobot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files, configurations
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # parameters for Nav2
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))),
        # Maps
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps/*'))),
        # Include URDF and RVIZ config files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # Include worlds
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tianyilim',
    maintainer_email='0.tianyi.lim@gmail.com',
    description='Entry point package for testing multiple-robot control in Gazebo',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_single_bot = multirobot_control.spawn_single_bot:main',
            'gt_odom = multirobot_control.gt_odom:main',
            'dwa_client = multirobot_control.dwa_client:main',
            'dwa_server = multirobot_control.dwa_server:main',
            'odom_distribution = multirobot_control.odom_distribution:main',
            'goal_creation = multirobot_control.goal_creation:main',
            'map_visualisation = multirobot_control.map_visualisation:main'
        ],
    },
)
