from setuptools import setup
import os
from glob import glob

package_name = 'robot_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # Include URDF and RVIZ config files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # include models (if they are files, glob returns files and directories)
        # (os.path.join('share', package_name, 'meshes'), [f for f in glob('meshes/**', recursive=True) if os.path.isfile(f)]),
        # (os.path.join('share', package_name, 'models'), [f for f in glob('models/**', recursive=True) if os.path.isfile(f)]),
        # (os.path.join('share', package_name, 'worlds'), [f for f in glob('worlds/**', recursive=True) if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tianyilim',
    maintainer_email='0.tianyi.lim@gmail.com',
    description='URDFs and behaviours for a 2WD warehouse autonomous mobile robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = robot_base.my_node:main'
        ],
    },
)
