from setuptools import setup

package_name = 'robot_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
