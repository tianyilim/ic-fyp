# Copyright (c) 2019 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''
Spawns a single robot in Gazebo with its own namespace.
'''

import argparse
import os
from re import A
import xml.etree.ElementTree as ET
import xacro

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import rclpy

from tf_transformations import quaternion_from_euler

def main():
    bringup_dir = get_package_share_directory('multirobot_control')
    urdf_dir = get_package_share_directory('robot_base')

    # Colours in the Materials section
    colour_palette = {
        0: "Gazebo/Red",
        1: "Gazebo/Green",	
        2: "Gazebo/Blue",	
        3: "Gazebo/Yellow",	
        4: "Gazebo/Purple",	
        5: "Gazebo/Turquoise",	
        6: "Gazebo/Orange",	
        7: "Gazebo/Indigo",	
        8: "Gazebo/DarkYellow",
        9: "Gazebo/SkyBlue",
        10: "Gazebo/ZincYellow",
        11: "Gazebo/RedBright",
        12: "Gazebo/BlueLaser",
        13: "Gazebo/Grey",
        14: "Gazebo/Black",
        15: "Gazebo/White"
    }

    # Get input arguments from user
    parser = argparse.ArgumentParser(description='Spawn Robot into Gazebo with Nav2')
    parser.add_argument('-n', '--robot_name', type=str, default='robot',
                        help='Name of the robot to spawn')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='robot',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-x', type=float, default=0,
                        help='the x component of the initial position [meters]')
    parser.add_argument('-y', type=float, default=0,
                        help='the y component of the initial position [meters]')
    parser.add_argument('-z', type=float, default=0,
                        help='the z component of the initial position [meters]')
    parser.add_argument('-Y', '--yaw', type=float, default=0,
                        help='the yaw component of the initial position [rad]')
    parser.add_argument('-k', '--timeout', type=float, default=10.0,
                        help="Seconds to wait. Block until the future is complete if negative. \
                            Don't wait if 0.")
    parser.add_argument('-u', '--urdf', type=str, 
                        # default=os.path.join(bringup_dir, 'urdf', 'mp_400.urdf' ),
                        default=os.path.join(urdf_dir, 'urdf', 'robot_base.xacro' ),
                       help="the path to the robot's model file (URDF)")
    parser.add_argument('--robot_num', type=int, default=15,
                        help="Robot's number to help with identification")

    args, unknown = parser.parse_known_args()

    # Start node
    rclpy.init()
    node = rclpy.create_node('entity_spawner')

    node.get_logger().info('spawning `{}` on namespace `{}` at x:{}, y:{}, z:{}, Yaw:{}'.format(
        args.robot_name, args.robot_namespace, args.x, args.y, args.z, args.yaw))

    node.get_logger().info('Taking URDF from {}'.format(args.urdf))

    if args.robot_num in colour_palette:
        color = colour_palette[args.robot_num]
    else:
        color = 'gray'
    node.get_logger().info("Setting colour to {}".format(color))

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info('...connected!')

    # We need to remap the transform (/tf) topic so each robot has its own.
    # We do this by adding `ROS argument entries` to the sdf file for
    # each plugin broadcasting a transform. These argument entries provide the
    # remapping rule, i.e. /tf -> /<robot_id>/tf
    
    root = ET.fromstring(xacro.process(args.urdf, mappings={'prefix': args.robot_namespace}))
    if args.robot_namespace:
        for plugin in root.iter('plugin'):
            # # Find all frames and add the relevant prefix
            # for elem in plugin:
            #     # if 'joint' in elem.tag or 'frame' in elem.tag:
            #     #     elem.text = args.robot_namespace + elem.text
            #     if 'frame' in elem.tag:
            #         elem.text = args.robot_namespace + elem.text

            ros_params = plugin.find('ros')
            if ros_params is not None:
                # only remap for diff drive plugin
                if 'ros_diff_drive' in plugin.get('filename'):
                    remap = ros_params.find('remapping')
                    if remap is None:
                        remap = ET.SubElement(ros_params, 'remapping')
                    remap.text = f'/tf:=/{args.robot_namespace}/tf'
                # add namespaces to all plugins
                ns = ros_params.find('namespace')
                if ns is None:
                    ns = ET.SubElement(ros_params, 'namespace')
                ns.text = '/' + args.robot_namespace
                ns.text = args.robot_namespace

        # Change robot colour as well
        links = root.findall('gazebo')
        for elem in links:
            # Use short-circuit eval to only get things with 'reference' attrib
            if 'reference' in elem.attrib and 'base_link' in elem.attrib['reference']:
                # print(elem.find('material').text)
                elem.find('material').text = color

    # Save file for reference
    # with open('out_{}.xml'.format(args.robot_name), 'wb') as f:
    #     ET.ElementTree(root).write(f)

    # Set data for request
    request = SpawnEntity.Request()
    request.name = args.robot_name
    request.xml = ET.tostring(root, encoding='unicode')
    request.robot_namespace = args.robot_namespace
    request.initial_pose.position.x = float(args.x)
    request.initial_pose.position.y = float(args.y)
    request.initial_pose.position.z = float(args.z)
    
    # Convert yaw into quarternion
    quat = quaternion_from_euler(0.0, 0.0, float(args.yaw))
    request.initial_pose.orientation.x = quat[0]
    request.initial_pose.orientation.y = quat[1]
    request.initial_pose.orientation.z = quat[2]
    request.initial_pose.orientation.w = quat[3]

    node.get_logger().info('Sending service request to `/spawn_entity`')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
    if future.result() is not None:
        print(f'response: {future.result()!r}')
    else:
        raise RuntimeError(
            f'exception while calling service: {future.exception()!r}')

    node.get_logger().info('Done! Shutting down node.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
