from multirobot_control.colour_palette import colour_palette, colour_palette_rviz_named
import xml.etree.ElementTree as ET
import xacro
import os

from typing import Dict
from multirobot_control.map_params import GOAL_ARRAY

import numpy as np

def parse_urdf(urdf_filepath:str, robot_num: int, robot_namespace:str):
    '''
    Modifies the input URDF file to take in the robot name and robot colour, remapping the 
    control inputs accordingly.
    Also modifies the visual characteristics of the robot so they are easily distinguishable.

    Returns the modified URDF filepath, typically in `tmp` of the current working directory.
    '''
    if robot_num in colour_palette:
        color = colour_palette[robot_num]
        color_rviz = colour_palette_rviz_named[robot_num]
    else:
        color = 'Gazebo/Grey'
        color_rviz = 'gray'

    root = ET.fromstring(xacro.process(urdf_filepath, mappings={'prefix': robot_namespace}))
    if robot_namespace:
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
                    remap.text = f'/tf:=/{robot_namespace}/tf'
                # add namespaces to all plugins
                ns = ros_params.find('namespace')
                if ns is None:
                    ns = ET.SubElement(ros_params, 'namespace')
                ns.text = '/' + robot_namespace
                ns.text = robot_namespace

        # Change robot colour as well
        links = root.findall('gazebo')
        for elem in links:
            # Use short-circuit eval to only get things with 'reference' attrib
            if 'reference' in elem.attrib and \
                ('base_link' in elem.attrib['reference'] or \
                 'bumper' in elem.attrib['reference']):
                # print(elem.find('material').text)
                elem.find('material').text = color

        links = root.findall('link')
        for elem in links:
            if 'name' in elem.attrib and \
                ('base_link' in elem.attrib['name'] or \
                 'bumper' in elem.attrib['name']):
                material = elem.find('visual').find('material')
                material.attrib['name'] = color_rviz

    # Save file for reference
    output_dir = os.path.join(os.getcwd(), "tmp")
    output_filepath = os.path.join(output_dir, f"out_{robot_namespace}.xml")
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
    with open(output_filepath, 'wb+') as f:
        ET.ElementTree(root).write(f)

    return output_filepath

def parse_world(world_filepath:str, realtime_factor:float):
    '''
    Modifies the input world xacro file to take in the realtime speedup factor.

    Returns the modified world filepath, typically in `tmp` of the current working directory.
    '''
    root = ET.fromstring(xacro.process(world_filepath))

    physics = root.find('world').find('physics').find('real_time_update_rate')
    if realtime_factor == 0.0:
        input_val = 0.0
    else:
        input_val = realtime_factor / 0.001 # default step size for gz

    physics.text = str(input_val)

    # Save file for reference
    output_dir = os.path.join(os.getcwd(), "tmp")
    output_filepath = os.path.join(output_dir, f"world.xml")
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
    with open(output_filepath, 'wb+') as f:
        ET.ElementTree(root).write(f)

    return output_filepath

def parse_scenario(scenario_configs:Dict):
    '''
    Reads a Dict corresponding to a Scenario YAML file.
    
    If the pose of all robots is defined, then return a list with those corresponding poses.

    If not, spawn robots in random locations chosen from GOAL_ARRAY with a random orientation.
    '''
    robots = []

    if len(scenario_configs['/**']['ros__parameters']['robot_starting_x']) == len(scenario_configs['/**']['ros__parameters']['robot_list']) \
    and len(scenario_configs['/**']['ros__parameters']['robot_starting_y']) == len(scenario_configs['/**']['ros__parameters']['robot_list']) \
    and len(scenario_configs['/**']['ros__parameters']['robot_starting_theta']) == len(scenario_configs['/**']['ros__parameters']['robot_list']):

        for idx in range(len(scenario_configs['/**']['ros__parameters']['robot_list'])):
            robots.append({
                'name': scenario_configs['/**']['ros__parameters']['robot_list'][idx],
                'x_pose': scenario_configs['/**']['ros__parameters']['robot_starting_x'][idx],
                'y_pose': scenario_configs['/**']['ros__parameters']['robot_starting_y'][idx],
                'z_pose': 0.10,
                'yaw_pose': scenario_configs['/**']['ros__parameters']['robot_starting_theta'][idx]
            })

    # If starting_x, y, theta do not line up with number of robots, randomly generate cooridnates
    else:
        used_idx = []

        for idx in range(len(scenario_configs['/**']['ros__parameters']['robot_list'])):

            # Prevent duplicate initial spawn positions
            while True:
                spawn_idx = np.random.randint(len(GOAL_ARRAY))
                if spawn_idx not in used_idx:
                    used_idx.append(spawn_idx)
                    break

            spawn_coords =  GOAL_ARRAY[spawn_idx]
            spawn_theta = np.random.random()*np.pi*2

            robots.append({
                'name': scenario_configs['/**']['ros__parameters']['robot_list'][idx],
                'x_pose': spawn_coords[0],
                'y_pose': spawn_coords[1],
                'z_pose': 0.10,
                'yaw_pose': spawn_theta
            })


    return robots