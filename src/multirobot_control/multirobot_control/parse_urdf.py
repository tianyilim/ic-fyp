from multirobot_control.colour_palette import colour_palette, colour_palette_rviz_named
import xml.etree.ElementTree as ET
import xacro
import os

def parse_urdf(urdf_filepath:str, robot_num: int, robot_namespace:str):
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