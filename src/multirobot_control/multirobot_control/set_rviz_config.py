import yaml
import os
import copy

def set_rviz_config(rviz_src:str, rviz_dest:str, config_path:str) -> None:
    
    with open(rviz_src, 'r') as f:
        d = yaml.safe_load(f)

    with open(config_path, 'r') as c:
        config = yaml.safe_load(c)

    # Get the number and name of robots in the simulation
    robot_list = config['/**']['ros__parameters']['robot_list']
    num_robots = len(robot_list)

    # We collect the base setting for the RViz display and append to a list
    robot_cfg_collection = []
    non_robot_cfg = []
    for elem in d['Visualization Manager']['Displays']:
        # Hardcoded to 'Robot1 {description}'
        print(elem['Name'])
        if 'Robot' in elem['Name']:
            robot_cfg_collection.append(elem)
        else:
            non_robot_cfg.append(elem)

    # Remove all "reference" entries from the list
    d['Visualization Manager']['Displays'] = non_robot_cfg

    # Rename and duplicate
    for i in range(1, num_robots+1):
        for elem in robot_cfg_collection:
            elem_copy = copy.deepcopy(elem)

            if elem_copy['Class'] == 'rviz_default_plugins/RobotModel':
                elem_copy['Description Topic']['Value'] = robot_list[i-1]+'/robot_description'
                elem_copy['Name'] = robot_list[i-1].capitalize() + ' Model'
            elif elem_copy['Class'] == 'rviz_default_plugins/Odometry':
                elem_copy['Topic']['Value'] = robot_list[i-1]+'/odom'
                elem_copy['Name'] = robot_list[i-1].capitalize() + ' Odometry'
            elif elem_copy['Class'] == 'rviz_default_plugins/MarkerArray' \
            or   elem_copy['Class'] == 'rviz_default_plugins/Marker':
                topic_suffix =  elem_copy['Topic']['Value'].split('/')[-1]
                elem_copy['Topic']['Value'] = robot_list[i-1] + '/' + topic_suffix
                suffix = elem_copy['Name'].split()[-1]
                elem_copy['Name'] = robot_list[i-1].capitalize() + ' ' + suffix

            d['Visualization Manager']['Displays'].append(elem_copy)

    with open(rviz_dest, 'w+') as dest:
        yaml.safe_dump(d, dest)