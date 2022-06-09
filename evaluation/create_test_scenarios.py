'''
Creates test scenarios based on a few parameters.

Modify this as desired to create a bunch of test cases to be run with `evaluate_planners.bash`.

Creates custom scenario files in `test_scenarios` and custom parameter files in `test_params`
Be sure to have the same name for matching files in `test_scenarios` and `test_params`.

Results can then be scored externally.
'''

import os
import copy
import yaml
import config
from datetime import datetime

TEST_SCENARIO_DIR = "/home/tianyilim/fyp/ic-fyp/evaluation/test_scenarios"
TEST_PARAMS_DIR = "/home/tianyilim/fyp/ic-fyp/evaluation/test_params"

if not os.path.exists(TEST_SCENARIO_DIR): os.mkdir(TEST_SCENARIO_DIR)
if not os.path.exists(TEST_PARAMS_DIR): os.mkdir(TEST_PARAMS_DIR)

scenario_settings = {
    'robot_list': ["robot1"],
    'robot_starting_x': [-5.5],
    'robot_starting_y': [1.15],
    'robot_starting_theta': [0.0],
    'total_goals': 10,
    'watchdog_timeout_s': 240,
    'result_folder': "result",
    'params_filepath': "/home/tianyilim/fyp/ic-fyp/src/multirobot_control/params/planner_params.yaml",
    # If random goals are desired, leave this as an empty list
    'goal_array': "[]",

    'realtime_factor': 1.0  # Max speedup
}

param_settings = {
    'pub_freq': 50.0,  # Match action duration

    # Typically for the DWA Action Server
    'robot_radius' : 0.35,
    'safety_thresh' : 0.3,
    'simulate_duration' : 0.4,
    'action_duration' : 0.03,
    'linear_speed_limit' : 0.6,
    'angular_speed_limit' : 1.5,
    'linear_step' : 0.1,
    'angular_step' : 0.2,
    'dist_thresh_hi' : 0.2,
    'dist_thresh_lo' : 0.05,
    'dist_method' : "L2",
    'inter_robot_dist' : 3.0,
    'orientation_ub_deg' : 180.0,
    'orientation_lb_deg' : 20.0,
    'angular_K' : 1.0, # the max proportion of dist_to_goal that angular contributes
    'goal_K' : 10.0,
    'obstacle_K': 1.0,
    'stall_det_period': 1.0,
    'stall_dist_thresh': 0.1,
    'replan_duration': 5.0,  # How long to wait to replan when an obstacle is reached

    # Typically for the RRT Action Server
    'rrt_path_bias': 0.1,
    'rrt_it_lim' : 2000,
    'rrt_it_min' : 200,
    'rrt_max_extend_length' : 1.5,
    'rrt_connect_circle_dist' : 1.5,
    'rrt_debug_plot' : False,
    'waypoint_skip': True,
    'waypoint_replan': False,
    'local_planner': 'dwa_action_server',

    # Typically for odom_distribution node,
    'odom_dist_thresh': 2.0,

    'num_robots': 1
}

# Tuning parameters
# local_planner = ['dwa_action_server', 'dwa_multirobot_server','dwa_replan_server']

# TODO : Set this to something legit

elems_search_grid = len(config.TEST_GRID)
start_time = datetime.now().strftime('%d%m%y_%H%M%S')

for i, setting in enumerate( config.TEST_GRID ):
    # Set filename as time of test
    filename = f"{start_time}_{i+1}_{elems_search_grid}"
    scenario_filename = f"{TEST_SCENARIO_DIR}/{filename}.yaml"
    param_filename = f"{TEST_PARAMS_DIR}/{filename}.yaml"

    scenario_settings_copy = copy.deepcopy(scenario_settings)
    param_settings_copy = copy.deepcopy(param_settings)

    # Ensure that the params filepath points to the correct one
    scenario_settings_copy['params_filepath'] = param_filename
    
    ##################### WRITE STUFF INTO SETTINGS HERE #####################
    
    for parameter in setting.keys():
        if parameter in param_settings_copy.keys():
            param_settings_copy[parameter] = setting[parameter]

        if parameter in scenario_settings_copy.keys():
            scenario_settings_copy[parameter] = setting[parameter]

    # number of robots is a special setting
    if 'robot_list' in setting.keys():
        param_settings_copy['num_robots'] = len(setting['robot_list'])

    ####################### END WRITE STUFF TO SETTINGS ######################

    scenario_tofile = {'/**': {'ros__parameters': scenario_settings_copy}}
    param_tofile = {'/**': {'ros__parameters': param_settings_copy}}
    
    print(f"Scenario/Parameter filename: {os.path.basename(scenario_filename)}")

    with open(scenario_filename, 'w') as f:
        yaml.safe_dump(scenario_tofile, f)
    
    with open(param_filename, 'w') as f:
        yaml.safe_dump(param_tofile, f)