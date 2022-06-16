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

import numpy as np
import sys # allow for us to use files from another directory
sys.path.append('/home/tianyilim/fyp/ic-fyp/src/multirobot_control')
from multirobot_control.map_params import GOAL_ARRAY

TEST_SCENARIO_DIR = "/home/tianyilim/fyp/ic-fyp/evaluation/test_scenarios"
TEST_PARAMS_DIR = "/home/tianyilim/fyp/ic-fyp/evaluation/test_params"

if not os.path.exists(TEST_SCENARIO_DIR): os.mkdir(TEST_SCENARIO_DIR)
if not os.path.exists(TEST_PARAMS_DIR): os.mkdir(TEST_PARAMS_DIR)

scenario_settings = {
    'robot_list': ["robot1"],
    'robot_starting_x': [],
    'robot_starting_y': [],
    'robot_starting_theta': [],
    'total_goals': 100,
    'watchdog_timeout_s': 120,
    'result_folder': "result",
    'params_filepath': "/home/tianyilim/fyp/ic-fyp/src/multirobot_control/params/planner_params.yaml",
    # If random goals are desired, leave this as an empty list
    'goal_array': "[]",
    # 'goal_array': "[[(0.0, 1.15), (0.0, -1.15), (-5.5, -1.15), (-5.5, 1.15)]]",

    'realtime_factor': 1.0  # Max speedup
}

param_settings = {
    'pub_freq': 25.0,  # Match action duration

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
    'rrt_it_min' : 50,
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

elems_search_grid = len(config.TEST_COMBINATIONS)*config.TEST_REPETITIONS
start_time = datetime.now().strftime('%d%m%y_%H%M%S')

# np.random.seed(42)  # Repeatable tests

# Create n identical test configs
for rep in range(config.TEST_REPETITIONS):
    # Define goal set, spawn positions
    
    start_pos_idx = np.random.choice(len(GOAL_ARRAY), config.max_num_robots, replace=False)
    start_orientation = np.random.rand(config.max_num_robots)*2*np.pi
    goal_pos_idx = np.random.choice(len(GOAL_ARRAY), (scenario_settings['total_goals'], config.max_num_robots), replace=True )

    for i, setting in enumerate( config.TEST_COMBINATIONS):
        # Set filename as time of test
        filename = f"{start_time}_{(i+1)+rep*len(config.TEST_COMBINATIONS):03d}_{elems_search_grid}"
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

        n_robots = len(scenario_settings_copy['robot_list'])
        param_settings_copy['num_robots'] = n_robots

        goals = []

        # Write to scenario settings
        for i in range(n_robots):
            scenario_settings_copy['robot_starting_x'].append(GOAL_ARRAY[start_pos_idx[i]][0])
            scenario_settings_copy['robot_starting_y'].append(GOAL_ARRAY[start_pos_idx[i]][1])
            scenario_settings_copy['robot_starting_theta'].append(float(start_orientation[i]))

            goals.append(
                [GOAL_ARRAY[idx] for idx in goal_pos_idx[:,i]]
            )

        scenario_settings_copy['goal_array'] = str(goals)

        ####################### END WRITE STUFF TO SETTINGS ######################

        scenario_tofile = {'/**': {'ros__parameters': scenario_settings_copy}}
        param_tofile = {'/**': {'ros__parameters': param_settings_copy}}
        
        print(f"Scenario/Parameter filename: {os.path.basename(scenario_filename)}")

        with open(scenario_filename, 'w') as f:
            yaml.safe_dump(scenario_tofile, f)
        
        with open(param_filename, 'w') as f:
            yaml.safe_dump(param_tofile, f)