'''
Creates test scenarios based on a few parameters.

Modify this as desired to create a bunch of test cases to be run with `evaluate_planners.bash`.

Creates custom scenario files in `test_scenarios` and custom parameter files in `test_params`
Be sure to have the same name for matching files in `test_scenarios` and `test_params`.

Results can then be scored externally
'''

import os
import copy
from pickle import LIST
import yaml

if not os.path.exists("/home/tianyilim/fyp/ic-fyp/src/multirobot_control/test_scenarios"):
    os.mkdir("/home/tianyilim/fyp/ic-fyp/src/multirobot_control/test_scenarios")

if not os.path.exists("/home/tianyilim/fyp/ic-fyp/src/multirobot_control/test_params"):
    os.mkdir("/home/tianyilim/fyp/ic-fyp/src/multirobot_control/test_params")

scenario_settings = {
    'robot_list': ["robot1"],
    'robot_starting_x': [-5.5],
    'robot_starting_y': [1.15],
    'robot_starting_theta': [0.0],
    'total_goals': 1,
    'watchdog_timeout_s': 30,
    'result_folder': "result",
    'params_filepath': "/home/tianyilim/fyp/ic-fyp/src/multirobot_control/params/planner_params.yaml",
    # If random goals are desired, leave this as an empty list
    'goal_array': "[]",
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
    'local_planner': 'dwa_replan_server',

    # Typically for odom_distribution node,
    'odom_dist_thresh': 2.0,
}

# Scenario parameters
num_robots = range(1,7)
# Set initial robot x,y,theta as well
total_goals = 10
timeout = 120
result_folder = ""
params_filepath = "/home/tianyilim/fyp/ic-fyp/src/multirobot_control/params/planner_params.yaml"

# Tuning parameters
local_planner = ['dwa_action_server', 'dwa_multirobot_server','dwa_replan_server']
simulate_duration = []
action_duration = []
linear_speed_limit = []
angular_speed_limit = []
linear_step : 0.1
angular_step : 0.2
dist_thresh_hi : 0.2
dist_thresh_lo : 0.05
orientation_ub_deg : 180.0
orientation_lb_deg : 20.0
angular_K : 1.0 # the max proportion of dist_to_goal that angular contributes
goal_K : 10.0
obstacle_K: 1.0
stall_det_period: 1.0
stall_dist_thresh: 0.1
replan_duration: 5.0  # How long to wait to replan when an obstacle is reached
rrt_it_lim : 2000
rrt_it_min : 200
rrt_max_extend_length : 1.5
rrt_connect_circle_dist : 1.5

LIST_OF_POSSIBLE_SETTINGS = [0.4, 0.5]

for i, setting in enumerate(LIST_OF_POSSIBLE_SETTINGS):
    scenario_filename = f"/home/tianyilim/fyp/ic-fyp/src/multirobot_control/test_scenarios/{i}.yaml"
    param_filename = f"/home/tianyilim/fyp/ic-fyp/src/multirobot_control/test_params/{i}.yaml"

    scenario_settings_copy = copy.deepcopy(scenario_settings)
    param_settings_copy = copy.deepcopy(param_settings)

    # Ensure that the params filepath points to the correct one
    scenario_settings_copy['params_filepath'] = param_filename
    
    ##################### WRITE STUFF INTO SETTINGS HERE #####################
    
    param_settings_copy['linear_speed_limit'] = setting

    ####################### END WRITE STUFF TO SETTINGS ######################

    scenario_tofile = {'/**': {'ros__parameters': scenario_settings_copy}}
    param_tofile = {'/**': {'ros__parameters': param_settings_copy}}
    
    print(f"Scenario filename: {scenario_filename}")
    print(f"Parameter filename: {param_filename}")

    with open(scenario_filename, 'w') as f:
        yaml.safe_dump(scenario_tofile, f)
    
    with open(param_filename, 'w') as f:
        yaml.safe_dump(param_tofile, f)