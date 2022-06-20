'''
Configuration of a given test scenario.

Defines the name of the config variable under test, the range of values it can take, and the
number of times to run each test.
'''

import itertools
from typing import Dict, List

# Edit this for the number of repeated experiements per combination
TEST_REPETITIONS = 10
max_num_robots = 16

# Edit this for the number of combinations
test_variables: Dict[str,List] = {
    # TEST FOR PLANNER
    'robot_list': [[f"robot{i+1}" for i in range(n+1)] for n in range(max_num_robots)],
    'local_planner' : ['dwa_action_server', 'dwa_replan_server'],

    # TEST FOR PARAMS
    # 'safety_thresh' : [0.25, 0.3, 0.35],
    # 'simulate_duration' : [0.25, 0.4, 0.55],
    # 'action_duration' : [0.01, 0.03, 0.05],
    # 'linear_speed_limit' : [0.4,0.6,0.8],
    # 'angular_speed_limit': [1.2,1.6],
    # 'angular_K' : [0.8,1.0,1.2],
    # 'goal_K' : [7.5,10.0,12.5],
    # 'obstacle_K': [0.8,1.0,1.2],
    # 'stall_det_period': [1.0],
    # 'stall_dist_thresh': [0.1],
    # 'move_towards_goal_hint': [True, False]

    # # Typically for the RRT Action Server
    # 'rrt_path_bias': [0.1],
    # 'rrt_it_min' : [200],
    # 'rrt_max_extend_length' : [1.5],
    # 'rrt_connect_circle_dist' : [1.5],
}

def make_grid(pars_dict: Dict[str,List]):
    '''Takes in a dictionary of parameters and finds the Cartesian product of input iterables.
    This gives us a list of combinations to test for.
    
    It also multiplies each combination by `iterations`.
    '''
    keys=pars_dict.keys()
    combinations=itertools.product(*pars_dict.values())
    ds=[dict(zip(keys,cc)) for cc in combinations]
    return ds

# Extract this for test values
TEST_COMBINATIONS = make_grid(test_variables)