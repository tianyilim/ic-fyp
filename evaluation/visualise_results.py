from math import dist
import matplotlib.pyplot as plt
from nbformat import write
import yaml
import os
import numpy as np
import glob

import sys # allow for us to use files from another directory
sys.path.append('/home/tianyilim/fyp/ic-fyp/src/multirobot_control')
from multirobot_control.goal_output import result_summary, goal_output
from config import TEST_COMBINATIONS
from ast import literal_eval

def write_to_attr(attr, value) -> None:
    '''Initialises attribute in `result_summary` if it is uninitialized (`None`).
    Else takes the average of the two values.'''
    if attr is None:
        attr = value
    else:
        attr += value
        attr /= 2

    return attr

def parse_res_dict(dict) -> goal_output:
    '''Parsing a YAML file returns strings of values instead of float values for some reason.
    This fixes that by returning a goal_output object with the correct values.'''

    parsed_dict = goal_output(
        goal_coords=( float(dict['goal_coords'][0]), float(dict['goal_coords'][1]) ),
        start_coords=(float(dict['start_coords'][0]), float(dict['start_coords'][1])),
        distance_travelled=float(dict['distance_travelled']),
        num_waypoints=int(dict['num_waypoints']),
        start_time=float(dict['start_time']),
        plan_time=float(dict['plan_time']),
        completion_time=float(dict['completion_time']) if dict['completion_time'] != '-1' \
            else -1
    )

    return parsed_dict

RESULT_DIR = "/home/tianyilim/fyp/ic-fyp/evaluation/result"

# This must match the parameters set in `create_test_scenarios.py`

# Get the test combinations
test_params = {}
test_param_dict = {}
for combi in TEST_COMBINATIONS:
    combi_key = str(combi.items())
    test_params[combi_key] = result_summary()
    test_param_dict[combi_key] = combi
test_param_names = TEST_COMBINATIONS[0].keys()

for file in glob.glob(RESULT_DIR+"/*.yaml"):
    with open(file, 'r') as f:
        res = yaml.load(f, Loader=yaml.BaseLoader)

    # print(f"Opening {os.path.basename(file)}")
    # Associate the value of the parameter file with the test combination
    params = res['/**']['ros__parameters']
    test_combination_key = {}
    for key in test_param_names:
        # This is needed for some reason because all the YAML files read as floats.
        try:
            test_combination_key[key] = literal_eval(params[key])
        except ValueError:
            test_combination_key[key] = params[key]
        # print(f"with {key}: {test_combination_key[key]}")

    if str(test_combination_key.items()) in test_params:
        res_summary = test_params[str(test_combination_key.items())]

        res_summary.num_iterations += 1

        for key in res.keys():
            if 'robot' in key:
                num_completed_goals = 0

                res_list = res[key]
                for elem_dict in res_list:
                    elem = parse_res_dict(elem_dict)

                    res_summary.plan_time.append(elem.plan_time)
                    res_summary.avg_plan_time = \
                        write_to_attr(res_summary.avg_plan_time, elem.plan_time)

                    if elem.completion_time != -1:
                        num_completed_goals += 1

                        manhattan_dist = np.linalg.norm(
                            np.array(elem.goal_coords)-np.array(elem.start_coords) )
                        res_summary.waypoint_dist.append(manhattan_dist)
                        res_summary.avg_waypoint_dist = \
                            write_to_attr(res_summary.avg_waypoint_dist, manhattan_dist)

                        res_summary.dist_travelled.append(elem.distance_travelled)
                        res_summary.avg_dist_travelled = \
                            write_to_attr(res_summary.avg_dist_travelled, elem.distance_travelled)

                        total_time = elem.completion_time-elem.start_time
                        move_time = elem.completion_time-(elem.start_time+elem.plan_time)
                        res_summary.total_time.append(total_time)
                        res_summary.avg_total_time = \
                            write_to_attr(res_summary.avg_total_time, total_time)

                        res_summary.move_time.append(move_time)
                        res_summary.avg_move_time = \
                            write_to_attr(res_summary.avg_move_time, move_time)

                res_summary.avg_num_completed_goals = \
                    write_to_attr(res_summary.avg_num_completed_goals, num_completed_goals)
                res_summary.num_completed_goals.append(num_completed_goals)
    
    else:
        print(f"{test_combination_key} not in test_params. Skipping.")

# for param in test_params.keys():
#     print(f"{param}\n{test_params[param]}")

x_val = np.arange(len(test_params), dtype='int')
# keys = [str(dict(k)) for k in test_params.keys()]
keys = [str(k) for k in test_params.keys()]
labels = []

num_iterations = [c.num_iterations for c in test_params.values()]
completed_goals_std = [np.std(np.array(c.num_completed_goals)) for c in test_params.values()]
completed_goals_sum = [np.sum(np.array(c.num_completed_goals)) for c in test_params.values()]
completed_goals = [c.avg_num_completed_goals for c in test_params.values()]
dist_travelled = [c.avg_dist_travelled for c in test_params.values()]
total_time = [c.avg_total_time for c in test_params.values()]
plan_time = [c.avg_plan_time for c in test_params.values()]
move_time = [c.avg_move_time for c in test_params.values()]

fig, ax = plt.subplots()
for x in x_val:
    bar = ax.bar(x, completed_goals[x], label=keys[x], yerr=completed_goals_std[x])
    d = test_param_dict[keys[x]]
    label = f"LP: {'A' if d['local_planner']=='dwa_action_server' else 'R'}, n_r: {d['num_robots']}"
    labels.append(label)
    ax.bar_label(bar, labels=[f'±{completed_goals_std[x]:.2f}, {num_iterations[x]} runs'],
             padding=8, color='k', fontsize=8)

ax.set_xticks(x_val, labels, rotation=-45)
# ax.legend()
ax.grid()
plt.tight_layout()
plt.show()