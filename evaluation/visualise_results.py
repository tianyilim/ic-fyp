import matplotlib.pyplot as plt
import yaml
import os
import numpy as np
import glob

import sys # allow for us to use files from another directory
sys.path.append('/home/tianyilim/fyp/ic-fyp/src/multirobot_control')
from multirobot_control.goal_output import result_summary, goal_output
from config import TEST_COMBINATIONS, max_num_robots
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

# If looking at tuning DWA values
RESULT_DIR = "/home/tianyilim/fyp/ic-fyp/evaluation/result"

# This must match the parameters set in `create_test_scenarios.py`

# Get the test combinations
test_params = {}
test_param_dict = {}
for combi in TEST_COMBINATIONS:
    # handle special case of 'robot_list'
    if 'robot_list' in combi:
        combi['num_robots'] = len(combi['robot_list'])
        combi.pop('robot_list')

    combi_key = str(combi.items())
    test_params[combi_key] = result_summary()
    test_param_dict[combi_key] = combi

test_param_names = TEST_COMBINATIONS[0].keys()

for file in glob.glob(RESULT_DIR+"/*.yaml"):
    with open(file, 'r') as f:
        res = yaml.load(f, Loader=yaml.BaseLoader)

    # Associate the value of the parameter file with the test combination
    params = res['/**']['ros__parameters']
    test_combination_key = {}
    for key in test_param_names:
        # This is needed for some reason because all the YAML files read as strings.
        try:
            test_combination_key[key] = literal_eval(params[key])
        except ValueError:
            test_combination_key[key] = params[key]

    added_already = False
    print(test_combination_key.items())
    print(test_params)
    if str(test_combination_key.items()) in test_params:
        res_summary = test_params[str(test_combination_key.items())]
        total_num_completed_goals = 0

        # Go through test combination keys (robots' indiv results per run)
        for key in res.keys():
            if 'robot' in key:
                num_completed_goals = 0 # per-robot score

                res_list = res[key]
                for elem_dict in res_list:
                    
                    if not added_already:
                        res_summary.num_iterations += 1
                        added_already = True

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
                res_summary.per_robot_num_completed_goals.append(num_completed_goals)
                
                total_num_completed_goals += num_completed_goals # all goals done per robot
        
        # Over all robots
        res_summary.per_run_num_completed_goals.append(total_num_completed_goals)
    
    else:
        pass
        # print(f"{test_combination_key} not in test_params. Skipping.")

    # Comment this in if there are issues with simulation
    # if added_already == False:
    #     os.remove(file)

# for param in test_params.keys():
#     print(f"{param}\n{test_params[param]}")

x_val = np.arange(len(test_params), dtype='int')
keys = [str(k) for k in test_params.keys()]
labels = []

num_iterations = [c.num_iterations for c in test_params.values()]
print("num_iterations", len(num_iterations), num_iterations)
completed_goals_std = [np.std(np.array(c.per_robot_num_completed_goals)) for c in test_params.values()]
# print("completed_goals_std", len(completed_goals_std), completed_goals_std)
completed_goals = [np.mean(np.array(c.per_robot_num_completed_goals)) for c in test_params.values()]
# print("completed_goals", len(completed_goals), completed_goals)
total_completed_goals_std = [np.std(np.array(c.per_run_num_completed_goals)) for c in test_params.values()]
# print("total_completed_goals_std", len(total_completed_goals_std), total_completed_goals_std)
completed_goals_sum = [np.mean(np.array(c.per_run_num_completed_goals)) for c in test_params.values()]
# print("completed_goals_sum", len(completed_goals_sum), completed_goals_sum)
dist_travelled = [np.mean(np.array(c.dist_travelled)) for c in test_params.values()]
# print("dist_travelled", len(dist_travelled), dist_travelled)
total_time = [np.mean(np.array(c.total_time)) for c in test_params.values()]
# print("total_time", len(total_time), total_time)
plan_time = [np.mean(np.array(c.plan_time)) for c in test_params.values()]
# print("plan_time", len(plan_time), plan_time)
move_time = [np.mean(np.array(c.move_time)) for c in test_params.values()]
# print("move_time", len(move_time), move_time)

max_goals_idx = np.argsort(-np.array(completed_goals_sum)) # large to small
min_time_idx = np.argsort(np.array(move_time))

for i in range(10):
    idx = max_goals_idx[i]
    print(keys[idx])
    print(f"Move time: {move_time[idx]:.3f}, total time: {total_time[idx]:.3f} dist: {dist_travelled[idx]:.2f}, goals: {completed_goals[idx]:.2f}+-{completed_goals_std[idx]}, n:{num_iterations[idx]}")
    print()

# Bar chart
'''
fig, ax = plt.subplots()
for x in x_val:
    bar = ax.bar(x, completed_goals[x], label=keys[x], yerr=completed_goals_std[x])
    d = test_param_dict[keys[x]]
    label = f"LP: {'A' if d['local_planner']=='dwa_action_server' else 'R'}, n_r: {d['num_robots']}"
    # label = f"st:{d['safety_thresh']},sd:{d['simulate_duration']},ad:{d['action_duration']},lsl:{d['linear_speed_limit']},asl:{d['angular_speed_limit']}"
    labels.append(label)
    ax.bar_label(bar, labels=[f'±{completed_goals_std[x]:.2f}, {num_iterations[x]} runs'],
             padding=8, color='k', fontsize=8)

ax.set_xticks(x_val, labels, rotation=-45)
# ax.legend()
ax.grid()
plt.tight_layout()
plt.show()
'''

a_total_goals = []
a_avg_goals = []
a_goal_std = []
a_total_goal_std = []
a_total_time = []
a_plan_time = []
a_move_time = []
a_dist_travelled = []

r_total_goals = []
r_avg_goals = []
r_goal_std = []
r_total_goal_std = []
r_total_time = []
r_plan_time = []
r_move_time = []
r_dist_travelled = []

max_r = 0

for x in x_val:
    if num_iterations[x] > 0:
        max_r += 1
        d = test_param_dict[keys[x]]
        if d['local_planner'] == 'dwa_action_server':
            a_total_goals.append( completed_goals_sum[x] )
            a_avg_goals.append(completed_goals[x])
            a_goal_std.append(completed_goals_std[x])
            a_total_goal_std.append(total_completed_goals_std[x])
            a_total_time.append(total_time[x])
            a_plan_time.append(plan_time[x])
            a_move_time.append(move_time[x])
            a_dist_travelled.append(dist_travelled[x])
        elif d['local_planner'] == 'dwa_replan_server':
            r_total_goals.append( completed_goals_sum[x] )
            r_avg_goals.append(completed_goals[x])
            r_goal_std.append(completed_goals_std[x])
            r_total_goal_std.append(total_completed_goals_std[x])
            r_total_time.append(total_time[x])
            r_plan_time.append(plan_time[x])
            r_move_time.append(move_time[x])
            r_dist_travelled.append(dist_travelled[x])


num_robots = np.arange(1, 1+(max_r//2), dtype='int')

plt.figure()
plt.xlabel("Number of Robots")
plt.ylabel("Total Goals Completed")
# plt.errorbar(num_robots, a_total_goals, fmt='x-', yerr=a_total_goal_std, capsize=4.0, barsabove=True, label="Action Server")
# plt.errorbar(num_robots, r_total_goals, fmt='x-', yerr=r_total_goal_std, capsize=4.0, barsabove=True, label="Replan Server")
plt.plot(num_robots, a_total_goals, 'x-', label="Action Server")
plt.plot(num_robots, r_total_goals, 'x-', label="Replan Server")
plt.xticks(np.arange(1, len(num_robots)+1, step=1))  # Set label locations
plt.grid()
plt.legend()

plt.figure()
plt.xlabel("Number of Robots")
plt.ylabel("Average Goals Completed Per Robot")
# plt.errorbar(num_robots, a_avg_goals, fmt='x-', yerr=a_goal_std, capsize=4.0, barsabove=True, label="Action Server")
# plt.errorbar(num_robots, r_avg_goals, fmt='x-', yerr=r_goal_std, capsize=4.0, barsabove=True, label="Replan Server")
plt.plot(num_robots, a_avg_goals, 'x-', label="Action Server")
plt.plot(num_robots, r_avg_goals, 'x-', label="Replan Server")
plt.xticks(np.arange(1, len(num_robots)+1, step=1))  # Set label locations
plt.grid()
plt.legend()

plt.figure()
plt.xlabel("Number of Robots")
plt.ylabel("Time spent per waypoint (s) ")
plt.plot(num_robots, a_total_time, 'x-', label="Action Server Total Time")
plt.plot(num_robots, r_total_time, 'x-', label="Replan Server Total Time")
# plt.plot(num_robots, a_plan_time, 'x-', label="Action Server Plan Time")
# plt.plot(num_robots, r_plan_time, 'x-', label="Replan Server Plan Time")
plt.xticks(np.arange(1, len(num_robots)+1, step=1))  # Set label locations
plt.grid()
plt.legend()

plt.figure()
plt.xlabel("Number of Robots")
plt.ylabel("Distance Travelled per waypoint (m) ")
plt.plot(num_robots, a_dist_travelled, 'x-', label="Action Server")
plt.plot(num_robots, r_dist_travelled, 'x-', label="Replan Server")
plt.xticks(np.arange(1, len(num_robots)+1, step=1))  # Set label locations
plt.grid()
plt.legend()

plt.tight_layout()
plt.show()