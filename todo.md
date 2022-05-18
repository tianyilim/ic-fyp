# Todo, Changelog, Roadmap for FYP

## Todo
- [x] Get the multiple robots spawned to be visualised with different colours in Gazebo (eg. Robot1 is red, 2 is blue, etc...)
- [x] Show a visualization in Gazebo(?) of each robot's goal position
- [ ] `spawn_single_bot.py`/`individual_bot.launch.py` : Xacro is now parsed twice. Try to get the URDF to only be parsed once, preferably in `individual_bot.launch.py`
- [ ] Programatically create Rviz files for visualising each robot (currently hardcoded)
- [ ] Figure out how to add VCS to the `neo_simulation2` and `aws-robomaker-small-warehouse-world` folders in `src`

---
## Changelog
### Tue 19/04/22
- Create package

### Mon 25/04/22
- Import AWS warehouse world and Neobotix robot packages for simulation
- Add in `multirobot_control` package for overall control (sunset `robot_base` package)

### Tue 26/04/22
- Modfiy URDF for Neobotix MP-400 robot and add to the `multirobot_control` package. It still refers to the meshes in `neo_simulation2` (less redundancy)
- Add in spawn scripts for multiple robots with their own namespaces.

### Wed 27/04/22
- Managed to learn how to get 2 robots to work within their own tf2 namespaces:
  1. Use `xacro` to add a prefix to each of the frames of the robot. For instance, `base_link` becomes `robot1base_link`, etc.
  2. Use the `relay` Node from `topic_tools` to relay topic info from each robot's namespace to the  main `tf` and `tf_static` topics. This was launched in `individual_bot.launch.py`.
  3. Used `GroupAction` in `individual_bot.launch.py` in the hopes of getting Nodes to launch in sync.
  4. Cleaned up the TF tree, so now it looks like this.
  5. ![tf tree today](doc/tf_tree_270422.png)
- Questions:
  - [ ] Robots behave poorly in the AWS warehouse environments but not the Neobotix workshops. Might try to figure out why, but probably better to just use the Neobotix workshop environment for now
  - [x] Is it bad if the wheel frames connect to `base_footprint` instead of `base_link`? Need to refer to reference `tf_tree`s from Neobotix or elsewhere.
    - This is the behaviour out-of-the-box from the Neobotix demo.

### Thu 28/04/22
- Created a new node, `GazeboOdomGroundTruth` in `gt_odom.py` that listens to the world pose of a given link and attempts to broadcast it (and its relavant tf transforms).
- This is a replacement for trying to call the `/get_entity_state` Gazebo ROS service, which somehow doesn't exist anymore. Hopefully this workaround helps the people in [this thread](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1287).
- Implement `localization.launch.py` which launches the map server and the lifecycle manager needed to get it running.

### Fri 29/04/22
- Added in `navigation.launch.py` which is supposed to bring up navigation, but so far has been quite a pain.

### Mon 02/05/22
- Create a new URDF file that is similar to the original Neobotix robot in dimension but without any funky physics
- Use a sample model world with 3 cylinders and attempt to implement a DWA node

### Tue 03/05/22
- Modernized original URDF in `robot_base` with what we learned from playing with the Neobotix robot. Now can have ground truth odometry by adding a (undocumented) feature from the `diff_drive` controller.
- Realised that the timestamps being published by `gt_odom.py` are wrong. Just echo back the transform timings by accessing the same header as the odometry transform.
- After all this, our command to run the experiment is: 
  ```bash
  ros2 launch multirobot_control spawn_bots.launch.py \
  world:="/home/tianyilim/fyp/ic-fyp/src/multirobot_control/worlds/test_world.world" \
  urdf:="/home/tianyilim/fyp/ic-fyp/src/robot_base/urdf/robot_base.xacro"
  ```

- [x] Fix Xacro errors for the `test_world` xacro file: perhaps xacro needs to use "urdf"-esque tags?
  - See solution on [Fri 13/05/22](#fri-130522) - essentially ran into a reserved keyword and there was not much of a warning

### Wed 04/05/22
- Create a custom action package in `planner_action_interfaces`, relying on `geometry_msgs`.
  - In addition to the tutorial on custom interfaces [here](https://roboticsbackend.com/ros2-create-custom-message/), we also need to remember to add dependencies to `rosidl` as per the forum post [here](https://answers.ros.org/question/326008/ros2-run-symbol-not-found-on-custom-msg/).
- Implemented a (naive) DWA local planner / controller. 
  - To run, minimally `launch` the simulation (as above) and then run `ros2 run multirobot_control dwa_server --ros-args -r odom:=robot1/odom -r cmd_vel:=robot1/cmd_vel` (the Action Server). 
  - Run `python3 ~/fyp/ic-fyp/src/multirobot_control/multirobot_control/dwa_client.py` (the Action Client) which can take in arbitrary x and y positions, which the DWA approach will then evaluate.
  - Alternatively, try `ros2 run multirobot_control dwa_client --ros-args -r /dwa/_action/feedback:=/robot1/dwa/_action/feedback -r /dwa/_action/status:=/robot1/dwa/_action/status`.
- [x] Implement tunable parameters as some sort of config file:
  - [x] Limits for linear and angular movement
  - [x] Safety thresholds for robot
  - [x] Forward simulation duration
- [ ] Implement parameterized map (perhaps by parsing the world file?)
- [ ] Tune DWA parameters (`goal_K`, `safety_thresh_K`, `non_thresh_K`)
- [x] Add cost/benefit to orientation wrt goal
- [x] What happens when all DWA options are equally bad?
- [x] Check if simulated input using `cmd_vel` is sufficient
- [x] Namespaced nodes
- [x] CVA

### Thu 05/05/22
- Made a new message `OtherRobotLocations` in `planner_action_interfaces` for a central node to broadcast the (absolute) positions of each robot to other robots.
- Made a new node, `odom_distribution` which simulates communication distance between the set of robots.
- Run by `ros2 run multirobot_control odom_distribution --ros-args -p 'robot_list:=["robot1", "robot2", "robot3"]' -p pub_freq:=0.5`, can adjust `robot_list` as required for the number (and name) of robots in the simulation, and `pub_freq` as how often the terminal should be spammed; could be the same as the calculation frequency of the DWA planner.
- [x] `OtherRobotLocations` should contain a `geometry_msgs/Twist` in the World frame as well - so that each individual DWA planner can use that to plan their eventual paths.
  - Not implemented as a `Twist`, see eventual implementation on [Fri 13/05/22](#fri-130522)
- [ ] Create Gazebo plugin to control individual wheels, while also publishing the TF and odom transforms of the robot.

### Fri 06/05/22
- Try remapping namespaces on the command line with 
- `ros2 run multirobot_control dwa_server --ros-args -r __ns:=/robot1` 
- `ros2 run multirobot_control dwa_client --ros-args -r __ns:=/robot1`

- `odom_distribution` has some skeleton code for robots to broadcast where they will be in the future. Currently doesn't work because of topic problems.
- `dwa_server` sees the _future_ positions of other robts as a obstacle (subscribe to `OtherRobotLocations`)

### Mon 09/05/22
- Do **not** update Python and Apt packages often - spent a lot of time fixing some dependency with `gazebo_msgs` deciding to break, had to uninstall and reinstall `ros-galactic`.
- Added in programatically changing the colour of robots in Gazebo so they can be distinguished from one another.

### Tue 10/05/22
- Added in `goal_creation`, a node that spawns goals and visualises them in Gazebo.
- Similar to `odom_distribution` run `ros2 run multirobot_control goal_creation --ros-args -p 'robot_list:=["robot1", "robot2", "robot3"]'`

### Thur 12/05/22
- Refined `goal_creation` node, using a timer callback to check if robots are ready for their next goal. This prevents nested callbacks and allows the simulation to run smoothly.
- [x] Robots to move to their original posiitons (guaranteed collision-free) after finishing goals to ensure goals do not end with colliding robots.
  - This is implemented as a parameter. Perhaps it can also be set in the launch file, eventually. But for now, we run 
  - `ros2 run multirobot_control goal_creation --ros-args -p 'robot_list:=["robot1", "robot2", "robot3"]' -p 'robot_starting_x:=[0.0, 0.0, 0.0]' -p 'robot_starting_y:=[2.2, 0.0, 4.5]'`
- [x] Use the params file with the DWA server as well as the `odom_distribution` nodes, and in the future the `goal_creation` node as well.
- [x] Use the params file as the source of truth for robot spawning also (using PyYaml)
- Now we can perhaps run `ros2 run multirobot_control goal_creation --ros-args --params-file "/home/tianyilim/fyp/ic-fyp/src/multirobot_control/params/planner_params.yaml"`

### Fri 13/05/22
- Use `PointStamped` instead of `Point` for robots to publish their planned positions through `dwa_server`. To this effect, added a `DWAServerStatus` `Enum` to `dwa_server`. If there is a current goal being moved towards, we publish the planned position. If there is no current goal, we publish the current position. `PointStamped` allows us to send the namespace (and thus name) of each robot, allowing for centralised association.
- Fixed xacro not working on world files. It turns out that somewhere, `cylinder` is a reserved keyword or something. Therefore, renamed the macro we used to be `cylinder_element`.
- Added `factory_world.world` in `worlds`. Remember to regenerate the xacro file whenever it is updated. 
- [x] Further tune DWA node parameters, they do poorly when trying to rotate the robot on the spot.
- [x] Scale up single robot DWA demo to the "warehouse" environment
  - [x] Simplify environment to look like Amazon warehouse (manhattan-like)
    - If global planner is needed perhaps use RRT.
- [ ] Set up multi robot joint planner
  - [ ] Each robot will run DWA by itself, unless they are in close proximity. Then they could perhaps check if they were in danger of colliding.
  - [ ] if 2 robots are in danger of colliding then we must use a 2-robot planner that searches the 9x9 space of possibilities of each robot
- [ ] Come up with metrics on how delivery time / package throughput is affected by number of robots
  - Dump a results file containing the params file as well as certain metrics, such as:
    - Average time per goal completion
    - Average goals per minute
    - Average robot speed

### Mon 16/05/22
- Put in shelf model from AWS warehouse repo into the `models` folder. Need to add `{repo-path}/src/multirobot_control/models` to the `GAZEBO_MODEL_PATH`, which should be handled in the `spawn_bots.launch.py` launch file.
- Fixed a bug in AWS shelves SDF files. Previously in their collision/geometry add tags, their `uri` was set to be `file://models/...` when it should be `model://...`. This meant that Gazebo could not find the DAE files.
- All files read from `params/planner_params.yaml` to decide how many robots to spawn.
- Added in collision detection with Axis-Aligned Bounding Boxes to model the shelves, as they are always aligned to the _x_ and _y_ axes of the world. The robot is modelled as a circle (as it is roughly one.) This algorithm is implemented in `dwa_server.py -> dist_to_aabb`.
- Added in `map_params`, a place to keep the AABB representation of the shelves and walls.
- However, the DWA node is insufficient to reach the goal by itself. The robot is unable to find a path around the rectangular boxes and settles to a local minima.
- [x] To fix: if robots are unable to find _any_ good trajectories, then they continue with their past trajectory, which often means they actually hit something.

### Tue 17/05/22
- Fixed a bug in the AABB distance finding code.
- Added in a heading weight to the code which allows the robots to orient themselves to face approximately the next goal (~+-70deg, specified in params->`angular_thresh`). However, it still fails if we use the rectangular obstacles as in our sample world.
- Solution:
  - [ ] Implement RRT* to obtain waypoints for each robot.
- [ ] Visualizations in RViz need to be figured out.
  - [x] Visualise obstacles from `OBSTACLE_LIST` in RViz using a new node, `map_visualisation.py`. This uses `Marker` messages in RViz to more cleanly display the static obstacles.
  - [ ] Visualise goals from `goal_creation`
  - [x] Visualise sample trajectories from `dwa_server`
  - [ ] Visualise waypoints from RRT node

### Wed 18/05/22
- Added in arrow visualisation for the robot, now it is easier to see what trajectory the DWA controller chooses.
- Reinstated reversing for the controller. This now lets the robot get out of sticky situations where it would otherwise previously be stuck.