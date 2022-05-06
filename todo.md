# Todo, Changelog, Roadmap for FYP

## Todo
- [ ] Get the multiple robots spawned to be visualised with different colours in Gazebo (eg. Robot1 is red, 2 is blue, etc...)
- [ ] Show a visualization in Gazebo(?) of each robot's goal position
- [ ] `spawn_single_bot.py`/`individual_bot.launch.py` : Xacro is now parsed twice. Try to get the URDF to only be parsed once, preferably in `individual_bot.launch.py`
- [ ] Programatically create Rviz files for visualising each robot (currently hardcoded)
- [ ] Figure out how to add VCS to the `neo_simulation2` and `aws-robomaker-small-warehouse-world` folders in `src`

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

- [ ] Fix Xacro errors for the `test_world` xacro file: perhaps xacro needs to use "urdf"-esque tags?

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
- [ ] Add cost/benefit to orientation wrt goal
- [ ] What happens when all DWA options are equally bad?
- [ ] Check if simulated input using `cmd_vel` is sufficient
- [ ] Namespaced nodes
- [ ] CVA

### Thu 05/05/22
- Made a new message `OtherRobotLocations` in `planner_action_interfaces` for a central node to broadcast the (absolute) positions of each robot to other robots.
- Made a new node, `odom_distribution` which simulates communication distance between the set of robots.
- Run by `ros2 run multirobot_control odom_distribution --ros-args -p 'robot_list:=["robot1", "robot2", "robot3"]' -p pub_freq:=0.5`, can adjust `robot_list` as required for the number (and name) of robots in the simulation, and `pub_freq` as how often the terminal should be spammed; could be the same as the calculation frequency of the DWA planner.
- [ ] `OtherRobotLocations` should contain a `geometry_msgs/Twist` in the World frame as well - so that each individual DWA planner can use that to plan their eventual paths.
- [ ] Create Gazebo plugin to control individual wheels, while also publishing the TF and odom transforms of the robot.

### Fri 06/05/22
- Try remapping namespaces on the command line with 
- `ros2 run multirobot_control dwa_server --ros-args -r __ns:=/robot1` 
- `ros2 run multirobot_control dwa_client --ros-args -r __ns:=/robot1`

- [ ] Scale up single robot DWA demo to the "warehouse" environment
  - [ ] Simplify enivrionment to look like Amazon warehouse (manhattan-like)
    - [ ] If global planner is needed perhaps use RRT?
- [ ] Set up multi robot joint planner
  - [ ] Each robot will run DWA by itself, unless they are in close proximity. Then they could perhaps check if they were in danger of colliding.
  - [ ] if 2 robots are in danger of colliding then we must use a 2-robot planner that searches the 9x9 space of possibilities of each robot
- [ ] 
- [ ] Come up with metrics on how delivery time / package throughput is affected by number of robots