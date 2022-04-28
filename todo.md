# Todo, Changelog, Roadmap for FYP

## Todo
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