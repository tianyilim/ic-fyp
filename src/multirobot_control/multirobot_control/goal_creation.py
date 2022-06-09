'''
Node that handles creating and visualizing 'goals' for each robot. 
'''
import xml.etree.ElementTree as ET
import os

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from planner_action_interfaces.action import LocalPlanner
from geometry_msgs.msg import Pose, Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID
from gazebo_msgs.srv import SpawnEntity, DeleteEntity

from planner_action_interfaces.msg import NamedFloat

from tf_transformations import euler_from_quaternion

# The dictionary containing the palette of robots--> gazebo colors
from multirobot_control.colour_palette import colour_palette
# Gloabl Goal array
from multirobot_control.map_params import GOAL_ARRAY
from multirobot_control.goal_output import goal_output

import numpy as np
from enum import Enum, auto
import yaml
from datetime import datetime
from ast import literal_eval

GOAL_SDF_PATH = os.path.join(get_package_share_directory('multirobot_control'),
    'urdf', 'goal.sdf')

class RobotGoalStatus(Enum):
    GOAL_STATUS_READY = auto()      # Ready to accept a new goal
    GOAL_STATUS_DOING = auto()      # Done with the goal
    GOAL_STATUS_PENDING = auto()    # Processing a goal

class GoalCreation(Node):

    def __init__(self):
        super().__init__('goal_creation')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_list', Parameter.Type.STRING_ARRAY),   # list of robot names to subscribe to
                ('total_goals', Parameter.Type.INTEGER),
                ('watchdog_timeout_s', Parameter.Type.INTEGER),
                ('result_folder', Parameter.Type.STRING),
                ('params_filepath', Parameter.Type.STRING),
                ('goal_array', Parameter.Type.STRING),
                ('realtime_factor', Parameter.Type.DOUBLE)
            ]
        )
        self.add_on_set_parameters_callback(self.parameter_callback)

        # 'goal_array' is a multidimensional array with
        # [ [r1g1,r1g2,...r1gN], [r2g1,...r2gN],... [rMg1,...rMgN] ]
        # Multidimensional arrays are not supported with ROS2 so we have to parse it as a string.
        try:
            self.goal_array = literal_eval(self.get_parameter('goal_array').value)
        except SyntaxError:
            self.get_logger().error(f"Goal array parameter has syntax error. Substituting empty list. Passed in: \'{self.get_parameter('goal_array').value}\'")
            self.goal_array = []
        
        # We follow goal generation rules based on whether the goals are specified or not
        # If goals were specified 
        self.randomly_generate_goals = ( len(self.goal_array) == 0 )
        if self.randomly_generate_goals:
            self.get_logger().info(f"Randomly generating goals from GOAL_ARRAY. Each robot has {self.get_parameter('total_goals').value} goals.")
        else:
            self.get_logger().info(f"Taking goals from list:\n{self.goal_array}")

        assert self.get_parameter("robot_list").value is not None
        assert len(self.get_parameter("robot_list").value) != 0
        if len(self.goal_array) != 0:
            assert len(self.goal_array) == len(self.get_parameter("robot_list").value), \
            "Ensure that goal array has same number of elements as the number of robots!"

        self.robot_remaining_goals = {}     # Collection of number goals each robot still has to finish
        self.robot_uuids = {}               # Collection of UUID objects corresponding to each robot
        self.gazebo_goals = {}              # Flag if a Gazebo entity has been spawned already
        self.robot_goal_status: dict[str, RobotGoalStatus] = {}       
        # Flag on the goal completion status of a robot, either `GOAL_STATUS_READY` or `GOAL_STATUS_DOING`
        self.action_clients = {}            # Send goal to DWA server
        self.robot_name_to_idx = {}         # Map robot names to respective indices (eg. robot1->1)
        self.goal_futures = {}              # One goal future corresponding to each robot
        self.result_futures = {}            # Same for results
        # Arrays to wait for robots to be done spawning in Gazebo before sending goals
        self.robots_done_spawning = {}
        self.robots_done_subscriber = {}
        self.plan_done_subscriber = {}

        self._sim_start_time = None     # Track run duration of the simulation

        # Code to visualise goals in Gazebo
        self.spawn_client = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawner service not available, waiting again...', once=True)

        self.delete_client = self.create_client(DeleteEntity, 'delete_entity')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Deleter service not available, waiting again...', once=True)

        self.get_logger().debug(f'Goal SDF path: {GOAL_SDF_PATH}')

        # Code to write to file
        self.parameter_file = self.get_parameter('params_filepath').value
        self.get_logger().info(f"Params file: {self.parameter_file}")

        result_dir = os.path.join(os.getcwd(), self.get_parameter('result_folder').value)
        if not os.path.exists(result_dir): os.mkdir(result_dir)
        self.result_file = os.path.join(result_dir, f"{datetime.now().strftime('%d%m%y_%H%M%S')}.yaml")
        self.get_logger().info(f"Writing results to {self.result_file}")

        now_s = self.get_clock().now().seconds_nanoseconds()
        now_s = float(now_s[0] + now_s[1]/1e9)
        self.results = {
            'start_time': now_s,
            'end_time': -1,
        }

        for index, robot in enumerate(self.get_parameter("robot_list").value):
            # Flag to wait for robots to be done spawning
            self.robots_done_spawning[robot] = False
            self.get_logger().debug(f"Subscribing to /{robot}/finished_spawning")
            self.robots_done_subscriber[robot] = self.create_subscription(String, 
                f"/{robot}/finished_spawning", self._handle_robot_done_spawning, 10)

            self.get_logger().debug(f"Subscribing to /{robot}/rrt_done")
            self.plan_done_subscriber[robot] = self.create_subscription(NamedFloat, 
                f"/{robot}/rrt_done", self._handle_robot_rrt_done, 10)
            
            self.action_clients[robot] = \
                ActionClient(self, LocalPlanner, robot+'/rrt_star')

            self.robot_name_to_idx[robot] = index

            # add to logging
            self.results[robot] = []

            # Generate goals or read from the list
            if self.randomly_generate_goals:
                self.robot_remaining_goals[robot] = self.get_parameter('total_goals').value
            else:
                self.robot_remaining_goals[robot] = len(self.goal_array[index])

            if self.robot_remaining_goals[robot] > 0:
                self.robot_goal_status[robot] = RobotGoalStatus.GOAL_STATUS_READY

        # Create a timer callback to periodically assign goals
        goal_timer_period = 1.0
        self.goal_assignment_timer = self.create_timer( goal_timer_period, self.goal_assignment_timer_callback )

        self.goal_assignment_timer = self.create_timer( 1.0, self.watchdog_timer_callback )
        self._watchdog_expiry_time = self.get_parameter('watchdog_timeout_s').value

    def send_goal(self, robot_name, goal_position: Point):
        '''
        Sends a Point goal to the robot (specified by `robot_name`)
        '''
        self.robot_goal_status[robot_name] = RobotGoalStatus.GOAL_STATUS_PENDING

        # Assign UUID
        uuid_arr = np.random.randint(256, size=16, dtype='uint8')   # Random UUID
        curr_uuid = UUID(uuid=uuid_arr)
        self.robot_uuids[robot_name] = uuid_arr
        
        goal_msg = LocalPlanner.Goal()
        goal_msg.goal_position = goal_position

        now_s = self.get_clock().now().seconds_nanoseconds()
        now_s = float(now_s[0] + now_s[1]/1e9)
        new_goal = goal_output(goal_coords=(
            goal_position.x, goal_position.y
        ), distance_travelled=0.0, start_time=now_s)
        self.results[robot_name].append(new_goal)

        # Delete existing object, if it exists
        if robot_name in self.gazebo_goals.keys():
            self.get_logger().debug(f"Attempting to delete goal for {robot_name}")
            self.delete_goal(robot_name)
        
        # Spawn object
        self.get_logger().debug(f"Attempting to spawn goal for {robot_name}")
        self.spawn_goal(robot_name, goal_position.x, goal_position.y)
        self.gazebo_goals[robot_name] = True

        self.get_logger().debug(f"Robot {robot_name} waiting for action server")
        self.action_clients[robot_name].wait_for_server()
        self.get_logger().debug(f"Robot {robot_name} action server done")

        # ? Remove feedback callback because RRT node doesn't have that yet?
        self.goal_futures[robot_name] = self.action_clients[robot_name].send_goal_async(
            goal_msg, goal_uuid=curr_uuid)
            # goal_msg, feedback_callback=self.feedback_callback, goal_uuid=goal_uuid)
            
        # Add a callback for when the future (goal response) is complete
        self.goal_futures[robot_name].add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        # self.get_logger().info("Goal ID {}".format(goal_handle.goal_id.uuid))

        # Extract information about the robot from the response (no way to do this via other means)
        for key in self.robot_uuids.keys():
            if np.all(self.robot_uuids[key] == goal_handle.goal_id.uuid):
                robot_name = key
                break

        if not goal_handle.accepted:
            # TODO: Respond to goal rejected by attempting to send another goal.
            # If goal cannot be completed in ~TIMEOUT seconds, reject goal too?
            self.get_logger().info(f'Goal rejected by robot {robot_name}')
            return

        self.get_logger().debug(f'Goal accepted by robot {robot_name}')
        self.robot_goal_status[robot_name] = RobotGoalStatus.GOAL_STATUS_DOING

        self.result_futures[robot_name] = goal_handle.get_result_async()
        self.result_futures[robot_name].add_done_callback(self.get_result_callback)
        return

    def get_result_callback(self, future):
        result = future.result().result

        # Extract information about the robot from the response by format
        robot_name = result.robot_name.data
        robot_name = robot_name.strip('/')   # get rid of topic name
        
        # Remove goal from current robot.
        self.robot_remaining_goals[robot_name] -= 1

        self.get_logger().info(f'Robot {robot_name} Success: {result.success.data} Final pos {result.final_position.x:.2f},{result.final_position.y:.2f} Remaining goals {self.robot_remaining_goals[robot_name]}')

        # Update field in goal log
        now_s = self.get_clock().now().seconds_nanoseconds()
        now_s = float(now_s[0] + now_s[1]/1e9)
        self.results[robot_name][-1].completion_time = now_s
        self.results[robot_name][-1].start_coords = (result.initial_position.x, result.initial_position.y)
        self.results[robot_name][-1].distance_travelled = result.distance_travelled.data
        self.results[robot_name][-1].num_waypoints = result.num_waypoints.data

        self.robot_goal_status[robot_name] = RobotGoalStatus.GOAL_STATUS_READY

    def goal_assignment_timer_callback(self):
        
        # Check if all robots are done spawning
        if all(self.robots_done_spawning.values()):
            # Check if all robots are done
            if all( remaining_goals == 0 for remaining_goals in self.robot_remaining_goals.values() ):
                now_s = self.get_clock().now().seconds_nanoseconds()
                now_s = float(now_s[0] + now_s[1]/1e9)
                self.results['end_time'] = now_s

                self.get_logger().info("All robots are done with their goals. Shutting down node.")

                self.dump_results()
                self.destroy_node()
                rclpy.shutdown()

            # Iterate over all robots
            for robot_name in self.robot_remaining_goals.keys():

                # Debug hook for checking if this is running
                self.get_logger().debug(f"Robot {robot_name} status {self.robot_goal_status[robot_name]} " + \
                f"Remaining Goals {self.robot_remaining_goals[robot_name]}")

                if self.robot_goal_status[robot_name] == RobotGoalStatus.GOAL_STATUS_READY \
                    and self.robot_remaining_goals[robot_name] > 0:
                    # assign a new goal
                    if self.randomly_generate_goals:
                        goal_idx = np.random.randint(len(GOAL_ARRAY))
                        goal_coords =  GOAL_ARRAY[goal_idx]
                    else:
                        robot_idx = self.robot_name_to_idx[robot_name]
                        goal_idx = len(self.goal_array[robot_idx]) - self.robot_remaining_goals[robot_name]
                        goal_coords = (self.goal_array[robot_idx][goal_idx][0], self.goal_array[robot_idx][goal_idx][1])

                    self.get_logger().info(f"Sending {robot_name} Goal {goal_coords[0]:.2f},{goal_coords[1]:.2f}. {self.robot_remaining_goals[robot_name]} goals left")

                    self.send_goal(robot_name, Point(x=goal_coords[0], y=goal_coords[1], z=0.0))
        else:
            self.get_logger().info(f"Waiting for all robots to spawn...")


    def watchdog_timer_callback(self):
        '''
        Periodically checks if the execution time is greater than the allowed time.
        '''
        if self._sim_start_time is not None:
            nt = self.get_clock().now().seconds_nanoseconds()
            now_time = float(nt[0] + nt[1]/1e9)
            time_diff = now_time-self._sim_start_time
            
            if time_diff > self._watchdog_expiry_time:
                self.get_logger().warn(f"Time: {time_diff:.2f}. Timeout of {self.get_parameter('watchdog_timeout_s').value}s reached. Shutting down node.")
                self.dump_results()

                self.destroy_node()
                rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.current_pose
        name = feedback_msg.feedback.robot_name
        curr_x = feedback.position.x
        curr_y = feedback.position.y
        curr_yaw = euler_from_quaternion([
            feedback.orientation.x,
            feedback.orientation.y,
            feedback.orientation.z,
            feedback.orientation.w
        ])[2]       # Only rotation about the z axis
        
        self.get_logger().debug(f'Robot {name} Current X:{curr_x:.2f} Y:{curr_y:.2f} Yaw:{np.degrees(curr_yaw):.2f}')

    def parameter_callback(self, params):
        # (don't modify robot_list)
        self.get_logger().warn("No parameters are able to be set for this node.")
        return SetParametersResult(successful=False)

    # TODO pass goal/uuid number into the spawn_delete arguments so that goals are deleted spawned appropriately.
    def spawn_goal(self, robot_name: str, goal_x: float, goal_y: float):
        '''
        Spawns a goal model in Gazebo for visualization.
        '''
        request = SpawnEntity.Request()
        request.name = robot_name + "_goal"
        
        root = ET.parse(GOAL_SDF_PATH)
        color = root.find('model').find('link').find('visual').find('material').find('script').find('name')
        robot_idx = self.robot_name_to_idx[robot_name]
        color.text = colour_palette[robot_idx]

        request.xml = ET.tostring(root.getroot(), encoding='unicode')
        request.initial_pose.position.x = goal_x
        request.initial_pose.position.y = goal_y
        request.initial_pose.position.z = 0.0

        if self.randomly_generate_goals:
            goal_num = self.get_parameter('total_goals').value-self.robot_remaining_goals[robot_name] + 1
        else:
            robot_idx = self.robot_name_to_idx[robot_name]
            goal_num = len(self.goal_array[robot_idx])-self.robot_remaining_goals[robot_name] + 1
        self.get_logger().debug(f"Spawning goal {goal_num} for {robot_name}.")
        # Send request
        self.spawn_future = self.spawn_client.call_async(request)
        self.spawn_future.add_done_callback(self.spawn_done)
    
    def spawn_done(self, response):
        self.get_logger().debug("Spawn Service done!")

    def delete_goal(self, robot_name: str):
        '''
        Deletes a goal model in Gazebo for visualization.
        '''
        request = DeleteEntity.Request()
        request.name = robot_name + "_goal" # match that in spawn_goal
        
        if self.randomly_generate_goals:
            goal_num = self.get_parameter('total_goals').value-self.robot_remaining_goals[robot_name]
        else:
            robot_idx = self.robot_name_to_idx[robot_name]
            goal_num = len(self.goal_array[robot_idx])-self.robot_remaining_goals[robot_name]
        self.get_logger().debug(f"Deleting goal {goal_num} for {robot_name}.")
        # Send request
        self.delete_future = self.delete_client.call_async(request)
        self.delete_future.add_done_callback(self.delete_done)
    
    def delete_done(self, response):
        self.get_logger().debug("Delete Service done!")

    def dump_results(self) -> None:
        '''
        Dumps the `results` dictionary to a YAML file. 
        '''
        # Go through all the dataclasses and convert to dictionary
        for label in self.results.keys():
            # Only go through the 'robots' keys
            if 'start_time' in label or 'end_time' in label:
                continue            
            for element in self.results[label]:
                element = element.to_dict()
        

        self.get_logger().debug(f"Saved results file in {self.result_file}")
        with open(self.result_file, 'w') as file:
            yaml.dump(self.results, file, sort_keys=True)

            with open(self.parameter_file, 'r') as f2:
                # Append planner params yaml file to the end of results file
                file.write('\n')
                file.write(f2.read())

    def _handle_robot_done_spawning(self, msg):
        '''Recieves a `std_msgs/String` on robot_namespace/finished_spawning.'''
        robot_name = msg.data
        self.get_logger().debug(f"{robot_name} done spawning.")
        self.robots_done_spawning[robot_name] = True

        if all(self.robots_done_spawning.values()):
            st = self.get_clock().now().seconds_nanoseconds()
            self._sim_start_time = float(st[0] + st[1]/1e9)

    def _handle_robot_rrt_done(self, msg):
        '''Recieves a `std_msgs/String` saying that robot is done with RRT planning.'''
        robot_name = msg.name.data
        plan_time = msg.data.data

        self.results[robot_name][-1].plan_time = plan_time
        self.get_logger().debug(f"{robot_name} took {self.results[robot_name][-1].plan_time:.2f}s CPU time to plan RRT.")
        
        if self.get_parameter('realtime_factor').value > 0.0:
            real_to_sim = self.get_parameter('realtime_factor').value
        else:
            real_to_sim = 10.0 # Approximation

        self._watchdog_expiry_time += plan_time*real_to_sim

def main(args=None):
    rclpy.init(args=args)

    try:
        goal_creation = GoalCreation()
        # MultiThreadedExecutor lets us listen to the current location while also doing 
        # Action Server stuff
        executor = MultiThreadedExecutor()  # use max number of threads, 12
        executor.add_node(goal_creation)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            goal_creation.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()