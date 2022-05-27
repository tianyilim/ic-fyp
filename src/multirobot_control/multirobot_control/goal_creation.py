'''
Node that handles creating and visualizing 'goals' for each robot. 
'''

from dataclasses import dataclass
from typing import Dict, Tuple, List
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

from tf_transformations import euler_from_quaternion

# The dictionary containing the palette of robots--> gazebo colors
from multirobot_control.colour_palette import colour_palette
# Gloabl Goal array
from multirobot_control.map_params import GOAL_ARRAY

import numpy as np
from enum import Enum, auto
import yaml
from datetime import datetime

GOAL_SDF_PATH = os.path.join(get_package_share_directory('multirobot_control'),
    'urdf', 'goal.sdf')

class RobotGoalStatus(Enum):
    GOAL_STATUS_READY = auto()      # Ready to accept a new goal
    GOAL_STATUS_DOING = auto()      # Done with the goal
    GOAL_STATUS_PENDING = auto()    # Processing a goal

@dataclass
class goal_output():
    goal_coords: Tuple[float, float]
    start_coords: Tuple[float, float] = (0,0)
    distance_travelled: float = -1
    num_waypoints: int = -1
    start_time: float = -1
    completion_time: float = -1

    def to_dict(self) -> Dict :
        ''' Returns the output as a dictionary, so that we can easily write to YAML log file later. '''
        return {
            'goal_coords' : self.goal_coords,
            'start_coords' : self.start_coords,
            'distance_travelled' : self.distance_travelled,
            'num_waypoints' : self.num_waypoints,
            'start_time' : self.start_time,
            'completion_time' : self.completion_time,
        }

class GoalCreation(Node):

    def __init__(self):
        super().__init__('goal_creation')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_list', Parameter.Type.STRING_ARRAY),   # list of robot names to subscribe to
                ('robot_starting_x', Parameter.Type.DOUBLE_ARRAY),
                ('robot_starting_y', Parameter.Type.DOUBLE_ARRAY),
                ('robot_starting_theta', Parameter.Type.DOUBLE_ARRAY),
                ('total_goals', Parameter.Type.INTEGER),
                ('watchdog_timeout_s', Parameter.Type.INTEGER),
                ('result_folder', Parameter.Type.STRING),
                ('params_filepath', Parameter.Type.STRING)
            ]
        )

        self.add_on_set_parameters_callback(self.parameter_callback)
        
        assert self.get_parameter("robot_list").value is not None
        assert len(self.get_parameter("robot_list").value) != 0
        assert len(self.get_parameter("robot_starting_x").value) == len(self.get_parameter("robot_list").value), \
            "Ensure all robot starting x are specified!"
        assert len(self.get_parameter("robot_starting_y").value) == len(self.get_parameter("robot_list").value), \
            "Ensure all robot starting y are specified!"

        self.robot_remaining_goals = {}     # Collection of goals each robot still has to finish
        self.robot_uuids = {}               # Collection of UUID objects corresponding to each robot
        self.gazebo_goals = {}              # Flag if a Gazebo entity has been spawned already
        self.robot_goal_status: dict[str, RobotGoalStatus] = {}       
        # Flag on the goal completion status of a robot, either `GOAL_STATUS_READY` or `GOAL_STATUS_DOING`
        self.action_clients = {}            # Send goal to DWA server
        self.robot_name_to_idx = {}         # Map robot names to respective indices (eg. robot1->1)
        self.goal_futures = {}              # One goal future corresponding to each robot
        self.result_futures = {}            # Same for results

        # Code to visualise goals in Gazebo
        self.spawn_client = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawner service not available, waiting again...')

        self.delete_client = self.create_client(DeleteEntity, 'delete_entity')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Deleter service not available, waiting again...')

        self.get_logger().debug('Goal SDF path: {}'.format(GOAL_SDF_PATH))

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
            self.action_clients[robot] = \
                ActionClient(self, LocalPlanner, robot+'/rrt_star')
                # ActionClient(self, LocalPlanner, robot+'/dwa')

            self.robot_name_to_idx[robot] = index
            self.robot_remaining_goals[robot] = self.get_parameter('total_goals').value + 1

            # add to logging
            self.results[robot] = []

            # Send initial first goal (random generated)
            goal_idx = np.random.randint(len(GOAL_ARRAY))
            initial_goal =  GOAL_ARRAY[goal_idx]
            uuid_arr = np.random.randint(256, size=16, dtype='uint8')   # Random UUID except the first number
            # uuid_arr = np.zeros(16, dtype='uint8') # This is the datatype used to uniquely identify action goals
            uuid_arr[-1] = index
            curr_uuid = UUID(uuid=uuid_arr)
            self.robot_uuids[robot] = uuid_arr
            self.robot_goal_status[robot] = RobotGoalStatus.GOAL_STATUS_READY

            self.get_logger().info("Sending {} Goal X:{:.2f} Y:{:.2f} with UUID {}".format(
                robot, initial_goal[0], initial_goal[1], curr_uuid.uuid[-1]
            ))

            self.send_goal(robot, Point(x=initial_goal[0], y=initial_goal[1], z=0.0), 
                goal_uuid=curr_uuid)

        # Create a timer callback to periodically assign goals
        goal_timer_period = 1.0
        self.goal_assignment_timer = self.create_timer( goal_timer_period, self.goal_assignment_timer_callback )

        watchdog_timer_period = self.get_parameter('watchdog_timeout_s').value
        self.goal_assignment_timer = self.create_timer( watchdog_timer_period, self.watchdog_timer_callback )

    def send_goal(self, robot_name, goal_position: Point, goal_uuid):
        '''Sends a Point goal to the robot (specified by robot name)'''
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
            self.get_logger().info(f"Attempting to delete goal for {robot_name}")
            self.delete_goal(robot_name)
        
        # Spawn object
        self.get_logger().info(f"Attempting to spawn goal for {robot_name}")
        self.spawn_goal(robot_name, goal_position.x, goal_position.y)
        self.gazebo_goals[robot_name] = True

        self.get_logger().debug(f"Robot {robot_name} waiting for action server")
        self.action_clients[robot_name].wait_for_server()
        self.get_logger().debug(f"Robot {robot_name} action server done")

        # ? Remove feedback callback because RRT node doesn't have that yet?
        self.goal_futures[robot_name] = self.action_clients[robot_name].send_goal_async(
            goal_msg, goal_uuid=goal_uuid)
            # goal_msg, feedback_callback=self.feedback_callback, goal_uuid=goal_uuid)
            
        # Add a callback for when the future (goal response) is complete
        self.goal_futures[robot_name].add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        # self.get_logger().info("Goal ID {}".format(goal_handle.goal_id))

        # Extract information about the robot from the response (no way to do this via other means)
        for key in self.robot_uuids.keys():
            if np.all(self.robot_uuids[key] == goal_handle.goal_id.uuid):
                robot_name = key
                break

        if not goal_handle.accepted:
            # TODO: Respond to goal rejected by attempting to send another goal.
            # If goal cannot be completed in ~TIMEOUT seconds, reject goal too?
            self.get_logger().info('Goal rejected by robot {}'.format(robot_name))
            return

        self.get_logger().info('Goal accepted by robot {}'.format(robot_name))
        self.robot_goal_status[robot_name] = RobotGoalStatus.GOAL_STATUS_DOING

        self.result_futures[robot_name] = goal_handle.get_result_async()
        self.result_futures[robot_name].add_done_callback(self.get_result_callback)
        return

    def get_result_callback(self, future):
        result = future.result().result

        # Extract information about the robot from the response by format
        robot_name = result.robot_name.data
        robot_name = robot_name.strip('/')   # get rid of topic name

        self.get_logger().info('Robot {} Success: {} Final_Position X:{:.2f} Y:{:.2f}'.format(
            robot_name,
            result.success.data, result.final_position.x, result.final_position.y))
        
        # Remove goal from current robot.
        self.robot_remaining_goals[robot_name] -= 1
        self.get_logger().info(f"Robot {robot_name} has {self.robot_remaining_goals[robot_name]} goals remaining.")

        # Update field in goal log
        now_s = self.get_clock().now().seconds_nanoseconds()
        now_s = float(now_s[0] + now_s[1]/1e9)
        self.results[robot_name][-1].completion_time = now_s
        self.results[robot_name][-1].start_coords = (result.initial_position.x, result.initial_position.y)
        self.results[robot_name][-1].distance_travelled = result.distance_travelled.data
        self.results[robot_name][-1].num_waypoints = result.num_waypoints.data

        self.robot_goal_status[robot_name] = RobotGoalStatus.GOAL_STATUS_READY

    def goal_assignment_timer_callback(self):
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

            if self.robot_goal_status[robot_name] == RobotGoalStatus.GOAL_STATUS_READY:
                # assign a new goal
                goal_idx = np.random.randint(len(GOAL_ARRAY))
                goal_coords =  GOAL_ARRAY[goal_idx]

                uuid_num = self.robot_uuids[robot_name][-1]
                uuid_arr = np.random.randint(256, size=16, dtype='uint8')   # Random UUID except the first number
                # uuid_arr = np.zeros(16, dtype='uint8') # This is the datatype used to uniquely identify action goals
                uuid_arr[-1] = uuid_num + len(self.robot_remaining_goals)
                curr_uuid = UUID(uuid=uuid_arr)
                self.robot_uuids[robot_name] = uuid_arr
                self.robot_goal_status[robot_name] = RobotGoalStatus.GOAL_STATUS_PENDING

                if self.robot_remaining_goals[robot_name] > 1:
                    self.get_logger().info("Sending {} Goal X:{:.2f} Y:{:.2f} with UUID {}".format(
                        robot_name, goal_coords[0], goal_coords[1], uuid_arr[-1]
                    ))

                    self.send_goal(robot_name, Point(x=goal_coords[0], y=goal_coords[1], z=0.0), 
                        goal_uuid=curr_uuid)

                elif self.robot_remaining_goals[robot_name] == 1:
                    # assign goal back to start
                    robot_index = self.robot_name_to_idx[robot_name]
                    start_x = self.get_parameter('robot_starting_x').value[robot_index]
                    start_y = self.get_parameter('robot_starting_y').value[robot_index]
                    self.get_logger().info("Sending {} back to starting position X:{:.2f} Y:{:.2f} with UUID {}".format(
                        robot_name, start_x, start_y, uuid_arr[-1]
                    ))

                    self.send_goal(robot_name, Point(x=start_x, y=start_y, z=0.0), 
                        goal_uuid=curr_uuid)
                    
    def watchdog_timer_callback(self):
        '''
        If this callback triggers, the time limit for execution is over.

        Shut down gracefully and dump existing values to CSV.
        '''
        self.get_logger().warn(f"Timeout of {self.get_parameter('watchdog_timeout_s').value}s reached. Shutting down node.")
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
        
        self.get_logger().debug('Robot {} Current X:{:.2f} Y:{:.2f} Yaw:{:.2f}'.format(
            name, curr_x, curr_y, np.degrees(curr_yaw) ))

    def parameter_callback(self, params):
        # (don't modify robot_list)
        self.get_logger().info("No parameters are able to be set for this node.")
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

        self.get_logger().info(f"Spawning goal {self.get_parameter('total_goals').value-self.robot_remaining_goals[robot_name]+1} for {robot_name}.")
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

        self.get_logger().info(f"Deleting goal {self.get_parameter('total_goals').value-self.robot_remaining_goals[robot_name]} for {robot_name}.")
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