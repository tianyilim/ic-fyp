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

from tf_transformations import euler_from_quaternion

# The dictionary containing the palette of robots--> gazebo colors
from multirobot_control.colour_palette import colour_palette

import numpy as np

# Global goal matrix (for now specific to the demo map)
GOAL_ARRAY = []
for x_goal in np.arange(-4, 4.1, 0.5):
    for y_goal in np.arange(-4, 4.1, 0.5):
        GOAL_ARRAY.append((x_goal, y_goal))

# Number of goals each robot has to finish
TOTAL_GOALS = 5

GOAL_SDF_PATH = os.path.join(get_package_share_directory('multirobot_control'),
    'urdf', 'goal.sdf')

class GoalCreation(Node):

    def __init__(self):
        super().__init__('goal_creation')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_list', Parameter.Type.STRING_ARRAY),   # list of robot names to subscribe to
            ]
        )

        self.add_on_set_parameters_callback(self.parameter_callback)
        
        assert self.get_parameter("robot_list").value is not None
        assert len(self.get_parameter("robot_list").value) != 0

        self.robot_goals = {}               # Collection of goals each robot still has to finish
        self.robot_uuids = {}               # Collection of UUID objects corresponding to each robot
        self.gazebo_goals = {}              # Collection of Goal entities
        self.action_clients = []            # Send goal to DWA server

        # Code to visualise goals in Gazebo
        self.spawn_client = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawner service not available, waiting again...')

        self.delete_client = self.create_client(DeleteEntity, 'delete_entity')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Deleter service not available, waiting again...')

        self.get_logger().info('Goal SDF path: {}'.format(GOAL_SDF_PATH))

        for index, robot in enumerate(self.get_parameter("robot_list").value):
            self.action_clients.append(
                ActionClient(self, LocalPlanner, robot+'/dwa')
            )

            # ? Will this be better to be indexed by number or by name?
            self.robot_goals[robot] = TOTAL_GOALS

            # Send initial first goal (random generated)
            goal_idx = np.random.randint(len(GOAL_ARRAY))
            initial_goal =  GOAL_ARRAY[goal_idx]
            self.get_logger().info("Sending {} Goal X:{:.2f} Y:{:.2f}".format(
                robot, initial_goal[0], initial_goal[1]
            ))
            uuid_arr = np.zeros(16) # This is the datatype used to uniquely identify action goals
            uuid_arr[-1] = index
            curr_uuid = UUID(uuid=uuid_arr)
            self.robot_uuids[robot] = curr_uuid

            self.send_goal(index, Point(x=initial_goal[0], y=initial_goal[1], z=0.0), 
                goal_uuid=curr_uuid)

    #####
    '''
     def __init__(self):
        super().__init__('spawn_entity')
        self.client = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()

    def send_request(self):
        self.req.name = str(sys.argv[1])
        self.req.xml = str(sys.argv[2])
        self.future = self.client.call_async(self.req)
    '''
    #####


    def send_goal(self, robot_idx, goal_position: Point, goal_uuid):
        '''Sends a Point goal to the robot (specified by index)'''
        goal_msg = LocalPlanner.Goal()
        goal_msg.goal_position = goal_position

        # ~ Spawn Gazebo object here (Shifting it if necessary)
        if robot_idx in self.gazebo_goals.keys():
            # Delete existing object
            self.delete_goal(robot_idx)
        
        # Spawn object
        self.spawn_goal(robot_idx, goal_position.x, goal_position.y)
        self.gazebo_goals[robot_idx] = True

        self.action_clients[robot_idx].wait_for_server()
        self._send_goal_future = self.action_clients[robot_idx].send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback, goal_uuid=goal_uuid)
        # Add a callback for when the future (goal response) is complete
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        # Extract information about the robot from the response
        for key in self.robot_uuids.keys():
            if self.robot_uuids[key] == goal_handle.goal_id:
                robot_name = key
                break

        if not goal_handle.accepted:
            # Todo: Respond to goal rejected by attempting to send another goal.
            # If goal cannot be completed in ~TIMEOUT seconds, reject goal too?
            self.get_logger().info('Goal rejected by robot {}'.format(robot_name))
            return

        self.get_logger().info('Goal accepted by robot {}'.format(robot_name))

        self._get_result_future = goal_handle.get_result_async()
        # Add a callback when the future (when action is complete) finishes
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        # Extract information about the robot from the response
        for index, key in enumerate(self.robot_uuids.keys()):
            if self.robot_uuids[key] == result.goal_id:
                robot_name = key
                break

        self.get_logger().info('Robot {} Success: {} Final_Position X:{:.2f} Y:{:.2f}'.format(
            robot_name,
            result.success.data, result.final_position.x, result.final_position.y))
        
        # Remove goal from current robot.
        self.robot_goals[robot_name] -= 1

        # Check if the current robot has finished all goals
        if self.robot_goals[robot_name] == 0:
            self.get_logger().info('Robot {} has finished all goals. Not sending any more.'\
                .format(robot_name))
            
            # check if all goals are complete, if so we shutdown this node.
            for remaining_goals in self.robot_goals.values():
                if remaining_goals != 0:
                    return  # We still have work to do
                
                self.get_logger().info("All robots have finished goals. Shutting down.")
                rclpy.shutdown()    # All nodes have reached all goals.

        else:
            # Generate a new goal
            goal_idx = np.random.randint(len(GOAL_ARRAY))
            goal_coords =  GOAL_ARRAY[goal_idx]
            self.get_logger().info("Sending {} Goal X:{:.2f} Y:{:.2f}".format(
                robot_name, goal_coords[0], goal_coords[1]
            ))
            uuid_num = future.goal_id[-1]
            uuid_arr = np.zeros(16) # This is the datatype used to uniquely identify action goals
            uuid_arr[-1] = uuid_num + len(self.robot_goals)
            curr_uuid = UUID(uuid=uuid_arr)
            self.robot_uuids[robot_name] = curr_uuid

            self.send_goal(index, Point(x=goal_coords[0], y=goal_coords[1], z=0.0), 
                goal_uuid=curr_uuid)

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
        return SetParametersResult(successful=False)

    def spawn_goal(self, robot_idx, goal_x: float, goal_y: float):
        '''
        Spawns a goal model in Gazebo for visualization.
        '''
        request = SpawnEntity.Request()
        request.name = str(robot_idx) + "_goal"
        
        root = ET.parse(GOAL_SDF_PATH)
        color = root.find('model').find('link').find('visual').find('material').find('script').find('name')
        color.text = colour_palette[robot_idx]

        request.xml = ET.tostring(root.getroot(), encoding='unicode')
        request.initial_pose.position.x = goal_x
        request.initial_pose.position.y = goal_y
        request.initial_pose.position.z = 0.0

        # Send request
        self.spawn_future = self.spawn_client.call_async(request)
        self.spawn_future.add_done_callback(self.spawn_done)
    
    def spawn_done(self, response):
        self.get_logger().info("Spawn Service done!")
        # print(dir(response))
        '''
        ['__await__', '__class__', '__del__', '__delattr__', '__dict__', '__dir__', 
        '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', 
        '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', 
        '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', 
        '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_callbacks', 
        '_cancelled', '_done', '_exception', '_exception_fetched', '_executor', '_lock', 
        '_result', '_schedule_or_invoke_done_callbacks', '_set_executor', 'add_done_callback', 
        'cancel', 'cancelled', 'done', 'exception', 'result', 'set_exception', 'set_result']
        '''


    def delete_goal(self, robot_idx):
        '''
        Deletes a goal model in Gazebo for visualization.
        '''
        request = DeleteEntity.Request()
        request.name = str(robot_idx) + "_goal"

        # Send request
        self.delete_future = self.delete_client.call_async(request)
        self.delete_future.add_done_callback(self.delete_done)
    
    def delete_done(self, response):
        self.get_logger().info("Delete Service done!")
        # print(dir(response))

def main(args=None):

    # TODO multiprocess executor
    rclpy.init(args=args)

    try:
        goal_creation = GoalCreation()
        # MultiThreadedExecutor lets us listen to the current location while also doing 
        # Action Server stuff
        executor = MultiThreadedExecutor()
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