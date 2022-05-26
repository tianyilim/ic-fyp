'''
Much like DWA, RRT server is an Action server that creates a series of waypoints and guides
the the robot to meet each of them.

By design choice, each robot calculates its own navigation tree, as it shows that the
algorithim can be run in a distributed manner.
'''

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rclpy.time import Duration
from rcl_interfaces.msg import SetParametersResult

from planner_action_interfaces.action import LocalPlanner
from planner_action_interfaces.msg import OtherRobotLocations

from std_msgs.msg import Bool, String, Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, PointStamped, Quaternion, Vector3
from visualization_msgs.msg import Marker

from tf_transformations import euler_from_quaternion, quaternion_from_euler

from multirobot_control.map_params import OBSTACLE_ARRAY, OBSTACLE_BOUND, OBS_WIDTH, OBS_HEIGHT, GOAL_Y_OFFSET
from multirobot_control.rrt_node import RRT as RRTPlanner
from multirobot_control.colour_palette import colour_palette_rviz

import time
import numpy as np
from typing import List, Tuple
from enum import Enum, auto

class PlannerStatus(Enum):
    PLANNER_EXEC = auto()
    PLANNER_PLAN = auto()
    PLANNER_READY = auto()

class RRTStarActionServer(Node):
    def __init__(self) -> None:

        super().__init__('rrt_star_action_server')
        self._action_server = ActionServer(
            self,
            LocalPlanner,
            'rrt_star',
            self.execute_callback)
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_radius', 0.35),             # Physical radius of robot
                ('safety_thresh', 0.5),             # How far away from obstacles do we want to keep?
                ('dist_thresh', 0.1),               # How close to the goal do we need to get?
                ('rrt_path_bias', 0.15),            # % chance to go straight towards goal
                ('rrt_it_lim', 2000),               # Max number of iterations to run RRT for
                ('rrt_it_min', 50),               # Min number of iterations to run RRT for
                ('rrt_max_extend_length', 0.5),     # How far each RRT node is away from the previous
                ('rrt_connect_circle_dist', 1.0),   # How far away nodes must be to be considered for rewiring in RRT*
                ('rrt_debug_plot', False),          # Debug plot RRT in MatPlotLib
                ('robot_num', -1),                  # Assign goal colour in Rviz
            ])

        self.add_on_set_parameters_callback(self.parameter_callback)

        # state variables (update from Odom)
        self._x = 0.0
        self._y = 0.0
        self._rpy = (0.0, 0.0, 0.0)
        self._yaw = 0.0

        # Saved map of the workspace
        self._RRT = None
        # Path from start to goal
        self.path = []
        self.global_planner_status = PlannerStatus.PLANNER_READY
        self.local_planner_status = PlannerStatus.PLANNER_READY

        self.goal_marker_topic = self.get_namespace() + '/goals'
        self.path_marker_topic = self.get_namespace() + '/waypoints'
        self.path_marker_ids = []

        # Parameters
        self.params = {
            "robot_radius" :            self.get_parameter("robot_radius").value,
            "safety_thresh" :           self.get_parameter("safety_thresh").value,
            "dist_thresh" :             self.get_parameter("dist_thresh").value,
            "rrt_path_bias":            self.get_parameter("rrt_path_bias").value,
            "rrt_it_lim" :              self.get_parameter("rrt_it_lim").value,
            "rrt_it_min" :              self.get_parameter("rrt_it_min").value,
            "rrt_max_extend_length" :   self.get_parameter("rrt_max_extend_length").value,
            "rrt_connect_circle_dist" : self.get_parameter("rrt_connect_circle_dist").value,
            "rrt_debug_plot" :          self.get_parameter("rrt_debug_plot").value,
        }

        self.robot_num = self.get_parameter('robot_num').value
    
        # DWA client (or other local planner) -> TODO make this into a parameter
        self._action_client = ActionClient(self, LocalPlanner, 'dwa')

        # Subscribe to odom which tells us robot pose
        self.state_sub = self.create_subscription(Odometry, 'odom', \
            self.handle_odom, qos_profile_sensor_data)

        # Publish to RViz
        self.vis_goal_pub = self.create_publisher(Marker, "goal_markers", 10)
        self.vis_waypoint_pub = self.create_publisher(Marker, "waypoint_markers", 10)

        # Periodically poll if a new task is ready
        self.create_timer(0.1, self.spin_callback)

    def execute_callback(self, goal_handle):
        '''Executes the RRT* action'''
        self.global_planner_status = PlannerStatus.PLANNER_PLAN

        self.remove_goal_marker()
        self.remove_path_marker()

        self.goal_x = goal_handle.request.goal_position.x
        self.goal_y = goal_handle.request.goal_position.y

        # Figure out which side of the obstacle the goal is on and ensure we approach the obstacle from a right angle
        # No need to modify x
        y_idx = np.round(self.goal_y / OBS_HEIGHT)
        up_down = (self.goal_y - y_idx*OBS_HEIGHT) > 0

        # find out if we are dealing with a regular starting pose or goal
        if abs( abs( (self.goal_y - y_idx*OBS_HEIGHT) ) - GOAL_Y_OFFSET ) < 0.1:
            # It's a goal pose, place an artificial extra waypoint slightly
            # away from the midpoint between two shelves
            # Around 0.8-0.4+1.5/2 + 0.1 = 0.45
            if up_down:
                # obstacle on top of goal
                self.intermediate_y = self.goal_y + 0.45
            else:
                # obstacle below goal
                self.intermediate_y = self.goal_y - 0.45
        else:
            # Not goal pose
            self.intermediate_y = self.goal_y

        # Show goal and start locations
        self.get_logger().info(f"Robot ID {self.robot_num}")
        self.display_goal_marker(self.goal_x, self.goal_y)
        self.display_goal_marker(self._x, self._y, id=1, alpha=0.4)
        
        start_x = self._x
        start_y = self._y
        self.dist_travelled = 0.0

        # Find a suitable path through the workspace (and save it for future use).
        rrt_planner = RRTPlanner( start_pos=(self._x, self._y), goal_pos=(self.goal_x, self.intermediate_y),
                                  obstacle_list=OBSTACLE_ARRAY, bounds=OBSTACLE_BOUND,
                                  path_bias=self.params['rrt_path_bias'],
                                  it_lim=self.params['rrt_it_lim'],
                                  it_min=self.params['rrt_it_min'],
                                  max_extend_length=self.params['rrt_max_extend_length'], 
                                  safety_radius=self.params['safety_thresh'], 
                                  robot_radius=self.params['robot_radius'],
                                  connect_circle_dist=self.params['rrt_connect_circle_dist'],
                                  debug_plot=self.params['rrt_debug_plot'],
                                  logger=self.get_logger()
        )

        self.get_logger().info(f"Finding path to goal at {self.goal_x:.2f}, {self.goal_y:.2f}")
        self.path, num_nodes = rrt_planner.explore()
        self.path.append((self.goal_x, self.goal_y))
        self.get_logger().info(f"Path found with {len(self.path)} segments in {num_nodes} nodes")
        self.display_path_marker()

        self.global_planner_status = PlannerStatus.PLANNER_EXEC
        self.waypoint_idx = 0

        # We need to wait until the other threads finish
        while self.global_planner_status != PlannerStatus.PLANNER_READY:
            time.sleep(0.1)
        
        goal_handle.succeed()
        self.get_logger().info("{} reached goal at X: {:.2f} Y: {:.2f}".format(
            self.get_namespace(), self.goal_x, self.goal_y
        ))

        result = LocalPlanner.Result()
        result.success = Bool(data=True)
        result.final_position.x = self._x
        result.final_position.y = self._y
        result.final_position.z = 0.0
        result.robot_name = String(data=self.get_namespace())
        result.initial_position.x = start_x
        result.initial_position.y = start_y
        result.initial_position.z = 0.0
        result.distance_travelled = self.dist_travelled
        result.num_waypoints = num_nodes
        return result

    def spin_callback(self):
        # Call the local planner action repeatedly for each waypoint until goal reached.
        # Only send a new goal when the local planner is done with its previous goal

        # TODO do this as a callback and not by polling...
        if len(self.path) > 0 and self.local_planner_status==PlannerStatus.PLANNER_READY:
            # TODO check if it is the last waypoint (goal) and tighten up the distance threshold
            self.local_planner_status = PlannerStatus.PLANNER_EXEC
            self.get_logger().info(f"Going to waypoint at {self.path[0][0]:.2f}, {self.path[0][1]:.2f}. {len(self.path)} segments left.")
            
            local_goal = LocalPlanner.Goal()
            local_goal.goal_position = Point(x=self.path[0][0], y=self.path[0][1], z=0.0)

            self.goal_future = self._action_client.send_goal_async(local_goal)
            self.goal_future.add_done_callback(self.local_planner_done_callback)
        
        # Global planner is ready only when we are done executing all waypoints
        if len(self.path) == 0 and self.global_planner_status == PlannerStatus.PLANNER_EXEC:
            self.get_logger().info("Readying global planner")
            self.global_planner_status = PlannerStatus.PLANNER_READY

    def local_planner_done_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            # TODO: Respond to goal rejected by attempting to send another goal.
            # If goal cannot be completed in ~TIMEOUT seconds, reject goal too?
            self.get_logger().info('Local Planner goal rejected')
            self.local_planner_status = PlannerStatus.PLANNER_READY
            return

        self.get_logger().debug('Local Planner goal accepted')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.local_planner_get_result_callback)

    def local_planner_get_result_callback(self, future):
        result = future.result().result

        # Extract information about the robot from the response by format
        robot_name = result.robot_name.data.strip('/')   # get rid of topic name

        self.get_logger().info('{} Success: {} Final_Position X:{:.2f} Y:{:.2f}'.format(
            robot_name,
            result.success.data, result.final_position.x, result.final_position.y))
        
        # Remove goal from current robot.
        self.path.pop(0)
        
        # Remove waypoint marker from RViz Visualisation
        self.remove_path_marker_by_idx(self.waypoint_idx)
        self.waypoint_idx += 1

        self.local_planner_status = PlannerStatus.PLANNER_READY
        self.get_logger().info(f"Robot {robot_name} has {len(self.path)} waypoints left.")

    def action_feedback_cb(self, feedback):
        '''
        Call this when we want to get feedback from the Local Planner (DWA or otherwise)
        '''
        raise NotImplementedError

    def handle_odom(self, msg):
        '''
        Handle incoming data on `odom` by updating private variables
        Also increments the absolute distance travelled.
        '''
        
        # Increment distance travelled if we are travelling
        if self.global_planner_status == PlannerStatus.PLANNER_EXEC:
            self.dist_travelled += np.linalg.norm(
                np.array((self._x, self._y)) - \
                np.array((msg.pose.pose.position.x,msg.pose.pose.position.y))
            )

        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._rpy = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        self._yaw = self._rpy[2]

        self.get_logger().debug("Current X:{} Y:{} Yaw:{}".format(
            self._x, self._y, self._yaw ))

    def display_goal_marker(self, x:float, y:float, 
        id:int=0, alpha:float=0.8) -> Marker:
        '''
        Displays a Marker corresponding to the overall goal of the robot.
        '''
        marker = Marker()
        # header stamp
        marker.header.frame_id = "/map"             # Set relative to global frame
        marker.header.stamp = self.get_clock().now().to_msg()

        # namespace and id
        marker.ns = self.goal_marker_topic
        marker.id = id

        # Type of marker
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # Position of marker
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = -0.05
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the scale of the marker
        marker.scale.x = 0.3    # diameter of cylinder x axis
        marker.scale.y = 0.3    # y axis
        marker.scale.z = 0.1    # height

        # Set the color -- be sure to set alpha to something non-zero!
        color = colour_palette_rviz[self.robot_num]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        
        marker.color.a = alpha

        # Duration of marker
        marker.lifetime = Duration().to_msg()

        self.vis_goal_pub.publish(marker)

    def display_path_marker(self) -> None:
        '''
        Displays a list of waypoints that the robot will follow using its local planner.
        '''
        for i, (x, y) in enumerate(self.path):
            marker = Marker()
            # header stamp
            marker.header.frame_id = "/map"             # Set relative to global frame
            marker.header.stamp = self.get_clock().now().to_msg()

            # namespace and id
            marker.ns = self.path_marker_topic
            marker.id = i

            # Type of marker
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Position of marker
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = -0.05
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # Set the scale of the marker
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            # Set the color -- be sure to set alpha to something non-zero!
            color = colour_palette_rviz[self.robot_num]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.8

            # Duration of marker
            marker.lifetime = Duration().to_msg()

            self.vis_waypoint_pub.publish(marker)

    def remove_path_marker(self) -> Marker:
        '''
        Remove all waypoint markers corresponding to the previous movement.
        '''
        marker = Marker()
        marker.header.frame_id = "/map"             # Set relative to global frame
        marker.header.stamp = self.get_clock().now().to_msg()

        # namespace and id
        marker.ns = self.path_marker_topic
        marker.id = 0

        # Type of marker
        marker.type = Marker.CUBE
        marker.action = Marker.DELETE

        self.vis_waypoint_pub.publish(marker)

    def remove_path_marker_by_idx(self, idx:int) -> Marker:
        '''
        Remove the previously-reached waypoint marker.
        '''
        marker = Marker()
        marker.header.frame_id = "/map"             # Set relative to global frame
        marker.header.stamp = self.get_clock().now().to_msg()

        # namespace and id
        marker.ns = self.path_marker_topic
        marker.id = idx

        # Type of marker
        marker.type = Marker.CUBE
        marker.action = Marker.DELETE

        self.vis_waypoint_pub.publish(marker)

    def remove_goal_marker(self) -> Marker:
        '''
        Remove all goal markers corresponding to the previous movement.
        '''
        marker = Marker()
        marker.header.frame_id = "/map"             # Set relative to global frame
        marker.header.stamp = self.get_clock().now().to_msg()

        # namespace and id
        marker.ns = self.goal_marker_topic
        marker.id = 0

        # Type of marker
        marker.type = Marker.CYLINDER
        marker.action = Marker.DELETEALL

        self.vis_goal_pub.publish(marker)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'pub_freq':
                if param.type_==Parameter.Type.DOUBLE or param.type_==Parameter.Type.INTEGER:
                    freq = param.value
                    period_ns = 1e9/freq
                    self.timer.timer_period_ns = period_ns
                    
                    # Sanity check
                    freq_check = 1/self.timer.timer_period_ns
                    freq_check /= 1e9

                    self.get_logger().debug("Set pub_freq {} to value {:.3f}".format(
                        param.name, freq_check ))

                    return SetParametersResult(successful=True)
            else:
                # TODO error handling/type check/bounds check
                self.params[param.name] = param.value
                self.get_logger().debug("Set parameter {} to value {}".format(
                    param.name, self.params[param.name] ))

        # Write parameter result change
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    try:
        rrt_star_action_server = RRTStarActionServer()
        # MultiThreadedExecutor lets us listen to the current location while also doing 
        # Action Server stuff
        executor = MultiThreadedExecutor()
        executor.add_node(rrt_star_action_server)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            rrt_star_action_server.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
