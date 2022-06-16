from time import time
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rclpy.time import Duration, Time
from rcl_interfaces.msg import SetParametersResult

from planner_action_interfaces.action import LocalPlanner
from planner_action_interfaces.msg import OtherRobotLocations
from planner_action_interfaces.msg import PlannerStatus as PlannerStatusMsg
from planner_action_interfaces.srv import GetPlannerStatus, GetIntValue, GetRRTWaypoints, SetRRTWaypoint, SetPoint

from std_msgs.msg import Header, Int32, String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, PointStamped, Quaternion, Vector3
from visualization_msgs.msg import MarkerArray, Marker

from tf_transformations import euler_from_quaternion, quaternion_from_euler

from multirobot_control.map_params import OBSTACLE_ARRAY
from multirobot_control.math_utils import check_collision, dist_to_aabb, get_point_on_connecting_line
from multirobot_control.planner_status import PlannerStatus
from multirobot_control.dwa_server import DWABaseNode

import numpy as np
from typing import Dict, List, Tuple

class DWAReplanServer(DWABaseNode):
    '''
    DWA node that also implements functionality for multiple robots.
    '''
    def __init__(self):
        super().__init__('dwa_replan_server')

        self.declare_parameter('replan_duration', 5.0)
        self.params['replan_duration'] = self.get_parameter('replan_duration').value

        # Periodically poll the RRT server for info about the global plan
        self.create_timer(1/2, self._get_rrt_timer_callback)
        self.curr_rrt_client = self.create_client(GetRRTWaypoints, f'{self.get_namespace()}/rrt_star_action_server/get_rrt_waypoints')

        # Check if replan
        self.create_timer(1/2, self._check_replan_timer_callback)

        self._last_replan = Time(seconds=0, nanoseconds=0, clock_type=self.get_clock().clock_type)
        '''The timestamp where replanning was done due to stall detection.'''

        self.closest_robot = None
        self._curr_manhattan_dist = None
        self._target_manhattan_dist = None

    def _check_replan_timer_callback(self):
        '''
        Periodically checks if a replan is necessary. Critiera:
        - If the nearest robot is closer than the current goal, and the nearest robot is 
        in between the current goal and this robot's location.
        - We roughly determine if the nearest robot is in between by comparing their relative
        bearings. If the difference in bearing is less than ~70 deg (a tunable parameter) we
        consider it to be "in between"
        - If the current robot's RRT manhattan distance is greater than the nearest robot's
        manhattan distance
        - If the last replan was more than `replan_duration` ago
        '''
        if self.closest_robot_pos is not None:
            # Check distances
            dist_to_goal = self.distToGoal(self._x, self._y, self.goal_x, self.goal_y)
            dist_to_robot = self.distToGoal(self._x, self._y, self.closest_robot_pos[0],
                self.closest_robot_pos[1])
            
            # Also account for the case where the other robot is ON the goal
            # if dist_to_goal-dist_to_robot > -0.05:

            # If there is a robot in the vicinity
            if dist_to_robot < self.params['robot_radius']*3.5:
                # Check bearing
                bearing_to_goal = np.arctan2((self.goal_y-self._y), (self.goal_x-self._x))
                bearing_to_robot = np.arctan2((self.closest_robot_pos[1]-self._y), (self.closest_robot_pos[0]-self._x))
                bearing_diff = np.abs(bearing_to_goal - bearing_to_robot)

                # Check relative headings
                curr_hdg_x = self.params['robot_radius']*np.cos(self._yaw) + self._x
                curr_hdg_y = self.params['robot_radius']*np.sin(self._yaw) + self._y

                other_hdg_x = self.params['robot_radius']*np.cos(self.closest_robot_pos[2]) + self.closest_robot_pos[0]
                other_hdg_y = self.params['robot_radius']*np.cos(self.closest_robot_pos[2]) + self.closest_robot_pos[1]

                dist_from_headings = np.linalg.norm(
                    np.array((curr_hdg_x, curr_hdg_y)) - \
                    np.array((other_hdg_x, other_hdg_y))
                )

                self.get_logger().info(
                    f"Curr {self._x:.2f},{self._y:.2f},{np.degrees(self._yaw):.2f} Other {self.closest_robot_pos[0]:.2f},{self.closest_robot_pos[1]:.2f},{self.closest_robot_pos[2]:.2f} Dist1: {dist_to_robot:.2f} Dist2: {dist_from_headings:.2f}"
                )
                
                if dist_from_headings < dist_to_robot and np.degrees(bearing_diff) < 70.0:
                    # Get the current time
                    now_time_f = self.get_clock().now().seconds_nanoseconds()
                    now_time_f = now_time_f[0] + now_time_f[1]/1e9
                    last_replan_f = self._last_replan.seconds_nanoseconds()
                    last_replan_f = last_replan_f[0] + last_replan_f[1]/1e9
                    time_diff = now_time_f - last_replan_f

                    if self._curr_manhattan_dist > self._target_manhattan_dist:

                        if time_diff > self.params['replan_duration']:
                            self.get_logger().info(f"{self.get_namespace()} aborting current goal.")
                            self._last_replan = self.get_clock().now()
                            self.set_planner_state(PlannerStatus.PLANNER_ABORT)
                        else:
                            self.get_logger().info(f"Time since last replanning: {time_diff:.2f}. Waiting for {self.params['replan_duration']}.")
        
        else:
            # No nearby robot for replanning to take place
            pass

    def handle_other_robot_state(self, msg):
        '''
        Wraps around the base implementation of `handle_other_robot_state`.

        Check if the current closest robot is the same as on the previous iteration of this
        callback, so we know to change the target of the RRT Query client.
        '''

        past_closest_robot = self.closest_robot

        super().handle_other_robot_state(msg)

        if self.closest_robot != past_closest_robot:
            if past_closest_robot is None:
                # Create new client
                self.target_rrt_client = self.create_client(GetRRTWaypoints, f'/{self.closest_robot}/rrt_star_action_server/get_rrt_waypoints')
                self.get_logger().info(f"{self.closest_robot} in range, creating RRT query client")
            else:
                self.target_rrt_client.destroy()

                if self.closest_robot is not None:
                    # Create new client
                    self.target_rrt_client = self.create_client(GetRRTWaypoints, f'/{self.closest_robot}/rrt_star_action_server/get_rrt_waypoints')
                    self.get_logger().info(f"{past_closest_robot} out of range, moving client to {self.closest_robot}.")
                else:
                    # Remove reference to this variable
                    del self.target_rrt_client
                    self.get_logger().info(f"{past_closest_robot} out of range, deleting RRT query client.")

    def _get_rrt_timer_callback(self):
        '''Periodically polls the RRT servers to get info about the high-level overview of the plan.'''
        if self.closest_robot is not None:
            curr_rrt_future = self.curr_rrt_client.call_async(GetRRTWaypoints.Request())
            curr_rrt_future.add_done_callback(self._get_curr_rrt_dist_callback)
            target_rrt_future = self.target_rrt_client.call_async(GetRRTWaypoints.Request())
            target_rrt_future.add_done_callback(self._get_target_rrt_dist_callback)

def main(args=None):
    rclpy.init(args=args)

    try:
        dwa_replan_server = DWAReplanServer()
        # MultiThreadedExecutor lets us listen to the current location while also doing 
        # Action Server stuff
        executor = MultiThreadedExecutor()
        executor.add_node(dwa_replan_server)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            dwa_replan_server.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
