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

        # Periodically poll the RRT server for info about the global plan
        self.create_timer(1/2, self._get_rrt_timer_callback)
        self.curr_rrt_client = self.create_client(GetRRTWaypoints, f'{self.get_namespace()}/rrt_star_action_server/get_rrt_waypoints')

        self._last_replan = Time(seconds=0, nanoseconds=0, clock_type=self.get_clock().clock_type)
        '''The timestamp where replanning was done due to stall detection.'''

        self.closest_robot = None
        self._curr_manhattan_dist = None
        self._target_manhattan_dist = None

    # override stall callback
    def stall_detection_callback(self):
        '''
        Checks if the robot has moved a certain threshold distance within a certain time. 
        
        If not, perform recovery action:
        - If dist to obstacle is too low, propose a vector that moves away from obstacle at large speed
        - Else, propose a random vector (break out of local minima)
        '''

        if self._planner_state == PlannerStatus.PLANNER_EXEC:
            if self._dist_travelled < self.params['stall_dist_thresh']:
                local_goal_x = self.goal_x
                local_goal_y = self.goal_y
                
                goal_poses = [] # A collection of poses that are made of vectors away from potential obstacles

                for robot_name in self.other_robots.keys():
                    robot_x, robot_y = self.other_robots[robot_name]
                    dist_to_robot = self.distToGoal( self._x, self._y, robot_x, robot_y )
                    self.get_logger().info(f'{self.get_namespace()} dist to {robot_name}: {dist_to_robot:.2f}')
                    # Calculate a path away from the robots that are too close
                    if dist_to_robot < (self.params['inter_robot_dist']*self.params['robot_radius']):
                        # Compare RRT Manhattan distances. 
                        # If distance is further:
                        self.get_logger().info(f"Curr Dist: {self._curr_manhattan_dist:.2f} Target Dist: {self._target_manhattan_dist:.2f}")

                        if self._curr_manhattan_dist > self._target_manhattan_dist:
                            now_time_f = self.get_clock().now().seconds_nanoseconds()
                            now_time_f = now_time_f[0] + now_time_f[1]/1e9
                            last_replan_f = self._last_replan.seconds_nanoseconds()
                            last_replan_f = last_replan_f[0] + last_replan_f[1]/1e9
                            time_diff = now_time_f - last_replan_f
                            if time_diff > 5.0:
                                # Abort current goal (await replanning)
                                self.get_logger().info(f"{self.get_namespace()} aborting current goal.")
                                self._last_replan = self.get_clock().now()
                                self.set_planner_state(PlannerStatus.PLANNER_ABORT)
                            else:
                                self.get_logger().info(f"Time since last replanning: {time_diff:.2f}. Waiting.")
                    
                        # Set a timer to clear the `replan` flag (eg. 5-10s) so that replanning only happens once in awhile
                        # Tag this replan flag to name in case of multiple robots getting tangled together

                        self_pos = np.array((self._x, self._y))
                        robot_pos = np.array((robot_x, robot_y))
                        goal_poses.append( get_point_on_connecting_line(robot_pos, self_pos, (self.params['inter_robot_dist']*self.params['robot_radius'])) )

                for obstacle in OBSTACLE_ARRAY:
                    dist_to_obstacle, closest_point = dist_to_aabb(self._x, self._y, obstacle, get_closest_point=True)
                    if dist_to_obstacle < 1.5*self.params['robot_radius']:
                        # Set local goal to be twice the robot radius away from the closest point at a right angle.
                        
                        self.get_logger().info(f"Robot too close to obstacle. Planning away from it.")

                        self_pos = np.array((self._x, self._y))
                        robot_pos = np.array((closest_point[0], closest_point[1]))
                        goal_poses.append( get_point_on_connecting_line(robot_pos, self_pos, self.params['robot_radius']*1.5) )

                # Overwrite the goal vector by taking the mean of all proposed vectors
                if len(goal_poses) > 0:
                    goal_pose = np.mean(np.array(goal_poses), axis=0)
                    local_goal_x = goal_pose[0]
                    local_goal_y = goal_pose[1]

                # If we are not too close to an obstacle, move toward the goal on an arc
                goal_hdg = np.arctan2(local_goal_y-self._y, local_goal_x-self._x)
                hdg_diff = goal_hdg - self._yaw
                # Sine and Cosine of the heading diff informs our proposed vector, 
                # So long we use the same hypotenuse (choose the linear speed limit so 
                # it's not possible to exceed it)
                # We cap the linear and angular speed limit so they are not exceeded
                lin_twist = min(
                    (self.params['linear_speed_limit'])*np.cos(hdg_diff),
                     self.params['linear_speed_limit'])
                ang_twist = min(
                    (self.params['linear_speed_limit'])*np.sin(hdg_diff),
                     self.params['linear_speed_limit'])

                # We also discretize the values to one of the steps of the planner.
                self._linear_twist = self.params['linear_step']*np.round(lin_twist/self.params['linear_step'])
                self._angular_twist = self.params['angular_step']*np.round(ang_twist/self.params['angular_step'])

                self.get_logger().info(f"Stall detected. Moving towards local goal {local_goal_x:.2f},{local_goal_y:.2f} with vect {self._linear_twist:.2f}, {self._angular_twist:.2f}")

        # Reset distance travelled accumulator
        self._dist_travelled = 0

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
