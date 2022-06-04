
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
from planner_action_interfaces.srv import GetPlannerStatus, GetIntValue, GetRRTWaypoints, SetRRTWaypoint

from std_msgs.msg import Header, Int32
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

class DWAMultirobotServer(DWABaseNode):
    '''
    DWA node that also implements functionality for multiple robots.

    Overrides several functions:
    - `handle_other_robot_state`:
        - Perform "handshaking" between robots in close proximity, and based on their 
        current states, decide what state each robot needs to go into.
        - If both robots are in `PlannerState.PLANNER_EXEC`, perform "goal arbitration" 
        to determine which goals to move towards.
    - 
    Also implements the following functionality:
    - `set_rrt_goal_cli`: Service client that modifies the current RRT* waypoint to better
    allow for collaboration
        - Sends either a valid `waypoint_idx` or a set of x,y goal coordinates.
        - If `waypoint_idx` valid, then the corresponding value in the RRT* server is 
        updated, either moving to a previous or next waypoint.
        - If the x,y goal coordinates are valid, then updates the current `waypoint_idx`
        with the modified goal coordinates.
    '''

    def __init__(self):
        super().__init__('dwa_multirobot_server')

        self.declare_parameter('rrt_max_extend_length', 1.5)
        self.params['rrt_max_extend_length'] = self.get_parameter('rrt_max_extend_length').value

        self._delay_10_hz = self.create_rate(10)
        self._main_loop_timer = self.create_timer(0.1, self._main_loop)

        self._other_robot_request = True
        self._curr_rrt_request = True
        self._target_rrt_request = True

    """
    def handle_other_robot_state(self, msg):
        '''Handle incoming data on `otherRobotLocations`, to where other robots are predicted to be'''
        
        # TODO move the state querying into odom_distribution.
        # Because everything is callback-based, it should return fairly quickly.
        # Should not loop in this callback but rather wait till the next iteration...
        # Or have a megaloop running somewhere.
        self.other_robots = {}
        for idx in range(len(msg.positions)):
            pose = msg.positions[idx]
            name = msg.names[idx].data
            self.get_logger().debug("[handle_other_robots] {} at x:{:.2f} y:{:.2f}".format(name, pose.x, pose.y))

            self.other_robots[name] = (pose.x, pose.y)
    """

    def _main_loop(self):
        '''
        Main loop for containing the logic for goal arbitration.
        '''
        # Check if we need to go undergo a state transition
        # Look for the closest robot
        if len(self.other_robots) > 0:
            if self._planner_state == PlannerStatus.PLANNER_EXEC or self._planner_state == PlannerStatus.PLANNER_READY:
                # only update this value when all below services return
                if self._other_robot_request and self._curr_rrt_request and self._target_rrt_request:
                    min_dist = np.inf
                    self.min_dist_robot_name = list(self.other_robots.keys())[0]
                    for key in self.other_robots.keys():
                        x,y = self.other_robots[key]
                        dist = np.linalg.norm(np.array((x,y)) - np.array((self._x, self._y)) )
                        if dist < min_dist:
                            min_dist = dist
                            self.min_dist_robot_name = key
                    self.get_logger().info(f'Closest robot to {self.get_name()}: {self.min_dist_robot_name}, dist:{min_dist:.2f}')
                else:
                    self.get_logger().info("Not running min_dist request, waiting for services to return.")
                
                # Check if the robot is within joint planning distance.
                # ? Currently there is no option for this. This can be done by tuning the
                # ? 'odom_dist_thresh' parameter.

                # Only call the service if it not been called before
                # We reset the flag at the end of this if-block (when all services have returned)
                if self._other_robot_request:
                    # self.min_dist_robot_name is the robot closest to this one.
                    robot_svc = f"/{self.min_dist_robot_name}/{self.get_name()}/get_dwa_server_status"
                    self.other_robot_state = None
                    self._other_robot_request = False
                    other_robot_state_cli = self.create_client(GetPlannerStatus, robot_svc)
                    other_robot_state_future = other_robot_state_cli.call_async(GetPlannerStatus.Request())    # is in the enum
                    other_robot_state_future.add_done_callback(self._get_other_robot_state_callback)
                
                # Wait for the service to complete. If no response yet, wait till next iteration of megaloop.
                if self.other_robot_state == None:
                    self.get_logger().info(f"Waiting for other_robot_state service")
                    return
                
                self.get_logger().info(f"curr status: {self._planner_state.name}, target status: {self.other_robot_state.name}")

                if self._planner_state == PlannerStatus.PLANNER_EXEC:
                    if self.other_robot_state == PlannerStatus.PLANNER_EXEC:
                        self._prev_planner_state = self._planner_state
                        
                        # Get current robot's manhattan distance
                        if self._curr_rrt_request:
                            self._curr_manhattan_dist = None
                            self._curr_rrt_request = False
                            curr_rrt_waypoints_cli = self.create_client(GetRRTWaypoints, 
                                f"{self.get_namespace()}/rrt_star_action_server/get_rrt_waypoints" )
                            curr_rrt_waypoints_future = curr_rrt_waypoints_cli.call_async(GetRRTWaypoints.Request())
                            curr_rrt_waypoints_future.add_done_callback(self._get_curr_rrt_dist_callback)
                        
                        # Get target robot's manhattan distance
                        if self._target_rrt_request:
                            self._target_manhattan_dist = None
                            self._target_rrt_request = False
                            target_rrt_waypoints_cli = self.create_client(GetRRTWaypoints, 
                                f"/{self.min_dist_robot_name}/rrt_star_action_server/get_rrt_waypoints" )
                            target_rrt_waypoints_future = target_rrt_waypoints_cli.call_async(GetRRTWaypoints.Request())
                            target_rrt_waypoints_future.add_done_callback(self._get_target_rrt_dist_callback)

                        if self._curr_manhattan_dist == None or self._target_manhattan_dist == None:
                            self.get_logger().info(f"Waiting for curr_rrt_waypoints and target_rrt_waypoints services")
                            return

                        self._set_curr_rrt_goal_cli = self.create_client(SetRRTWaypoint, 
                            f'{self.get_namespace()}/rrt_star_action_server/set_rrt_waypoint')
                        self._set_target_rrt_goal_cli = self.create_client(SetRRTWaypoint, 
                            f'/{self.min_dist_robot_name}/rrt_star_action_server/set_rrt_waypoint')
                        
                        # priority based on the shorter distance to the goal.
                        if self._curr_manhattan_dist < self._target_manhattan_dist:
                            self.set_planner_state(PlannerStatus.PLANNER_EXEC_JOINT)

                            curr_goal = self._curr_path[self._curr_waypoint_idx]
                            target_goal = self._target_path[self._target_waypoint_idx]

                            # Current goals cannot coexist
                            if np.linalg.norm(np.array(curr_goal)-np.array(target_goal)) \
                                < (self.params['inter_robot_dist'] * self.params['robot_radius']):
                                
                                curr_idx, curr_goal, target_idx, target_goal = \
                                    self.get_collision_free_waypoint_set()

                                # Call service to modify goals
                                self.set_curr_waypoint_vals(curr_idx, curr_goal)
                                self.set_target_waypoint_vals(target_idx, target_goal)

                        else:
                            # Stop robot
                            self.set_planner_state(PlannerStatus.PLANNER_DEFERRED)
                            self._linear_twist = 0.0
                            self._angular_twist = 0.0
                            self.publish_cmd_vel(self._linear_twist, self._angular_twist)

                        self.get_logger().info(f"{self.get_namespace()}: {self._prev_planner_state.name}->{self._planner_state.name}")

                    if self.other_robot_state == PlannerStatus.PLANNER_READY:
                        self._prev_planner_state = self._planner_state
                        self.set_planner_state(PlannerStatus.PLANNER_EXEC_JOINT)
                        self.get_logger().info(f"{self.get_namespace()}: {self._prev_planner_state.name}->{self._planner_state.name}.")

                if self._planner_state == PlannerStatus.PLANNER_READY:
                    if self.other_robot_state == PlannerStatus.PLANNER_EXEC:
                        self._prev_planner_state = self._planner_state
                        self.set_planner_state(PlannerStatus.PLANNER_DEFERRED)
                        self.get_logger().info(f"{self.get_namespace()}: {self._prev_planner_state.name}->{self._planner_state.name}.")

                self._other_robot_request = True
                self._curr_rrt_request = True
                self._target_rrt_request = True
        
        else:
            self.get_logger().info("Not running goal arbitration, no robots nearby.", once=True)
            self._other_robot_request = True
            self._curr_rrt_request = True
            self._target_rrt_request = True
            
            # No robots in proximity.
            # Check if we need to transition back to regular PLANNER_EXEC or PLANNER_PLAN state
            if self._planner_state == PlannerStatus.PLANNER_DEFERRED \
            or self._planner_state == PlannerStatus.PLANNER_EXEC_JOINT:
                # Restore state to previous state
                self.get_logger().info(f"{self.get_namespace()}: {self._planner_state.name}->{self._prev_planner_state.name}.")
                self.set_planner_state(self._prev_planner_state)

                # Set velocity to 0
                self._linear_twist = 0.0
                self._angular_twist = 0.0
                self.publish_cmd_vel(self._linear_twist, self._angular_twist)

    def get_collision_free_waypoint_set(self):
        '''
        Given the current and target waypoint set, check 

        Returns a value for curr_idx, curr_goal, target_idx, and target_goal.

        If a value needs to be modified, it will not be None. Therefore, either curr_idx OR
        curr_goal will not be None, for instance (caught by assertion)
        '''

        curr_goal_is_last = self._curr_waypoint_idx == len(self._curr_path)-1
        target_goal_is_last = self._target_waypoint_idx == len(self._target_path)-1

        curr_idx = None
        target_idx = None
        curr_goal = None
        target_goal = None

        if curr_goal_is_last and target_goal_is_last:
            # Both goals are the last goal (can't be reassigned) and  collide with each other.
            # Give current robot priority by assigning target goal backwards (move idx back)
            target_idx = self._target_waypoint_idx-1
        else:
            if not curr_goal_is_last:
                # Do not modify goal for current robot.
                curr_idx, curr_goal = self.get_collision_free_waypoint(self._curr_path, self._curr_waypoint_idx)
            if not target_goal_is_last:
                # Do not modify goal for target robot.
                target_idx, target_goal = self.get_collision_free_waypoint(self._target_path, self._target_waypoint_idx)

        assert not(curr_idx is not None and curr_goal is not None), \
            "[get_collision_free_waypoint_set] attempting to write to both curr_idx and curr_goal."
        assert not(target_idx is not None and target_goal is not None), \
            "[get_collision_free_waypoint_set] attempting to write to both target_idx and target_goal."
        
        # Check if these waypoints are far enough apart.
        if target_idx is not None:
            target_pos = self._target_path[target_idx]
        else:
            target_pos = target_goal
        
        if curr_idx is not None:
            curr_pos = self._curr_path[curr_idx]
        elif curr_goal is not None:
            curr_pos = curr_goal
        else:
            curr_pos = self._curr_path[self._curr_waypoint_idx]

        proposed_dist = np.linalg.norm(np.array(target_pos)-np.array(curr_pos))
        if proposed_dist < (self.params['inter_robot_dist'] * self.params['robot_radius']):
            self.get_logger().error(f"Proposed curr {curr_pos} and target {target_pos} are still too close together with distance {proposed_dist:.2f}")

        return curr_idx, curr_goal, target_idx, target_goal

    def get_collision_free_waypoint(self, path:List[Tuple[float,float]], idx:int) -> Tuple[int, Tuple[float,float]]:
        '''
        Finds a point along the connecting line between the current waypoint and the next
        waypoint in `path`.
        
        If the current and next waypoint are close enough, then we just modify the waypoint
        index (skip to the next waypoint on the list).
        
        If the next waypoint is too far away (half the rrt_max_extend_length), we connect
        a point between the both of them and ensure that it is collision-free.
        '''

        waypoint = np.array(path[idx])
        next_waypoint = np.array(path[idx+1])
        dist_waypoints = np.linalg.norm(waypoint- next_waypoint)

        prop_point = None
        next_waypoint_idx = None

        skip_to_next_waypoint = dist_waypoints < self.params['rrt_max_extend_length']/2
        if skip_to_next_waypoint:
            next_waypoint_idx = idx + 1
        else:
            # Get point on the connecting line (proposed waypoint)
            prop_point = get_point_on_connecting_line(waypoint, next_waypoint,
                self.params['rrt_max_extend_length']/2 )

            # Check if waypoint is collision free.
            # Do not use safety radius for a bit more wiggle room
            _, prop_point = check_collision(prop_point, OBSTACLE_ARRAY, self.params['safety_thresh'],
                self.params['robot_radius'], use_safety_radius=False )

        return next_waypoint_idx, prop_point

    def set_curr_waypoint_vals(self, curr_idx, curr_goal):
        ''' Checks if curr_idx or curr_goal need to be changed in class variables, and also
        calls the service to SetRRTWaypoint for the curr RRT Server.
        '''
        req = None

        if curr_idx is not None:
            req = SetRRTWaypoint.Request()
            req.sel_waypoint_idx = SetRRTWaypoint.Request.EDIT_WAYPOINT_IDX
            req.waypoint_idx = Int32(data=curr_idx)
            self.get_logger().info(f"Modifying curr_waypoint idx from {self._curr_waypoint_idx} to {curr_idx}")
            self._curr_waypoint_idx = curr_idx
        elif curr_goal is not None:
            req = SetRRTWaypoint.Request()
            req.sel_waypoint_idx = SetRRTWaypoint.Request.EDIT_WAYPOINT_IDX
            req.waypoint = Point(x=curr_goal[0], y=curr_goal[1], z=0.0)
            self.get_logger().info(f"Modifying curr_waypoint from {self._curr_path[self._curr_waypoint_idx][0]:.2f},{self._curr_path[self._curr_waypoint_idx][1]:.2f} to {curr_goal[0]:.2f},{curr_goal[1]:.2f}")
            self._curr_path[self._curr_waypoint_idx] = curr_goal

        if req is not None:
            self._set_curr_rrt_goal_future = self._set_curr_rrt_goal_cli.call_async(req)
            self._set_curr_rrt_goal_future.add_done_callback(self._set_curr_rrt_goal_callback)
        # Else no action reqd

    def set_target_waypoint_vals(self, target_idx, target_goal):
        ''' Checks if target_idx or target_goal need to be changed in class variables, and also
        calls the service to SetRRTWaypoint for the target RRT Server.
        '''
        req = None

        if target_idx is not None:
            req = SetRRTWaypoint.Request()
            req.sel_waypoint_idx = SetRRTWaypoint.EDIT_WAYPOINT_IDX
            req.waypoint_idx = Int32(data=target_idx)
            self.get_logger().info(f"Modifying target_waypoint idx from {self._target_waypoint_idx} to {target_idx}")
            self._target_waypoint_idx = target_idx
        elif target_goal is not None:
            req = SetRRTWaypoint.Request()
            req.sel_waypoint_idx = SetRRTWaypoint.EDIT_WAYPOINT_IDX
            req.waypoint = Point(x=target_goal[0], y=target_goal[1], z=0.0)
            self.get_logger().info(f"Modifying target_waypoint from {self._target_path[self._target_waypoint_idx][0]:.2f},{self._target_path[self._target_waypoint_idx][1]:.2f} to {target_goal[0]:.2f},{target_goal[1]:.2f}")
            self._target_path[self._target_waypoint_idx] = target_goal

        if req is not None:
            self._set_target_rrt_goal_future = self._set_target_rrt_goal_cli.call_async(req)
            self._set_target_rrt_goal_future.add_done_callback(self._set_target_rrt_goal_callback)
        # Else no action reqd

    def planned_pos_timer_callback(self):
        ''' 
        Sends out planned position if DWA server is BUSY (a action is currently executing).
        Else, sends the current position.
        '''
        if self._planner_state == PlannerStatus.PLANNER_EXEC:
            # If an action is currently being executed
            self._planned_pos_pub.publish(PointStamped(
                header=Header(frame_id=self.get_namespace()),
                point=Point(
                    x=self._planned_pose[0], y=self._planned_pose[1], z=0.0
                )))
        elif self._planner_state == PlannerStatus.PLANNER_EXEC_JOINT:
            # Send for both target robot and own robot.
            self._planned_pos_pub.publish(PointStamped(
                header=Header(frame_id=self.get_namespace()),
                point=Point(
                    x=self._planned_pose[0], y=self._planned_pose[1], z=0.0
                )))
            # send for target robot
            self._planned_pos_pub.publish(PointStamped(
                header=Header(frame_id=f"{self._joint_plan_target}"),
                point=Point(
                    x=self._joint_plan_target_planned_pose[0], y=self._joint_plan_target_planned_pose[1], z=0.0
                )))
        elif self._planner_state == PlannerStatus.PLANNER_READY:
            # No action executed, send current x and y values
            self._planned_pos_pub.publish(PointStamped(
                header=Header(frame_id=self.get_namespace()),
                point=Point(
                    x=self._x, y=self._y, z=0.0
                )))
        # No need to send anything if we are in DEFERRED (the EXEC_JOINT node will send for us)

    def dwa_action_callback(self):
        # TODO add in stuff for planner_exec_joint and other stuff...
        # ...
        if self._planner_state == PlannerStatus.PLANNER_EXEC:
            # Enumerate possible actions, checking for bounds
            possible_linear = [ self._linear_twist ]
            if (self._linear_twist + self.params['linear_step']) <= self.params['linear_speed_limit']:
                possible_linear.append(self._linear_twist + self.params['linear_step'])
            if (self._linear_twist - self.params['linear_step']) >= -self.params['linear_speed_limit']:
                possible_linear.append(self._linear_twist - self.params['linear_step'])

            possible_angular = [ self._angular_twist ]
            if (self._angular_twist + self.params['angular_step']) <= self.params['angular_speed_limit']:
                possible_angular.append(self._angular_twist + self.params['angular_step'])
            if (self._angular_twist - self.params['angular_step']) >= -self.params['angular_speed_limit']:
                possible_angular.append(self._angular_twist - self.params['angular_step'])

            top_score = -np.inf
            top_pose = (self._x, self._y, self._yaw)

            # Variables for visualising trajectories in RViz
            vis_msg_array = []
            vis_idx = 0
            top_idx = 0

            # Forward simulate actions based on dynamics model (?)
            # Assumes that robot can instantly achieve the commanded velocity (might not be the case)
            for linear_twist in possible_linear:
                for angular_twist in possible_angular:
                    
                    # Rotation matrix for current angle
                    effective_yaw = self._yaw + angular_twist*self.params['simulate_duration']
                    c, s = np.cos(effective_yaw), np.sin(effective_yaw)
                    R = np.array(((c, -s), (s, c)))

                    # displacement vector
                    displacement = np.array([ linear_twist*self.params['simulate_duration'], 0 ])
                    displacement = R @ displacement

                    # Eventual position and orientation
                    end_pose = displacement + np.array([self._x, self._y])
                    end_pose = (end_pose[0], end_pose[1], effective_yaw)

                    # Rank score
                    self.get_logger().debug(f"Lin: {linear_twist:.2f} Ang: {angular_twist:.2f}")
                    score = self.rankPose(end_pose, linear_twist, angular_twist)

                    marker_vis = self.marker_from_traj(vis_idx, score, end_pose)
                    vis_msg_array.append(marker_vis)

                    if score > top_score:
                        top_score = score
                        top_idx = vis_idx
                        self._linear_twist = linear_twist
                        self._angular_twist = angular_twist
                        
                        top_pose = end_pose

                    vis_idx += 1
            
            self.get_logger().debug(f"---- Chose Lin: {self._linear_twist:.2f} Ang {self._angular_twist:.2f} Idx {top_idx}/{len(vis_msg_array)} ----\n")

            # Label top score with color. Invalid -> red, Not top score -> yellow, Top score -> Green
            vis_msg_array[top_idx].color.r = 0.0
            vis_msg_array[top_idx].color.g = 1.0

            # Visualise trajectories in RViz
            self.vis_planned_pos_pub.publish( MarkerArray(markers=vis_msg_array) )

            # only write this here because we don't want the value of the class var to be corrupted
            self._planned_pose = top_pose

            if top_score <= -np.inf:
                # Currently deadlock is handled by stopping the robot.
                self.get_logger().warn(f"No satisfactory new trajectories found. Stopping robot.", once=True)
                self._linear_twist = 0.0
                self._angular_twist = 0.0

            self.publish_cmd_vel(self._linear_twist, self._angular_twist)
            
            self.get_logger().debug('{} Curr X:{:.2f} Y:{:.2f} Yaw:{:.2f} Lin:{:.2f}, Ang:{:.2f}'.format(
                self.get_namespace(),
                self._x, self._y, np.degrees(self._yaw), 
                self._linear_twist, self._angular_twist ))

            if self.closeToGoal(self._x, self._y, self.goal_x, self.goal_y):
                # Ensure robot is stopped
                self._linear_twist = 0.0
                self._angular_twist = 0.0
                self.publish_cmd_vel(self._linear_twist, self._angular_twist)
                
                self.set_planner_state(PlannerStatus.PLANNER_READY)

            # ? Extensions: Check for collision / Timeout and send 'goal failed'?

        elif self._planner_state == PlannerStatus.PLANNER_EXEC_JOINT:
            self._linear_twist = 0.0
            self._angular_twist = 0.0
            self.publish_cmd_vel(self._linear_twist, self._angular_twist)

            self._target_linear_twist = 0.0
            self._target_angular_twist = 0.0
            self.publish_target_cmd_vel(self._target_linear_twist, self._target_angular_twist)

            self._planned_pose = (self._x, self._y)
            self._joint_plan_target_planned_pose = (self._joint_plan_target_x, self._joint_plan_target_y)   # Current position

    def stall_detection_callback(self):
        '''
        Checks if the robot has moved a certain threshold distance within a certain time. 
        
        If not, perform recovery action:
        - If dist to obstacle is too low, propose a vector that moves away from obstacle at large speed
        - Else, propose a random vector (break out of local minima)
        '''

        # ? Check if stall detection requried for PLANNER_EXEC_JOINT.

        if self._planner_state == PlannerStatus.PLANNER_EXEC:
            if self._dist_travelled < self.params['stall_dist_thresh']:
                local_goal_x = self.goal_x
                local_goal_y = self.goal_y
                
                goal_poses = [] # A collection of poses that are made of vectors away from potential obstacles

                for robot_name in self.other_robots.keys():
                    robot_x, robot_y = self.other_robots[robot_name]
                    # Calculate a path away from the robots that are too close
                    if self.distToGoal( self._x, self._y, robot_x, robot_y ) < (self.params['inter_robot_dist']*self.params['robot_radius']):
                        self.get_logger().info(f"Robot too close to other robot. Moving away from it.")
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

    # Helper functions
    def rankPose(self, end_pose, linear_twist, angular_twist):
        '''
        The core of the DWA algorithm. Scores final positions based on different factors.

        Add bonus to moving forward. Add cost for staying still.
        '''

        goal_K = self.params['goal_K']                              # P factor for proximity to goal
        orientation_ub_deg = self.params['orientation_ub_deg']      # cutoff in deg for distance to goal
        orientation_lb_deg = self.params['orientation_lb_deg']
        orientation_K = self.params['angular_K']                    # Max proportion of orientation that this can take

        safety_thresh_K = self.params['obstacle_K']
        
        # Staying still will have a cost of half of the best possible score.
        # This should reduce the chance of the robot stalling far away from the goal.
        if np.allclose(linear_twist, 0.0, atol=1e-3) and np.allclose(angular_twist, 0.0, atol=1e-3):
            score = -(goal_K/self.params['dist_thresh'])/4
        else:
            score = 0

        # Else, see how close it gets to the goal.
        # Cap the score to the maximum cutoff value.
        dist_to_goal = self.distToGoal(end_pose[0], end_pose[1], self.goal_x, self.goal_y)
        # dist_plus = min( goal_K / dist_to_goal,
        #                  goal_K / (self.params['dist_thresh']/2)
        #                 )
        '''
        dist_plus = goal_K / dist_to_goal
        '''
        dist_plus = 0.0
        if dist_to_goal < 1.0:
            dist_plus = goal_K / dist_to_goal
        else:
            goal_lb = 1.5   # The distance at which there is a dist_plus of 0
            dist_m = goal_K/(1.0-goal_lb)
            dist_c = -dist_m*goal_lb
            dist_plus = dist_m*dist_to_goal + dist_c

        score += dist_plus

        obstacle_acc = 0
        # Check for collisions or close shaves
        # OBSTACLE_ARRAY also handles the walls
        od_min = 1e9
        for obstacle in OBSTACLE_ARRAY:
            dist_to_obstacle = dist_to_aabb(end_pose[0], end_pose[1], obstacle)
            
            # print(f"Dist to obstacle for end_pose x:{end_pose[0]:.2f} y:{end_pose[1]:.2f}" + \
            #     f" is {dist_to_obstacle:.2f} for obstacle at x:{obstacle[0]:.2f} y:{obstacle[1]:.2f}")

            if dist_to_obstacle < self.params['robot_radius']:
                self.get_logger().debug(f"End pose ({end_pose[0]:.2f},{end_pose[1]:.2f},{np.degrees(end_pose[2]):.2f}) collision @ {(obstacle[0]+obstacle[2])/2:.2f}|{(obstacle[1]+obstacle[3])/2:.2f} dist {dist_to_obstacle:.2f}")
                score = -np.inf # Collision, don't process further
                return score

            elif dist_to_obstacle < (self.params['robot_radius'] + self.params['safety_thresh']):
                '''
                score -= safety_thresh_K / dist_to_obstacle # Too close for comfort
                '''
                if dist_to_obstacle < od_min: od_min = dist_to_obstacle
                
                # Use % instead of flat scaling
                obstacle_m = 1.0/self.params['safety_thresh']
                obstacle_c = -obstacle_m*(self.params['safety_thresh']+self.params['robot_radius'])
                obstacle_cost = (obstacle_m*dist_to_obstacle + obstacle_c)*safety_thresh_K
                obstacle_acc += obstacle_cost
                score += obstacle_cost # obstacle cost is negative

        for robot_name in self.other_robots.keys():
            x, y = self.other_robots[robot_name]
            dist_to_robot = self.distToGoal(end_pose[0], end_pose[1], x, y)
            if dist_to_robot < 2*self.params['robot_radius']:
                self.get_logger().debug(f"End pose {end_pose[0]:.2f}|{end_pose[1]:.2f}|{np.degrees(end_pose[2]):.2f} collision with robot at {x:.2f}|{y:.2f} dist {dist_to_robot:.2f}")
                score = -np.inf # Collision, don't process further
                return score
            elif dist_to_robot < (self.params['inter_robot_dist']*self.params['robot_radius']):
                score -= safety_thresh_K / dist_to_robot

        # Calculate heading difference to goal. Set the difference in degrees for more accurate scaling
        # This is proportional to the distance score. Otherwise it will 'overwhelm' the pathing intuition of the robot
        goal_hdg = np.arctan2(self.goal_y-end_pose[1], self.goal_x-end_pose[0])
        hdg_diff = min( orientation_ub_deg, abs( np.degrees(goal_hdg - end_pose[2])) )
        self.get_logger().debug(f"Ref hdg {np.degrees(goal_hdg):.2f}, curr hdg {np.degrees(end_pose[2]):.2f}")
        # Clamp heading between max and min
        hdg_clamp = max(min(hdg_diff, orientation_ub_deg), orientation_lb_deg)
        # y = mx + c
        #   0 = m*(orientation_lb_deg) + c
        # -orientation_K = m*(orientation_ub_deg) + c
        # c = -m*orientation_lb_deg
        # -orientation_K = m*(orientation_ub_deg-orientation_lb_deg)
        # m = orientation_K/-(orientation_ub_deg-orientation_lb_deg)
        # c follows
        hdg_m = -orientation_K/(orientation_ub_deg-orientation_lb_deg)
        hdg_c = -hdg_m*orientation_lb_deg
        hdg_cost = (hdg_m*hdg_clamp + hdg_c)*dist_plus
        score += hdg_cost # hdg cost is negative

        # Score breakdown
        self.get_logger().debug(
            f"{linear_twist:.2f}|{angular_twist:.2f}" + \
            f" | end: ({end_pose[0]:.2f}, {end_pose[1]:.2f}, {np.degrees(end_pose[2]):.2f})" + \
            f" | d: {dist_to_goal:.2f},{dist_plus:.2f} | h: {hdg_diff:.2f},{hdg_cost:.2f}" + \
            f" | o: {od_min:.2f},{obstacle_acc:.2f} | {score:.2f}"
        )

        return score

    def rankPoseJoint(self):
        '''
        Searches through the possible combination of two robots' possible vectors and
        returns the highest-scoring pair of scores.
        '''
        pass

    def _get_other_robot_state_callback(self, future):
        response = future.result()
        self.other_robot_state = PlannerStatus(response.planner_status.data)
        self.get_logger().info(f"Other_robot_state srv callback: {self.other_robot_state.name}")

    @staticmethod
    def _parse_get_rrt_response(response:GetRRTWaypoints.Response):
        manhattan_dist = response.remaining_dist.data
        waypoint_idx = response.waypoint_idx.data
        path = []
        for point in response.waypoints:
            path.append((point.x, point.y))
            
        return manhattan_dist, waypoint_idx, path

    def _get_curr_rrt_dist_callback(self, future):
        response = future.result()
        self._curr_manhattan_dist, self._curr_waypoint_idx , self._curr_path = \
            self._parse_get_rrt_response(response)
        self.get_logger().info(f"Curr RRT srv callback: Manhattan Dist: {self._curr_manhattan_dist:.2f}, Curr waypt idx: {self._curr_waypoint_idx}\n{self._curr_path}")

    def _get_target_rrt_dist_callback(self, future):
        response = future.result()
        self._target_manhattan_dist, self._target_waypoint_idx , self._target_path = \
            self._parse_get_rrt_response(response)
        self.get_logger().info(f"Target RRT srv callback: Manhattan Dist: {self._target_manhattan_dist:.2f}, Curr waypt idx: {self._target_waypoint_idx}\n{self._target_path}")

    def _set_curr_rrt_goal_callback(self, future):
        response = future.result()
        self.get_logger().info(f"Set Curr RRT srv callback: {'success' if response.success else 'failure'}")

    def _set_target_rrt_goal_callback(self, future):
        response = future.result()
        self.get_logger().info(f"Set Target RRT srv callback: {'success' if response.success else 'failure'}")

    def set_planner_state(self, state: PlannerStatus) -> None:
        self._planner_state = state
        self._dwa_status_pub.publish(PlannerStatusMsg(data=int(self._planner_state)))
        if state == PlannerStatus.PLANNER_EXEC_JOINT:
            self._joint_plan_target = f"/{self.min_dist_robot_name}"
            self.get_logger().info(f"Subscribing to {self._joint_plan_target}/odom")
            self._target_state_sub = self.create_subscription(Odometry, f"{self._joint_plan_target}/odom", 
                    self._handle_target_odom_callback, qos_profile_sensor_data)

            self.get_logger().info(f"Publishing to {self._joint_plan_target}/cmd_vel")
            self._target_cmd_vel_pub = self.create_publisher(Twist, f"{self._joint_plan_target}/cmd_vel", 10)
        else:
            del self._joint_plan_target
            try:
                self._target_state_sub.destroy()
                self._target_cmd_vel_pub.destroy()
            except AttributeError:
                pass

    def publish_target_cmd_vel(self, lin_vel: float, ang_vel: float):
        '''Publish a cmd_vel to the current dwa target node'''
        cmd_vel = Twist(
            linear=Vector3(x=lin_vel, y=0.0, z=0.0), 
            angular=Vector3(x=0.0, y=0.0, z=ang_vel)
        )
        self._target_cmd_vel_pub.publish(cmd_vel)

    def _handle_target_odom_callback(self, msg):
        self._joint_plan_target_x = msg.pose.pose.position.x
        self._joint_plan_target_y = msg.pose.pose.position.y
        self._joint_plan_target_rpy = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        self._joint_plan_target_yaw = self._rpy[2]

        self.get_logger().debug("Joint Plan X:{} Y:{} Yaw:{}".format(
            self._joint_plan_target_x, self._joint_plan_target_y, self._joint_plan_target_yaw ))
        pass

def main(args=None):
    rclpy.init(args=args)

    try:
        dwa_multirobot_server = DWAMultirobotServer()
        # MultiThreadedExecutor lets us listen to the current location while also doing 
        # Action Server stuff
        executor = MultiThreadedExecutor()
        executor.add_node(dwa_multirobot_server)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            dwa_multirobot_server.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
