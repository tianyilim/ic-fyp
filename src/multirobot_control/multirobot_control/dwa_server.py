'''
Create a DWA node.
'''
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
from planner_action_interfaces.srv import GetPlannerStatus, GetRRTWaypoints

from std_msgs.msg import Bool, String, Header, Int8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, PointStamped, Quaternion, Vector3
from visualization_msgs.msg import MarkerArray, Marker

from tf_transformations import euler_from_quaternion, quaternion_from_euler

from multirobot_control.map_params import OBSTACLE_ARRAY
from multirobot_control.math_utils import dist_to_aabb, get_point_on_connecting_line
from multirobot_control.planner_status import PlannerStatus

import numpy as np
from typing import Dict, List, Tuple

class DWABaseNode(Node):

    def __init__(self, impl_name:str):

        super().__init__(impl_name)
        self._action_server = ActionServer(
            self, LocalPlanner, impl_name,      # Action Server should be the same name as node name.
            self.execute_callback, cancel_callback=self.cancel_callback)
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_radius', 0.7),
                ('safety_thresh', 0.5),
                ('simulate_duration', 1.0),
                ('action_duration', 0.1),
                ('linear_speed_limit', 0.5),
                ('angular_speed_limit', 0.5),
                ('linear_step', 0.1),
                ('angular_step', 0.1),
                ('dist_thresh', 0.1),
                ('dist_method', "L2"),
                ('pub_freq', 20.0),
                ('inter_robot_dist', 3.0),
                ('orientation_ub_deg', 180.0),
                ('orientation_lb_deg', 20.0),
                ('angular_K', 0.5),
                ('goal_K', 10.0),
                ('obstacle_K', 1.0),
                ('stall_det_period', 1.0),
                ('stall_dist_thresh', 0.1),
            ])

        self.add_on_set_parameters_callback(self._set_parameter_callback)

        # state variables
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # Inputs to cmd_vel
        self._linear_twist = 0.0
        self._angular_twist = 0.0

        self._planned_pose = None

        # Distance accumulator
        self._dist_travelled = 0.0

        # Parameters
        self.params = {
            "robot_radius" :        self.get_parameter("robot_radius").value,
            "safety_thresh" :       self.get_parameter("safety_thresh").value,
            "simulate_duration" :   self.get_parameter("simulate_duration").value,
            "action_duration" :     self.get_parameter("action_duration").value,
            "linear_speed_limit" :  self.get_parameter("linear_speed_limit").value,
            "angular_speed_limit" : self.get_parameter("angular_speed_limit").value,
            "linear_step" :         self.get_parameter("linear_step").value,
            "angular_step" :        self.get_parameter("angular_step").value,
            "dist_thresh" :         self.get_parameter("dist_thresh").value,
            "dist_method" :         self.get_parameter("dist_method").value,
            "pub_freq" :            self.get_parameter("pub_freq").value,
            "inter_robot_dist" :    self.get_parameter("inter_robot_dist").value,
            "orientation_ub_deg" :  self.get_parameter("orientation_ub_deg").value,
            "orientation_lb_deg" :  self.get_parameter("orientation_lb_deg").value,
            "angular_K" :           self.get_parameter("angular_K").value,
            "goal_K" :              self.get_parameter("goal_K").value,
            "obstacle_K" :          self.get_parameter("obstacle_K").value,
            "stall_det_period" :    self.get_parameter("stall_det_period").value,
            "stall_dist_thresh" :   self.get_parameter("stall_dist_thresh").value,
        }
        
        # other robots on the map
        self.other_robots:Dict[str, Tuple[float,float]] = {} 

        # Subscribe to Odom (which has ground truth)
        self.state_sub = self.create_subscription(Odometry, 'odom', \
            self.handle_odom, qos_profile_sensor_data)

        # Sub to messages about the pose of other robots (and their predicted positions)
        self.other_robot_sub = self.create_subscription(OtherRobotLocations, 
            'otherRobotLocations', self.handle_other_robot_state, 10)

        # Publish to cmd_vel
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Publish predicted movement
        self._planned_pos_pub = self.create_publisher(PointStamped, "planned_pos", 10)
        self.vis_planned_pos_pub = self.create_publisher(MarkerArray, "planned_pos_markers", 10)

        # Timer callback for planned position
        self.planned_pos_timer = self.create_timer(1/self.params['pub_freq'], self.planned_pos_timer_callback)

        # Timer callback for stall detection
        self.stall_detection_timer = self.create_timer(self.params['stall_det_period'], self.stall_detection_callback)

        # Wait item to implement DWA node
        self._dwa_wait_rate = self.create_rate(1/self.params['action_duration'])
        self._dwa_action_timer = self.create_timer(self.params['action_duration'], self.dwa_action_callback)

        # Service to check for server status
        self._srv_dwa_status = self.create_service(GetPlannerStatus, f'{self.get_name()}/get_dwa_server_status', self._srv_dwa_status_callback)

        # Publish status changes 
        self._dwa_status_pub = self.create_publisher(PlannerStatusMsg, f'{self.get_name()}/dwa_status', 10)
        self.set_planner_state(PlannerStatus.PLANNER_READY)

        self.get_logger().info(f"{self.get_namespace()}/{self.get_name()} ready.")

    def handle_odom(self, msg):
        '''Handle incoming data on `odom` by updating private variables'''
        # Accumulate distance travelled
        self._dist_travelled += np.linalg.norm(
            np.array((self._x, self._y)) -
            np.array((msg.pose.pose.position.x, msg.pose.pose.position.y))
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

    def handle_other_robot_state(self, msg):
        '''Handle incoming data on `otherRobotLocations`, to where other robots are predicted to be'''
        self.other_robots = {}
        self.closest_robot = None
        closest_dist = np.inf

        for idx in range(len(msg.positions)):
            pose = msg.positions[idx]
            name = msg.names[idx].data
            self.get_logger().debug("[handle_other_robots] {} at x:{:.2f} y:{:.2f}".format(name, pose.x, pose.y))

            self.other_robots[name] = (pose.x, pose.y)

            dist = np.linalg.norm( np.array((pose.x, pose.y))-np.array((self._x, self._y)) )
            if dist < closest_dist:
                closest_dist = dist
                self.closest_robot = name
    
    def planned_pos_timer_callback(self):
        ''' 
        Sends out planned position if DWA server is EXEC (a action is currently executing).
        Else, sends the current position.
        '''
        if self._planner_state == PlannerStatus.PLANNER_EXEC:
            # If an action is currently being executed=
            self._planned_pos_pub.publish(PointStamped(
                header=Header(frame_id=self.get_namespace()),
                point=Point(
                    x=self._planned_pose[0], y=self._planned_pose[1], z=0.0
                )))
        else:
            # No action executed, send current x and y values
            self._planned_pos_pub.publish(PointStamped(
                header=Header(frame_id=self.get_namespace()),
                point=Point(
                    x=self._x, y=self._y, z=0.0
                )))

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

    def execute_callback(self, goal_handle):
        ''' Executes the DWA action. '''
        self.set_planner_state(PlannerStatus.PLANNER_EXEC)

        self.goal_x = goal_handle.request.goal_position.x
        self.goal_y = goal_handle.request.goal_position.y

        self.get_logger().info('{} moving towards goal ({:.2f}, {:.2f})'.format(self.get_namespace(), self.goal_x, self.goal_y))

        while not self._planner_state == PlannerStatus.PLANNER_READY:
            # Publish feedback
            feedback_msg = LocalPlanner.Feedback()
            feedback_msg.robot_name = String(data=self.get_namespace())
            feedback_msg.current_pose.position.x = self._x
            feedback_msg.current_pose.position.y = self._y
            feedback_msg.current_pose.position.z = 0.0
            curr_quaternion = quaternion_from_euler(0, 0, self._yaw)
            feedback_msg.current_pose.orientation.x = curr_quaternion[0]
            feedback_msg.current_pose.orientation.y = curr_quaternion[1]
            feedback_msg.current_pose.orientation.z = curr_quaternion[2]
            feedback_msg.current_pose.orientation.w = curr_quaternion[3]
            goal_handle.publish_feedback(feedback_msg)

            # Stop the robot
            if self._planner_state == PlannerStatus.PLANNER_ABORT:
                goal_handle.abort()

                self._linear_twist = 0.0
                self._angular_twist = 0.0
                self.publish_cmd_vel(0.0, 0.0)

                result = LocalPlanner.Result()
                
                result.success = Bool(data=False)
                result.final_position.x = self._x
                result.final_position.y = self._y
                result.final_position.z = 0.0
                result.robot_name = String(data=self.get_namespace())

                self.set_planner_state(PlannerStatus.PLANNER_READY)
                return result

            self._dwa_wait_rate.sleep()
        # Set planner status back to free so that correct planned messages are recieved

        # After DWA reaches goal
        goal_handle.succeed()
        self.get_logger().info("{} reached goal ({:.2f}, {:.2f}), currently at ({:.2f}, {:.2f})".format(
            self.get_namespace(), self.goal_x, self.goal_y, self._x, self._y
        ))

        result = LocalPlanner.Result()
        result.success = Bool(data=True)
        result.final_position.x = self._x
        result.final_position.y = self._y
        result.final_position.z = 0.0
        result.robot_name = String(data=self.get_namespace())
        return result

    def cancel_callback(self, cancelRequest):
        # ? When will we need to check if goal cannot be cancelled?
        self.get_logger().info(f"Cancel requested, stopping {self.get_namespace()}.")

        # Stop the robot
        self.set_planner_state(PlannerStatus.PLANNER_READY)

        self._linear_twist = 0.0
        self._angular_twist = 0.0
        self.publish_cmd_vel(self._linear_twist, self._angular_twist)

        return rclpy.action.server.CancelResponse.ACCEPT

    def dwa_action_callback(self):
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

                    marker_vis = self.marker_from_traj(vis_idx, score, (self._x, self._y), end_pose)
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

    def distToGoal(self, curr_x, curr_y, goal_x, goal_y):
        '''
        Returns distance to goal (or other x,y coordinate) based the `method`.
        `'L2'`: Calculates the L2 norm
        '''

        method = self.params['dist_method']

        if method=='L2':
            dist = np.hypot( curr_x-goal_x, curr_y-goal_y )
            return dist
        else:
            raise NotImplementedError("distance function {} not implemented yet".format(method))

    def closeToGoal(self, curr_x, curr_y, goal_x, goal_y):
        '''Returns True if we are `self.params['dist_thresh']` away from the goal'''
        
        if self.distToGoal(curr_x, curr_y, goal_x, goal_y) < self.params['dist_thresh']:
            return True
        else:
            return False

    def _list_possible_endpoints(self, current_pos:Tuple[float,float], current_yaw:float, 
        current_lin_vel:float, current_ang_vel:float) -> List[Tuple[float,float,float,float,float]]:
        ''' Based on the current position, linear, and angular velocities of a robot, report
        their possible poses after `simulate_duration`.
        
        Returns a list of end poses (x,y,theta,linear_vel,angular_vel).
        '''
        possible_linear = [ current_lin_vel ]
        if (current_lin_vel + self.params['linear_step']) <= self.params['linear_speed_limit']:
            possible_linear.append(current_lin_vel + self.params['linear_step'])
        if (current_lin_vel - self.params['linear_step']) >= -self.params['linear_speed_limit']:
            possible_linear.append(current_lin_vel - self.params['linear_step'])

        possible_angular = [ current_ang_vel ]
        if (current_ang_vel + self.params['angular_step']) <= self.params['angular_speed_limit']:
            possible_angular.append(current_ang_vel + self.params['angular_step'])
        if (current_ang_vel - self.params['angular_step']) >= -self.params['angular_speed_limit']:
            possible_angular.append(current_ang_vel - self.params['angular_step'])

        end_poses = []

        for linear_twist in possible_linear:
            for angular_twist in possible_angular:
                # Rotation matrix for current angle
                effective_yaw = current_yaw + angular_twist*self.params['simulate_duration']
                c, s = np.cos(effective_yaw), np.sin(effective_yaw)
                R = np.array(((c, -s), (s, c)))

                # displacement vector
                displacement = np.array([ linear_twist*self.params['simulate_duration'], 0 ])
                displacement = R @ displacement

                # Eventual position and orientation
                end_pose = displacement + np.array(current_pos)
                end_pose = (end_pose[0], end_pose[1], effective_yaw, linear_twist, angular_twist) 
                end_poses.append(end_pose)

        return end_poses

    def _set_parameter_callback(self, params):
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


    def marker_from_traj(self, idx: int, score: float, start_pos: Tuple[float,float], 
        end_pose: Tuple[float, float, float]) -> Marker :
        '''
        Returns a `Marker` object from the proposed trajectory.

        Args:
        - idx: An index uniquely identifying the proposed trajectory
        - score: Trajectory's score from `rankPose`
        - start_pos: Tuple of (x,y)
        - end_pose: Tuple of (x, y, yaw) - only x, y are used to determine the shape of the marker

        Returns:
        - `Marker` arrow object to be passed on to a MarkerArray and subsequently published.
        '''

        marker = Marker()
        # header stamp
        marker.header.frame_id = "/map"             # Set relative to global frame
        marker.header.stamp = self.get_clock().now().to_msg()

        # namespace and id
        marker.ns = self.get_namespace() + '/trajectories'
        marker.id = idx

        # Type of marker
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Position of marker
        marker.pose.position.x = start_pos[0]
        marker.pose.position.y = start_pos[1]
        marker.pose.position.z = 0.25
        # Orientation is diff between current pose and future pose
        yaw = np.arctan2((end_pose[1]-start_pos[1]), (end_pose[0]-start_pos[0]))
        quat = quaternion_from_euler(0, 0, yaw)
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        # Set the scale of the marker
        # x: Arrow length
        marker.scale.x = np.hypot((end_pose[1]-start_pos[1]), (end_pose[0]-start_pos[0])) 
        # y: Arrow width
        marker.scale.y = 0.05
        # z: Head length
        marker.scale.z = np.hypot((end_pose[1]-start_pos[1]), (end_pose[0]-start_pos[0])) * 0.2

        # Set the color -- be sure to set alpha to something non-zero!
        if score==-np.inf:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        
        marker.color.a = 0.8

        # Duration of marker, set to be the update rate of the 
        # controller so we don't have to delete objects
        period = self.params['action_duration']
        period_sec = int( np.round(period) )
        period_nanosec = int( (period-period_sec)*1e9 )
        marker.lifetime = Duration(seconds=period_sec, nanoseconds=period_nanosec).to_msg()

        return marker

    def set_planner_state(self, state:PlannerStatus) -> None:
        '''Sets the state of the DWA server internally and publishes it.'''
        self._planner_state = state
        self._dwa_status_pub.publish(PlannerStatusMsg(data=int(self._planner_state)))

    def publish_cmd_vel(self, lin_vel:float, ang_vel:float) -> None:
        '''Creates a Twist with the specified `lin` and `ang` velocities and publishes it 
        to `cmd_vel` for the current robot.'''
        cmd_vel = Twist(
            linear=Vector3(x=lin_vel, y=0.0, z=0.0), 
            angular=Vector3(x=0.0, y=0.0, z=ang_vel)
        )
        self._cmd_vel_pub.publish(cmd_vel)

    # Service handlers
    def _srv_dwa_status_callback(self, _, response):
        '''Handles request to check the state of the DWA server.'''
        self.get_logger().debug(f'enter dwa status callback: {self._planner_state.name}:{int(self._planner_state)}')
        response.planner_status = PlannerStatusMsg(data=int(self._planner_state))

        return response

    @staticmethod
    def _parse_get_rrt_response(response:GetRRTWaypoints.Response):
        '''Parses the response from the RRT server about the current list of waypoints.
        Converts the Point List into a list of x,y values for pythonic interpretation.'''
        manhattan_dist = response.remaining_dist.data
        waypoint_idx = response.waypoint_idx.data
        path = []
        for point in response.waypoints:
            path.append((point.x, point.y))
            
        return manhattan_dist, waypoint_idx, path

    def _get_curr_rrt_dist_callback(self, future):
        '''Response from the current RRT server about the waypoints.'''
        response = future.result()
        self._curr_manhattan_dist, self._curr_waypoint_idx , self._curr_path = \
            self._parse_get_rrt_response(response)
        self.get_logger().debug(f"Curr RRT srv callback: Manhattan Dist: {self._curr_manhattan_dist:.2f}, Curr waypt idx: {self._curr_waypoint_idx}\n{self._curr_path}")

    def _get_target_rrt_dist_callback(self, future):
        '''Response from the target RRT server about the waypoints.'''
        response = future.result()
        self._target_manhattan_dist, self._target_waypoint_idx , self._target_path = \
            self._parse_get_rrt_response(response)
        self.get_logger().debug(f"Target RRT srv callback: Manhattan Dist: {self._target_manhattan_dist:.2f}, Curr waypt idx: {self._target_waypoint_idx}\n{self._target_path}")

def main(args=None):
    rclpy.init(args=args)

    try:
        dwa_action_server = DWABaseNode('dwa_action_server')
        # MultiThreadedExecutor lets us listen to the current location while also doing 
        # Action Server stuff
        executor = MultiThreadedExecutor()
        executor.add_node(dwa_action_server)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            dwa_action_server.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
