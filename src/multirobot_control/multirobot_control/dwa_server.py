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

from std_msgs.msg import Bool, String, Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, PointStamped, Quaternion, Vector3
from visualization_msgs.msg import MarkerArray, Marker

from tf_transformations import euler_from_quaternion, quaternion_from_euler

from multirobot_control.map_params import OBSTACLE_ARRAY
from multirobot_control.math_utils import dist_to_aabb

import time
import numpy as np
from enum import Enum, auto
from typing import List, Tuple

class DWAServerStatus(Enum):
    STATUS_BUSY = auto()
    STATUS_FREE = auto()


class DWAActionServer(Node):

    def __init__(self):

        super().__init__('dwa_action_server')
        self._action_server = ActionServer(
            self,
            LocalPlanner,
            'dwa',
            self.execute_callback)
        
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
                ('angular_thresh', 70.0),
                ('angular_K', 2.0),
                ('goal_K', 10.0)
            ])

        self.add_on_set_parameters_callback(self.parameter_callback)

        # state variables
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # Inputs to cmd_vel
        self._linear_twist = 0.0
        self._angular_twist = 0.0

        self._planned_pose = None
        self._planner_state = DWAServerStatus.STATUS_FREE

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
            "angular_thresh" :      self.get_parameter("angular_thresh").value,
            "angular_K" :           self.get_parameter("angular_K").value,
            "goal_K" :              self.get_parameter("goal_K").value,
        }
        
        # other robots on the map
        self.other_robots = []

        # Subscribe to Odom (which has ground truth)
        self.state_sub = self.create_subscription(Odometry, 'odom', \
            self.handle_odom, qos_profile_sensor_data)

        # Sub to messages about the pose of other robots (and their predicted positions)
        self.other_robot_sub = self.create_subscription(OtherRobotLocations, 
            'otherRobotLocations', self.handle_other_robot_state, 10)

        # Publish to cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Publish predicted movement
        self.planned_pos_pub = self.create_publisher(PointStamped, "planned_pos", 10)
        self.vis_planned_pos_pub = self.create_publisher(MarkerArray, "planned_pos_markers", 10)

        # Timer callback
        timer_period = 1 / self.params['pub_freq']
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def handle_odom(self, msg):
        '''Handle incoming data on `odom` by updating private variables'''
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
        
        # print(msg)
        self.other_robots = []
        for pose in msg.positions:
            self.get_logger().debug("[handle_other_robots] appending x:{:.2f} y:{:.2f}".format(pose.x, pose.y))
            self.other_robots.append(
                (pose.x, pose.y)
            )
    
    def timer_callback(self):
        ''' Sends out planned position if DWA server is BUSY (a action is currently executing).
            Else, sends the current position.
        '''
        if self._planner_state == DWAServerStatus.STATUS_BUSY:
            # If an action is currently being executed=
            self.planned_pos_pub.publish(PointStamped(
                header=Header(frame_id=self.get_namespace()),
                point=Point(
                    x=self._planned_pose[0], y=self._planned_pose[1], z=0.0
                )))
        else:
            # No action executed, send current x and y values
            self.planned_pos_pub.publish(PointStamped(
                header=Header(frame_id=self.get_namespace()),
                point=Point(
                    x=self._x, y=self._y, z=0.0
                )))

    def execute_callback(self, goal_handle):
        ''' Executes the DWA action. '''

        self._planner_state = DWAServerStatus.STATUS_BUSY

        feedback_msg = LocalPlanner.Feedback()

        # Actual DWA algorithm here
        self.goal_x = goal_handle.request.goal_position.x
        self.goal_y = goal_handle.request.goal_position.y

        self.get_logger().info('Moving towards goal X:{:.2f} Y:{:.2f}'.format(self.goal_x, self.goal_y))

        while not self.closeToGoal(self._x, self._y, self.goal_x, self.goal_y):
            # Enumerate possible actions, checking for bounds
            possible_linear = [ self._linear_twist ]
            if (self._linear_twist + self.params['linear_step']) <= self.params['linear_speed_limit']: 
                possible_linear.append(self._linear_twist + self.params['linear_step'])
            # ? Temporary
            if (self._linear_twist - self.params['linear_step']) >= -self.params['linear_speed_limit']: 
            # if (self._linear_twist - self.params['linear_step']) >= 0: 
                possible_linear.append(self._linear_twist - self.params['linear_step'])

            possible_angular = [ self._angular_twist ]
            if (self._angular_twist + self.params['angular_step']) <= self.params['angular_speed_limit']: 
                possible_angular.append(self._angular_twist + self.params['angular_step'])
            if (self._angular_twist - self.params['angular_step']) >= -self.params['angular_speed_limit']: 
                possible_angular.append(self._angular_twist - self.params['angular_step'])

            top_score = -np.inf
            top_pose = None

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
                    end_pose = [end_pose[0], end_pose[1], effective_yaw]

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
            
            self.get_logger().debug(f"---- Chose Lin: {self._linear_twist:.2f} Ang {self._angular_twist:.2f} Idx {top_idx}/{len(vis_msg_array)} ----")

            # Label top score with color. Invalid -> red, Not top score -> yellow, Top score -> Green
            vis_msg_array[top_idx].color.r = 0.0
            vis_msg_array[top_idx].color.g = 1.0

            # Visualise trajectories in RViz
            self.vis_planned_pos_pub.publish( MarkerArray(markers=vis_msg_array) )

            # only write this here because we don't want the value of the class var to be corrupted
            self._planned_pose = top_pose

            if top_score <= -np.inf:
                # Currently deadlock is handled by stopping the robot.
                self.get_logger().warn(f"No satisfactory new trajectories found. Stopping robot.")
                self._linear_twist = 0.0
                self._angular_twist = 0.0

            # Apply proposed velocity for a specified time
            cmd_vel = Twist(
                linear=Vector3(x=self._linear_twist, y=0.0, z=0.0), 
                angular=Vector3(x=0.0, y=0.0, z=self._angular_twist)
            )
            self.cmd_vel_pub.publish(cmd_vel)

            # Publish feedback
            feedback_msg.robot_name = String(data=self.get_namespace())
            feedback_msg.current_pose.position.x = self._x
            feedback_msg.current_pose.position.y = self._y
            feedback_msg.current_pose.position.z = 0.0
            
            curr_quaternion = quaternion_from_euler(
                0, 0, self._yaw
            )
            feedback_msg.current_pose.orientation.x = curr_quaternion[0]
            feedback_msg.current_pose.orientation.y = curr_quaternion[1]
            feedback_msg.current_pose.orientation.z = curr_quaternion[2]
            feedback_msg.current_pose.orientation.w = curr_quaternion[3]
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().debug('{} Curr X:{:.2f} Y:{:.2f} Yaw:{:.2f} Lin:{:.2f}, Ang:{:.2f}'.format(
                self.get_namespace(),
                self._x, self._y, np.degrees(self._yaw), 
                self._linear_twist, self._angular_twist ))

            # Sleep for awhile before applying the next motion
            start_time = self.get_clock().now()
            while True:
                curr_time = self.get_clock().now()
                s_diff = (curr_time.nanoseconds - start_time.nanoseconds) * 1e9
                
                if s_diff > self.params['action_duration']:
                    break

            # ? Extensions: Check for collision / Timeout and send 'goal failed'?
            
        # Ensure robot is stopped
        self._linear_twist = 0.0
        self._angular_twist = 0.0
        cmd_vel = Twist(
            linear=Vector3(x=self._linear_twist, y=0.0, z=0.0), 
            angular=Vector3(x=0.0, y=0.0, z=self._angular_twist)
        )
        self.cmd_vel_pub.publish(cmd_vel)

        # Set planner status back to free so that corret planned messages are recieved
        self._planner_state = DWAServerStatus.STATUS_FREE

        # After DWA reaches goal
        goal_handle.succeed()
        self.get_logger().info("Robot {} reached goal at X: {:.2f} Y: {:.2f}".format(
            self.get_namespace(), self.goal_x, self.goal_y
        ))

        result = LocalPlanner.Result()
        result.success = Bool(data=True)
        result.final_position.x = self._x
        result.final_position.y = self._y
        result.final_position.z = 0.0
        result.robot_name = String(data=self.get_namespace())
        return result

    # Helper functions
    def rankPose(self, end_pose, linear_twist, angular_twist):
        '''
        The core of the DWA algorithm. Scores final positions based on different factors.

        Add bonus to moving forward. Add cost for staying still.
        '''
        score = 0 if linear_twist!=0 and angular_twist!=0 else -5

        goal_K = self.params['goal_K']                              # P factor for proximity to goal
        orientation_thresh = self.params['angular_thresh']          # cutoff in deg for distance to goal
        orientation_K = orientation_thresh*self.params['angular_K'] # P factor in angle heading.

        safety_thresh_K = 2
        non_thresh_K = 1
        
        # Check for collisions or close shaves
        # OBSTACLE_ARRAY also handles the walls
        for obstacle in OBSTACLE_ARRAY:
            dist_to_obstacle = dist_to_aabb(end_pose[0], end_pose[1], obstacle)
            
            # print(f"Dist to obstacle for end_pose x:{end_pose[0]:.2f} y:{end_pose[1]:.2f}" + \
            #     f" is {dist_to_obstacle:.2f} for obstacle at x:{obstacle[0]:.2f} y:{obstacle[1]:.2f}")

            if dist_to_obstacle < self.params['robot_radius']:
                self.get_logger().debug(f"End pose {end_pose[0]:.2f}|{end_pose[1]:.2f}|{np.degrees(end_pose[2]):.2f} collision @ {(obstacle[0]+obstacle[2])/2:.2f}|{(obstacle[1]+obstacle[3])/2:.2f} dist {dist_to_obstacle:.2f}")
                score = -np.inf # Collision, don't process further
                return score
            elif dist_to_obstacle < (self.params['robot_radius'] + self.params['safety_thresh']):
                score -= safety_thresh_K / dist_to_obstacle # Too close for comfort

        for x, y in self.other_robots:
            dist_to_robot = self.distToGoal(end_pose[0], end_pose[1], x, y)
            if dist_to_robot < self.params['robot_radius']:
                self.get_logger().debug(f"End pose {end_pose[0]:.2f}|{end_pose[1]:.2f}|{np.degrees(end_pose[2]):.2f} collision with robot at {x:.2f}|{y:.2f} dist {dist_to_robot:.2f}")
                score = -np.inf # Collision, don't process further
                return score
            elif dist_to_robot < (self.params['inter_robot_dist']*self.params['robot_radius']):
                score -= safety_thresh_K / dist_to_robot

        # Else, see how close it gets to the goal.
        # Cap the score to the maximum cutoff value.
        dist_to_goal = self.distToGoal(end_pose[0], end_pose[1], self.goal_x, self.goal_y)
        dist_plus = goal_K / dist_to_goal
        score += dist_plus

        # Calculate heading difference to goal. Set the difference in degrees for more accurate scaling
        # This is proportional to the distance score. Otherwise it will 'overwhelm' the pathing intuition of the robot
        goal_hdg = np.arctan2(self.goal_y-end_pose[1], self.goal_x-end_pose[0])
        # We don't want to fall into a local minima so we cap the error the robot is facing
        hdg_diff = min( orientation_thresh, abs( np.degrees(goal_hdg - end_pose[2])) )
        # self.get_logger().debug(f"Ref hdg {np.degrees(goal_hdg):.2f}, curr hdg {np.degrees(end_pose[2]):.2f}")
        hdg_plus = min( orientation_K*dist_plus, 
                        orientation_K*dist_plus / hdg_diff )
        score += hdg_plus

        # Score breakdown
        self.get_logger().debug(
            f"end_x: {end_pose[0]:.2f} end_y: {end_pose[1]:.2f} end_yaw: {np.degrees(end_pose[2]):.2f}" + \
            f" | dist+: {dist_plus:.2f} | hdg+: {hdg_plus:.2f} | hdg_diff: {hdg_diff:.2f}" + \
            f" | score: {score:.2f}"
        )

        return score

    def distToGoal(self, curr_x, curr_y, goal_x, goal_y):
        '''
        Returns distance to goal based the `method`.
        `'L2'`: Calculates the L2 norm
        '''

        method = self.params['dist_method']

        if method=='L2':
            dist = np.hypot( curr_x-goal_x, curr_y-goal_y )
            return dist
        else:
            raise NotImplementedError("distance function {} not implemented yet".format(method))

    def closeToGoal(self, curr_x, curr_y, goal_x, goal_y):
        '''Returns True if we are `thresh` away from the goal'''
        
        if self.distToGoal(curr_x, curr_y, goal_x, goal_y) < self.params['dist_thresh']:
            return True
        else:
            return False

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

    def marker_from_traj(self, idx: int, score: float, end_pose: Tuple[float, float, float]) -> Marker :
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
        marker.pose.position.x = self._x
        marker.pose.position.y = self._y
        marker.pose.position.z = 0.25
        # Orientation is diff between current pose and future pose
        yaw = np.arctan2((end_pose[1]-self._y), (end_pose[0]-self._x))
        quat = quaternion_from_euler(0, 0, yaw)
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        # Set the scale of the marker
        # x: Arrow length
        marker.scale.x = np.hypot((end_pose[1]-self._y), (end_pose[0]-self._x)) 
        # y: Arrow width
        marker.scale.y = 0.05
        # z: Head length
        marker.scale.z = np.hypot((end_pose[1]-self._y), (end_pose[0]-self._x)) * 0.2

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
        period = self.params['action_duration']*1.1
        period_sec = int( np.round(period) )
        period_nanosec = int( (period-period_sec)*1e9 )
        marker.lifetime = Duration(seconds=period_sec, nanoseconds=period_nanosec).to_msg()

        return marker

def main(args=None):
    rclpy.init(args=args)

    try:
        dwa_action_server = DWAActionServer()
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


    # rclpy.spin(dwa_action_server)


if __name__ == '__main__':
    main()
