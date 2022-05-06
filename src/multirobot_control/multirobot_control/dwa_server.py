'''
Create a DWA node.
'''

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult

from planner_action_interfaces.action import LocalPlanner
from planner_action_interfaces.msg import OtherRobotLocations

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion, Vector3

from tf_transformations import euler_from_quaternion, quaternion_from_euler

import time
import numpy as np

# TODO add a config file to adjust parameters such as closeToGoal Threshold

class DWAActionServer(Node):

    def __init__(self, map_path=""):

        super().__init__('dwa_action_server')
        self._action_server = ActionServer(
            self,
            LocalPlanner,
            'dwa',
            self.execute_callback)
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_radius', 0.3),
                ('safety_thresh', 0.5),
                ('simulate_duration', 1.0),
                ('action_duration', 0.1),
                ('linear_speed_limit', 0.5),
                ('angular_speed_limit', 0.5),
                ('linear_step', 0.1),
                ('angular_step', 0.1),
                ('dist_thresh', 0.1),
                ('dist_method', "L2"),
            ])

        self.add_on_set_parameters_callback(self.parameter_callback)

        # state variables
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # Inputs to cmd_vel
        self._linear_twist = 0.0
        self._angular_twist = 0.0

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
        }

        # Obstacles
        self.obstacles = [
            (0, -2, 0.5),
            (2, 1, 0.5),
            (-1, 2, 0.5),
        ]
        
        # other robots on the map
        self.other_robots = []

        # Walls
        self.ub_x = 4.5
        self.lb_x = -4.5
        self.ub_y = 4.5
        self.lb_y = -4.5

        # Subscribe to Odom (which has ground truth)
        self.state_sub = self.create_subscription(Odometry, 'odom', \
            self.handle_odom, qos_profile_sensor_data)

        # Sub to messages about the pose of other robots (and their predicted positions)
        self.other_robot_sub = self.create_subscription(OtherRobotLocations, 
            'otherRobotLocations', self.handle_other_robot_state, 10)

        # Publish to cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Publish predicted movement
        self.planned_pos_pub = self.create_publisher(Point, "planned_pos", 10)

    def handle_odom(self, msg):
        '''Handle incoming data on `odom` by updating private variables'''
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])[2]

        # self.get_logger().debug("Current X:{} Y:{} Yaw:{}".format(
        #     self._x, self._y, self._yaw ))

    def handle_other_robot_state(self, msg):
        '''Handle incoming data on `otherRobotLocations`, to where other robots are predicted to be'''
        
        # print(msg)
        self.other_robots = []
        for pose in msg.positions:
            print("[handle_other_robots] appending x:{:.2f} y:{:.2f}".format(pose.x, pose.y))
            self.other_robots.append(
                (pose.x, pose.y)
            )


    ''' Executes the action. '''
    def execute_callback(self, goal_handle):
        feedback_msg = LocalPlanner.Feedback()

        # Actual DWA algorithm here
        self.goal_x = goal_handle.request.goal_position.x
        self.goal_y = goal_handle.request.goal_position.y

        self.get_logger().info('Moving towards goal X:{:.2f} Y:{:.2f}'.format(self.goal_x, self.goal_y))

        while not self.closeToGoal(self._x, self._y, self.goal_x, self.goal_y):
            # Enumerate possible actions, checking for bounds
            possible_linear = [ self._linear_twist ]
            if (self._linear_twist + self.params['linear_step']) < self.params['linear_speed_limit']: 
                possible_linear.append(self._linear_twist + self.params['linear_step'])
            if (self._linear_twist - self.params['linear_step']) > -self.params['linear_speed_limit']: 
                possible_linear.append(self._linear_twist - self.params['linear_step'])

            possible_angular = [ self._angular_twist ]
            if (self._angular_twist + self.params['angular_step']) < self.params['angular_speed_limit']: 
                possible_angular.append(self._angular_twist + self.params['angular_step'])
            if (self._angular_twist - self.params['angular_step']) > -self.params['angular_speed_limit']: 
                possible_angular.append(self._angular_twist - self.params['angular_step'])

            top_score = -np.inf
            planned_pose = None

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

                    # Eventual position
                    end_pose = displacement + np.array([self._x, self._y])

                    # Rank score
                    score = self.rankPose(end_pose)
                    if score > top_score:
                        top_score = score
                        self._linear_twist = linear_twist
                        self._angular_twist = angular_twist
                        
                        planned_pose = end_pose

            # TODO what if there is no goal sent?
            self.planned_pos_pub.publish(Point(x=planned_pose[0], y=planned_pose[1], z=0.0))

            if top_score == 0:
                # TODO handle 'deadlock'
                self.get_logger().warn('No satisfactory velocities found')

            # Apply proposed velocity for a specified time
            cmd_vel = Twist(
                linear=Vector3(x=self._linear_twist, y=0.0, z=0.0), 
                angular=Vector3(x=0.0, y=0.0, z=self._angular_twist)
            )
            self.cmd_vel_pub.publish(cmd_vel)

            # Publish feedback
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
            
            self.get_logger().info('Current X:{:.2f} Y:{:.2f} Yaw:{:.2f} lin:{:.2f}, ang:{:.2f}'.format(
                self._x, self._y, np.degrees(self._yaw), self._linear_twist, self._angular_twist ))

            # Sleep for awhile before applying the next motion
            time.sleep(self.params['action_duration'])
            # TODO publish feedback at regular intervals

        # Ensure robot is stopped
        self._linear_twist = 0.0
        self._angular_twist = 0.0
        cmd_vel = Twist(
            linear=Vector3(x=self._linear_twist, y=0.0, z=0.0), 
            angular=Vector3(x=0.0, y=0.0, z=self._angular_twist)
        )
        self.cmd_vel_pub.publish(cmd_vel)

        # After DWA reaches goal
        goal_handle.succeed()
        
        result = LocalPlanner.Result()
        result.success = Bool(data=True)
        result.final_position.x = self._x
        result.final_position.y = self._y
        result.final_position.z = 0.0
        return result

    # Helper functions
    def rankPose(self, end_pose):
        score = 0

        goal_K = 10
        safety_thresh_K = 2
        non_thresh_K = 1
        
        if end_pose[0] > self.ub_x or \
            end_pose[0] < self.lb_x or \
            end_pose[1] > self.ub_y or \
            end_pose[1] < self.lb_y:
            
            return 0    # Out of bounds

        # Else, see how close it gets to the goal
        score += \
            goal_K / self.distToGoal(end_pose[0], end_pose[1], self.goal_x, self.goal_y)

        for x, y, r in self.obstacles:
            # Use distToGoal to get a simple distance
            dist_to_cylinder = self.distToGoal(end_pose[0], end_pose[1], x, y)
            
            # Collision!
            if dist_to_cylinder < (r+self.params['robot_radius']):
                score = 0
            elif dist_to_cylinder < (r+self.params['robot_radius']+self.params['safety_thresh']):
                score -= safety_thresh_K / dist_to_cylinder
            else:
                score -= non_thresh_K / dist_to_cylinder

        for x, y in self.other_robots:
            dist_to_robot = self.distToGoal(end_pose[0], end_pose[1], x, y)
            # TODO hack, set this as a parameter
            if dist_to_robot < (2.5*self.params['robot_radius']):
                score -= safety_thresh_K / dist_to_robot

        # score cannot be negative
        return max(score, 0)

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
            # TODO error handling/type check/bounds check
            self.params[param.name] = param.value
            self.get_logger().debug("Set parameter {} to value {}".format(
                param.name, self.params[param.name] ))

        # Write parameter result change
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)

    try:
        dwa_action_server = DWAActionServer()
        # MultiThreadedExecutor lets us listen to the current location while also doing 
        # Action Server stuff
        executor = MultiThreadedExecutor(num_threads=2)
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
