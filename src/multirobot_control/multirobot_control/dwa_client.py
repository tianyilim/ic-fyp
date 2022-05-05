import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci
from planner_action_interfaces.action import LocalPlanner
from geometry_msgs.msg import Point

from tf_transformations import euler_from_quaternion

import numpy as np

class DWAActionClient(Node):

    def __init__(self):
        super().__init__('dwa_action_client')
        self._action_client = ActionClient(self, LocalPlanner, 'dwa')

    def send_goal(self, goal_position):
        goal_msg = LocalPlanner.Goal()
        goal_msg.goal_position = goal_position

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        # Add a callback for when the future (goal response) is complete
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        # Add a callback when the future (when action is complete) finishes
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Success: {} Final_Position X:{:.2f} Y:{:.2f}'.format(
            result.success.data, result.final_position.x, result.final_position.y))

        # Shutdown ROS2 for clean exit
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.current_pose
        curr_x = feedback.position.x
        curr_y = feedback.position.y
        curr_yaw = euler_from_quaternion([
            feedback.orientation.x,
            feedback.orientation.y,
            feedback.orientation.z,
            feedback.orientation.w
        ])[2]       # Only rotation about the z axis
        
        self.get_logger().info('Current X:{:.2f} Y:{:.2f} Yaw:{:.2f}'.format(
            curr_x, curr_y, np.degrees(curr_yaw) ))

def main(args=None):
    rclpy.init(args=args)

    action_client = DWAActionClient()

    goal_pos_x = float( input("Enter Goal X:") )
    goal_pos_y = float( input("Enter Goal Y:") )
    # goal_pos_x = -1.0
    # goal_pos_y = -1.0
    goal_pos = Point(x=goal_pos_x, y=goal_pos_y, z=0.0)

    action_client.send_goal(goal_pos)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()