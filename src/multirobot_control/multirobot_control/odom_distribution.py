'''
This node loops through all robots in the simulation and gets a distance between each of them.

If robots are mutually close by, then we send velocity information to both robots.
'''

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


from planner_action_interfaces.msg import OtherRobotLocations
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

import numpy as np

class OdomDistribution(Node):

    def __init__(self):
        super().__init__('odom_distribution')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_list', None)    # list of robot names to subscribe to
                ('pub_freq', 100)       # Frequency to publish at
            ]
        )

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.robot_locs = {}            # Collection of robot poses
        self.odom_subscriptions = []    # Collection of subscription objects
        self.odom_publications = []     # Collection of publisher objects
        
        for robot in self.get_parameter("robot_list").value:
            self.robot_locs[robot] = Point(x=0.0, y=0.0, z=0.0) # Dummy position for now

            self.odom_subscriptions.append(
                self.create_subscription(Odometry, robot+'/odom', \
                self.handle_odom, qos_profile_sensor_data)
            )
            self.odom_publications.append(
                self.create_publisher(OtherRobotLocations, robot+'/otherRobotLocations', 10)
            )

        timer_period = self.get_parameter("pub_freq").value
        self.timer = self.create_timer(timer_period, self.timer_callback)



    def timer_callback(self):
        # TODO
        # Calculate the NN for each pair of robots

        # Publish relevant info for each robot
        pass


    def handle_odom(self, msg):
        # update the location of each robot as a class variable
        # ? Hack - no way to recieve the topic the message was published on, so have to 
        # ? assume that the child_frame_id is the same string.
        self.robot_locs[msg.child_frame_id] = msg.pose.pose
        self.get_logger().info("Updated robot name {} with pose x:{:.2f} y:{:.2f} z:{:.2f}".format(
            msg.child_frame_id, msg.pose.pose.x, msg.pose.pose.y, msg.pose.pose.z
        ))

    def parameter_callback(self, params):
        # type/bounds check on pub_freq (don't modify robot_list)
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
            return SetParametersResult(successful=False)

def main(args=None):
    rclpy.init(args=args)

    odom_distribution = OdomDistribution()

    rclpy.spin(odom_distribution)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_distribution.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()