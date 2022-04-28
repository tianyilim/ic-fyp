'''
Delivers the ground truth transformation between map and odom.
'''

import rclpy
from rclpy.node import Node

import argparse

from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from tf_transformations import euler_from_quaternion

class GazeboOdomGroundTruth(Node):

    def __init__(self, link_name):
        super().__init__('gazebo_odom_ground_truth')

        self.link_name = link_name
        self.link_name_rectified = link_name.replace("::", "_")

        if not self.link_name:
            raise ValueError("'link_name' is an empty string")
        else:
            self.get_logger().info('link_name:{}'.format(self.link_name_rectified))

        self.states_sub = self.create_subscription(LinkStates, '/gazebo/link_states', self.callback, 10)
        self.publisher = self.create_publisher(Pose, '~/' + self.link_name_rectified, 10)
        # self.publisher = self.create_publisher(Pose, 'groundtruth_pose', 10)

    '''
    Callback called when a message is recieved on the '/gazebo/link_states' topic
    '''
    def callback(self, data):
        try:
            ind = data.name.index(self.link_name)
            self.link_pose = data.pose[ind]
            
            self.__yaw = euler_from_quaternion(
                self.link_pose.orientation[0],
                self.link_pose.orientation[1],
                self.link_pose.orientation[2],
                self.link_pose.orientation[3]
            )[2]    # yaw is about the z axis

            self.publisher.publish(self.link_pose)
            self.get_logger().info('Publishing x:{}, y:{}, z:{} yaw:{}'.format(
                self.link_pose.position.x,
                self.link_pose.position.y,
                self.link_pose.position.z,
                self.__yaw
            ))

        except ValueError:
            pass

def main(args=None):
    parser = argparse.ArgumentParser(description="Get ground truth of a link in Gazebo")
    parser.add_argument('-l', '--link_name', type=str, \
        default='base_footprint',   \
        help="Name of link")

    args, unknown = parser.parse_known_args()

    # start node
    # rclpy.init(args=args)
    rclpy.init()

    gazebo_odom_ground_truth = GazeboOdomGroundTruth(args.link_name)

    rclpy.spin(gazebo_odom_ground_truth)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gazebo_odom_ground_truth.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
