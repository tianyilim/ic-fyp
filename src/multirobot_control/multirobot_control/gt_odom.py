'''
Delivers the ground truth transformation between `prefix`base_footprint <-> `prefix`odom and map.
'''

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import argparse

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, TransformStamped, PoseWithCovariance
from std_msgs.msg import String

from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class GazeboOdomGroundTruth(Node):

    def __init__(self, link_name, namespace=''):
        super().__init__('gazebo_odom_ground_truth')

        self.world_frame = 'map'
        self.link_name = link_name
        self.namespace = namespace
        self.odom_frame = self.namespace + 'odom'
        if not self.link_name:
            raise ValueError("'link_name' is an empty string")
        else:
            self.get_logger().info('link_name:{}'.format(self.link_name))

        self.map_odom_transform = self.make_map_odom_transform()
        # define odom and map to be the same frame
        self.map_odom_br = TransformBroadcaster(self)

        self.br = TransformBroadcaster(self)

        # remember to set the QOS to 'best effort'
        self.states_sub = self.create_subscription(Odometry, '/'+self.namespace+'/odom', \
            self.handle_odom, qos_profile_sensor_data)

    '''
    Callback called when a message is recieved on the incoming odometry topic
    '''
    def handle_odom(self, msg):
        t = TransformStamped()

        frame_pos = msg.pose.pose.position
        frame_ori = msg.pose.pose.orientation
        frame_child_id = msg.child_frame_id

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        # t.child_frame_id = self.link_name
        t.child_frame_id = frame_child_id

        t.transform.translation.x = frame_pos.x
        t.transform.translation.y = frame_pos.y
        t.transform.translation.z = frame_pos.z


        t.transform.rotation.x = frame_ori.x
        t.transform.rotation.y = frame_ori.y
        t.transform.rotation.z = frame_ori.z
        t.transform.rotation.w = frame_ori.w

        # self.get_logger().info("got x:{} y:{} z:{} yaw:{}".format(
        #     frame_pos.x, frame_pos.y, frame_pos.z,
        #     euler_from_quaternion(
        #         [frame_ori.x, frame_ori.y, frame_ori.z, frame_ori.w]
        #     )
        # ))
        
        # Send the transformations
        self.br.sendTransform(t)

        # * don't do this here - do it in its own node instead
        # self.map_odom_br.sendTransform(self.map_odom_transform)

    def make_map_odom_transform(self):
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = self.world_frame
        st.child_frame_id = self.odom_frame
        st.transform.translation.x = 0.0
        st.transform.translation.y = 0.0
        st.transform.translation.z = 0.0
        st.transform.rotation.x = 0.0
        st.transform.rotation.y = 0.0
        st.transform.rotation.z = 0.0
        st.transform.rotation.w = 1.0

        return st

def main(args=None):
    parser = argparse.ArgumentParser(description="Get ground truth of a link in Gazebo")
    parser.add_argument('-l', '--link_name', type=str, \
        default='base_footprint',   \
        help="Name of link")
    parser.add_argument('-n', '--namespace', type=str, \
        default='',   \
        help="Namespace")

    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = GazeboOdomGroundTruth(args.link_name, args.namespace)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
