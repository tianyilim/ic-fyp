'''
Visualises the map in `map_params` in RViz by publishing Marker objects.
'''

import rclpy
from rclpy.time import Time
from rclpy.node import Node

from multirobot_control.map_params import OBSTACLE_ARRAY
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

from time import sleep
import numpy as np

class MapVisualisation(Node):

    def __init__(self):
        super().__init__('map_visualisation')

        self.get_logger().info("Starting up Map Visualisation Node.")
        self.map_marker_pub = self.create_publisher(Marker, 'map_markers', len(OBSTACLE_ARRAY))
        self.publish_markers()

    def publish_markers(self):
        for idx, (x0, y0, x1, y1) in enumerate(OBSTACLE_ARRAY):
            marker = Marker()
            # header stamp
            marker.header.frame_id = "/map"             # Set relative to global frame
            marker.header.stamp = self.get_clock().now().to_msg()

            # namespace and id
            marker.ns = "collision_map"
            marker.id = idx

            # Type of marker
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Position of marker
            marker.pose.position.x = (x0 + x1)/2
            marker.pose.position.y = (y0 + y1)/2
            marker.pose.position.z = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = x1-x0
            marker.scale.y = y1-y0
            marker.scale.z = 2.0;

            # Set the color -- be sure to set alpha to something non-zero!
            marker.color.r = idx/len(OBSTACLE_ARRAY) * 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            # Duration of marker, set to be infinite
            marker.lifetime = Duration()

            # Publish the marker
            while self.map_marker_pub.get_subscription_count() < 1:
                self.get_logger().warn("Please create a subscriber to the marker.", once=True)
                sleep(1)

            self.map_marker_pub.publish(marker)
            self.get_logger().info(f"Published object idx {idx} at {(x0 + x1)/2:.2f}, {(y0 + y1)/2:.2f} with scale {x1-x0:.2f}, {y1-y0:.2f}")

            # sleep(0.01) # if not we flood RViz and it drops stuff...

        self.get_logger().info("Done! Destroying node.")
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    map_vis = MapVisualisation()
    # rclpy.spin_once(map_vis)
    rclpy.spin(map_vis)

    map_vis.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()