'''
This node loops through all robots in the simulation and gets a distance between each of them.

If robots are mutually close by, then we send velocity information to both robots.
'''

import enum
from re import M
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


from planner_action_interfaces.msg import OtherRobotLocations
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import numpy as np

class OdomDistribution(Node):

    def __init__(self):
        super().__init__('odom_distribution')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_list', Parameter.Type.STRING_ARRAY),   # list of robot names to subscribe to
                ('pub_freq', 10.0),      # Frequency to publish at
                ('dist_thresh', 2.0),   # Distance range robots can 'communicate'
            ]
        )

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.robot_locs = {}            # Collection of robot poses
        self.odom_subscriptions = []    # Collection of subscription objects
        self.odom_publications = []     # Collection of publisher objects
        
        assert self.get_parameter("robot_list").value is not None
        assert len(self.get_parameter("robot_list").value) != 0

        for index, robot in enumerate(self.get_parameter("robot_list").value):
            # Dummy position for now
            dummyPoint = (Point(x=0.0, y=0.0, z=0.0), robot)
            self.robot_locs[robot] = dummyPoint
            self.robot_locs[index] = dummyPoint
            # We use a dict because otherwise subscribing to many robots' locations will become tough
            # but we index with both names (for topics) and indices

            self.odom_subscriptions.append(
                self.create_subscription(Odometry, robot+'/odom', \
                self.handle_odom, qos_profile_sensor_data)
            )
            self.odom_publications.append(
                self.create_publisher(OtherRobotLocations, robot+'/otherRobotLocations', 10)
            )

        timer_period = 1 / self.get_parameter("pub_freq").value
        self.timer = self.create_timer(timer_period, self.timer_callback)



    def timer_callback(self):
        # Calculate the NN for each pair of robots
        # This is represented as a symmetric matrix to prevent extra calculation.
        num_robots = len(self.get_parameter("robot_list").value)

        distances = np.zeros(( num_robots, num_robots ))

        for curr_idx in range( num_robots ):
            curr_x = self.robot_locs[curr_idx][0].x
            curr_y = self.robot_locs[curr_idx][0].y

            for other_idx in range( num_robots ):
                if curr_idx == other_idx \
                or distances[curr_idx, other_idx] != 0:
                    # Don't need to calculate distance between the same element
                    # Or the distance between stuff that has already been calculated
                    continue
                
                other_x = self.robot_locs[other_idx][0].x
                other_y = self.robot_locs[other_idx][0].y

                # print(curr_x, curr_y, other_x, other_y)

                # Symmetric matrix. For some reason `hypot` needs to be indexed
                
                distances[curr_idx, other_idx] = np.hypot((curr_x-other_x), (curr_y-other_y))
                distances[other_idx, curr_idx] = distances[curr_idx, other_idx]

                # print("Dist between robot {} (x:{:.2f} y:{:.2f}) and {} (x:{:.2f} y:{:.2f}): {:.2f}".format(
                #     curr_idx+1, curr_x, curr_y, 
                #     other_idx+1, other_x, other_y,
                #     distances[curr_idx, other_idx]
                # ))

        # Filter for distances not on the diagonal
        np.fill_diagonal(distances, 1e9)    # large to avoid filter below
        i_index, j_index = np.nonzero( distances <= self.get_parameter("dist_thresh").value )
        # We know the matrix is symmetrical so this cuts down our search space
        i_index = np.split(i_index, 2)[0]
        j_index = np.split(j_index, 2)[0]

        # Iterate through
        for robot_idx in range(num_robots):
            # Publish relevant info for each robot
            message = OtherRobotLocations()
            robot_names = []
            robot_poses = []

            # We only want the robots where i_index!=j_index (handled earlier where we removed diagonals)
            # We compare if i or j is a match, and use it to get the other pair (as we have removed duplicates)
            for idx in range(len(i_index)):
                i = i_index[idx]
                j = j_index[idx]
                if  i==robot_idx:
                    robot_names.append(String(data=self.robot_locs[j][1]))
                    robot_poses.append(self.robot_locs[j][0])
                elif j==robot_idx:
                    robot_names.append(String(data=self.robot_locs[i][1]))
                    robot_poses.append(self.robot_locs[i][0])

            message.name = robot_names
            message.position = robot_poses

            self.get_logger().info("Robot idx {} ({}) publishing names\n{}\nand poses\n{}".format(
                robot_idx, self.robot_locs[robot_idx][1],
                robot_names, robot_poses
            ))

            self.odom_publications[robot_idx].publish(message)


    def handle_odom(self, msg):
        # update the location of each robot as a class variable
        # ? Hack - no way to recieve the topic the message was published on, so have to 
        # ? assume that the child_frame_id is the same string.
        # ? Currently it's always set as (eg.) robot1_{LINK}, so it's all good for now...
        robot_id = msg.child_frame_id.split('_')[0]

        self.robot_locs[robot_id][0].x = msg.pose.pose.position.x
        self.robot_locs[robot_id][0].y = msg.pose.pose.position.y
        self.robot_locs[robot_id][0].z = msg.pose.pose.position.z

        self.get_logger().debug("Updated robot name {} with pose x:{:.2f} y:{:.2f} z:{:.2f}".format(
            robot_id, self.robot_locs[robot_id][0].x, 
            self.robot_locs[robot_id][0].y, self.robot_locs[robot_id][0].z
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
            elif param.name == 'dist_thresh':
                if param.type_==Parameter.Type.DOUBLE or param.type_==Parameter.Type.INTEGER:
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