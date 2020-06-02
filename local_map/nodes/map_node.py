#!/usr/bin/env python3
import rospy
import numpy as np
from local_map.map_logic import Mapper
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud
from teraranger_array.msg import RangeArray


class Server:
    def __init__(self):
        self.pose = PoseStamped()
        self.mapper = Mapper()
        self.pub = rospy.Publisher("phx/local_map", PointCloud, queue_size=1)


    def ranges_callback(self, msg):
        ranges = np.array(list(map(lambda x: x.range, msg.ranges)))
        map = self.mapper.update(ranges, self.pose)

        # TODO Generate the correct point cloud message
        msg = PointCloud(map)
        self.pub.publish(msg)

    def _pose_callback(self, msg: PoseStamped):
        # TODO: Convert the pose to the desired format
        self.pose = msg


if __name__ == '__main__':
    rospy.init_node('local_mapper')

    server = Server()

    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, server._pose_callback) # TODO: Find out which topic is relevant
    rospy.Subscriber('/multiflex_1/ranges_raw', RangeArray, server.ranges_callback)
    rospy.loginfo("Local mapper is initialized")

    rospy.spin()
