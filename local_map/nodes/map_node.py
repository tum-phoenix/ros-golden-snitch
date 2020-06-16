#!/usr/bin/env python3
import rospkg
import rospy
import numpy as np
import yaml
from local_map.map_logic import Mapper
from geometry_msgs.msg import PoseStamped, Point32
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header
from teraranger_array.msg import RangeArray


class Server:
    def __init__(self):

        rospack = rospkg.RosPack()
        with open(rospack.get_path("phx_launch")+"/../config/hardware_config.yaml") as f:
            mechanical_config = yaml.load(f, Loader=yaml.SafeLoader)
        dirOfRangeSensors = mechanical_config["direction_of_range_sensors"]
        self.mapper = Mapper(dirOfRangeSensors)
        self.pub = rospy.Publisher("phx/local_map", PointCloud, queue_size=1)


    def ranges_callback(self, msg : RangeArray):
        if self.pos is None:
            # We have still not received the first pose from the FC
            return
        ranges = np.array(list(map(lambda x: x.range, msg.ranges)))
        worldmap = self.mapper.update(ranges, self.pos, self.rot)

        points = list(map(lambda x: Point32(x[0], x[1], x[2]), worldmap))
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = "map" # Needs to correspond to the settings in rviz.
        msg = PointCloud(header, points, [])
        self.pub.publish(msg)

    def _pose_callback(self, msg: PoseStamped):
        self.pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,)
        self.rot = (msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,)


if __name__ == '__main__':
    rospy.init_node('local_mapper')

    server = Server()

    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, server._pose_callback)
    rospy.Subscriber('/multiflex_1/ranges_raw', RangeArray, server.ranges_callback)
    rospy.loginfo("Local mapper is initialized")

    rospy.spin()
