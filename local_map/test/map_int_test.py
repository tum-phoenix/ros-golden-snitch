#!/usr/bin/env python3
print("Starting map integration test")
import time
import unittest
import sys
import numpy as np
import rospkg
import rospy
import yaml

from local_map.map_logic import Mapper
from sensor_msgs.msg import PointCloud, Range
from std_msgs.msg import Header
from teraranger_array.msg import RangeArray
from geometry_msgs.msg import PoseStamped

PKG = 'local_map'

POSE_TOPIC = "/mavros/local_position/pose"
RANGES_TOPIC = '/multiflex_1/ranges_raw'
MAP_TOPIC = "phx/local_map"


class Rx:

    def __init__(self):
        self.receivedMsg = False

    def receiveMap(self, msg: PointCloud):
        self.receivedMsg = True
        print("Received message")
        self.map = msg


class TestLocalMapper(unittest.TestCase):

    def setUp(self) -> None:
        rospy.init_node("test_fsm", anonymous=True)
        self.rx = Rx()
        rospy.Subscriber(MAP_TOPIC, PointCloud, self.rx.receiveMap)
        self.range_pub = rospy.Publisher(RANGES_TOPIC, RangeArray, queue_size=1)
        self.pose_pub = rospy.Publisher(POSE_TOPIC, PoseStamped, queue_size=2)

        rospack = rospkg.RosPack()
        with open(rospack.get_path("phx_launch") + "/../config/hardware_config.yaml") as f:
            mechanical_config = yaml.load(f, Loader=yaml.SafeLoader)
        self.NUM_RANGE_SENSORS = mechanical_config["number_of_range_sensors"]
        rospy.sleep(0.5)

    def test_map_received(self):
        poseMsg = PoseStamped()
        poseMsg.header = Header()
        poseMsg.pose.position.x = 0
        poseMsg.pose.position.y = 0
        poseMsg.pose.position.z = 0
        poseMsg.pose.orientation.x = 0
        poseMsg.pose.orientation.y = 0
        poseMsg.pose.orientation.z = 0
        poseMsg.pose.orientation.w = 1
        self.pose_pub.publish(poseMsg)

        rangeMsg = RangeArray()
        rangeMsg.header = Header()
        ranges = [0, 0.2, -1, 1, 1.5, 8, 3, 1, 8, 4, 2.3, 0.2, 0.1, 2, 3, 1, 9]
        for rangerange in ranges:
            range = Range()
            range.header = Header()
            range.range = rangerange
            range.field_of_view = 1
            range.min_range = 0.1
            range.max_range = 2.1
            rangeMsg.ranges.append(range)
        rangeMsg.ranges = rangeMsg.ranges[:self.NUM_RANGE_SENSORS]

        self.range_pub.publish(rangeMsg)
        rospy.sleep(0.2)
        self.assertEqual(self.rx.receivedMsg, True)
        self.assertGreater(len(self.rx.map.points), 0)


if __name__ == '__main__':
    import rosunit
    import rostest

    # rosunit.unitrun(PKG, 'test_fsm_code_level', TestFsm)
    rostest.rosrun(PKG, 'map_int_test', TestLocalMapper, sys.argv)
