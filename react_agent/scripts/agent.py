#!/usr/bin/env python



# this was copied from https://stackoverflow.com/questions/48830056/use-data-from-multiple-topics-in-ros-python
import rospy
from std_msgs.msg import Float64MultiArray
from teraranger_array.msg import RangeArray
from human_info.msg import HumanPos
from decision_tree import Decision_tree
from geometry_msgs.msg import PoseStamped


class Server:
    def __init__(self):
        self.human_dir = None
        self.human_dist = None
        self.distances = []

        self.ai = Decision_tree()

        self.pub = rospy.Publisher('/setpoint_position/local', PoseStamped, queue_size=1) # Should be the one mavros wants, but we don't know if Ardupilot will accept it.

    def orientation_callback(self, msg):
        # "Store" message received.
        self.orientation = msg

        # Compute stuff.
        self.compute_stuff()

    def ranges_callback(self, msg):
        # "Store" the message received.
        # TODO: Check what kind of format the data becomes in python.
        ranges = map(lambda x: x["range"], msg.ranges)
        self.distances = msg
        print("Terraranger array messages:", ranges)

        # Compute stuff.
        self.compute_stuff()

    def compute_stuff(self):
        if self.human_dir is not None and len(self.distances) !=0:
            res = self.ai.update()
            # TODO: Convert res to the correct fromat for the ROS message
            self.pub.publish(res)



if __name__ == '__main__':
    rospy.init_node('decicion_maker')

    server = Server()

    rospy.Subscriber('/human_info/human_info', Float64MultiArray, server.orientation_callback)
    rospy.Subscriber('/multiflex_1/ranges_raw', RangeArray, server.ranges_callback)

    rospy.spin()
