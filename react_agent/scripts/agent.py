#!/usr/bin/env python



# this was copied from https://stackoverflow.com/questions/48830056/use-data-from-multiple-topics-in-ros-python
import rospy
from std_msgs.msg import Float64MultiArray
from teraranger_array.msg import RangeArray
from human_info.msg import HumanPos
import decision_tree


class Server:
    def __init__(self):
        self.human_dir = None
        self.human_dist = None
        self.distances = []

        self.ai = decision_tree.Decision_tree()

        #TODO: Make publisher here

    def orientation_callback(self, msg):
        # "Store" message received.
        self.orientation = msg

        # Compute stuff.
        self.compute_stuff()

    def ranges_callback(self, msg):
        # "Store" the message received.
        # TODO: Check what kind of format the data becomes in python.
        self.distances = msg
        print("Terraranger array messages:", msg)

        # Compute stuff.
        self.compute_stuff()

    def compute_stuff(self):
        if self.orientation is not None and self.velocity is not None:
            res = self.ai.update()



if __name__ == '__main__':
    rospy.init_node('listener')

    server = Server()

    rospy.Subscriber('/human_info/human_info', Float64MultiArray, server.orientation_callback)
    rospy.Subscriber('/multiflex_1/ranges_raw', RangeArray, server.ranges_callback)

    rospy.spin()
