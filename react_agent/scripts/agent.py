#!/usr/bin/env python



# this was copied from https://stackoverflow.com/questions/48830056/use-data-from-multiple-topics-in-ros-python
import rospy
from std_msgs.msg import Float64MultiArray


class Server:
    def __init__(self):
        self.orientation = None
        self.velocity = None

    def orientation_callback(self, msg):
        # "Store" message received.
        self.orientation = msg

        # Compute stuff.
        self.compute_stuff()

    def velocity_callback(self, msg):
        # "Store" the message received.
        self.velocity = msg

        # Compute stuff.
        self.compute_stuff()

    def compute_stuff(self):
        if self.orientation is not None and self.velocity is not None:
            pass  # Compute something.


if __name__ == '__main__':
    rospy.init_node('listener')

    server = Server()

    rospy.Subscriber('/orientation', Float64MultiArray , server.orientation_callback)
    rospy.Subscriber('/velocity', Float64MultiArray, server.velocity_callback)

    rospy.spin()
