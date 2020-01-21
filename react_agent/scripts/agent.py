#!/usr/bin/env python



# this was copied from https://stackoverflow.com/questions/48830056/use-data-from-multiple-topics-in-ros-python
import rospy
from std_msgs.msg import Float64MultiArray
from teraranger_array.msg import RangeArray
from human_info.msg import HumanPos
from decision_tree import Decision_tree
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler


class Server:
    def __init__(self):
        self.human_dir = None
        self.human_dist = None
        self.distances = []

        self.ai = Decision_tree()

        self.pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1) # Should be the one mavros wants, but we don't know if Ardupilot will accept it.

    def orientation_callback(self, msg):
        # "Store" message received.
        self.orientation = msg

        # Compute stuff.
        self.compute_stuff()

    def ranges_callback(self, msg):
        # "Store" the message received.
        # TODO: Check what kind of format the data becomes in python.
        ranges = map(lambda x: x.range, msg.ranges)
        self.distances = ranges
        print("Terraranger array messages:", ranges)

        # Compute stuff.
        self.compute_stuff()

    def compute_stuff(self):
        if self.human_dir is not None and len(self.distances) !=0:
            new_setpoint = self.ai.update(self.distances, self.human_dir, self.human_dist)
            res = _convert_to_mavros_message(new_setpoint)
            self.pub.publish(res)

def _convert_to_mavros_message(setpoint):
    res = PoseStamped()
    res.header = Header()
    res.header.stamp = rospy.Time.now()
    res.pose.position.x = setpoint[0]
    res.pose.position.y = setpoint[1]
    res.pose.position.z = setpoint[2]

    quaternion = quaternion_from_euler(0, 0, setpoint[3])
    res.pose.orientation = Quaternion(*quaternion)
    return res



if __name__ == '__main__':
    rospy.init_node('decicion_maker')

    server = Server()

    rospy.Subscriber('/human_info/human_info', Float64MultiArray, server.orientation_callback)
    rospy.Subscriber('/multiflex_1/ranges_raw', RangeArray, server.ranges_callback)

    rospy.spin()
