#!/usr/bin/env python3



# this was copied from https://stackoverflow.com/questions/48830056/use-data-from-multiple-topics-in-ros-python
import rospy
from std_msgs.msg import Float64MultiArray
from mavros import setpoint as SP
from teraranger_array.msg import RangeArray
from human_info.msg import HumanPos
# from react_agent import Decision_tree
from react_agent.decision_tree import Decision
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Vector3
from mavros_msgs.msg import PositionTarget
from tf.transformations import quaternion_from_euler
import numpy as np


class Server:
    def __init__(self):
        self.human_dir = None
        self.human_dist = None
        self.distances = []

        self.ai = Decision()

        # For velocities
        # self.pub = rospy.Publisher("phx/setpoint", Twist, queue_size=1)

        # For positions
        self.pub = rospy.Publisher("phx/setpoint/pos", PositionTarget, queue_size=1)


    def human_tracking_callback(self, msg):
        self.human_dist = msg.distance
        self.human_dir = np.array([msg.h_angle, msg.v_angle])


        self.generate_setpiont()

    def ranges_callback(self, msg):
        ranges = np.array(list(map(lambda x: x.range, msg.ranges)))
        self.distances = ranges
        print("Terraranger array messages:", ranges)

        self.generate_setpiont()

    def generate_setpiont(self):
        if self.human_dir is not None and len(self.distances) != 0:
            new_setpoint = self.ai.update_perception(self.distances, self.human_dir, self.human_dist)
            res = _convert_to_mavros_vel_message(new_setpoint)
            self.pub.publish(res)

def _convert_to_mavros_vel_message(setpoint) -> Twist:
    res = Twist()
    res.header = Header()
    res.header.stamp = rospy.Time.now()

    res.linear = Vector3(*setpoint[:3])
    res.angular = Vector3(0, 0, setpoint[3])

    return res

def _convert_to_mavros_pos_message_pose_stamped(setpoint) -> PoseStamped:
    # To be used with geometry_msg.Pose
    res = PoseStamped()
    res.header = Header()
    res.header.stamp = rospy.Time.now()

    res.pose.position.x = setpoint[0]
    res.pose.position.y = setpoint[1]
    res.pose.position.z = setpoint[2]
    quaternion = quaternion_from_euler(0, 0, setpoint[3])
    res.pose.orientation = Quaternion(*quaternion)
    return res

def _convert_to_mavros_pos_message(setpoint) -> PositionTarget:
    # To be used with geometry_msg.Pose
    res = PositionTarget()
    res.header = Header()
    res.header.stamp = rospy.Time.now()

    res.coordinate_frame = PositionTarget.FRAME_BODY_NED
    res.type_mask = res.IGNORE_VX + res.IGNORE_VY + res.IGNORE_VZ + res.IGNORE_AFX + res.IGNORE_AFY + res.IGNORE_AFZ + res.IGNORE_YAW_RATE


    res.position.x = setpoint[0]
    res.position.y = setpoint[1]
    res.position.z = setpoint[2]
    res.yaw = (setpoint[3] + 90) * np.pi/180

    return res


if __name__ == '__main__':
    rospy.init_node('decicion_maker')

    server = Server()

    rospy.Subscriber('/human_info/human_info', HumanPos, server.human_tracking_callback)
    rospy.Subscriber('/multiflex_1/ranges_raw', RangeArray, server.ranges_callback)

    rospy.spin()
