#!/usr/bin/env python3
import rospkg
import rospy
import yaml
from example_node.example_logic import Example_logic_class
from std_msgs.msg import Header


class Server:
    def __init__(self):

        rospack = rospkg.RosPack()
        with open(rospack.get_path("phx_launch")+"/../config/hardware_config.yaml") as f:
            mechanical_config = yaml.load(f, Loader=yaml.SafeLoader)
        # TODO: Save
        self.example = Example_logic_class(4)
        self.pub = rospy.Publisher("phx/example_pub", Header, queue_size=1)


    def _example_callback(self, msg : Header):
        # TODO: convert the message to desired format
        res = self.mapper.update(msg.stamp, msg.seq)

        # TODO: convert the result to the reasonable message
        msg = Header(res)
        self.pub.publish(msg)



if __name__ == '__main__':
    rospy.init_node('example_node')

    server = Server()

    rospy.Subscriber("/phx/example_sub", Header, server._example_callback)
    rospy.loginfo("Example node is initialized")

    rospy.spin()
