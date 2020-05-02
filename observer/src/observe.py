#!/usr/bin/python3
import rospy
from cv_bridge import CvBridge
from human_info.msg import HumanPos
from sensor_msgs.msg import Image
import logging

class Observer:
    def __init__(self):
        self.pub = rospy.Publisher('observe', Image, queue_size=1)
        self.bridge = CvBridge()

    def receive_frame(self, frame):
        frame = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        
        # add information to the image
        frame = self.add_human_info(frame)
        
        frame = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        self.pub.publish(frame)


    def add_human_info(self, frame):
        # TODO
        return frame

def ros_run():
    rospy.init_node('observer', anonymous=True)
    observer = Observer()
    sub = rospy.Subscriber('/camera/image_raw', Image, observer.receive_frame)
    logging.info('Subscribed to camera')
    rospy.spin()

if __name__ == "__main__":
    ros_run()
