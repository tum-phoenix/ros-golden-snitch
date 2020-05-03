import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import logging
import cv2
import sys

class Viewer:

    def __init__(self, topic):
        rospy.init_node('viewer', anonymous=True)
        self.topic = topic
        self.bridge = CvBridge()

    def start(self):
        sub = rospy.Subscriber(self.topic, Image, self._show)
        logging.info(f'Subscribed to topic {self.topic}')
        rospy.spin()

    def _show(self, frame):
        frame = self.bridge.imgmsg_to_cv2(frame, 'bgr8')
        cv2.imshow(self.topic, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            logging.info(f'Stop viewer')
            sys.exit(0)
