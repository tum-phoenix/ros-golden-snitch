#!/usr/bin/python3
import rospy
from cv_bridge import CvBridge
from human_info.msg import HumanPos
from sensor_msgs.msg import Image
import cv2

FONT = cv2.FONT_HERSHEY_SIMPLEX

class Observer:
    def __init__(self):
        self.pub = rospy.Publisher('observe', Image, queue_size=1)
        self.bridge = CvBridge()
        self.human_info = None

    def receive_frame(self, frame):
        frame = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        
        # add information to the image
        frame = self.add_human_info(frame)
        
        frame = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub.publish(frame)

    def receive_human_info(self, msg):
        self.human_info = msg
            
    def add_human_info(self, frame):
        width, hight, _ = frame.shape
        if self.human_info is not None:
            distance = self.human_info.distance
            cv2.putText(frame, str(distance),
                (width//2, hight//2), FONT, 1, (255, 0, 0), 2)
        return frame

def ros_run():
    rospy.init_node('observer', anonymous=True)
    observer = Observer()
    sub = rospy.Subscriber('/camera/image_raw', Image, observer.receive_frame)
    sub = rospy.Subscriber('/human_info', HumanPos, observer.receive_human_info)
    rospy.spin()

if __name__ == "__main__":
    ros_run()
