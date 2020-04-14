#!/usr/bin/python3
"""
Script for processing the camera frames.
Calcualtes the distance and the player position for each frame.
The communication with other system components is organised via ros.
"""
from config import *
from filter import *
import numpy as np
import cv2
#import rospy
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
#from human_info.msg import HumanPos


def pos_from_center(poses, shape):
	"""
	In progress
	Calculate the avg of the pose and returns the angle of the human related
	to the camera
	"""
	avg = []
	y, x = np.array(shape)/2
	center = np.array([x, y])
	for pose in poses:
		points = [point.yx for point in pose.keypoints.values()
				  if point.score > THRESHOLD]
		if points:
			pose_avg = sum(points)/len(points)
			avg.append(pose_avg)
	if avg:
		y, x = sum(avg)/len(avg)
		avg = np.array([x, y])
		x, y = (center - avg) * np.array([-1, 1])
		horizontal_angle = np.arctan(x/FOCAL_LENGTH)
		vertical_angle = np.arctan(y/FOCAL_LENGTH)
		return center.astype('int'), avg.astype('int'), horizontal_angle, vertical_angle


def cal_distance(poses):
	"""
	Calculates the distance to the player based on the predicted poses

	@param poses the list of poses predicted by the NN
	@param threshold the score threshold for the pose prediction
	"""

	if len(poses) > 0:
		pose_dis = []
		for pose in poses:
			feature_dis = []
			keypoints = pose.keypoints
			for f_name_1, f_name_2, dis in FEATURES:
				keypoint_1 = keypoints[f_name_1]
				keypoint_2 = keypoints[f_name_2]
				if keypoint_1.score > THRESHOLD and keypoint_2.score > THRESHOLD:
					# quality sufficient -> use keypoints
					pix_distance = np.linalg.norm(
						keypoint_1.yx - keypoint_2.yx, ord=1)
					distance = dis * FOCAL_LENGTH / pix_distance
					#filtering odd values
					self.outlier_rejection = Outlier_Rejection()
					distance = self.outlier_rejection.update(distance)
					# TODO remove debug when testing is done
					feature_dis.append(distance)

			if len(feature_dis) > 0:
				# avg over feature distances
				distance = sum(feature_dis)/len(feature_dis)
				pose_dis.append(distance)
		if len(pose_dis) > 0:
			# max of pose distances -> wost case
			distance = max(pose_dis)
			return distance
	return None


def add_pose(poses, frame):
	"""
	Draws the predicted feature points on the frame. Further, it draws lines between the
	feature points it uses for prediction.

	@param poses the poses predicted by the NN
	@param frame the camera frame
	@param threshold the score threshold for the pose prediction
	"""
	for pose in poses:
		keypoints = pose.keypoints
		for name in keypoints:
			if keypoints[name].score > THRESHOLD:
				y, x = tuple(keypoints[name].yx)
				frame = cv2.circle(frame, (x, y), 2, (255, 0, 0), 2)
				#frame[int(position[0])-1][int(position[1])-1] = 255
		for feature1, feature2, _ in FEATURES:
			point_1 = keypoints[feature1]
			point_2 = keypoints[feature2]
			if point_1.score > THRESHOLD and point_2.score > THRESHOLD:
				y1, x1 = point_1.yx
				y2, x2 = point_2.yx
				frame = cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
	return frame


def add_centroids(frame, center, avg):
	"""
	Draws the image center and the pose center on the frame

	@param frame the camera frame
	@param center image center
	@param avg center of the pose
	"""
	frame = cv2.circle(frame, tuple(center), 10, (0, 255, 0), 2)
	frame = cv2.circle(frame, tuple(avg), 10, (0, 0, 255), 2)
	return frame


def process_frame(frame):
	"""
	Calculatest he distance and position of the player and the detected human features

	@param frame The camera frame
	@return (distance, (frame center, pose average, horizontal_angle, vertical_angle), poses)
	"""

	# prediction
	poses, inference_time = ENGINE.DetectPosesInImage(np.uint8(frame))

	distance = cal_distance(poses)
	position = pos_from_center(poses, frame.shape[:-1])
	return distance, position, poses


class Processor:
	def __init__(self):
		self.pub = rospy.Publisher('human_info', HumanPos, queue_size=1)

	def ros_callback(self, frame):
		"""
		Starts the frame processing and publishes on the channel

		@param frame The camera frame
		"""
		frame = BRIDGE.imgmsg_to_cv2(frame, "bgr8")
		frame = cv2.resize(frame, (640, 480))
		distance, position, poses = process_frame(frame)
		if position is not None:
			center, avg, h_angle, v_angle = position
			frame = add_pose(poses, frame)
			frame = add_centroids(frame, center, avg)
		msg = HumanPos()
		if distance is not None:
			msg.h_angle = h_angle
			msg.v_angle = v_angle
			msg.distance = distance
			# filtering odd values
			self.filter = average_Filter()
			msg.distance = self.filter.update(msg.distance)
		else:
			msg.h_angle = 0
			msg.v_angle = 0
			msg.distance = -1
		self.pub.publish(msg)
		cv2.imshow('CAM', frame)
		cv2.waitKey(1)


def ros_run():
	"""
	Subscribs to the camera channel and starts the frame processing loop
	"""
	rospy.init_node('human_distance', anonymous=True)
	processor = Processor()
	sub = rospy.Subscriber('/camera/image_raw', Image, processor.ros_callback)
	print('Subscribed to camera channel')
	rospy.spin()


if __name__ == "__main__":
	ros_run()
