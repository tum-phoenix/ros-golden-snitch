#!/usr/bin/python3
import sys
import numpy as np
import cv2
import pyttsx3
sys.path.append('project-posenet/')
from pose_engine import PoseEngine
from argparse import ArgumentParser
from utils.camera_utils import *

from config import *

def pos_from_center(poses, threshold, shape):
	"""
	In progress
	Calculate the avg of the pose and returns the angle of the human related 
	to the camera 
	"""
	avg = []
	for pose in poses:
		points = [point.yx for point in pose.keypoints.values() if point.score > threshold]
		if points:
			pose_avg = sum(points)/len(points)
			avg.append(pose_avg)
	if avg:
		avg = sum(avg)/len(avg)
		difference = (np.array(shape)/2 - avg) * np.array([1,-1])
		return difference

def cal_distance(poses, threshold):
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
				if keypoint_1.score > threshold and keypoint_2.score > threshold:
					# quality sufficient -> use keypoints
					pix_distance = np.linalg.norm(keypoint_1.yx - keypoint_2.yx, ord=1)
					distance = dis * FOCAL_LENGTH / pix_distance
					#TODO remove debug when testing is done
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

def add_pose(poses, frame, threshold):
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
			if keypoints[name].score > threshold:
				y, x = tuple(keypoints[name].yx)
				frame = cv2.circle(frame, (x,y), 2, (255, 0, 0), 2)
				#frame[int(position[0])-1][int(position[1])-1] = 255
		for feature1, feature2, _ in FEATURES:
			point_1 = keypoints[feature1]
			point_2 = keypoints[feature2]
			if point_1.score > threshold and point_2.score > threshold:
				y1, x1 = point_1.yx
				y2, x2 = point_2.yx
				frame = cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
	return frame

def main(args):

	# Const
	THRESHOLD = args.threshold
	SHOW = args.show
	ACC = args.accuracy if args.accuracy > 0 else None
	MAX_AVG = args.smooth
	PRINT_DELAY = args.frames
	if MAX_AVG < 0 or PRINT_DELAY < 0:
		raise ValueError('Either SMOOTH or FRAMES is smaller than 0')

	# TODO consider to use model with a higher resolution
	# increases maximum distance of the prediction
	# increases power consumption
	# might reduce fps 
	print('Load model ...')

	speak_engine = pyttsx3.init()

	engine = PoseEngine('project-posenet/models/posenet_mobilenet_v1_075_481_641_quant_decoder_edgetpu.tflite')
	said = False
	if args.ids is True:
		stream = IDSCamera
	else:
		stream = CVCamera
	print('Starte video stream')
	with stream(args.camera) as st:
		dis_history = []
		print_count = 0
		# TODO implement nice keyboard interupt
		while(True):
			frame = st.read()

			pil_image = cv2.resize(frame, (640, 480))

			# prediction
			poses, inference_time = engine.DetectPosesInImage(np.uint8(pil_image))

			distance = cal_distance(poses, THRESHOLD)
			position = pos_from_center(poses, THRESHOLD, frame.shape[:-1])
			if distance is None:
				distance = 'UNKOWN'
			else:
				dis_history.append(distance)
				if len(dis_history) > MAX_AVG:
					dis_history.pop(0)				
				#TODO sliding window alg + alternative smoothing ???
				distance = sum(dis_history)/len(dis_history)
				if ACC is not None:
					distance = round(distance, ACC)
			if print_count >= PRINT_DELAY and type(distance) is not str:
				print_count = 0
				if not said and distance < MAX:
					speak_engine.say("Stop")
					speak_engine.runAndWait()
					said = True
				elif distance > MAX:
					said = False
				print(f'Distance to human: {distance}; Position of Human: {position}', end='\r')
			else:
				print_count += 1
			sys.stdout.write(ERASE_LINE)
			if SHOW is True:
				frame = add_pose(poses, pil_image, THRESHOLD)
				cv2.imshow('Camera', frame)
				if cv2.waitKey(1) & 0xFF == ord('q'):
					break

#def ros_run():
#	rospy.init_node('human_distance', anonymous=True)
#	sub = rospy.Subscriber('human_distance', )

if __name__ == "__main__":
	parser = ArgumentParser(description="Human depth estimation tool", prog="hde")
	parser.add_argument("--ids", dest="ids", action="store_true", default=False,
			help="Uses the ids camera")
	parser.add_argument("--sm", dest="smooth", type=int, default=10,
			help="Sets the max averaging frame. Default: 10")
	parser.add_argument("--acc", dest="accuracy", type=int, default=0,
			help="Sets the accuracy of the output. Uses max accuracy if value is smaller equal 0")
	parser.add_argument("--pf", dest="frames", type=int, default=0,
			help="Sets the amount of frames between each print output")
	parser.add_argument("--show", dest="show", action="store_true",
			help="Show camera stream")
	parser.add_argument("--cam", dest="camera", type=int, default=0,
			help="Sets the camera. Default: 0")
	parser.add_argument("--th", dest="threshold", type=float, default=0.6,
			help="Set the threshold for the model points. Default: 0.6")
	args = parser.parse_args()
	main(args)	
