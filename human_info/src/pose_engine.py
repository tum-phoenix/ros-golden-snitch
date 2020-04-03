import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import collections
import math

import numpy as np
from PIL import Image
import tensorflow as tf
import cv2 

KEYPOINTS = (
  'nose',
  'left eye',
  'right eye',
  'left ear',
  'right ear',
  'left shoulder',
  'right shoulder',
  'left elbow',
  'right elbow',
  'left wrist',
  'right wrist',
  'left hip',
  'right hip',
  'left knee',
  'right knee',
  'left ankle',
  'right ankle'
)

class Keypoint:
	__slots__ = ['k', 'yx', 'score']

	def __init__(self, k, yx, score=None):
		self.k = k
		self.yx = yx
		self.score = score

	def __repr__(self):
		return 'Keypoint(<{}>, {}, {})'.format(KEYPOINTS[self.k], self.yx, self.score)


class Pose:
	__slots__ = ['keypoints', 'score']

	def __init__(self, keypoints, score=None):
		assert len(keypoints) == len(KEYPOINTS)
		self.keypoints = keypoints
		self.score = score

	def __repr__(self):
		return 'Pose({}, {})'.format(self.keypoints, self.score)

def argmax_2d(tensor):
  """
  Uses implementation of https://stackoverflow.com/questions/36388431/tensorflow-multi-dimension-argmax
  """
  # input format: BxHxWxD
  assert rank(tensor) == 4, f'Got tensor rank {rank(tensor)} but expected 4'

  # flatten the Tensor along the height and width axes
  flat_tensor = tf.reshape(tensor, (tf.shape(tensor)[0], -1, tf.shape(tensor)[3]))

  # argmax of the flat tensor
  argmax = tf.cast(tf.argmax(flat_tensor, axis=1), tf.int32)

  # convert indexes into 2D coordinates
  argmax_x = argmax // tf.shape(tensor)[2]
  argmax_y = argmax % tf.shape(tensor)[2]

  # stack and return 2D coordinates
  return tf.stack((argmax_x, argmax_y), axis=1)

def rank(tensor):

  # return the rank of a Tensor
  return len(tensor.shape)


class PoseEngine:

	def __init__(self, model_path):
		
		self.interpreter = tf.compat.v2.lite.Interpreter(model_path=model_path)
		self.interpreter.allocate_tensors()

		self.input_details = self.interpreter.get_input_details()
		self.output_details = self.interpreter.get_output_details()
		
		self.image_height = self.input_details[0]['shape'][1]
		self.image_width = self.input_details[0]['shape'][2]
		
	def DetectPosesInImage(self, img):	
		
		self.interpreter.set_tensor(self.input_details[0]['index'], img[np.newaxis, :])

		self.interpreter.invoke()

		outputs = [self.interpreter.get_tensor(detail['index']) for detail in self.output_details]

		return self.ParseOutput(outputs) # TODO uncomment , 0
		

	def ParseOutput(self, outputs):
		
		heatmaps = tf.cast(outputs[0], dtype=tf.float32) # (1 x 9 x 9 x 17)
		offsets = outputs[1][0] # (1 x 9 x 9 x 34)
		
		scores = tf.sigmoid(heatmaps)
		heatmapPositions = argmax_2d(scores)[0] # (2 x 17)
		heatmapPositions = tf.transpose(heatmapPositions) # (17 x 2)
		
		heatmapPositions = tf.matmul(heatmapPositions, tf.constant([[1, 0], [0, 1]]))

		offsetVectors = [
			[offsets[y, x, k], offsets[y, x, k + 17]] 
			for k, (y, x) in enumerate(heatmapPositions)
		]

		offsetVectors = tf.cast(tf.constant(offsetVectors), dtype=tf.int32)
		
		outputStride = 16
		
		keypointPositions = heatmapPositions * outputStride + offsetVectors
		keypointScores = [scores[0, y, x, i] for i, (y, x) in enumerate(heatmapPositions)]
		keypoints = []
		for i in range(len(keypointScores)):
			keypoint = Keypoint(i, keypointPositions[i], score=keypointScores[i])
			keypoints.append(keypoint)
		keypointMap = {KEYPOINTS[pos]:keypoint for pos, keypoint in enumerate(keypoints)}
		pose = Pose(keypointMap, score=tf.math.reduce_mean(keypointScores))
		return pose

def add_pose(pose, frame):
	"""
	Draws the predicted feature points on the frame. Further, it draws lines between the
	feature points it uses for prediction.

	@param poses the poses predicted by the NN
	@param frame the camera frame
	@param threshold the score threshold for the pose prediction
	"""
	keypoints = pose.keypoints
	for name in keypoints:
		point = keypoints[name]
		y, x = tuple(point.yx)
		frame = cv2.circle(frame, (x, y), 2, (255, 0, 0), 2)
	return frame

def show_img(img, wait=0):
	cv2.imshow('image', img)
	cv2.waitKey(wait)

def run():
	cap = cv2.VideoCapture(0)
	while True:
		_, img = cap.read()
		img = cv2.resize(np.float32(img), (257, 257))
		model = PoseEngine('models/posenet_mobilenet_v1_100_257x257_multi_kpt_stripped.tflite')
		pose = model.DetectPosesInImage(img)
		img = add_pose(pose, img)
		img = np.uint8(img)
		show_img(img, wait=1)

def get_image():
	return cv2.imread('img.jpg') #np.asarray(Image.open('img.jpg'), dtype=np.float32)
