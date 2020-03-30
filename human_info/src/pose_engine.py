import collections
import math

import numpy as np
from PIL import Image
import tensorflow as tf

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

def run():
	img = get_image()
	model = PoseEngine('models/posenet_mobilenet_v1_100_257x257_multi_kpt_stripped.tflite')
	out = model.DetectPosesInImage(img)
	pose = model.ParseOutput(out)
	return pose, model

def get_image():
	return np.asarray(Image.open('img.jpg'), dtype=np.float32)

def sigmoid(x):
	return 1 / (1 + np.exp(-x))

class PoseEngine:

	def __init__(self, model_path):
		
		self.interpreter = tf.compat.v2.lite.Interpreter(model_path=model_path)
		self.interpreter.allocate_tensors()

		self.input_details = self.interpreter.get_input_details()
		self.output_details = self.interpreter.get_output_details()
		
		self.image_height = self.input_details[0]['shape'][1]
		self.image_width = self.input_details[0]['shape'][2]
		
	def DetectPosesInImage(self, img):	
		
		# Extend or crop the input to match the input shape of the network.
		if img.shape[0] < self.image_height or img.shape[1] < self.image_width:
			img = np.pad(img, [[0, max(0, self.image_height - img.shape[0])],
							   [0, max(0, self.image_width - img.shape[1])], [0, 0]],
						 mode='constant')
		img = img[0:self.image_height, 0:self.image_width]
		
		self.interpreter.set_tensor(self.input_details[0]['index'], img[np.newaxis, :])

		self.interpreter.invoke()

		outputs = [self.interpreter.get_tensor(detail['index']) for detail in self.output_details]

		return self.ParseOutput(outputs), 0
		

	def ParseOutput(self, outputs):
		"""
		Implementation based on https://github.com/tensorflow/examples/blob/master/lite/examples/posenet/android/posenet/src/main/java/org/tensorflow/lite/examples/posenet/lib/Posenet.kt
		# TODO make the code more pythonic 
		"""
		heatmaps = outputs[0]
		offsets = outputs[1]
		
		height, width = heatmaps[0, ..., 0].shape

		key_points = []

		# get maximum point of headmap for each keypoint
		for key_point_num in range(17):
			max_val = heatmaps[0,0,0, key_point_num]
			max_row = 0
			max_col = 0
			for row in range(height):
				for col in range(width):
					if heatmaps[0,row,col,key_point_num] > max_val:
						max_val = heatmaps[0,row,col,key_point_num]
						max_row = row
						max_col = col
			key_points.append((max_row, max_col))
		
		key_point_map = {}
		scores = []
		# calculate pixle coordinates based on offset
		for pos, (row, col) in enumerate(key_points):
			x = int((row / (height - 1)) * self.image_height) # + offsets[0, row, col, pos])
			y = int((col / (width - 1)) * self.image_width) #+ offsets[0, row, col, pos + 17])
			score = sigmoid(heatmaps[0, row, col, pos]) * 100 
			key_point = Keypoint(pos, (y,x), score)
			scores.append(score)
			key_point_map[KEYPOINTS[pos]] = key_point
		pose = Pose(key_point_map, sum(scores))
		return [pose]
