import numpy as np
from cv_bridge import CvBridge
import os
import sys
import rospkg
rospack = rospkg.RosPack()
path = rospack.get_path('human_info')
sys.path.append(os.path.join(path, 'src/project-posenet/'))
from pose_engine import PoseEngine

ERASE_LINE = '\x1b[2K'

# possible FEATURES
FEATURES = ['nose', 'left eye', 'right eye', 'left ear', 'right ear', 'left shoulder', 'right shoulder', 'left elbow', 'right elbow', 'left wrist', 'right wrist', 'left hip', 'right hip', 'left knee', 'right knee', 'left ankle', 'right ankle']


#TODO search better FEATURE_DISTANCES and test them
# parameters
FEATURE_DISTANCES = [
	#('left elbow', 'left wrist', 0.26), # avg male 0.26; avg female 0,235 (estimate)
	#('right elbow', 'right wrist', 0.26), # avg male 0.26; avg female 0,235 (estimate)
	('left ankle', 'left knee', 0.45), # avg male 0.45; avg female 0.415 (valid)
	('right ankle', 'right knee', 0.45), # avg male 0.45; avg female 0.415 (valid)
	('left shoulder', 'left elbow', 0.35), #avg male 0.35; avg female 0.325 (valid)
	('right shoulder', 'right elbow', 0.35), #avg male 0.35; avg female 0.325 (valid)
	('right shoulder', 'right hip', 0.56), #avg male 0.5; avg female 0.485 (estimate)
	('right shoulder', 'right hip', 0.56), #avg male 0.5; avg female 0.485 (estimate)
]

MAX = 2

THRESHOLD = 0.5

# depends on the camera that is used
# FOCAL_LENGTH = distance_to_obj * pixle_size_of_obj / real_obj_size
#FOCAL_LENGTH = 6.6761817924503907e+02 #616.728321429  
FOCAL_LENGTH = 7.395847924773853e+02/2.3 #4.533079645790183e+02

ENGINE = PoseEngine(os.path.join(path,
            'src/project-posenet/models/posenet_mobilenet_v1_075_481_641_quant_decoder_edgetpu.tflite'))
