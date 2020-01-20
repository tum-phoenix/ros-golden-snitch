import numpy as np
import cv2
# This module is a pain in the ass :(
# If someone has a nice way to handle the import error if no
# ids camera is attached change this thx
try:
	from pyueye import ueye
except ImportError:
	pass

class CVCamera():
	def __init__(self, camera):
	    self.camera = camera
	def __enter__(self):
	    self.cap = cv2.VideoCapture(self.camera)
	    return self
	def read(self):
	    _, frame = self.cap.read()
	    return frame
	def __exit__(self, *_):
	    # TODO free camera and memory
	    pass
	def open(self):
	    self.__enter__()
	def close(self):
	    self.__exit__()

class IDSCamera:
	"""
	IDS Camera access
	"""
	def __init__(self, _):
		pass # TODO this is ugly -> make it better
	def __enter__(self):
		"""
		Sets parameters to read the data from the camera
		TODO understan what it does and extend
		"""

		#Variables
		hCam = ueye.HIDS(0)			 #0: first available camera;  1-254: The camera with the specified camera ID
		sInfo = ueye.SENSORINFO()
		cInfo = ueye.CAMINFO()
		pcImageMemory = ueye.c_mem_p()
		MemID = ueye.int()
		rectAOI = ueye.IS_RECT(1280,1024)
		pitch = ueye.INT()
		nBitsPerPixel = ueye.INT(24)	#24: bits per pixel for color mode; take 8 bits per pixel for monochrome
		channels = 3					#3: channels for color mode(RGB); take 1 channel for monochrome
		m_nColorMode = ueye.INT(1)		# Y8/RGB16/RGB24/REG32
		bytes_per_pixel = int(nBitsPerPixel / 8)

		# Starts the driver and establishes the connection to the camera
		nRet = ueye.is_InitCamera(hCam, None)
		if nRet != ueye.IS_SUCCESS:
			raise Exception("is_InitCamera ERROR")

		# Reads out the data hard-coded in the non-volatile camera memory and writes it to the data structure that cInfo points to
		nRet = ueye.is_GetCameraInfo(hCam, cInfo)
		if nRet != ueye.IS_SUCCESS:
			raise Exception("is_GetCameraInfo ERROR")

		# You can query additional information about the sensor type used in the camera
		nRet = ueye.is_GetSensorInfo(hCam, sInfo)
		if nRet != ueye.IS_SUCCESS:
			raise Exception("is_GetSensorInfo ERROR")

		nRet = ueye.is_ResetToDefault( hCam)
		if nRet != ueye.IS_SUCCESS:
			raise Exception("is_ResetToDefault ERROR")

		# Set display mode to DIB
		nRet = ueye.is_SetDisplayMode(hCam, ueye.IS_SET_DM_DIB)

		# Set the right color mode
		if int.from_bytes(sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_BAYER:
			# setup the color depth to the current windows setting
			ueye.is_GetColorDepth(hCam, nBitsPerPixel, m_nColorMode)
			bytes_per_pixel = int(nBitsPerPixel / 8)

		elif int.from_bytes(sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_CBYCRY:
			# for color camera models use RGB32 mode
			m_nColorMode = ueye.IS_CM_BGRA8_PACKED
			nBitsPerPixel = ueye.INT(32)
			bytes_per_pixel = int(nBitsPerPixel / 8)

		elif int.from_bytes(sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_MONOCHROME:
			# for color camera models use RGB32 mode
			m_nColorMode = ueye.IS_CM_MONO8
			nBitsPerPixel = ueye.INT(8)
			bytes_per_pixel = int(nBitsPerPixel / 8)
		
		else:
			# for monochrome camera models use Y8 mode
			m_nColorMode = ueye.IS_CM_MONO8
			nBitsPerPixel = ueye.INT(8)
			bytes_per_pixel = int(nBitsPerPixel / 8)

		# Can be used to set the size and position of an "area of interest"(AOI) within an image
		nRet = ueye.is_AOI(hCam, ueye.IS_AOI_IMAGE_GET_AOI, rectAOI, ueye.sizeof(rectAOI))
		if nRet != ueye.IS_SUCCESS:
			raise Exception("is_AOI ERROR")

		width = rectAOI.s32Width
		height = rectAOI.s32Height

		# Allocates an image memory for an image having its dimensions defined by width and height and its color depth defined by nBitsPerPixel
		nRet = ueye.is_AllocImageMem(hCam, width, height, nBitsPerPixel, pcImageMemory, MemID)
		if nRet != ueye.IS_SUCCESS:
			raise Exception("is_AllocImageMem ERROR")
		else:
			# Makes the specified image memory the active memory
			nRet = ueye.is_SetImageMem(hCam, pcImageMemory, MemID)
			if nRet != ueye.IS_SUCCESS:
				raise Exception("is_SetImageMem ERROR")
			else:
				# Set the desired color mode
				nRet = ueye.is_SetColorMode(hCam, m_nColorMode)



		# Activates the camera's live video mode (free run mode)
		nRet = ueye.is_CaptureVideo(hCam, ueye.IS_DONT_WAIT)
		if nRet != ueye.IS_SUCCESS:
			raise Exception("is_CaptureVideo ERROR")

		# Enables the queue mode for existing image memory sequences
		nRet = ueye.is_InquireImageMem(hCam, pcImageMemory, MemID, width, height, nBitsPerPixel, pitch)
		if nRet != ueye.IS_SUCCESS:
			raise Exception("is_InquireImageMem ERROR")

		# set attributes
		self.pcImageMemory = pcImageMemory
		self.width = width
		self.height = height
		self.nBitsPerPixel = nBitsPerPixel
		self.pitch = pitch
		self.ueye = ueye
		self.bytes_per_pixel = bytes_per_pixel
		self.hCam = hCam
		return self	

	def open(self):
		"""See __enter__"""
		self.__enter__()	

	def read(self):
		"""
		Reads the next frame from the camera

		Returns the frame as a numpy array
		"""
		# get data from camera
		array = self.ueye.get_data(self.pcImageMemory, self.width, self.height, self.nBitsPerPixel, self.pitch, copy=False)
		# get frame as numpy array
		frame = np.reshape(array,(self.height.value, self.width.value, self.bytes_per_pixel))
		
		"""
		camera_matrix = np.array([
			[4.5330796457901283e+02, 0., 6.1902229288626302e+02],
			[0., 4.5369175559310276e+02, 5.1298362120979994e+02],
			[0., 0., 1.]])
		
		dist_coeffs = np.array([
			-3.1812973406286371e-01, 9.6396352148682182e-02,
			2.9601124432187590e-03, 9.7700591472463412e-04,
			-1.1929681608809075e-02
		])

		frame = cv2.undistort(frame, camera_matrix, dist_coeffs, camera_matrix)
		"""

		return frame
		
	def __exit__(self, *_):
		"""
		Frees the allocated memory and exist the camera
		"""
		self.ueye.is_FreeImageMem(self.hCam, self.pcImageMemory, self.MemID)

		# Disables the hCam camera handle and releases the data structures and memory areas taken up by the uEye camera
		self.ueye.is_ExitCamera(self.hCam)
	
	def close():
		"""See __exit__"""
		self.__exit__()
