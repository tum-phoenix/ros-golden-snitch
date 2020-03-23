#!/usr/bin/env python

class Decision_tree:
	def __init__(self):
		pass

	# @returns : a python list of length 4 on the format : [pox_x, pos_y, pos_z, yaw] in the local frame. Yaw is in degrees
	def update(self, distances, dir_human, dist_human, ):  # this takes all inputs from all sensors etc.
		# case = 0 => No human detected
		# case = 1 =>
		newPos = [0, 0] # We support for now only 2D movement
		yaw = 0

		case = 0
		d_threshold = 1
		if (case) == 0:
			if dir_human == None or dist_human == None:
				case = 1
			if distances[0] < d_threshold:
				case = 2
		elif(case) == 1:
			pass
		elif(case) == 2:
			pass
		elif(case) == 3:
			pass
		elif(case) == 4:
			pass
		else:
			pass



		# ...

		# Yaw is controlled independently of the other movements

		if case != 0: # we can see the human
			yaw = - dir_human # TODO: Test if this is the correct sign

		return [newPos[0],newPos[1],0,yaw]
