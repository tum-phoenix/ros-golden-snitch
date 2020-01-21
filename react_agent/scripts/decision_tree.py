#!/usr/bin/env python

class Decision_tree:
	def __init__(self):
		pass

	# @returns : a python list of length 4 on the format : [pox_x, pos_y, pos_z, yaw] in the local frame
	def update(self, distances, where_is_human, how_far_is_human, ):  # this takes all inputs from all sensors etc.
		state_var = 0
		d_threshold = 1
		if (state_var) == 0:
			if where_is_human == None or how_far_is_human == None:
				state_var = 1
			if distances[0] < d_threshold:
				state_var = 2
		elif(state_var) == 1:
			pass
		elif(state_var) == 2:
			pass
		elif(state_var) == 3:
			pass
		elif(state_var) == 4:
			pass
		else:
			pass



		# ...

		return [0,0,0,0]
