#!/usr/bin/env python
import math

def closestDist(distances):
    closestIndex, closestLength = -1, -1
    for (i, dist) in enumerate(distances):
	if closestLength < dist:
	    closestIndex = i
	    closestLength = dist

def human_not_detected(distanes):
	if dir_human == None or dist_human == None:
		case = 1
	if distances[0] < d_threshold:
		case = 2
		
class Decision_tree:
	
	def __init__(self):
		self.LastPos = 0
		pass

	# @returns : a python list of length 4 on the format : [vel_x, vel_y, vel_z, yaw] in the local frame. Yaw is in degrees
	def update(self, distances, dir_human, dist_human, ):  # this takes all inputs from all sensors etc.
		
		
		# case = 0 => No human detected
		# case = 1 =>
		newPos = [0, 0] # We support for now only 2D movement
		yaw = 0
		if dist_human < 0:
			human_not_detected()
		else:
			humand_detected()
		
		if cosesestDist(distances) < 1:
			human_detected_wall_close()
		elif closesestDist(distances)<2:
			human_detecded_medium_far()
		elif closestDist(distances)< 10:
			#human_detected_far()
			newDir = -dir_human
			vel_x = math.arccos(dir_human) # Not sure if arccos
			vel_y = math.arcsin(dir_human) # Not sure if arcsin
			
		case = 0
		d_threshold = 1
		if (case) == 0:
			human_not_detected(distances)
			#if dir_human == None or dist_human == None:
			#	case = 1
			#if distances[0] < d_threshold:
			#	case = 2
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
		if human_dir = -1000:
			yaw = self.lastDirHuman
			
		self.lastDirHuman = human_dir

		return [newPos[0],newPos[1],0,yaw]
	
def main():
	DC = Decision_tree()
	#...
	DC.update([2,2,2,2,1,2,1], [0,0.2],1.2)
	##//
	res = DC.update([2,2,2,2,1,2,1], [0,0.2],1.2)
	if res == [0,3,1,2]:
		print("Test correct")
	else:
		print("Test failed")

		
if __name__ == '__main__':
	main()
