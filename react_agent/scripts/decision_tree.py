#!/usr/bin/env python
import math

def closestDist(distances):
    closestIndex, closestLength = -1, -1
    for (i, dist) in enumerate(distances):
	if closestLength < dist:
	    closestIndex = i
	    closestLength = dist

def human_detected():
    if cosesestDist(distances) < 1:
			human_detected_wall_close()
		elif closesestDist(distances)<2:
			human_detecded_medium_far()
		elif closestDist(distances)< 10:
			#human_detected_far()
			newDir = -dir_human
			vel_x = math.arccos(dir_human) # Not sure if arccos
			vel_y = math.arcsin(dir_human) # Not sure if arcsin

def human_not_detected(self, distances,yaw):
    self.Human_detected = 0
	self.lastDirHuman = human_dir
	yaw = self.lastDirHuman

def human_detected(self, distances,yaw):
	self.Human_detected = 1
	yaw = - dir_human # TODO: Test if this is the correct sign


def check_if_a_wall_is_near(self, distances):
    i=0
    while(i<9):
        if distances[i] > 0 && i<:
            self.Wall_detected = 0
            break
    if self.Wall_detected == 0 # Wall detected

def decision(self,distances): #didn't make use of sensors 2,4 & 8 yet and there is no randomness in the algorithm
    if(self.Human_detected && distances[5]<0): # no wall detected behind the drone: by default we fly backwards
        return 
    elif(self.Human_detected && distances[5]>0 && distance[3]<0] # no wall detected on our right
        return
    elif(self.Human_detected && distances[5]>0 && distance[3]>0 && distances[7]<0] # only left is an option
        return
		
class Decision_tree:
	
	def __init__(self):
		self.LastPosHuman = [0, 0]
        self.Human_detected = False
        self.Wall_detected = 1
        self.Wall_left = 1 # I was thinking of making a 2 dimensional array later so we can work with more exact coordinates of the wall
        self.Wall_right = 1
        self.Wall_inFront = 1
        self.Wall_behind = 1
        self.Sensor_ancle_positioning=[0,45,90,135,180,225,270,315]
		pass

	# @returns : a python list of length 4 on the format : [vel_x, vel_y, vel_z, yaw] in the local frame. Yaw is in degrees
    # dir_human, distances, dist_human 2-dimensional array
    # distances is a 8th dimensional array consisting of the measurments of the 8 sensors
	def update(self, distances, dir_human, dist_human):  # this takes all inputs from all sensors etc.
		
		# case = 0 => No human detected
		# case = 1 => human is close
        # case = 2 => human is far away 
        # case = 3 => wall detected
        # case = 4 => nothing detected

		newPos = [0, 0] # We support for now only 2D movement
		yaw = 0

        # case 0
		if dist_human < 0:
			human_not_detected(self,distances, yaw)
		else:
        # deciding in human_detected() if it's case 1, 2 or 3
			human_detected(self,distances, dir_human,dist_human, yaw)
        
        decision(self, distances)

		# Yaw is controlled independently of the other movements

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
