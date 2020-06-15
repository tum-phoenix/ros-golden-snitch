#!/usr/bin/env python3
import numpy as np
from rospy import loginfo
from math import cos,sin,tan

class Mapper:
    def __init__(self, dirOfRangeSensors):
        # TODO: Decide for feasible map datastructure --> list of points
        self.map = []
        self.dirOfRangeSensors = dirOfRangeSensors


    def update(self, ranges : np.ndarray, position, orientation):
        """
        @param position: 3 element list x, y, z
        @param orientation: 4 element unit quaternion with w first: [w, x, y, x]
        """
        self.map = _update_map(self.map,self.dirOfRangeSensors, ranges, position, orientation)
        return self.map

def _update_map(map,dirOfRangeSensors, ranges, position, orientation):
    # splitting position for further calculations
    pos_x = position[0]
    pos_y = position[1]
    pos_z = position[2]

    #distance of range sensors from center of drone TODO: measure on drone and include in config file
    distCenterDroneRangeSens = 0.1 #initial guess

    # rotation matrix from orientation quaternion
    q_w = orientation[0]
    q_x = orientation[1]
    q_y = orientation[2]
    q_z = orientation[3]
    orientation_matrix = np.array([[1 - 2 * (q_y **2 + q_z **2), 2 * (q_x * q_y - q_w * q_z), 2 * (q_x * q_z + q_y * q_w)],
                          [2 * (q_x * q_y + q_w * q_z), 1 - 2 * (q_x **2 + q_z **2), 2 * (q_y * q_z - q_x * q_w)],
                          [2 * (q_x * q_z - q_w * q_y), 2 * (q_y * q_z + q_w * q_x), 1 - 2 * (q_x **2 + q_y **2)]])

    # range sensor direction vectors in drone frame of reference
    row1 = np.cos(dirOfRangeSensors)
    row2 = np.sin(dirOfRangeSensors)
    row3 = np.zeros((1,dirOfRangeSensors.size))
    dirVectInDroneFOR = np.vstack([row1,row2,row3])

    pointCoordInDroneFOR = dirVectInDroneFOR.copy
    #either do vector after vector --> col1, col2, ... , col8 and then hstack or for-loop TODO
    #basic idea: pointCoordInDroneFOR[i] = ranges[i] * dirVectInDroneFOR[:,i]


    #transformation to world frame of reference
    pointCoordInWorldFOR = np.matmul(orientation_matrix,pointCoordInDroneFOR)

    #add new points to map --> pointCoordInWorldFOR(:,i) TODO


    # TODO: Create the logic
    return map