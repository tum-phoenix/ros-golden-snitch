#!/usr/bin/env python3
import numpy as np
#from rospy import loginfo


class Mapper:
    def __init__(self, dirOfRangeSensors, distCenterDroneRangeSens):
        # The map is a list of 3d point coordinates where each point is (x, y, z,)
        self.map = []
        self.dirOfRangeSensors = dirOfRangeSensors
        self.distCenterDroneRangeSens = distCenterDroneRangeSens


    def update(self, ranges : np.ndarray, position, orientation):
        """
        @param position: 3 element list x, y, z
        @param orientation: 4 element unit quaternion with w first: [w, x, y, x]
        """
        self.map = _update_map(self.map,self.dirOfRangeSensors, ranges, position, orientation)
        return self.map


def _update_map(map, dirOfRangeSensors, distCenterDroneRangeSens, ranges, position, orientation):

    # delete values where range value is out of range
    ranges = ranges[ranges < 2.1]
    dirOfRangeSensors = dirOfRangeSensors[ranges < 2.1]
    numberRangeSensors = ranges.size

    # rotation matrix from orientation quaternion
    q_w = orientation[0]
    q_x = orientation[1]
    q_y = orientation[2]
    q_z = orientation[3]
    orientation_matrix = np.array(
        [[1 - 2 * (q_y ** 2 + q_z ** 2), 2 * (q_x * q_y - q_w * q_z), 2 * (q_x * q_z + q_y * q_w)],
         [2 * (q_x * q_y + q_w * q_z), 1 - 2 * (q_x ** 2 + q_z ** 2), 2 * (q_y * q_z - q_x * q_w)],
         [2 * (q_x * q_z - q_w * q_y), 2 * (q_y * q_z + q_w * q_x), 1 - 2 * (q_x ** 2 + q_y ** 2)]])

    # range sensor direction vectors in drone FoR
    row1 = np.cos(dirOfRangeSensors)
    row2 = np.sin(dirOfRangeSensors)
    row3 = np.zeros((1, numberRangeSensors))
    dirVectInDroneFOR = np.vstack([row1, row2, row3])

    # calculate the coordinates in drone FoR
    pointCoordInDroneFOR = dirVectInDroneFOR.copy()
    for x in range(0, numberRangeSensors):
        pointCoordInDroneFOR[:, x] = (ranges[x] + distCenterDroneRangeSens) * dirVectInDroneFOR[:, x]

    # transformation to world FoR
    dronePos = np.repeat(position, numberRangeSensors, axis=1)
    pointCoordInWorldFOR = np.matmul(orientation_matrix,
                                     pointCoordInDroneFOR) + dronePos

    # add new points to map
    for x in range(0, numberRangeSensors):
        map = np.hstack(map, pointCoordInWorldFOR[:, x])

    return map
