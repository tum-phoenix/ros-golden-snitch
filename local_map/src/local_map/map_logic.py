#!/usr/bin/env python3
import numpy as np
#from rospy import loginfo


class Mapper:
    def __init__(self, dirOfRangeSensors, distCenterDroneRangeSens, max_readingage):
        # The map is a list of 3d point coordinates where each point is (x, y, z,)
        self.local_map = []
        self.age = []
        self.max_readingage = max_readingage
        self.dirOfRangeSensors = dirOfRangeSensors
        self.distCenterDroneRangeSens = distCenterDroneRangeSens
        iteration = 0 #tracks the current "version" of readings to assign an age to s


    def update(self, ranges : np.ndarray, position, orientation):
        """
        @param position: 3 element list x, y, z
        @param orientation: 4 element unit quaternion with w first: [w, x, y, x]
        """
        self.map = _update_map(self.local_map, self.age, self.dirOfRangeSensors, self.distCenterDroneRangeSens, ranges, position, orientation, max_readingage, iteration)
        return self.map


def _update_map(local_map, age, dirOfRangeSensors, distCenterDroneRangeSens, ranges, position, orientation, max_readingage, iteration):

    # delete values where range value is out of range
    dirOfRangeSensors = dirOfRangeSensors[(ranges > 0) & ranges < 2.1]
    ranges = ranges[(ranges > 0) & (ranges < 2.1)]
    numberRangeSensors = ranges.size

    iteration += 1 # always increment iteration independently of sensor values
    iter_vec = iteration * np.ones((1, numberRangeSensors), dtype='int32')

    # check whether the oldest sensor readings are too old
    if len(set(age)) < max_readingage:
        age = np.append(age, iter_vec)

    else:   # delete earlier values TODO

        # then add new ones
        age = np.append(age, iter_vec)

    # rotation matrix from orientation quaternion
    q_w = orientation[0]
    q_x = orientation[1]
    q_y = orientation[2]
    q_z = orientation[3]
    # rotation from drone FoR to world FoR
    R_dw = np.array(
        [[1 - 2 * (q_y ** 2 + q_z ** 2), 2 * (q_x * q_y - q_w * q_z), 2 * (q_x * q_z + q_y * q_w)],
         [2 * (q_x * q_y + q_w * q_z), 1 - 2 * (q_x ** 2 + q_z ** 2), 2 * (q_y * q_z - q_x * q_w)],
         [2 * (q_x * q_z - q_w * q_y), 2 * (q_y * q_z + q_w * q_x), 1 - 2 * (q_x ** 2 + q_y ** 2)]])

    # range sensor direction vectors in drone FoR
    row1 = np.cos(dirOfRangeSensors)
    row2 = np.sin(dirOfRangeSensors)
    row3 = np.zeros((1, numberRangeSensors))
    dirVectInDroneFOR = np.vstack([row1, row2, row3]) #dimension:(3, numberRangeSensors)

    # calculate the coordinates in drone FoR
    pointCoordInDroneFOR = dirVectInDroneFOR.copy()
    for x in range(0, numberRangeSensors):
        pointCoordInDroneFOR[:, x] = (ranges[x] + distCenterDroneRangeSens) * dirVectInDroneFOR[:, x]

    # transformation to world FoR
    dronePos = np.repeat(position, numberRangeSensors, axis=1)
    pointCoordInWorldFOR = np.matmul(R_dw, pointCoordInDroneFOR) + dronePos

    # add new points to map
    for x in range(0, numberRangeSensors):
        local_map = np.hstack(local_map, pointCoordInWorldFOR[:, x])

    return local_map
