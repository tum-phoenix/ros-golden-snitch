#!/usr/bin/env python3
import numpy as np
from rospy import loginfo


class Mapper:
    def __init__(self, dirOfRangeSensosr):
        # TODO: Decide for feasible map datastructure
        self.map = []
        self.dirOfRangeSensors = dirOfRangeSensosr


    def update(self, ranges : np.ndarray, position, orientation):
        """
        @param position: 3 element list x, y, z
        @param orientation: 4 element unit quaternion with w first: [w, x, y, x]
        """
        self.map = _update_map(self.map,self.dirOfRangeSensors, ranges, position, orientation)
        return self.map

def _update_map(map,dirOfRangeSensors, ranges, position, orientation):
    # TODO: Create the logic
    return map