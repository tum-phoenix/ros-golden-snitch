#!/usr/bin/env python3
import numpy as np
from rospy import loginfo


class Mapper:
    def __init__(self):
        # TODO: Decide for feasible map datastructure
        self.map = []

    def update(self, ranges : np.ndarray, pose):
        map = _update_map(self.map, ranges, pose)

def _update_map(map, ranges, pose):
    # TODO: Create the logic
    pass