#!/usr/bin/env python3
import unittest
import sys
import numpy as np

from local_map.map_logic import Mapper, _update_map

PKG = 'local_map'


class TestLocalMapper(unittest.TestCase):

    def setUp(self) -> None:
        pass

    def test_import(self):
        mapper = Mapper(np.array([0, 180]), 0.1, 50)
        local_map = []
        age = []
        dirOfRangeSensors = np.array([0, 180])
        distCenterDroneRangeSens = 0
        ranges = np.array([0, 1])
        position = np.zeros(3)
        orientation = np.zeros(4)
        orientation[0] = 1
        max_readingage = 1
        iteration = 0
        mapper.update(ranges, position, orientation)
        _update_map(local_map, age, dirOfRangeSensors, distCenterDroneRangeSens, ranges, position, orientation,
                    max_readingage, iteration)
        self.assertTrue(True, "Should be true if the import is working")

    def test_rotation_matrix(self):
        # give different values for the orientation quaternion and verify that the correct matrix is calculated
        # self.assertEqual(R_dw, #TODO) calculate the value according to the quaternion
        pass

    def test_dynamic_age(self):
        # verify that older values in the local_map variables are successfully replaced if too old
        pass


if __name__ == '__main__':
    import rosunit
    import rostest

    # rosunit.unitrun(PKG, 'test_fsm_code_level', TestFsm)
    rostest.rosrun(PKG, 'test_map_logic', TestLocalMapper, sys.argv)
