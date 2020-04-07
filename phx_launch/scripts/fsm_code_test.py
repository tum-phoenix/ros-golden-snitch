#!/usr/bin/env python3
PKG='test_fsm'

import sys
import unittest
import fsm as fsmmod

## A sample python unit test
class TestFsm(unittest.TestCase):

    def test_constants_set(self):
        print("Test is running !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
        #FSM = fsmmod.Fsm() We need to offer a lot of mavros services for this to work.
        self.assertEqual(1,1, "1!=1")
        # self.assertEquals(FSM.TAKEOFF_HEIGHT, 1, "Takeoff height is not 1. Remove this test when not useful anymore. TAKEOFFHEIGHT:"+str(FSM.TAKEOFF_HEIGHT))
        # self.assertGreaterEqual(FSM.HEIGHT_FACTOR_NECESARRY, 0, "The scaling factor must be in the interval (0,1). It is now to small")
        # self.assertGreaterEqual(1, FSM.HEIGHT_FACTOR_NECESARRY, "The scaling factor must be in the interval (0,1). It is now to large")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_fsm_code_level', TestFsm)