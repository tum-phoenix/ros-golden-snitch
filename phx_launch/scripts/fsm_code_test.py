#!/usr/bin/env python3
import time

PKG='phx_launch'

import sys
import unittest
import rospy
import fsm as fsmmod

from std_msgs.msg import Empty
from mavros_msgs.srv import SetMode, SetModeResponse, CommandBool, CommandBoolResponse, CommandBoolRequest, CommandTOL, CommandTOLResponse, SetMavFrame, SetMavFrameResponse
from geographic_msgs.msg import GeoPointStamped


class MavrosMock:

    def handle_set_mode(self, msg):
        self.modeSet = 1
        return SetModeResponse(True)

    def handle_arm(self, msg : CommandBoolRequest):
        self.armed = msg.value
        self.armed_too_early = 0
        for parameter in [self.modeSet, self.set_vel_FOR, self.origin_set]:
            if parameter is None:
                self.armed_too_early = 1

        return CommandBoolResponse(True,0)

    def handle_takeoff(self, msg):
        self.taken_off = 1
        self.takeoff_without_arm = 0
        if self.armed is None:
            self.takeoff_without_arm = 1
        return CommandTOLResponse(True, 0)

    def handle_landing(self, msg):
        self.taken_off = 0
        return CommandTOLResponse(True, 0)

    def handle_set_vel_FOR(self, msg):
        self.set_vel_FOR = 1
        return SetMavFrameResponse(True)

    def handle_set_gp_origin(self, msg):
        self.origin_set = 1


## A sample python unit test
class TestFsm(unittest.TestCase):

    def setUp(self) -> None:
        rospy.init_node("test_fsm", anonymous=True)
        self.mm = MavrosMock()

        rospy.Service("/mavros/set_mode",SetMode, self.mm.handle_set_mode)
        rospy.Service("/mavros/cmd/arming", CommandBool, self.mm.handle_arm)
        rospy.Service("/mavros/cmd/takeoff", CommandTOL, self.mm.handle_takeoff)
        rospy.Service("/mavros/cmd/land", CommandTOL, self.mm.handle_landing)
        rospy.Service("/mavros/setpoint_velocity/mav_frame", SetMavFrame, self.mm.handle_set_vel_FOR)

        rospy.Subscriber("/mavros/global_position/set_gp_origin", GeoPointStamped, self.mm.handle_set_gp_origin)

        self.fsm = fsmmod.Fsm()

    # def test_setup(self):
    #
    #     time.sleep(10)
    #     self.assertTrue(self.mm.origin_set is not None, "The initializaiton is not done.")

    def test_arm(self):
        time.sleep(5)
        self.assertEqual(self.mm.armed, 0, "The drone is armed before arming.")
        self.fsm.takeoff_handler(Empty)
        self.assertEqual(self.mm.armed, 1, "The drone is not armed after arming")
        self.assertEqual(self.mm.armed_too_early, 0, "We armed before calling some necesarry setupcalls.")
        self.assertEqual(self.mm.taken_off, 1, "We did not take off when we should have.")


if __name__ == '__main__':
    import rosunit
    import rostest
    #rosunit.unitrun(PKG, 'test_fsm_code_level', TestFsm)
    rostest.rosrun(PKG, 'test_range_filter_valid', TestFsm, sys.argv)