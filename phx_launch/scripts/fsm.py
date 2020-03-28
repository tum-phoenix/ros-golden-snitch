#!/usr/bin/env python3

import rospy
import scripts.mavrosInterface as mavrosInterface
from enum import Enum
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State as UAV_State
from sensor_msgs.msg import Imu



class States(Enum):
    IDLE = 0
    TAKING_OFF = 1
    FLYING = 2
    LANDING = 3

class Fsm:
    def __init__(self):
        self.state = States.IDLE
        self.UAV = mavrosInterface.MavrosUAV(initUAV=True)

        self.land_sub = rospy.Subscriber("phx/land", Empty, self.land_handler)
        self.takeoff_sub = rospy.Subscriber("phx/takeoff", Empty, self.takeoff_handler)
        self.control_input_sub = rospy.Subscriber("phx/setpoint", Twist,)
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.altitude_handler) # To check if we have reached the desired altitude to change state.
        self.uav_state_sub = rospy.Subscriber("/mavros/state", UAV_State, self.uav_state_handler)
        self.imu_pub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_handler)

        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=2)


        self.ACCELERATION_TRESHOLD_FOR_BEING_UPSIDE_DOWN = 5
        self.NUM_OF_CONFIRMED_IMUS_MSGS_NEEDED = 7
        self.num_of_imu_msgs_upside_down = 0


    def land_handler(self):
        if self.state != States.LANDING:
            self.UAV.land(block=True,verbose=False)
            self.state = States.LANDING

    def takeoff_handler(self):
        if self.state == States.IDLE:
            self.UAV.arm()
            self.UAV.takeoff(1,block=True)
            self.state = States.TAKING_OFF

    def altitude_handler(self, pose):
        if self.state == States.TAKING_OFF:
            # TODO: Check if we have reached the desired altitude
            so = False
            if so:
                self.state = States.FLYING
            pass
        elif self.state == States.LANDING:
            # We try to detect landing by makeing ardupilot disarm automatically, and then detect that.
            # so = False
            # if so:
            #     self.UAV.disarm()
            #     self.state = States.IDLE

    def control_handler(self, cmd_msg):
        if self.state == States.FLYING:
            self.vel_pub.publish(cmd_msg)


    def uav_state_handler(self, uav_state_msg):
        if self.state == States.LANDING:
            # We try to use the ardupilot landing detector. We need to configure the drone to disarm on landing, and then we detect landing by checking if it is armed.
            # TODO: Configure ardupilot to disarm on landing and test that it works.
            if not uav_state_msg.armed:
                self.state = States.IDLE

    def imu_handler(self, imu_msg):
        if self.state == States.FLYING:
            # TODO: Test that this detection does not give false detections, which will lead to crashes. This must be tested on the real drone!
            if imu_msg.linear_acceleration[3] > self.ACCELERATION_TRESHOLD_FOR_BEING_UPSIDE_DOWN:
                self.num_of_imu_msgs_upside_down += 1
                if self.num_of_imu_msgs_upside_down >= self.NUM_OF_CONFIRMED_IMUS_MSGS_NEEDED:
                    # The player have grabed the drone and turned it upside down.
                    self.state = States.IDLE
                    # self.UAV.disarm() # Test it without disarming first
                    rospy.logwarn("The drone is now detected to be grabbed, but we do not disarm it yet!")

            else:
                self.num_of_imu_msgs_upside_down = 0







if __name__ == '__main__':
    rospy.init_node("phx_GS_launcher")
    fsm = Fsm()

    rospy.spin()