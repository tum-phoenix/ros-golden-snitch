#!/usr/bin/env python3

import rospy
import mavrosInterface as mavrosInterface
from enum import Enum
from std_msgs.msg import Empty, Float64
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State as UAV_State
from sensor_msgs.msg import Imu



class States(Enum):
    IDLE = "IDLE"
    TAKING_OFF = "TAKING_OFF"
    FLYING = "FLYING"
    LANDING = "LANDING"

class Fsm:
    def __init__(self):
        self.state = States.IDLE
        self.TAKEOFF_HEIGHT = 1
        self.HEIGHT_FACTOR_NECESARRY = 0.8 # The persentage/100 of our target we have to reach before we accept it.

        self.ACCELERATION_TRESHOLD_FOR_BEING_UPSIDE_DOWN = -5
        self.NUM_OF_CONFIRMED_IMUS_MSGS_NEEDED = 7
        self.num_of_imu_msgs_upside_down = 0

        self.UAV = mavrosInterface.MavrosUAV(initUAV=True)

        self.land_sub = rospy.Subscriber("phx/land", Empty, self.land_handler)
        self.takeoff_sub = rospy.Subscriber("phx/takeoff", Empty, self.takeoff_handler)
        self.control_input_sub = rospy.Subscriber("phx/setpoint", Twist,)
        # self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.altitude_handler) # To check if we have reached the desired altitude to change state.
        self.uav_state_sub = rospy.Subscriber("/mavros/state", UAV_State, self.uav_state_handler)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_handler)
        self.alt_sub = rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.altitude_handler)

        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=2)





    def land_handler(self, empty):
        if self.state != States.LANDING:
            self.UAV.land(block=True,verbose=False)
            self.set_state(States.LANDING)

    def takeoff_handler(self, empty):
        if self.state == States.IDLE:
            self.UAV.arm()
            self.UAV.takeoff(self.TAKEOFF_HEIGHT, block=False)
            self.set_state(States.TAKING_OFF)

    def altitude_handler(self, alt):
        if self.state == States.TAKING_OFF:
            if alt.data > self.HEIGHT_FACTOR_NECESARRY * self.TAKEOFF_HEIGHT:
                self.set_state(States.FLYING)

        elif self.state == States.LANDING:
            pass
            # We try to detect landing by makeing ardupilot disarm automatically, and then detect that.
            # so = False
            # if so:
            #     self.UAV.disarm()
            #     self.new_state(States.IDLE)

    def control_handler(self, cmd_msg):
        if self.state == States.FLYING:
            self.vel_pub.publish(cmd_msg)


    def uav_state_handler(self, uav_state_msg):
        if self.state == States.LANDING:
            # We try to use the ardupilot landing detector. We need to configure the drone to disarm on landing, and then we detect landing by checking if it is armed.
            # TODO: Test that it works. Might be that we need to set PILOT_THR_BHV = 4, but it seems not like it.
            if not uav_state_msg.armed:
                self.UAV.set_mode("GUIDED")
                self.set_state(States.IDLE)

    def imu_handler(self, imu_msg):
        if self.state == States.FLYING:
            # TODO: Test that this detection does not give false detections, which will lead to crashes. This must be tested on the real drone!
            if imu_msg.linear_acceleration.z < self.ACCELERATION_TRESHOLD_FOR_BEING_UPSIDE_DOWN:
                self.num_of_imu_msgs_upside_down += 1
                if self.num_of_imu_msgs_upside_down >= self.NUM_OF_CONFIRMED_IMUS_MSGS_NEEDED:
                    # The player have grabed the drone and turned it upside down.
                    self.set_state(States.IDLE)
                    # self.UAV.disarm() # Test it without disarming first
                    rospy.logwarn("The drone is now detected to be grabbed, but we do not disarm it yet!")

            else:
                self.num_of_imu_msgs_upside_down = 0

    def set_state(self, nState : States, txtMsg=""):
        rospy.loginfo("Changing state from "+str(self.state)+ " to "+ str(nState)+". "+txtMsg)
        self.state = nState






if __name__ == '__main__':
    print("Starting fsm")
    rospy.init_node("phx_GS_launcher")
    fsm = Fsm()

    rospy.spin()