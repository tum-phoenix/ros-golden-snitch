#!/usr/bin/env python3

import mavros
import rospy
import mavrosInterface as mavrosInterface
from mavros_msgs.srv import CommandHome, CommandTOL, CommandBool, SetMode
from geographic_msgs.msg import GeoPointStamped, GeoPoint
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Header


OPERATION_POSITION = [-35.36294910983843, 149.16928445322435, 579.717312261560] # Latitude, Longtitude, Altitude TODO: Find the position of Munich

def main():

    # MT = TestMavros()
    # MT.init_uav()
    MI = mavrosInterface.MavrosUAV()
    MI.init_uav(verbose=True)
    # input("Uav initialized")

    MI.arm(True)
    # input("Armed")

    MI.takeoff(1, block=True,verbose=True)
    print("taken off")

    MI.set_vel_setpoint([1,0,0], [0,0,0])
    rospy.sleep(2)

    MI.land(False,True)



if __name__ == '__main__':
    main()

