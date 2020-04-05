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
    rospy.init_node("Simple_flight_node", anonymous=False)
    MI = mavrosInterface.MavrosUAV()
    MI.init_uav(verbose=True)
    # input("Uav initialized")

    MI.arm(True)
    # input("Armed")

    MI.takeoff(1, block=True,verbose=True)
    print("taken off")
    try:
        while(True):
            ans = input("Where do you want to fly?")
            if ans == 'q':
                break
            elif ans == 's':  # Stop
                MI.set_vel_setpoint([0, 0, 0], [0, 0, 0])
            elif ans[0] == 'l': # Move linearly
                MI.set_vel_setpoint([float(ans[1]),float(ans[2]),float(ans[3])], [0,0,0])
            elif ans[0] == 'r': # Rotate
                MI.set_vel_setpoint([0, 0, 0,], [float(ans[1]), float(ans[2]), float(ans[3])])
    except IndexError:
        print("Invalid input, landing")

    MI.land(False,True)



if __name__ == '__main__':
    main()

