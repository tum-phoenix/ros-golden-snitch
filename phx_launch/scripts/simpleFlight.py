#!/usr/bin/env python3

import mavros
import rospy
import mavrosInterface as mavrosInterface
from mavros_msgs.srv import CommandHome, CommandTOL, CommandBool, SetMode
from geographic_msgs.msg import GeoPointStamped, GeoPoint
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Header


OPERATION_POSITION = [-35.36294910983843, 149.16928445322435, 579.717312261560] # Latitude, Longtitude, Altitude TODO: Find the position of Munich

def printRestult(sucsess: bool, result: int):
    print("Sucsess:", sucsess, ", result:", result)


class TestMavros:
    def __init__(self):
        rospy.init_node('testMavros', anonymous=False)

        self.set_origin_pub = rospy.Publisher("/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10)
        self.set_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=2)
        print("Waiting for service setmode")
        rospy.wait_for_service("/mavros/set_mode")
        self.setmode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        print("Waiting for service sethome")
        rospy.wait_for_service('/mavros/cmd/set_home')
        self.set_home_srv = rospy.ServiceProxy("/mavros/cmd/set_home", CommandHome)
        print("Waiting for service arming")
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        print("Waiting for takeoff service")
        rospy.wait_for_service("/mavros/cmd/takeoff")
        self.takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        print("Waiting for landing service")
        rospy.wait_for_service("/mavros/cmd/land")
        self.land_srv = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)


    def init_uav(self):
        print("Starting init_uav(). Atm we have to manualy accept every new command, but just remove the `input()` from the code to do this faster.")
        res_setMode = self.setmode_srv(0,'GUIDED')
        print("Res of setmode:", res_setMode)
        input()


        origin = GeoPointStamped()
        origin.header = Header() # TODO: Perhaps fill this with correct timestamp? It seems to work without in simulation.
        origin.position = GeoPoint()
        origin.position.latitude = OPERATION_POSITION[0]  # TODO: Find the position of Phoneix HQ
        origin.position.longitude = OPERATION_POSITION[1]
        origin.position.altitude = OPERATION_POSITION[2]
        self.set_origin_pub.publish(origin)
        print("Published ",origin)
        input()

        res_arm = self.arm_srv(True)
        print("Result of arming:", res_arm)
        input()

        res_takeoff = self.takeoff_srv(min_pitch=0.0, yaw=0.0,latitude=OPERATION_POSITION[0], longitude=OPERATION_POSITION[1], altitude=1)
        print("Result of takeoff", res_takeoff)
        input()

        new_vel = Twist(Vector3(1,0,0), Vector3(0,0,0)) # TODO: Should this be StapmedTwist instead?
        self.set_vel_pub.publish(new_vel)
        input("Just sent command to move")

        res_land = self.land_srv(min_pitch=0.0, yaw=0.0,latitude=OPERATION_POSITION[0], longitude=OPERATION_POSITION[1], altitude=1)
        print("Result of landing", res_land)







def main():
    MI = mavrosInterface.MavrosUAV()
    MI.init_uav()
    input("Uav initialized")

    MI.arm(True)
    input("Armed")

    MI.takeoff(1, False)
    print("taken off")

    MI.set_vel_setpoint([1,0,0], [0,0,0])
    rospy.sleep(2)

    MI.land(False,True)



if __name__ == '__main__':
    main()
