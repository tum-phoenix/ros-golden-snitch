#!/usr/bin/env python3
import time

import mavros
import rospy
from mavros_msgs.srv import  CommandTOL, CommandBool, SetMode
from geographic_msgs.msg import GeoPointStamped, GeoPoint
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Header

OPERATION_POSITION = [-35.36294910983843, 149.16928445322435, 579.717312261560] # Latitude, Longtitude, Altitude TODO: Find the position of Munich

class MavrosUAV:
    def __init__(self):
        rospy.init_node('phx/gs_launcher', anonymous=False)

        set_origin_topic_name = "/mavros/global_position/set_gp_origin"
        setpoint_vel_topic_name = "/mavros/setpoint_velocity/cmd_vel_unstamped"
        set_mode_srvname = "/mavros/set_mode"
        arming_srv_name = "/mavros/cmd/arming"
        takeoff_srv_name = "/mavros/cmd/takeoff"
        landing_srv_name = "/mavros/cmd/land"

        self.set_origin_pub = rospy.Publisher(set_origin_topic_name, GeoPointStamped, queue_size=10)
        self.set_vel_pub = rospy.Publisher(setpoint_vel_topic_name, Twist, queue_size=2)

        print("Waiting for service setmode")
        rospy.wait_for_service(set_mode_srvname)
        self.setmode_srv = rospy.ServiceProxy(set_mode_srvname, SetMode)

        print("Waiting for service arming")
        rospy.wait_for_service(arming_srv_name)
        self.arm_srv = rospy.ServiceProxy(arming_srv_name, CommandBool)

        print("Waiting for takeoff service")
        rospy.wait_for_service(takeoff_srv_name)
        self.takeoff_srv = rospy.ServiceProxy(takeoff_srv_name, CommandTOL)

        print("Waiting for landing service")
        rospy.wait_for_service(landing_srv_name)
        self.land_srv = rospy.ServiceProxy(landing_srv_name, CommandTOL)


    def init_uav(self, arm=False, takeoff=False):
        print("Starting init_uav(). Atm we have to manualy accept every new command, but just remove the `input()` from the code to do this faster.")
        res_setMode = self.setmode_srv(0,'GUIDED')
        print("Res of setmode:", res_setMode)
        input()


        origin = GeoPointStamped()
        origin.header = Header() # TODO: Perhaps fill this with correct timestamp? It seems to work without in simulation.
        origin.position = GeoPoint()
        origin.position.latitude = OPERATION_POSITION[0]
        origin.position.longitude = OPERATION_POSITION[1]
        origin.position.altitude = OPERATION_POSITION[2]
        self.set_origin_pub.publish(origin)
        print("Published ",origin)

        if arm or takeoff:
            self.arm()
            if takeoff:
                self.takeoff()


    def arm(self):
        res_arm = self.arm_srv(True)
        print("Result of arming:", res_arm)
        input()

    # block: True if the call should block until we have reached the specified altitude.
    def takeoff(self, altitude=1, block=True):
        res_takeoff = self.takeoff_srv(min_pitch=0.0, yaw=0.0,latitude=OPERATION_POSITION[0], longitude=OPERATION_POSITION[1], altitude=altitude)
        print("Result of takeoff", res_takeoff)
        if block:
            time.sleep(5) # TODO: Replace this with a check for the height and see that it is >=0.9*altitude.

    # block is True if the call should block until the UAV is landed.
    def land(self, block=True):
        res_land = self.land_srv(min_pitch=0.0, yaw=0.0,latitude=OPERATION_POSITION[0], longitude=OPERATION_POSITION[1], altitude=1)
        print("Result of landing", res_land)

    # This is in the MAV frame.
    def set_vel_setpoint(self, dir, rot):
        new_vel = Twist(Vector3(*dir), Vector3(*rot))  # TODO: Should this be StapmedTwist instead?
        self.set_vel_pub.publish(new_vel)
        input("Just sent command to move")