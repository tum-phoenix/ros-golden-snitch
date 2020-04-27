#!/usr/bin/env python3
import time

import mavros
import rospy
import rospkg
from mavros_msgs.srv import  CommandTOL, CommandBool, CommandBoolRequest, SetMode, SetMavFrame
from geographic_msgs.msg import GeoPointStamped, GeoPoint
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from mavros_msgs.msg import State as UAV_State
import numpy as np
import yaml


class MavrosUAV:
    def __init__(self, block = True, initUAV=False):
        rospack = rospkg.RosPack()
        with open(rospack.get_path("phx_launch")+"/../physical.yaml") as f:
            mechanical_config = yaml.load(f, Loader=yaml.FullLoader)
        self.OPERATION_POSITION = mechanical_config["operation_position"]

        set_origin_topic_name = "/mavros/global_position/set_gp_origin"
        setpoint_vel_topic_name = "/mavros/setpoint_velocity/cmd_vel_unstamped"
        setpoint_pos_topic_name = "/mavros/setpoint_position/local"
        set_mode_srvname = "/mavros/set_mode"
        arming_srv_name = "/mavros/cmd/arming"
        takeoff_srv_name = "/mavros/cmd/takeoff"
        landing_srv_name = "/mavros/cmd/land"
        ref_frame_vel_srv_name = "/mavros/setpoint_velocity/mav_frame"

        self.uav_state = 0
        self.currentPose = Pose()

        self.set_origin_pub = rospy.Publisher(set_origin_topic_name, GeoPointStamped, queue_size=10)
        self.set_vel_pub = rospy.Publisher(setpoint_vel_topic_name, Twist, queue_size=2)
        self.set_pos_pub = rospy.Publisher(setpoint_pos_topic_name, PoseStamped, queue_size=2)

        self.uav_state_sub = rospy.Subscriber("/mavros/state", UAV_State, self.__uav_state_handler)
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.__pose_handler)

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

        print("Waiting for reference frame service")
        rospy.wait_for_service(ref_frame_vel_srv_name)
        self.set_ref_frame_srv = rospy.ServiceProxy(ref_frame_vel_srv_name, SetMavFrame)

        rospy.loginfo("All (necessary) services are found.")


        if initUAV:
            self.init_uav()

    def __uav_state_handler(self, state_msg):
        self.uav_state = state_msg.system_status # It did not work

    def __pose_handler(self, pose_msg):
        self.currentPose = pose_msg.pose


    def init_uav(self, arm=False, takeoff=False, verbose=False):
        if verbose:
            print("Starting init_uav(). Atm we have to manualy accept every new command, but just remove the `input()` from the code to do this faster.")
        res_setMode = self.setmode_srv(0,'GUIDED')
        if verbose:
            print("Res of setmode:", res_setMode)

        res_set_ref_mode = self.set_ref_frame_srv(1) # FRAME_BODY_NED = 8 works for velocities
        if verbose:
            print("Res of set_ref_frame", res_set_ref_mode)

        rospy.sleep(1)
        print("Waiting for ekf initialisation")
        origin = GeoPointStamped()
        origin.header = Header()
        origin.header.stamp = rospy.Time.now()
        origin.position = GeoPoint()
        origin.position.latitude = self.OPERATION_POSITION[0]
        origin.position.longitude = self.OPERATION_POSITION[1]
        origin.position.altitude = self.OPERATION_POSITION[2]
        self.set_origin_pub.publish(origin)
        if verbose:
            print("Published ",origin)

        while(True):
            res   = self.arm()
            if res.success:
                break
            rospy.sleep(1)
        self.disarm()
        # input("Ekf initialized?")
        #time.sleep(40) # For some reason, it does not work if we use rospy.sleep(...)

        if arm or takeoff:
            self.arm()
            if takeoff:
                self.takeoff()

        print("Uav is now initialized. We are done with waiting.")

    def set_mode(self,mode):
        acceptedModes = ["GUIDED"]
        if mode in acceptedModes:
            return self.setmode_srv(0,mode)
        else:
            print("MOde not supported:", mode)
            return -1
    def arm(self, verbose=False):
        success = self.arm_srv(True)
        if verbose:
            print("Result of arming:", success)
        return success

    def disarm(self, verbose=False):
        success = self.arm_srv(False)
        if verbose:
            print("Result of disarming:", success)
        return success

    # block: True if the call should block until we have reached the specified altitude.
    def takeoff(self, altitude=1, block=True, verbose=False):
        # TODO: Check if armed
        self.arm(verbose)

        success = self.takeoff_srv(min_pitch=0.0, yaw=0.0,latitude=self.OPERATION_POSITION[0], longitude=self.OPERATION_POSITION[1], altitude=altitude)
        if verbose:
            print("Result of takeoff", success)
        if block:
            rospy.sleep(10) # TODO: Replace this with a check for the height and see that it is >=0.9*altitude.
        return success

    # block is True if the call should block until the UAV is landed.
    def land(self, block=True, verbose=False):
        success = self.land_srv(min_pitch=0.0, yaw=0.0,latitude=self.OPERATION_POSITION[0], longitude=self.OPERATION_POSITION[1], altitude=1)
        if verbose:
            print("Result of landing", success)
        return success

    def set_vel_setpoint(self, dir, rot):
        new_vel = Twist(Vector3(*dir), Vector3(*rot))  # TODO: Should this be StapmedTwist instead?
        self.set_vel_pub.publish(new_vel)
        print("Sendt setpoint ", new_vel)

    def set_pos_setpoint(self, relpos, relrot):
        # print("Current pos:", self.currentPose.position)
        # print("Rel pos:", relpos)
        pos = [relpos[0] + self.currentPose.position.x, relpos[1] + self.currentPose.position.y, relpos[2] + self.currentPose.position.z]
        newPose = PoseStamped()
        newPose.header = Header()
        newPose.header.stamp = rospy.Time.now()
        newPose.pose = Pose()
        newPose.pose.position = Point(*pos)
        relquat = euler_to_quaternion(*relrot)
        curquat = [self.currentPose.orientation.w, self.currentPose.orientation.x, self.currentPose.orientation.y, self.currentPose.orientation.z]
        quat = quaternion_multiply(curquat, relquat)
        newPose.pose.orientation = Quaternion(quat[1], quat[2], quat[3], quat[0])
        self.set_pos_pub.publish(newPose)

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
        yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
        yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)

    return [qw, qx, qy, qz]

def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)


