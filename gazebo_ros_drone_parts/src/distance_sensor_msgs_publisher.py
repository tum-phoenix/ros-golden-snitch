#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
@author: Martin Zimmermann

'''

import rospy

from sensor_msgs.msg import Range
from teraranger_array.msg import RangeArray


dist_00 = Range()
dist_01 = Range()
dist_02 = Range()
dist_03 = Range()
dist_04 = Range()
dist_05 = Range()
dist_06 = Range()
dist_07 = Range()


###############################################################################
## callback functions
###############################################################################

def callback_dist_00(data):
    global dist_00 
    dist_00 = data
    
def callback_dist_01(data):
    global dist_01 
    dist_01 = data
    
def callback_dist_02(data):
    global dist_02 
    dist_02 = data
    
def callback_dist_03(data):
    global dist_03 
    dist_03 = data
    
def callback_dist_04(data):
    global dist_04 
    dist_04 = data
    
def callback_dist_05(data):
    global dist_05 
    dist_05 = data
    
def callback_dist_06(data):
    global dist_06 
    dist_06 = data
    
def callback_dist_07(data):
    global dist_07 
    dist_07 = data
    
###############################################################################
## distance sensor measurement melting
###############################################################################
    
def distance_sensor_sub_pub():

    ###########################################################################
    ## init ros framework
    ###########################################################################
    
    ## subscriber node
    rospy.init_node('distance_sensor',anonymous=True)
    
    ## single sensor subscriber
    rospy.Subscriber("/iris/distance_sensor_00", Range, callback_dist_00)
    rospy.Subscriber("/iris/distance_sensor_01", Range, callback_dist_01)
    rospy.Subscriber("/iris/distance_sensor_02", Range, callback_dist_02)
    rospy.Subscriber("/iris/distance_sensor_03", Range, callback_dist_03)
    rospy.Subscriber("/iris/distance_sensor_04", Range, callback_dist_04)
    rospy.Subscriber("/iris/distance_sensor_05", Range, callback_dist_05)
    rospy.Subscriber("/iris/distance_sensor_06", Range, callback_dist_06)
    rospy.Subscriber("/iris/distance_sensor_07", Range, callback_dist_07)

    ## init publisher
    pub = rospy.Publisher('/iris/distance_sensor', RangeArray, queue_size=10)
    
    r = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        dist_ = RangeArray()
        ## define range array
        dist_.ranges = [dist_00, dist_01, dist_02, dist_03, dist_04, dist_05, dist_06, dist_07]
        pub.publish(dist_)
        r.sleep()



if __name__=='__main__':
    try:
        distance_sensor_sub_pub()
    except rospy.ROSInterruptException:
        pass
    


