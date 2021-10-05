#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from numpy import random
position_human = Odometry()
position_drone = Odometry()
distance_human_drone = Odometry()

subscribed = None

def callback_human_pose(data):

    #rospy.loginfo(rospy.get_caller_id() + " usbl: position: x=%f y=%f", 
    #              data.pose.pose.position.x, 
    #              data.pose.pose.position.y)
    
    global position_human, subscribed
    subscribed = data.pose.pose.position.x # checks if already subcribed
    position_human = data

def callback_drone_pose(data):

    #rospy.loginfo(rospy.get_caller_id() + " usbl: position: x=%f y=%f", 
    #              data.pose.pose.position.x, 
    #              data.pose.pose.position.y)
    
    global position_drone, subscribed
    subscribed = data.pose.pose.position.x # checks if already subcribed
    position_drone = data

    
def subscriber():
    rospy.init_node('distance_calculator', anonymous=True)
    rospy.Subscriber("/human/ground_truth", Odometry, callback_human_pose)
    rospy.Subscriber("/iris/ground_truth", Odometry, callback_drone_pose)
    pub_distance_calculator = rospy.Publisher('distance_human_drone', Odometry, queue_size = 10)
    #rospy.spin()
    
    print 'Waiting to subscribe to sensor topics'
    print ' ... '
    
    while type(subscribed) is None and not rospy.is_shutdown():
        rospy.Rate(10).sleep()

    print 'Subscribed to topic'
    print ' '
    print 'Distance calculation is online.'
    
    freq = 30    # 1/secs 
    rate = rospy.Rate(freq)
    
    while not rospy.is_shutdown():
	distance_human_drone.pose.pose.position.x = position_human.pose.pose.position.x - position_drone.pose.pose.position.x;
	distance_human_drone.pose.pose.position.y = position_human.pose.pose.position.y - position_drone.pose.pose.position.y;
	distance_human_drone.pose.pose.position.z = position_human.pose.pose.position.z - position_drone.pose.pose.position.z;
        pub_distance_calculator.publish(distance_human_drone)
        
        rate.sleep()
    
if __name__=='__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
    
