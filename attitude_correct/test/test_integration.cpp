//
// Created by henrik on 16.04.2020.
//


#include "gtest/gtest.h"
#include <ros/ros.h>

#include <teraranger_array/RangeArray.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

// TODO: Check what happens when the range sensor reading is out of range.

const int numOfRangeSensors = 8;

class CallbackContainer{
    CallbackContainer() : count(0){ }

public:
    uint32_t count;
    teraranger_array::RangeArray lastMsg;
    void cb(teraranger_array::RangeArray msg)
    {
        ++count;
        lastMsg = msg;
    }
};

class DistanceCorrectorTest : public testing::Test {
protected:
    teraranger_array::RangeArray range_msg;
    geometry_msgs::PoseStamped att_msg;
    std_msgs::Float64 alt_msg;
    ros::NodeHandle n;
    CallbackContainer *cbc;
    virtual void SetUp() {
        float ranges[8] = {1, 2, 1.5, 0.2, 0.7, 1.3, 0.2, 0.5};


        range_msg = teraranger_array::RangeArray();
        range_msg.header = std_msgs::Header();
        range_msg.header.stamp = ros::Time::now();
        for (int i = 0; i< numOfRangeSensors;i++){
            range_msg.ranges[i] = sensor_msgs::Range();
            range_msg.ranges[i].header = range_msg.header;
            range_msg.ranges[i].field_of_view = 0.1;
            range_msg.ranges[i].max_range = 2.3;
            range_msg.ranges[i].min_range = 0.1;
            range_msg.ranges[i].radiation_type = range_msg.ranges[i].INFRARED;
            range_msg.ranges[i].range = ranges[i];
        }

        att_msg = geometry_msgs::PoseStamped();
        att_msg.header = std_msgs::Header();
        att_msg.header.stamp = ros::Time::now();
        att_msg.pose = geometry_msgs::Pose();
        att_msg.pose.position = geometry_msgs::Point();
        att_msg.pose.position.x = 0;
        att_msg.pose.position.y = 0;
        att_msg.pose.position.z = 0;
        att_msg.pose.orientation = geometry_msgs::Quaternion();
        att_msg.pose.orientation.x = 0;
        att_msg.pose.orientation.y = 0;
        att_msg.pose.orientation.z = 0;
        att_msg.pose.orientation.w = 1;

        alt_msg = std_msgs::Float64();
        alt_msg.data = 2.0;

    }




};

TEST_F(DistanceCorrectorTest, OneIsOne) {
    ASSERT_EQ(1, 1);
}

TEST_F(DistanceCorrectorTest, NumOfRangeSensorsSet) {
    ros::Subscriber sub_ranges = n.subscribe<teraranger_array::RangeArray>("phx_attitude_corrected_ranges", 1, &CallbackContainer::cb, cbc);
    ros::Publisher pub_ranges = n.advertise<teraranger_array::RangeArray>("/multiflex_1/ranges_raw", 1);
    ros::Publisher pub_attitude = n.advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1);
    ros::Publisher pub_altitude = n.advertise<std_msgs::Float64>("/mavros/global_position/rel_alt", 1);

    pub_ranges.publish(range_msg);
    ros::Duration(0.1).sleep();
    ASSERT_EQ(1, cbc->count);
    ASSERT_EQ(range_msg, cbc->lastMsg);

    pub_altitude.publish(alt_msg);
    pub_attitude.publish(att_msg);
    pub_ranges.publish(range_msg);
    ros::Duration(0.1).sleep();
    ASSERT_EQ(2, cbc->count);
    ASSERT_EQ(range_msg, cbc->lastMsg);


}


int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "int_test_attitude_correct");
    return RUN_ALL_TESTS();
}