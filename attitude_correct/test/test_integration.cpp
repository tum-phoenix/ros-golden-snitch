//
// Created by henrik on 16.04.2020.
//


#include "gtest/gtest.h"
#include <ros/ros.h>
#include <cstdint>
#include "attitude_correct/callback_synchronizer.h"

#include <teraranger_array/RangeArray.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

// TODO: Check what happens when the range sensor reading is out of range.

//const int numOfRangeSensors = 8;

class CallbackContainer{
public:
    CallbackContainer() : count(0){ }

    uint32_t count;
    teraranger_array::RangeArray lastMsg;
    void cb(teraranger_array::RangeArray msg)
    {
        ROS_INFO_STREAM("Received a msg in the testter.");
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
    CallbackContainer cbc;
    virtual void SetUp() {
        cbc = CallbackContainer();

        float ranges[8] = {1, 2, 1.5, 0.2, 0.7, 1.3, 0.2, 0.5};

        range_msg = teraranger_array::RangeArray();
        range_msg.header = std_msgs::Header();
        range_msg.header.stamp = ros::Time::now();
        range_msg.ranges = std::vector<sensor_msgs::Range>();
        for (int i = 0; i< numOfRangeSensors;i++){
            sensor_msgs::Range rng = sensor_msgs::Range();
            range_msg.ranges.push_back(rng);
            range_msg.ranges[i].header = std_msgs::Header();
            range_msg.ranges[i].header.stamp = ros::Time::now();
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

TEST_F(DistanceCorrectorTest, IntegrationTest) {
    ROS_INFO("Starting integration test.");
//    CallbackSynchronizer m = CallbackSynchronizer(false);
    std::cout << "This is  where cout ends up.\n";
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        std::cout << "topic_" << it - master_topics.begin() << ": " << info.name << ", " << info.datatype << std::endl;
    }

    ros::Subscriber sub_ranges = n.subscribe<teraranger_array::RangeArray>("phx_attitude_corrected_ranges", 3, &CallbackContainer::cb, &cbc);
    ros::Publisher pub_ranges = n.advertise<teraranger_array::RangeArray>("/multiflex_1/ranges_raw", 3);
    ros::Publisher pub_attitude = n.advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 3);
    ros::Publisher pub_altitude = n.advertise<std_msgs::Float64>("/mavros/global_position/rel_alt", 3);

    ros::spinOnce();
//    for(unsigned long i = 0; i<2e9;i++){}
    ros::Duration(2).sleep();
    ros::WallDuration(2).sleep();

    ROS_INFO_STREAM("Number of ranges subscribers: " << pub_ranges.getNumSubscribers() << ". Topic: " << pub_ranges.getTopic() );
    ROS_INFO_STREAM("Number of ranges in publishers: " << sub_ranges.getNumPublishers() << ". Topic: " << sub_ranges.getTopic());
//    ROS_INFO_STREAM("Number of ranges in publishers in node under test: " << m.rangesIn.getNumPublishers() << ". Topic: " << m.rangesIn.getTopic());

    ASSERT_EQ(0, cbc.count);
    pub_ranges.publish(range_msg);
//    ros::spinOnce();
//    for(unsigned long i = 0; i<2e9;i++){}
    ros::Duration(0.1).sleep();
//    ros::WallDuration(2).sleep();
    ros::spinOnce();
    ASSERT_EQ(1, cbc.count);
    ASSERT_EQ(range_msg, cbc.lastMsg);

    pub_altitude.publish(alt_msg);
    pub_attitude.publish(att_msg);
    pub_ranges.publish(range_msg);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    ASSERT_EQ(2, cbc.count);
    range_msg.header.seq = cbc.lastMsg.header.seq;
    ASSERT_EQ(range_msg, cbc.lastMsg);




}


int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "int_test_attitude_correct");

//    main(argc, argv);
    return RUN_ALL_TESTS();
}