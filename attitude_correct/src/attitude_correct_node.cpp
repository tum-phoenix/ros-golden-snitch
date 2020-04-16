#include <iostream>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "teraranger_array/RangeArray.h"

#include "../include/attitude_correct/callback_synchronizer.h"

void attitudeCallback(geometry_msgs::PoseStamped msg){

}

void rangesCallback(teraranger_array::RangeArray msg){

}

int main(int argc, char **argv){
    std::cout << "Attutude_correct_node is now running. \n";
    std::string attitudeTopicName  = "/mavros/local_position/pose";
    std::string rangesOutName = "phx_attitude_corrected_ranges";
    std::string rangesInName = "/multiflex_1/ranges_raw";

    ros::init(argc, argv, "attitude_corrector");
    ros::NodeHandle n;
    ros::Publisher rangesOut = n.advertise<teraranger_array::RangeArray>(rangesOutName, 1);
    ros::Subscriber rangesIn = n.subscribe(rangesInName, 1, rangesCallback);
    ros::Subscriber attitude = n.subscribe(attitudeTopicName, 1, attitudeCallback);
    LOG_INFO("attitude_correct_node initialized, now spinning.\n");
    ros::spin();
    return 0;
}