//
// Created by henrik on 12.04.2020.
//

#include "callback_synchronizer.h"
#include "sensor_msgs/Range.h"
#include "ros/ros.h"

void CallbackSynchronizer::rangesCallback(teraranger_array::RangeArray msg) {
    std::array<double, numOfRangeSensors> res{}; // = new std::array<double, numOfRangeSensors>();
    for (int i=0;i<msg.ranges.size();i++) {
        res[i] = msg.ranges[i].range;
    }
    this->ranges = res;
}

void CallbackSynchronizer::attitudeCallback(geometry_msgs::PoseStamped msg) {

}


int main(int argc, char **argv){
    std::string attitudeTopicName  = "/mavros/local_position/pose";
    std::string rangesOutName = "phx_attitude_corrected_ranges";
    std::string rangesInName = "/multiflex_1/ranges_raw";

    ros::init(argc, argv, "attitude_corrector");
    ros::NodeHandle n;

    CallbackSynchronizer cs = CallbackSynchronizer();
    CallbackSynchronizer *csp = &cs;

    ros::Publisher rangesOut = n.advertise<teraranger_array::RangeArray>(rangesOutName, 1);
    ros::Subscriber rangesIn = n.subscribe(rangesInName, 1, &CallbackSynchronizer::rangesCallback, csp);
    ros::Subscriber attitude = n.subscribe(attitudeTopicName, 1, &CallbackSynchronizer::attitudeCallback, csp);
    ros::spin();
    return 0;
}