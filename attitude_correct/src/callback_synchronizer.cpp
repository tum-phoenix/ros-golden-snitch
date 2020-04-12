//
// Created by henrik on 12.04.2020.
//

#include "callback_synchronizer.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/ros.h"
#include <cmath>

#include "distance_corrector.h"

// TODO: Get the real values and get this from somewhere else.
std::array<double, numOfRangeSensors> dirOfRangeSensors = {0, M_PI/4, M_PI/2, 3*M_PI/4, M_PI, 5*M_PI/4, 3*M_PI/2, 7*M_PI/4};

double sq(double x){
    return x*x;
}
// Shamelessly stolen from https://stackoverflow.com/questions/14447338/converting-quaternions-to-euler-angles-problems-with-the-range-of-y-angle
std::array<double,3> eulerFromQuat(geometry_msgs::Quaternion quat){
    std::array<double,3> res{};
    res[2] = (atan2(2.0 * (quat.x*quat.y + quat.z*quat.w),(sq(quat.x) - sq(quat.y) - sq(quat.z) + sq(quat.w))) * (180.0f/M_PI));// Z
    res[0] = (atan2(2.0 * (quat.y*quat.z + quat.x*quat.w),(-sq(quat.x) - sq(quat.y) + sq(quat.z) + sq(quat.w))) * (180.0f/M_PI)); // X
    res[1] = (asin(-2.0 * (quat.x*quat.z - quat.y*quat.w)) * (180.0f/M_PI)); // Y
    return res;
}


void CallbackSynchronizer::rangesCallback(teraranger_array::RangeArray msg) {
    std::array<double, numOfRangeSensors> res{}; // = new std::array<double, numOfRangeSensors>();
    for (int i=0;i<msg.ranges.size();i++) {
        res[i] = msg.ranges[i].range;
    }
    std::array<double, numOfRangeSensors> correctRanges{};
    std::array<bool, numOfRangeSensors> seesFloor{}; // We do nothing with the detected floors for now, but we should soon.
    correctDistances(res,this->attitude,this->altitude, dirOfRangeSensors, correctRanges, seesFloor);

}
// Stolen from https://stackoverflow.com/questions/19152178/printing-an-stdarray
template <class T, std::size_t N>
std::ostream& operator<<(std::ostream& o, const std::array<T, N>& arr){
    copy(arr.cbegin(), arr.cend(), std::ostream_iterator<T>(o, " "));
    return o;
}

void CallbackSynchronizer::attitudeCallback(geometry_msgs::PoseStamped msg) {
    std::array<double, 3> orientationEuler = eulerFromQuat(msg.pose.orientation);
    this->attitude = orientationEuler;
    std::cout << "The orientation in euler angles(XYZ)" << orientationEuler << '\n';

}

void CallbackSynchronizer::altitudeCallback(std_msgs::Float64 msg) {
    this->altitude = msg.data;
}


int main(int argc, char **argv){
    std::string attitudeTopicName  = "/mavros/local_position/pose";
    std::string altitudeTopicName = "/mavros/global_position/rel_alt";
    std::string rangesOutName = "phx_attitude_corrected_ranges";
    std::string rangesInName = "/multiflex_1/ranges_raw";

    ros::init(argc, argv, "attitude_corrector");
    ros::NodeHandle n;

    CallbackSynchronizer cs = CallbackSynchronizer();
    CallbackSynchronizer* csp = &cs;

    ros::Publisher rangesOut = n.advertise<teraranger_array::RangeArray>(rangesOutName, 1);
    ros::Subscriber rangesIn = n.subscribe(rangesInName, 1, &CallbackSynchronizer::rangesCallback, csp);
    ros::Subscriber attitude = n.subscribe(attitudeTopicName, 1, &CallbackSynchronizer::attitudeCallback, csp);
    ros::Subscriber altitude = n.subscribe(altitudeTopicName, 1, &CallbackSynchronizer::altitudeCallback, csp);
    ROS_INFO("Attitude_correct is now initialized");
    ros::spin();
    return 0;
}