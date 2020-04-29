//
// Created by henrik on 12.04.2020.
//

#include "../include/attitude_correct/callback_synchronizer.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/ros.h"
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <algorithm>

double sq(double x) {
    return x * x;
}

// Shamelessly stolen from https://stackoverflow.com/questions/14447338/converting-quaternions-to-euler-angles-problems-with-the-range-of-y-angle
std::array<double, 3> eulerFromQuat(geometry_msgs::Quaternion quat) {
    std::array<double, 3> res{};
    res[2] = (atan2(2.0 * (quat.x * quat.y + quat.z * quat.w), (sq(quat.x) - sq(quat.y) - sq(quat.z) + sq(quat.w))) *
              (180.0f / M_PI));// Z
    res[0] = (atan2(2.0 * (quat.y * quat.z + quat.x * quat.w), (-sq(quat.x) - sq(quat.y) + sq(quat.z) + sq(quat.w))) *
              (180.0f / M_PI)); // X
    res[1] = (asin(-2.0 * (quat.x * quat.z - quat.y * quat.w)) * (180.0f / M_PI)); // Y
    return res;
}

// Note that this might change the 'old' message.
teraranger_array::RangeArray getMessage(teraranger_array::RangeArray old,
                                        std::vector<double> ranges,
                                        unsigned int numOfRangeSensors) {
    for (int i = 0; i < numOfRangeSensors; i++) {
        old.ranges[i].range = ranges[i];
    }
    return old;
}

void CallbackSynchronizer::rangesCallback(teraranger_array::RangeArray msg) {
    std::vector<double> res{}; // = new std::vector<double>();
    for (int i = 0; i < msg.ranges.size(); i++) {
        res.push_back(msg.ranges[i].range);
    }
    std::vector<double> correctRanges{};
    std::vector<bool> seesFloor{}; // We do nothing with the detected floors for now, but we should soon.
    correctDistances(this->numOfRangeSensors, res, this->attitude, this->altitude, dirOfRangeSensors,
                     correctRanges, seesFloor);
    teraranger_array::RangeArray out = getMessage(msg, correctRanges, this->numOfRangeSensors);
    this->rangesOut.publish(out);
}

// Stolen from https://stackoverflow.com/questions/19152178/printing-an-stdarray
template<class T, std::size_t N>
std::ostream &operator<<(std::ostream &o, const std::array<T, N> &arr) {
    copy(arr.cbegin(), arr.cend(), std::ostream_iterator<T>(o, " "));
    return o;
}

void CallbackSynchronizer::attitudeCallback(geometry_msgs::PoseStamped msg) {
    ROS_INFO_STREAM("Received attitudes");
    std::array<double, 3> orientationEuler = eulerFromQuat(msg.pose.orientation);
    this->attitude = orientationEuler;
    std::cout << "The orientation in euler angles(XYZ)" << orientationEuler << '\n';

}

void CallbackSynchronizer::altitudeCallback(std_msgs::Float64 msg) {
    ROS_INFO_STREAM("Received altitude");
    this->altitude = msg.data;


}

CallbackSynchronizer::CallbackSynchronizer() {
    YAML::Node config = YAML::LoadFile("/home/henrik/catkin_ws2/src/phx-flight-ros-golden-snitch/physical.yaml");
    this->dirOfRangeSensors = config["direction_of_range_sensors"].as<std::vector<double>>();
    this->numOfRangeSensors = config["number_of_range_sensors"].as<unsigned int>();
    //Converts the direction of the range sensors to radians instead of degrees:
    std::for_each(this->dirOfRangeSensors.begin(), this->dirOfRangeSensors.end(), [](double &deg){deg*M_PI/180;});
    this->altitude = 10;
    this->attitude = {0, 0, 0,};

    std::string attitudeTopicName = "/mavros/local_position/pose";
    std::string altitudeTopicName = "/mavros/global_position/rel_alt";
    std::string rangesOutName = "phx_attitude_corrected_ranges";
    std::string rangesInName = "/multiflex_1/ranges_raw";

    ros::NodeHandle n;

    this->rangesOut = n.advertise<teraranger_array::RangeArray>(rangesOutName, 1);

    rangesIn = n.subscribe(rangesInName, 1, &CallbackSynchronizer::rangesCallback, this);
    attitude_sub = n.subscribe(attitudeTopicName, 1, &CallbackSynchronizer::attitudeCallback, this);
    altitude_sub = n.subscribe(altitudeTopicName, 1, &CallbackSynchronizer::altitudeCallback, this);
    ROS_INFO("Attitude_correct is now initialized");
    ros::spin();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "attitude_corrector");
    CallbackSynchronizer m = CallbackSynchronizer();
    return 0;
}