//
// Created by henrik on 12.04.2020.
//

#ifndef ATTITUDE_CORRECT_CALLBACK_SYNCHRONIZER_H
#define ATTITUDE_CORRECT_CALLBACK_SYNCHRONIZER_H

#include <array>
#include "teraranger_array/RangeArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include "distance_corrector.h"


//const unsigned int numOfRangeSensors = 8;


class CallbackSynchronizer {
private:
    //std::array<double, numOfRangeSensors> ranges;
    std::array<double, 3> attitude;
    double altitude;

public:
    void rangesCallback(teraranger_array::RangeArray msg);
    void attitudeCallback(geometry_msgs::PoseStamped msg);
    void altitudeCallback(std_msgs::Float64 msg);
};


#endif //ATTITUDE_CORRECT_CALLBACK_SYNCHRONIZER_H
