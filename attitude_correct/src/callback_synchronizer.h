//
// Created by henrik on 12.04.2020.
//

#ifndef ATTITUDE_CORRECT_CALLBACK_SYNCHRONIZER_H
#define ATTITUDE_CORRECT_CALLBACK_SYNCHRONIZER_H

#include <array>
#include "teraranger_array/RangeArray.h"
#include "geometry_msgs/PoseStamped.h"


const unsigned int numOfRangeSensors = 8;


class CallbackSynchronizer {
private:
    std::array<double, numOfRangeSensors> ranges;
public:
    void rangesCallback(teraranger_array::RangeArray msg);
    void attitudeCallback(geometry_msgs::PoseStamped msg);
};


#endif //ATTITUDE_CORRECT_CALLBACK_SYNCHRONIZER_H
