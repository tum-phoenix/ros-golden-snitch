//
// Created by henrik on 04.04.2020.
//

#include "distance_corrector.h"




void correctDistances(
        std::array<double, numOfRangeSensors> ranges,
        std::vector<double> attitude,
        double altitude,
        std::array<double, numOfRangeSensors> dirOfRangeSensors,
        std::array<double, numOfRangeSensors> &correctRanges,
        std::array<bool, numOfRangeSensors> &seesFloor){

    // std::array<double, numOfRangeSensors> yawRotatedRanges{};
    for(int i = 0; i<numOfRangeSensors;i++){
        double yawRotatedRangeDir = attitude[2] + dirOfRangeSensors[i];
        

    }
    return;
}