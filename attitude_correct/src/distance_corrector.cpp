//
// Created by henrik on 04.04.2020.
//

#include "../include/attitude_correct/distance_corrector.h"
#include <cmath>


void correctDistances(
        std::array<double, numOfRangeSensors> ranges,
        std::array<double, 3> attitude,
        double altitude,
        std::array<double, numOfRangeSensors> dirOfRangeSensors,
        std::array<double, numOfRangeSensors> &correctRanges,
        std::array<bool, numOfRangeSensors> &seesFloor) {

    for (int i = 0; i < numOfRangeSensors; i++) {
        double yawRotatedRangeDir = attitude[2] + dirOfRangeSensors[i];
        // Decompose the direction into x and y. (The axes of the roll and pitch we get)
        double x_dir = cos(yawRotatedRangeDir);
        double y_dir = sin(yawRotatedRangeDir);

        // Check if it hits the floor
        double max_inc = x_dir * attitude[1] + y_dir * attitude[0];
        seesFloor[i] = ranges[i] * sin(max_inc) > altitude;

        // Calculate correct ranges
        double correct_dist_x = x_dir * ranges[i] * cos(attitude[1]);
        double correct_dist_y = y_dir * ranges[i] * cos(attitude[0]);
        correctRanges[i] = sqrt(correct_dist_x * correct_dist_x + correct_dist_y * correct_dist_y);
    }
}