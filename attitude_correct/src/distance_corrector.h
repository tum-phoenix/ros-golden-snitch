//
// Created by henrik on 04.04.2020.
//

#ifndef ATTITUDE_CORRECT_DISTANCE_CORRECTOR_H
#define ATTITUDE_CORRECT_DISTANCE_CORRECTOR_H

#include <array>
#include <vector>

const unsigned int numOfRangeSensors = 8;

/*
 * This function assumes level, flat floor, vertical walls and no other objects that does not conform to these assumptions.
 *  (It can still probably be used, but will give wrong results)
 * @param: attitude asumes for now that it is the roll, pitch, yaw euler angles in that order.
 * @param: dirOfRangeSensors contains the direction of the range sensors in degrees, where 0 is straight forward and increasing with direction of the clock.
 */
void correctDistances(
        std::array<double, numOfRangeSensors> ranges,
        std::array<double, 3> attitude,
        double altitude,
        std::array<double, numOfRangeSensors> dirOfRangeSensors,
        std::array<double, numOfRangeSensors> &correctRanges,
        std::array<bool, numOfRangeSensors> &seesFloor);

#endif //ATTITUDE_CORRECT_DISTANCE_CORRECTOR_H
