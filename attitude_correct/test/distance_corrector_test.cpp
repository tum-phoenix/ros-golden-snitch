//
// Created by henrik on 16.04.2020.
//


#include "attitude_correct/distance_corrector.h"
#include "gtest/gtest.h"
#include <cmath>
#include <algorithm>

// TODO: Check what happens when the range sensor reading is out of range.

unsigned int numOfRangeSensors = 8;

class DistanceCorrectorTest : public testing::Test {
protected:
    std::vector<double> ranges;
    std::array<double, 3> attitude;
    double altitude;
    std::vector<double> dirOfRangeSensors;
    std::vector<bool> noneDetected;


    virtual void SetUp() {
        ranges = {1, 2, 3, 3, 3, 1, 0.5, 0.8};
        attitude = {0, 0, 0};
        altitude = 5;
        dirOfRangeSensors = {0, M_PI/4, M_PI/2, 3*M_PI/4, M_PI, 5*M_PI/4, 3*M_PI/2, 7*M_PI/4};
        noneDetected = {false, false, false, false, false, false, false, false};
    }
};

TEST_F(DistanceCorrectorTest, OneIsOne) {
    ASSERT_EQ(1, 1);
}

TEST_F(DistanceCorrectorTest, SimpleTests) {
    std::vector<double> correctRanges{};
    std::vector<bool> seesFloor{};

    correctDistances(8, ranges, attitude, altitude, dirOfRangeSensors,
                     correctRanges, seesFloor);
    ASSERT_EQ(ranges, correctRanges);
    ASSERT_EQ(seesFloor, noneDetected);

    correctRanges = {};
    seesFloor = {};
    attitude = {0, 0, M_PI/2};
    correctDistances(8, ranges, attitude, altitude, dirOfRangeSensors,
                     correctRanges, seesFloor);
    ASSERT_EQ(ranges, correctRanges);
    ASSERT_EQ(seesFloor, noneDetected);


    std::vector<double> refRanges = correctRanges;
    correctRanges = {};
    seesFloor = {};
    attitude = {M_PI/18, 0, 0};
    correctDistances(8, ranges, attitude, altitude, dirOfRangeSensors,
                     correctRanges, seesFloor);
    ASSERT_GE(refRanges, correctRanges);
    for (int i = 0; i<numOfRangeSensors;i++){
        if (dirOfRangeSensors[i] !=  0 && dirOfRangeSensors[i] != M_PI){
            ASSERT_GT(refRanges[i], correctRanges[i]);
        }else{
            ASSERT_EQ(refRanges[i], correctRanges[i]);
        }
    }

    correctRanges = {};
    seesFloor = {};
    attitude = {0, M_PI/3, 0};
    altitude = 1;
    ranges[0] = 2.5;
    correctDistances(8, ranges, attitude, altitude, dirOfRangeSensors,
                     correctRanges, seesFloor);
    ASSERT_EQ(true, seesFloor[0]);
}

TEST_F(DistanceCorrectorTest, PreciseTests){
    std::vector<double> correctRanges{};
    std::vector<bool> seesFloor{};

    attitude = {0, M_PI/4, 0};
    correctDistances(8, ranges, attitude, altitude, dirOfRangeSensors,
                     correctRanges, seesFloor);
    ASSERT_EQ(cos(M_PI/4) * ranges[0], correctRanges[0]);

    attitude = {0, M_PI/4, M_PI/2};
    correctDistances(8, ranges, attitude, altitude, dirOfRangeSensors,
                     correctRanges, seesFloor);
    ASSERT_EQ(cos(M_PI/4) * ranges[2], correctRanges[2]);
}



int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}