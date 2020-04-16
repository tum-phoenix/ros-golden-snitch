//
// Created by henrik on 16.04.2020.
//


#include "attitude_correct/distance_corrector.h"
#include "gtest/gtest.h"
#include <cmath>

// TODO: Check what happens when the range sensor reading is out of range.

class DistaneCorrectorTest : public testing::Test {
protected:
    std::array<double, numOfRangeSensors> ranges;
    std::array<double, 3> attitude;
    double altitude;
    std::array<double, numOfRangeSensors> dirOfRangeSensors;


    virtual void SetUp() {
        ranges = {1,2, 3, 3, 3, 1, 0.5, 0.8};
        attitude = {0, 0, 0};
        altitude = 1;
        dirOfRangeSensors = {0, 45, 90, };
        dirOfRangeSensors = {0, M_PI/4, M_PI/2, 3*M_PI/4, M_PI, 5*M_PI/4, 3*M_PI/2, 7*M_PI/4};
    }
};

TEST_F(DistaneCorrectorTest, OneIsOne) {
    EXPECT_EQ(1, 1);
}

TEST_F(DistaneCorrectorTest, NumOfRangeSensorsSet) {
    EXPECT_GE(numOfRangeSensors,0);
}

TEST_F(DistaneCorrectorTest, SimpleTests) {
    std::array<double, numOfRangeSensors> correctRanges{};
    std::array<bool, numOfRangeSensors> seesFloor{};

    correctDistances(ranges, attitude, altitude,dirOfRangeSensors,correctRanges,seesFloor);
    EXPECT_EQ(ranges, correctRanges);

}

int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}