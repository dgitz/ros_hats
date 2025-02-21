/*! \file test_GPSHatDriver.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "../../include/GPSHatDriver.h"
using namespace eros;
using namespace ros_hats;
using namespace ros_hats;
TEST(BasicTest, TestDefinitions) {
    for (uint8_t i = 0; i <= (uint8_t)GPSHatDriver::StatusType::END_OF_LIST; ++i) {
        if ((i == (uint8_t)GPSHatDriver::StatusType::UNKNOWN) ||
            (i == (uint8_t)GPSHatDriver::StatusType::END_OF_LIST)) {
            EXPECT_EQ(GPSHatDriver::StatusTypeString((GPSHatDriver::StatusType)i), "UNKNOWN");
        }
        else {
            EXPECT_NE(GPSHatDriver::StatusTypeString((GPSHatDriver::StatusType)i), "UNKNOWN");
        }
        EXPECT_NE(GPSHatDriver::get_level((GPSHatDriver::StatusType)i), eros::Level::Type::UNKNOWN);
        EXPECT_NE(GPSHatDriver::get_level((GPSHatDriver::StatusType)i),
                  eros::Level::Type::END_OF_LIST);
    }
    for (uint8_t i = 0; i <= (uint8_t)GPSHatDriver::FixType::END_OF_LIST; ++i) {
        if ((i == (uint8_t)GPSHatDriver::FixType::UNKNOWN) ||
            (i == (uint8_t)GPSHatDriver::FixType::END_OF_LIST)) {
            EXPECT_EQ(GPSHatDriver::FixTypeString((GPSHatDriver::FixType)i), "UNKNOWN");
        }
        else {
            EXPECT_NE(GPSHatDriver::FixTypeString((GPSHatDriver::FixType)i), "UNKNOWN");
        }
    }
}
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestGPSHatDriver");
    GPSHatDriver SUT;
    SUT.init(logger);

    double timeToRun = 10.0;
    double dt = 0.1;
    double timer = 0.0;
    while (timer <= timeToRun) {
        EXPECT_TRUE(SUT.update(dt));
        SUT.get_gps_data();

        logger->log_debug(SUT.pretty());
        timer += dt;
    }

    // delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
