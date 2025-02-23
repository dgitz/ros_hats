/*! \file test_MockServoHatDriver.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "MockServoHatDriver.h"
using namespace eros;
using namespace ros_hats;
TEST(BasicTest, FailureScenarios) {
    Logger* logger = new Logger("DEBUG", "UnitTestMockServoHatDriver");
    IServoHatDriver* SUT = new (MockServoHatDriver);
    SUT->init(logger);
    EXPECT_GT(SUT->get_channels().size(), 0);
    logger->log_debug(SUT->pretty());

    // Invalid Pin
    EXPECT_FALSE(SUT->setServoValue(-1, 0));

    delete logger;
    delete SUT;
}

TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestMockServoHatDriver");
    IServoHatDriver* SUT = new (MockServoHatDriver);
    SUT->init(logger);
    EXPECT_GT(SUT->get_channels().size(), 0);
    logger->log_debug(SUT->pretty());

    double timeToRun = 10.0;
    double dt = 0.1;
    double timer = 0.0;
    uint16_t counter = IServoHatDriver::MIN_SERVO_VALUE;
    while (timer <= timeToRun) {
        EXPECT_TRUE(SUT->update(dt));

        logger->log_debug(SUT->pretty("simple"));
        EXPECT_TRUE(SUT->setServoValue(0, counter));
        counter += 10;
        timer += dt;
    }

    delete logger;
    delete SUT;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
