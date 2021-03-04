/*! \file test_definitions.cpp
 */
#include <eros/Logger.h>
#include <gtest/gtest.h>
#include <ros_hats/Channel/PWMOutputChannel.h>
#include <ros_hats/Hat/PWMHat.h>
#include <ros_hats/Port/PWMOutputPort.h>
#include <ros_hats/ROSHATS_Definitions.h>
#include <stdio.h>
#include <sys/time.h>
double time_diff(struct timeval A, struct timeval B) {
    double t1 = A.tv_sec + A.tv_usec / 1000000.0;
    double t2 = B.tv_sec + B.tv_usec / 1000000.0;
    return t2 - t1;
}
TEST(BasicTest, Definitions) {
    printf("All Currently Supported Relay Hats:\n");
    int counter = 0;
    for (uint8_t i = 1; i < (uint8_t)(PWMHat::HatModel::END_OF_LIST); ++i) {
        EXPECT_FALSE(PWMHat::HatModelString((PWMHat::HatModel)(i)) == "UNKNOWN");
        if ((PWMHat::HatModel)(i) != PWMHat::HatModel::UNKNOWN) {
            printf("\t[%d] Supported Hat Model: %s\n",
                   counter,
                   PWMHat::HatModelString((PWMHat::HatModel)(i)).c_str());
            counter++;
        }
    }
}
TEST(BasicTest, TestOperation_PWMHat) {
    printf(
        "NOTE: Executing this test DOES Require to be ran with a Hat Installed.  It SHOULD "
        "Only be ran on a Raspberry Pi.\n");
    {
        printf("Testing Adafruit Servo Hat.\n");
        Logger* logger = new Logger("DEBUG", "UnitTest_PWMHat");
        PWMHat hat(PWMHat::HatModel::ADAFRUIT_SERVOHAT_16CH);
        HatConfig _config("PWMHat1", "PWMHat", "Adafruit 16Ch Servo Hat", true);

        bool status = hat.init(logger, _config);
        EXPECT_TRUE(hat.get_diagnostic().level <= Level::Type::ERROR);

        printf("%s\n", hat.pretty().c_str());
        if (status == true) {
            logger->log_notice("Continuing tests...");
        }
        else {
            logger->log_warn("Not running anymore tests.");
            return;
        }
    }
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
