/*! \file test_servohat.cpp
 */
#include <eros/Logger.h>
#include <gtest/gtest.h>
#include <ros_hats/Hat/ServoHat.h>
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
    for (uint8_t i = 1; i < (uint8_t)(ServoHat::HatModel::END_OF_LIST); ++i) {
        EXPECT_FALSE(ServoHat::HatModelString((ServoHat::HatModel)(i)) == "UNKNOWN");
        if ((ServoHat::HatModel)(i) != ServoHat::HatModel::UNKNOWN) {
            printf("\t[%d] Supported Hat Model: %s\n",
                   counter,
                   ServoHat::HatModelString((ServoHat::HatModel)(i)).c_str());
            counter++;
        }
    }
}
TEST(BasicTest, TestOperation_ServoHat) {
    printf(
        "NOTE: Executing this test DOES Require to be ran with a Hat Installed.  It SHOULD "
        "Only be ran on a Raspberry Pi.\n");
    {
        printf("Testing Adafruit Servo Hat.\n");
        Logger* logger = new Logger("DEBUG", "UnitTest_ServoHat");
        ServoHat hat(ServoHat::HatModel::ADAFRUIT_SERVOHAT_16CH);
        HatConfig _config("ServoHat1", "ServoHat", "Adafruit 16Ch Servo Hat", true);

        bool status = hat.init(logger, _config);
        EXPECT_TRUE(hat.get_diagnostic().level <= Level::Type::ERROR);

        printf("%s\n", hat.pretty(" ").c_str());
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
