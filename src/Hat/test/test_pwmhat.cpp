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
TEST(BasicTest, TestOperation_RelayHat) {
    //#ifdef __arm__
    printf(
        "NOTE: Executing this test DOES Require to be ran with a Hat Installed.  It SHOULD "
        "Only be ran on a Raspberry Pi.\n");
    {
        printf("Testing Adafruit Servo Hat.\n");
        Logger* logger = new Logger("DEBUG", "UnitTest_PWMHat");
        PWMHat hat(PWMHat::HatModel::ADAFRUIT_SERVOHAT_16CH);
        EXPECT_TRUE(hat.init(logger, "PWMHat"));
        EXPECT_TRUE(hat.get_diagnostic().level <= Level::Type::NOTICE);

        printf("%s\n", hat.pretty().c_str());
        /*
                int v = 0;
                for (int i = 0; i < 10; ++i) {
                    EXPECT_TRUE(hat.update_pin("20", v) !=
                                ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND);
                    usleep(500000);
                    EXPECT_TRUE(hat.get_diagnostic().level <= Level::Type::NOTICE);
                    if (v == 0) {
                        v = 1;
                    }
                    else {
                        v = 0;
                    }
                }

                EXPECT_TRUE(hat.update_pin("20", 0) !=
                            ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND);
                usleep(5000000);
                EXPECT_TRUE(hat.update_pin("20", 1) !=
                            ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND);
                usleep(5000000);
                EXPECT_TRUE(hat.update_pin("20", 0) !=
                            ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND);
                usleep(5000000);
                EXPECT_TRUE(hat.get_diagnostic().level <= Level::Type::NOTICE);
        */
    }
    //#else
    // printf("[WARN]: Not running tests as this is only supported on Raspberry Pi's.\n");
    //#endif
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
