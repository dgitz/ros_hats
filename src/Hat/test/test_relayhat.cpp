/*! \file test_definitions.cpp
 */
#include <eros/Logger.h>
#include <gtest/gtest.h>
#include <ros_hats/Channel/DigitalOutputChannel.h>
#include <ros_hats/Hat/RelayHat.h>
#include <ros_hats/Port/DigitalOutputPort.h>
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
    for (uint8_t i = 1; i < (uint8_t)(RelayHat::HatModel::END_OF_LIST); ++i) {
        EXPECT_FALSE(RelayHat::HatModelString((RelayHat::HatModel)(i)) == "UNKNOWN");
        if ((RelayHat::HatModel)(i) != RelayHat::HatModel::UNKNOWN) {
            printf("\t[%d] Supported Hat Model: %s\n",
                   counter,
                   RelayHat::HatModelString((RelayHat::HatModel)(i)).c_str());
            counter++;
        }
    }
}
TEST(BasicTest, TestOperation_RelayHat) {
#ifdef __arm__
    printf(
        "NOTE: Executing this test DOES NOT Require to be ran with a Hat Installed.  It SHOULD "
        "Only be ran on a Raspberry Pi.\n");
    printf(
        "While this test is running, if a Relay Hat is installed you should hear a series of "
        "clicks.\n");
    {
        printf(
            "Testing RPi Relay Hat.  This Test assumes the default configuration of jumpers "
            "installed.\n");
        Logger* logger = new Logger("DEBUG", "UnitTest_RelayHat");
        RelayHat hat(RelayHat::HatModel::RPI_RELAY_HAT);
        HatConfig _config("RelayHat1", "RelayHat", "RPi Relay Hat", true);
        EXPECT_TRUE(hat.init(logger, _config));
        EXPECT_TRUE(hat.get_diagnostic().level <= Level::Type::NOTICE);

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
    }
#else
    printf("[WARN]: Not running tests as this is only supported on Raspberry Pi's.\n");
#endif
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
